#include "vibrating_string.h"

#include <algorithm>
#include <cmath>

namespace {
VibratingStringEngine* gAudioOwner = nullptr;
constexpr double kPi = 3.14159265358979323846;
constexpr double kAudioGain = 42.0;
constexpr double kAudioLowpass = 0.30;
constexpr double kSoftLimitDrive = 1.15;
constexpr double kDcBlockerR = 0.995;
constexpr double kTemperatureHistoryPeriod = 1.0 / 30.0;
}

VibratingStringEngine::VibratingStringEngine() {
    massPerLength_ = density_ * kPi * radius_ * radius_;
    dx_ = length_ / static_cast<double>(kSegments);
    waveSpeed_ = std::sqrt(static_cast<double>(tension_) / massPerLength_);
    for (auto& value : snapshotDisplacement_) value.store(0.0f);
    for (auto& value : snapshotTemperatureHistory_) value.store(20.0f);
    for (auto& value : snapshotSpectrumDb_) value.store(-80.0f);
    audioHistory_.fill(0.0f);
    fftReal_.fill(0.0f);
    fftImag_.fill(0.0f);
    ResetModel();
    PublishSnapshot();
}

VibratingStringEngine::~VibratingStringEngine() { StopAudio(); }

bool VibratingStringEngine::StartAudio() {
    if (!IsAudioDeviceReady()) { InitAudioDevice(); audioDeviceOwned_ = true; }
    if (!IsAudioDeviceReady()) return false;
    audioStream_ = LoadAudioStream(kSampleRate, 32, 1);
    SetAudioStreamCallback(audioStream_, AudioCallback);
    callbackOwner_ = this;
    gAudioOwner = this;
    streamLoaded_ = true;
    PlayAudioStream(audioStream_);
    snapshotAudioReady_.store(true, std::memory_order_release);
    return true;
}

void VibratingStringEngine::StopAudio() {
    snapshotAudioReady_.store(false, std::memory_order_release);
    if (streamLoaded_) { StopAudioStream(audioStream_); UnloadAudioStream(audioStream_); streamLoaded_ = false; }
    callbackOwner_ = nullptr;
    gAudioOwner = nullptr;
    if (audioDeviceOwned_ && IsAudioDeviceReady()) { CloseAudioDevice(); audioDeviceOwned_ = false; }
}

void VibratingStringEngine::SetActive(bool active) { active_.store(active, std::memory_order_release); }
void VibratingStringEngine::SetPaused(bool paused) { paused_.store(paused, std::memory_order_release); }
void VibratingStringEngine::SetMuted(bool muted) { muted_.store(muted, std::memory_order_release); }
void VibratingStringEngine::RequestReset() { pendingCommands_.fetch_or(kCommandReset, std::memory_order_release); }
void VibratingStringEngine::RequestPluck() { pendingCommands_.fetch_or(kCommandPluck, std::memory_order_release); }
void VibratingStringEngine::PlayFrequency(float frequency) {
    const double targetTension = std::pow(2.0 * length_ * static_cast<double>(frequency), 2.0) * massPerLength_;
    requestedTension_.store(std::clamp(static_cast<float>(targetTension), 0.05f, 3000.0f), std::memory_order_relaxed);
    RequestPluck();
}
void VibratingStringEngine::RequestHotStart() { pendingCommands_.fetch_or(kCommandHotStart, std::memory_order_release); }
void VibratingStringEngine::AdjustTension(float delta) { requestedTension_.store(std::clamp(requestedTension_.load() + delta, 20.0f, 300.0f)); }
void VibratingStringEngine::AdjustDamping(float delta) { requestedDamping_.store(std::clamp(requestedDamping_.load() + delta, 0.0f, 8.0f)); }
void VibratingStringEngine::AdjustHeatingFraction(float delta) { requestedHeatingFraction_.store(std::clamp(requestedHeatingFraction_.load() + delta, 0.0f, 1.0f)); }
void VibratingStringEngine::AdjustHeatLoss(float delta) { requestedHeatLoss_.store(std::clamp(requestedHeatLoss_.load() + delta, 0.0f, 1.0f)); }
void VibratingStringEngine::AdjustPluckAmplitude(float delta) { requestedPluckAmplitude_.store(std::clamp(requestedPluckAmplitude_.load() + delta, 0.001f, 0.03f)); }
void VibratingStringEngine::AdjustPickupPosition(float delta) { requestedPickupPosition_.store(std::clamp(requestedPickupPosition_.load() + delta, 0.05f, 0.95f)); }

void VibratingStringEngine::ApplyRequests() {
    tension_ = requestedTension_.load(std::memory_order_relaxed);
    dampingRate_ = requestedDamping_.load(std::memory_order_relaxed);
    heatingFraction_ = requestedHeatingFraction_.load(std::memory_order_relaxed);
    heatLossCoefficient_ = requestedHeatLoss_.load(std::memory_order_relaxed);
    pickupPosition_ = requestedPickupPosition_.load(std::memory_order_relaxed);
    pluckAmplitude_ = requestedPluckAmplitude_.load(std::memory_order_relaxed);
    ambientTemperature_ = requestedAmbientTemperature_.load(std::memory_order_relaxed);
    waveSpeed_ = std::sqrt(static_cast<double>(tension_) / massPerLength_);
    const uint32_t commands = pendingCommands_.exchange(0, std::memory_order_acq_rel);
    if (commands & kCommandReset) ResetModel();
    if (commands & kCommandHotStart) temperature_ = ambientTemperature_ + 80.0;
    if (commands & kCommandPluck) PluckModel();
}

void VibratingStringEngine::ResetModel() {
    displacement_.fill(0.0); velocity_.fill(0.0); acceleration_.fill(0.0);
    temperature_ = ambientTemperature_; generatedHeat_ = 0.0; heatToEnvironment_ = 0.0; time_ = 0.0;
    historyTimer_ = 0.0; snapshotTimer_ = 0.0; spectrumTimer_ = 0.0; previousAudioInput_ = 0.0; previousAudioOutput_ = 0.0; previousAudioLowpass_ = 0.0;
    audioHistory_.fill(0.0f); audioHistoryWrite_ = 0; clippingCount_ = 0;
    temperatureHistory_.fill(static_cast<float>(temperature_));
    snapshotSpectrumPeakFrequency_.store(0.0f, std::memory_order_relaxed);
    snapshotSpectrumPeakDb_.store(-80.0f, std::memory_order_relaxed);
    for (auto& value : snapshotSpectrumDb_) value.store(-80.0f, std::memory_order_relaxed);
    initialMechanicalEnergy_ = 0.0;
}

void VibratingStringEngine::PluckModel() {
    for (std::size_t i = 0; i <= kSegments; ++i) {
        const double position = static_cast<double>(i) / static_cast<double>(kSegments);
        displacement_[i] = position <= 0.5 ? static_cast<double>(pluckAmplitude_) * position / 0.5 : static_cast<double>(pluckAmplitude_) * (1.0 - position) / 0.5;
        velocity_[i] = 0.0;
    }
    displacement_[0] = 0.0; displacement_[kSegments] = 0.0;
    initialMechanicalEnergy_ = ComputeMechanicalEnergy(); generatedHeat_ = 0.0; heatToEnvironment_ = 0.0; time_ = 0.0; clippingCount_ = 0;
}

void VibratingStringEngine::StepSubstep(double dt) {
    const double dampingFactor = std::exp(-static_cast<double>(dampingRate_) * dt * 0.5);
    double dampingLoss = 0.0;
    for (std::size_t i = 1; i < kSegments; ++i) { const double before = 0.5 * massPerLength_ * dx_ * velocity_[i] * velocity_[i]; velocity_[i] *= dampingFactor; const double after = 0.5 * massPerLength_ * dx_ * velocity_[i] * velocity_[i]; dampingLoss += before - after; }
    const double c2 = waveSpeed_ * waveSpeed_;
    for (std::size_t i = 1; i < kSegments; ++i) { const double lap = (displacement_[i + 1] - 2.0 * displacement_[i] + displacement_[i - 1]) / (dx_ * dx_); velocity_[i] += 0.5 * dt * c2 * lap; }
    for (std::size_t i = 1; i < kSegments; ++i) displacement_[i] += dt * velocity_[i];
    displacement_[0] = 0.0; displacement_[kSegments] = 0.0;
    for (std::size_t i = 1; i < kSegments; ++i) { const double lap = (displacement_[i + 1] - 2.0 * displacement_[i] + displacement_[i - 1]) / (dx_ * dx_); velocity_[i] += 0.5 * dt * c2 * lap; }
    for (std::size_t i = 1; i < kSegments; ++i) { const double before = 0.5 * massPerLength_ * dx_ * velocity_[i] * velocity_[i]; velocity_[i] *= dampingFactor; const double after = 0.5 * massPerLength_ * dx_ * velocity_[i] * velocity_[i]; dampingLoss += before - after; }
    velocity_[0] = 0.0; velocity_[kSegments] = 0.0;
    const double heatCapacity = massPerLength_ * length_ * specificHeat_;
    const double heatPower = heatingFraction_ * dampingLoss / dt;
    const double coolingPower = static_cast<double>(heatLossCoefficient_) * (temperature_ - ambientTemperature_);
    const double decay = std::exp(-static_cast<double>(heatLossCoefficient_) * dt / heatCapacity);
    if (heatLossCoefficient_ > 1.0e-9f) temperature_ = ambientTemperature_ + (temperature_ - ambientTemperature_) * decay + heatPower / static_cast<double>(heatLossCoefficient_) * (1.0 - decay);
    else temperature_ += heatPower / heatCapacity * dt;
    generatedHeat_ += heatingFraction_ * dampingLoss; heatToEnvironment_ += coolingPower * dt; time_ += dt; historyTimer_ += dt; snapshotTimer_ += dt;
    if (historyTimer_ >= kTemperatureHistoryPeriod) RecordTemperatureHistory();
}

void VibratingStringEngine::RecordTemperatureHistory() {
    historyTimer_ = 0.0;
    for (std::size_t i = 1; i < temperatureHistory_.size(); ++i) temperatureHistory_[i - 1] = temperatureHistory_[i];
    temperatureHistory_.back() = static_cast<float>(temperature_);
}

double VibratingStringEngine::ComputeMechanicalEnergy() const {
    double kinetic = 0.0; for (std::size_t i = 1; i < kSegments; ++i) kinetic += 0.5 * massPerLength_ * dx_ * velocity_[i] * velocity_[i];
    double potential = 0.0; for (std::size_t i = 0; i < kSegments; ++i) { const double slope = (displacement_[i + 1] - displacement_[i]) / dx_; potential += 0.5 * tension_ * dx_ * slope * slope; }
    return kinetic + potential;
}

double VibratingStringEngine::PickupDisplacement() const {
    const double node = pickupPosition_ * static_cast<double>(kSegments); const std::size_t left = std::min<std::size_t>(static_cast<std::size_t>(node), kSegments - 1); const double fraction = node - static_cast<double>(left);
    return displacement_[left] * (1.0 - fraction) + displacement_[left + 1] * fraction;
}

void VibratingStringEngine::ComputeSpectrum() {
    constexpr float pi = 3.14159265358979323846f;
    for (std::size_t i = 0; i < kFftSize; ++i) { const std::size_t source = (audioHistoryWrite_ + i) % kFftSize; const float window = 0.5f * (1.0f - std::cos(2.0f * pi * static_cast<float>(i) / static_cast<float>(kFftSize - 1))); fftReal_[i] = audioHistory_[source] * window; fftImag_[i] = 0.0f; }
    for (std::size_t i = 1, j = 0; i < kFftSize; ++i) { std::size_t bit = kFftSize >> 1; for (; j & bit; bit >>= 1) j ^= bit; j ^= bit; if (i < j) { std::swap(fftReal_[i], fftReal_[j]); std::swap(fftImag_[i], fftImag_[j]); } }
    for (std::size_t length = 2; length <= kFftSize; length <<= 1) { const float angle = -2.0f * pi / static_cast<float>(length); const float wReal = std::cos(angle); const float wImag = std::sin(angle); for (std::size_t start = 0; start < kFftSize; start += length) { float currentReal = 1.0f; float currentImag = 0.0f; const std::size_t half = length >> 1; for (std::size_t offset = 0; offset < half; ++offset) { const std::size_t even = start + offset; const std::size_t odd = even + half; const float productReal = currentReal * fftReal_[odd] - currentImag * fftImag_[odd]; const float productImag = currentReal * fftImag_[odd] + currentImag * fftReal_[odd]; fftReal_[odd] = fftReal_[even] - productReal; fftImag_[odd] = fftImag_[even] - productImag; fftReal_[even] += productReal; fftImag_[even] += productImag; const float nextReal = currentReal * wReal - currentImag * wImag; currentImag = currentReal * wImag + currentImag * wReal; currentReal = nextReal; } } }
    float peakDb = -80.0f; std::size_t peakBin = 0;
    for (std::size_t bin = 0; bin < StringSnapshot::kSpectrumBins; ++bin) { const float magnitude = 2.0f * std::sqrt(fftReal_[bin] * fftReal_[bin] + fftImag_[bin] * fftImag_[bin]) / static_cast<float>(kFftSize); const float db = 20.0f * std::log10(std::max(magnitude, 1.0e-6f)); snapshotSpectrumDb_[bin].store(db, std::memory_order_relaxed); if (bin > 0 && db > peakDb) { peakDb = db; peakBin = bin; } }
    snapshotSpectrumPeakFrequency_.store(static_cast<float>(peakBin) * static_cast<float>(kSampleRate) / static_cast<float>(kFftSize), std::memory_order_relaxed); snapshotSpectrumPeakDb_.store(peakDb, std::memory_order_relaxed);
}

void VibratingStringEngine::PublishSnapshot() {
    const double mechanicalEnergy = ComputeMechanicalEnergy(); const double heatCapacity = massPerLength_ * length_ * specificHeat_; const double thermalExcess = heatCapacity * (temperature_ - ambientTemperature_); const double residual = mechanicalEnergy + thermalExcess + heatToEnvironment_ + (1.0 - static_cast<double>(heatingFraction_)) * generatedHeat_ - initialMechanicalEnergy_; const double coolingPower = static_cast<double>(heatLossCoefficient_) * (temperature_ - ambientTemperature_);
    for (std::size_t i = 0; i < StringSnapshot::kDisplayNodes; ++i) { const std::size_t source = i * kSegments / (StringSnapshot::kDisplayNodes - 1); snapshotDisplacement_[i].store(static_cast<float>(displacement_[source]), std::memory_order_relaxed); }
    for (std::size_t i = 0; i < StringSnapshot::kHistoryNodes; ++i) snapshotTemperatureHistory_[i].store(temperatureHistory_[i], std::memory_order_relaxed);
    snapshotTemperature_.store(static_cast<float>(temperature_), std::memory_order_relaxed); snapshotHeatingPower_.store(static_cast<float>(heatingFraction_ * dampingRate_ * PickupDisplacement() * PickupDisplacement()), std::memory_order_relaxed); snapshotCoolingPower_.store(static_cast<float>(coolingPower), std::memory_order_relaxed); snapshotMechanicalEnergy_.store(static_cast<float>(mechanicalEnergy), std::memory_order_relaxed); snapshotThermalExcessEnergy_.store(static_cast<float>(thermalExcess), std::memory_order_relaxed); snapshotGeneratedHeat_.store(static_cast<float>(generatedHeat_), std::memory_order_relaxed); snapshotHeatToEnvironment_.store(static_cast<float>(heatToEnvironment_), std::memory_order_relaxed); snapshotResidual_.store(static_cast<float>(residual), std::memory_order_relaxed); snapshotTension_.store(tension_, std::memory_order_relaxed); snapshotDamping_.store(dampingRate_, std::memory_order_relaxed); snapshotHeatingFraction_.store(heatingFraction_, std::memory_order_relaxed); snapshotHeatLoss_.store(heatLossCoefficient_, std::memory_order_relaxed); snapshotPickup_.store(pickupPosition_, std::memory_order_relaxed); snapshotPluckAmplitude_.store(pluckAmplitude_, std::memory_order_relaxed); snapshotWaveSpeed_.store(static_cast<float>(waveSpeed_), std::memory_order_relaxed); snapshotFundamental_.store(static_cast<float>(waveSpeed_ / (2.0 * length_)), std::memory_order_relaxed); snapshotSimulatedTime_.store(static_cast<float>(time_), std::memory_order_relaxed); snapshotAmbient_.store(ambientTemperature_, std::memory_order_relaxed); snapshotHistoryCount_.store(static_cast<int>(std::min<std::size_t>(temperatureHistory_.size(), static_cast<std::size_t>(std::max(1.0, time_ / kTemperatureHistoryPeriod)))), std::memory_order_relaxed); snapshotClippingCount_.store(clippingCount_, std::memory_order_relaxed); snapshotMuted_.store(muted_.load(std::memory_order_relaxed), std::memory_order_relaxed); snapshotPaused_.store(paused_.load(std::memory_order_relaxed), std::memory_order_relaxed);
}

void VibratingStringEngine::ProcessAudio(float* output, unsigned int frames) {
    ApplyRequests();
    const bool active = active_.load(std::memory_order_acquire);
    const bool paused = paused_.load(std::memory_order_acquire);
    const bool muted = muted_.load(std::memory_order_acquire);
    const double dt = 1.0 / static_cast<double>(kSampleRate * kSubsteps);

    for (unsigned int frame = 0; frame < frames; ++frame) {
        if (active && !paused) {
            for (int substep = 0; substep < kSubsteps; ++substep) StepSubstep(dt);
            if (snapshotTimer_ >= 1.0 / 60.0) { snapshotTimer_ = 0.0; PublishSnapshot(); }
        }

        const double sample = active && !paused ? PickupDisplacement() * kAudioGain : 0.0;
        const double dcBlocked = sample - previousAudioInput_ + kDcBlockerR * previousAudioOutput_;
        previousAudioInput_ = sample;
        previousAudioOutput_ = dcBlocked;
        const double filtered = kAudioLowpass * dcBlocked + (1.0 - kAudioLowpass) * previousAudioLowpass_;
        previousAudioLowpass_ = filtered;
        const double limited = std::tanh(kSoftLimitDrive * filtered) / std::tanh(kSoftLimitDrive);
        output[frame] = muted || !active || paused ? 0.0f : static_cast<float>(limited);

        if (active && !paused) {
            audioHistory_[audioHistoryWrite_] = static_cast<float>(filtered);
            audioHistoryWrite_ = (audioHistoryWrite_ + 1) % kFftSize;
            spectrumTimer_ += 1.0 / static_cast<double>(kSampleRate);
            if (spectrumTimer_ >= 1.0 / 20.0) { spectrumTimer_ = 0.0; ComputeSpectrum(); }
        }
    }
}
void VibratingStringEngine::AudioCallback(void* bufferData, unsigned int frames) { if (gAudioOwner == nullptr) { std::fill_n(static_cast<float*>(bufferData), frames, 0.0f); return; } gAudioOwner->ProcessAudio(static_cast<float*>(bufferData), frames); }

StringSnapshot VibratingStringEngine::ReadSnapshot() const {
    StringSnapshot result; for (std::size_t i = 0; i < result.kDisplayNodes; ++i) result.displacement[i] = snapshotDisplacement_[i].load(std::memory_order_relaxed); for (std::size_t i = 0; i < result.kHistoryNodes; ++i) result.temperatureHistory[i] = snapshotTemperatureHistory_[i].load(std::memory_order_relaxed); for (std::size_t i = 0; i < result.kSpectrumBins; ++i) result.spectrumDb[i] = snapshotSpectrumDb_[i].load(std::memory_order_relaxed);
    result.temperature = snapshotTemperature_.load(); result.ambientTemperature = snapshotAmbient_.load(); result.heatingPower = snapshotHeatingPower_.load(); result.coolingPower = snapshotCoolingPower_.load(); result.mechanicalEnergy = snapshotMechanicalEnergy_.load(); result.thermalExcessEnergy = snapshotThermalExcessEnergy_.load(); result.generatedHeat = snapshotGeneratedHeat_.load(); result.heatToEnvironment = snapshotHeatToEnvironment_.load(); result.numericalResidual = snapshotResidual_.load(); result.tension = snapshotTension_.load(); result.dampingRate = snapshotDamping_.load(); result.heatingFraction = snapshotHeatingFraction_.load(); result.heatLossCoefficient = snapshotHeatLoss_.load(); result.pickupPosition = snapshotPickup_.load(); result.pluckAmplitude = snapshotPluckAmplitude_.load(); result.length = static_cast<float>(length_); result.waveSpeed = snapshotWaveSpeed_.load(); result.fundamentalFrequency = snapshotFundamental_.load(); result.simulatedTime = snapshotSimulatedTime_.load(); result.historyCount = snapshotHistoryCount_.load(); result.clippingCount = snapshotClippingCount_.load(); result.audioReady = snapshotAudioReady_.load(); result.muted = snapshotMuted_.load(); result.paused = snapshotPaused_.load(); result.displacementScale = 5.0f; result.spectrumPeakFrequency = snapshotSpectrumPeakFrequency_.load(); result.spectrumPeakDb = snapshotSpectrumPeakDb_.load(); return result;
}
