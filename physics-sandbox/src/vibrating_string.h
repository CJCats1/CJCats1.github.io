#pragma once

#include "raylib.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

struct StringSnapshot {
    static constexpr std::size_t kDisplayNodes = 65;
    static constexpr std::size_t kSpectrumBins = 64;
    static constexpr std::size_t kHistoryNodes = 90;

    float length = 3.0f;
    float temperature = 20.0f;
    float ambientTemperature = 20.0f;
    float heatingPower = 0.0f;
    float coolingPower = 0.0f;
    float generatedHeat = 0.0f;
    float mechanicalEnergy = 0.0f;
    float numericalResidual = 0.0f;
    float fundamentalFrequency = 110.0f;
    float waveSpeed = 660.0f;
    float spectrumPeakFrequency = 110.0f;
    float spectrumPeakDb = -80.0f;
    float pluckAmplitude = 0.045f;
    float displacementScale = 5.0f;
    int historyCount = 90;
    std::array<float, kDisplayNodes> displacement{};
    std::array<float, kSpectrumBins> spectrumDb{};
    std::array<float, kHistoryNodes> temperatureHistory{};
};

class VibratingStringEngine {
public:
    VibratingStringEngine() { RequestReset(); }
    ~VibratingStringEngine() { StopAudio(); }

    VibratingStringEngine(const VibratingStringEngine&) = delete;
    VibratingStringEngine& operator=(const VibratingStringEngine&) = delete;

    bool StartAudio() {
        if (audioReady_) return true;
        if (!IsAudioDeviceReady()) InitAudioDevice();
        if (!IsAudioDeviceReady()) return false;
        SetAudioStreamBufferSizeDefault(kAudioBufferSize);
        audioStream_ = LoadAudioStream(kAudioSampleRate, 32, 1);
        if (!IsAudioStreamValid(audioStream_)) { CloseAudioDevice(); return false; }
        PlayAudioStream(audioStream_);
        audioReady_ = true;
        return true;
    }

    void StopAudio() {
        if (!audioReady_) return;
        StopAudioStream(audioStream_);
        UnloadAudioStream(audioStream_);
        CloseAudioDevice();
        audioReady_ = false;
    }

    void SetActive(bool active) {
        active_ = active;
        if (!audioReady_) return;
        if (active_) ResumeAudioStream(audioStream_);
        else PauseAudioStream(audioStream_);
    }
    void SetPaused(bool paused) { paused_ = paused; }
    void SetMuted(bool muted) { muted_ = muted; }

    void RequestPluck() { pluckPending_ = true; }
    void RequestHotStart() { snapshot_.temperature = 85.0f; }

    void RequestReset() {
        snapshot_ = {};
        snapshot_.length = 3.0f;
        snapshot_.temperature = 20.0f;
        snapshot_.ambientTemperature = 20.0f;
        snapshot_.fundamentalFrequency = 110.0f;
        snapshot_.waveSpeed = 660.0f;
        snapshot_.spectrumPeakFrequency = 110.0f;
        snapshot_.spectrumPeakDb = -80.0f;
        snapshot_.pluckAmplitude = 0.045f;
        snapshot_.displacementScale = 5.0f;
        snapshot_.historyCount = static_cast<int>(StringSnapshot::kHistoryNodes);
        for (std::size_t i = 0; i < snapshot_.temperatureHistory.size(); ++i) snapshot_.temperatureHistory[i] = snapshot_.temperature;
        for (std::size_t i = 0; i < snapshot_.spectrumDb.size(); ++i) snapshot_.spectrumDb[i] = -80.0f;

        damping_ = 0.25f;
        heatingFraction_ = 0.25f;
        heatLoss_ = 0.05f;
        pickupPosition_ = 0.5f;
        generatedHeat_ = 0.0f;
        audioAmplitude_ = 0.0f;
        audioPhase_ = 0.0;
        previousMechanicalEnergy_ = 0.0f;
        pluckPending_ = false;
        std::fill(displacement_.begin(), displacement_.end(), 0.0f);
        std::fill(velocity_.begin(), velocity_.end(), 0.0f);
        std::fill(audioHistory_.begin(), audioHistory_.end(), 0.0f);
        audioHistoryWrite_ = 0;
        UpdateSnapshot();
    }

    void PlayFrequency(float frequency) {
        snapshot_.fundamentalFrequency = std::clamp(frequency, 20.0f, 1000.0f);
        snapshot_.waveSpeed = snapshot_.fundamentalFrequency * 2.0f * snapshot_.length;
        snapshot_.spectrumPeakFrequency = snapshot_.fundamentalFrequency;
    }
    void AdjustTension(float delta) { PlayFrequency(snapshot_.fundamentalFrequency + delta * 2.0f); }
    void AdjustDamping(float delta) { damping_ = std::clamp(damping_ + delta, 0.0f, 3.0f); }
    void AdjustPluckAmplitude(float delta) { snapshot_.pluckAmplitude = std::clamp(snapshot_.pluckAmplitude + delta, 0.001f, 0.25f); }
    void AdjustHeatingFraction(float delta) { heatingFraction_ = std::clamp(heatingFraction_ + delta, 0.0f, 1.0f); }
    void AdjustHeatLoss(float delta) { heatLoss_ = std::clamp(heatLoss_ + delta, 0.0f, 2.0f); }
    void AdjustPickupPosition(float delta) { pickupPosition_ = std::clamp(pickupPosition_ + delta, 0.05f, 0.95f); }

    void Step(float dt) {
        if (paused_) return;
        const float clampedDt = std::clamp(dt, 0.0f, 0.05f);
        if (clampedDt <= 0.0f) return;
        if (pluckPending_) { ApplyPluck(); pluckPending_ = false; }

        const float dx = snapshot_.length / static_cast<float>(kSegments);
        const float c = snapshot_.waveSpeed;
        const int substeps = std::clamp(static_cast<int>(std::ceil(clampedDt * c / (0.35f * dx))), 1, 512);
        const float h = clampedDt / static_cast<float>(substeps);
        double dissipatedPower = 0.0;
        for (int substep = 0; substep < substeps; ++substep) {
            for (std::size_t i = 1; i < kNodes - 1; ++i) {
                const float laplacian = (displacement_[i - 1] - 2.0f * displacement_[i] + displacement_[i + 1]) / (dx * dx);
                velocity_[i] += (c * c * laplacian - damping_ * velocity_[i]) * h;
                displacement_[i] += velocity_[i] * h;
            }
            displacement_.front() = 0.0f;
            displacement_.back() = 0.0f;
            for (std::size_t i = 1; i < kNodes - 1; ++i) dissipatedPower += damping_ * velocity_[i] * velocity_[i] * h;
        }

        const float pickup = SampleDisplacement(pickupPosition_);
        const float heat = static_cast<float>(heatingFraction_ * dissipatedPower);
        snapshot_.heatingPower = heat / clampedDt;
        snapshot_.coolingPower = heatLoss_ * (snapshot_.temperature - snapshot_.ambientTemperature);
        snapshot_.temperature += (heat - snapshot_.coolingPower * clampedDt) / kThermalCapacity;
        generatedHeat_ += heat;
        snapshot_.generatedHeat = generatedHeat_;
        snapshot_.mechanicalEnergy = ComputeMechanicalEnergy(dx);
        snapshot_.numericalResidual = snapshot_.mechanicalEnergy - previousMechanicalEnergy_ + static_cast<float>(dissipatedPower);
        previousMechanicalEnergy_ = snapshot_.mechanicalEnergy;
        PushAudioHistory(pickup);
        UpdateSnapshot();
        UpdateAudio();
    }

    StringSnapshot ReadSnapshot() const { return snapshot_; }

private:
    static constexpr std::size_t kSegments = 128;
    static constexpr std::size_t kNodes = kSegments + 1;
    static constexpr int kAudioSampleRate = 48000;
    static constexpr int kAudioBufferSize = 512;
    static constexpr float kThermalCapacity = 1.2f;

    void ApplyPluck() {
        for (std::size_t i = 0; i < kNodes; ++i) {
            const float x = static_cast<float>(i) / static_cast<float>(kSegments);
            displacement_[i] = snapshot_.pluckAmplitude * (x < 0.5f ? 2.0f * x : 2.0f * (1.0f - x));
            velocity_[i] = 0.0f;
        }
        audioAmplitude_ = snapshot_.pluckAmplitude;
    }

    float SampleDisplacement(float position) const {
        const float coordinate = std::clamp(position, 0.0f, 1.0f) * static_cast<float>(kSegments);
        const std::size_t left = std::min(kSegments - 1, static_cast<std::size_t>(coordinate));
        const float fraction = coordinate - static_cast<float>(left);
        return displacement_[left] * (1.0f - fraction) + displacement_[left + 1] * fraction;
    }

    float ComputeMechanicalEnergy(float dx) const {
        const float massPerNode = 0.001f * dx;
        double energy = 0.0;
        for (std::size_t i = 1; i < kNodes - 1; ++i) {
            const float slope = (displacement_[i + 1] - displacement_[i]) / dx;
            energy += 0.5 * massPerNode * (velocity_[i] * velocity_[i] + snapshot_.waveSpeed * snapshot_.waveSpeed * slope * slope);
        }
        return static_cast<float>(energy);
    }

    void PushAudioHistory(float sample) {
        audioHistory_[audioHistoryWrite_] = sample;
        audioHistoryWrite_ = (audioHistoryWrite_ + 1) % audioHistory_.size();
    }

    void UpdateSnapshot() {
        for (std::size_t i = 0; i < snapshot_.kDisplayNodes; ++i) snapshot_.displacement[i] = displacement_[std::min(kSegments, i * 2)];
        for (std::size_t i = snapshot_.temperatureHistory.size() - 1; i > 0; --i) snapshot_.temperatureHistory[i] = snapshot_.temperatureHistory[i - 1];
        snapshot_.temperatureHistory[0] = snapshot_.temperature;

        float strongest = -80.0f;
        std::size_t strongestBin = 1;
        for (std::size_t bin = 1; bin < snapshot_.kSpectrumBins; ++bin) {
            double real = 0.0;
            double imag = 0.0;
            for (std::size_t n = 0; n < audioHistory_.size(); ++n) {
                const double angle = 2.0 * 3.14159265358979323846 * static_cast<double>(bin * n) / static_cast<double>(audioHistory_.size());
                const float value = audioHistory_[(audioHistoryWrite_ + n) % audioHistory_.size()];
                real += value * std::cos(angle);
                imag -= value * std::sin(angle);
            }
            const float db = std::max(-80.0f, static_cast<float>(20.0 * std::log10(std::max(1.0e-6, std::sqrt(real * real + imag * imag) / static_cast<double>(audioHistory_.size())))));
            snapshot_.spectrumDb[bin] = db;
            if (db > strongest) { strongest = db; strongestBin = bin; }
        }
        snapshot_.spectrumDb[0] = -80.0f;
        snapshot_.spectrumPeakDb = strongest;
        snapshot_.spectrumPeakFrequency = strongest > -79.9f ? static_cast<float>(strongestBin) * static_cast<float>(kAudioSampleRate) / static_cast<float>(audioHistory_.size()) : snapshot_.fundamentalFrequency;
    }

    void UpdateAudio() {
        if (!audioReady_ || !active_ || !IsAudioStreamProcessed(audioStream_)) return;
        std::array<float, kAudioBufferSize> samples{};
        for (float& sample : samples) {
            sample = muted_ ? 0.0f : audioAmplitude_ * 8.0f * std::sin(static_cast<float>(audioPhase_));
            audioPhase_ += 2.0 * 3.14159265358979323846 * snapshot_.fundamentalFrequency / static_cast<double>(kAudioSampleRate);
            if (audioPhase_ > 2.0 * 3.14159265358979323846) audioPhase_ -= 2.0 * 3.14159265358979323846;
            audioAmplitude_ *= 0.9996f;
        }
        UpdateAudioStream(audioStream_, samples.data(), kAudioBufferSize);
    }

    StringSnapshot snapshot_{};
    std::array<float, kNodes> displacement_{};
    std::array<float, kNodes> velocity_{};
    std::array<float, 512> audioHistory_{};
    float damping_ = 0.25f;
    float heatingFraction_ = 0.25f;
    float heatLoss_ = 0.05f;
    float pickupPosition_ = 0.5f;
    float generatedHeat_ = 0.0f;
    float audioAmplitude_ = 0.0f;
    double audioPhase_ = 0.0;
    float previousMechanicalEnergy_ = 0.0f;
    std::size_t audioHistoryWrite_ = 0;
    bool pluckPending_ = false;
    bool active_ = false;
    bool paused_ = false;
    bool muted_ = false;
    bool audioReady_ = false;
    AudioStream audioStream_{};
};
