#pragma once

#include "raylib.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

struct StringSnapshot {
    static constexpr std::size_t kDisplayNodes = 65;
    static constexpr std::size_t kHistoryNodes = 90;
    static constexpr std::size_t kSpectrumBins = 64;

    std::array<float, kDisplayNodes> displacement{};
    std::array<float, kHistoryNodes> temperatureHistory{};
    std::array<float, kSpectrumBins> spectrumDb{};
    float temperature = 20.0f;
    float ambientTemperature = 20.0f;
    float heatingPower = 0.0f;
    float coolingPower = 0.0f;
    float mechanicalEnergy = 0.0f;
    float thermalExcessEnergy = 0.0f;
    float generatedHeat = 0.0f;
    float heatToEnvironment = 0.0f;
    float numericalResidual = 0.0f;
    float tension = 80.0f;
    float dampingRate = 0.8f;
    float heatingFraction = 0.85f;
    float heatLossCoefficient = 0.035f;
    float pickupPosition = 0.5f;
    float pluckAmplitude = 0.008f;
    float length = 0.65f;
    float waveSpeed = 0.0f;
    float fundamentalFrequency = 0.0f;
    float simulatedTime = 0.0f;
    float displacementScale = 5.0f;
    float spectrumPeakFrequency = 0.0f;
    float spectrumPeakDb = -80.0f;
    int historyCount = 0;
    uint32_t clippingCount = 0;
    bool audioReady = false;
    bool muted = false;
    bool paused = false;
};

class VibratingStringEngine {
public:
    VibratingStringEngine();
    ~VibratingStringEngine();

    bool StartAudio();
    void StopAudio();
    void Update(double dt);

    void SetActive(bool active);
    void SetPaused(bool paused);
    void SetMuted(bool muted);
    void RequestReset();
    void RequestPluck();
    void PlayFrequency(float frequency);
    void RequestHotStart();

    void AdjustTension(float delta);
    void AdjustDamping(float delta);
    void AdjustHeatingFraction(float delta);
    void AdjustHeatLoss(float delta);
    void AdjustPluckAmplitude(float delta);
    void AdjustPickupPosition(float delta);

    StringSnapshot ReadSnapshot() const;

private:
    static constexpr std::size_t kSegments = 128;
    static constexpr std::size_t kNodes = kSegments + 1;
    static constexpr unsigned int kSampleRate = 48000;
    static constexpr int kSubsteps = 4;
    static constexpr uint32_t kCommandReset = 1u << 0;
    static constexpr uint32_t kCommandPluck = 1u << 1;
    static constexpr uint32_t kCommandHotStart = 1u << 2;

    static void AudioCallback(void* bufferData, unsigned int frames);
    void ProcessAudio(float* output, unsigned int frames);
    void ApplyRequests();
    void ResetModel();
    void PluckModel();
    void StepSubstep(double dt);
    void PublishSnapshot();
    void RecordTemperatureHistory();
    void ComputeSpectrum();
    double ComputeMechanicalEnergy() const;
    double PickupDisplacement() const;
    void StoreAtomic(std::atomic<float>& target, float value);

    AudioStream audioStream_{};
    bool streamLoaded_ = false;
    bool audioDeviceOwned_ = false;
    VibratingStringEngine* callbackOwner_ = nullptr;

    std::array<double, kNodes> displacement_{};
    std::array<double, kNodes> velocity_{};
    std::array<double, kNodes> acceleration_{};
    std::array<float, StringSnapshot::kHistoryNodes> temperatureHistory_{};

    double length_ = 0.65;
    double radius_ = 0.00045;
    double density_ = 7800.0;
    double specificHeat_ = 385.0;
    double massPerLength_ = 0.0;
    double dx_ = 0.0;
    double waveSpeed_ = 0.0;
    double time_ = 0.0;
    double temperature_ = 20.0;
    double generatedHeat_ = 0.0;
    double heatToEnvironment_ = 0.0;
    double initialMechanicalEnergy_ = 0.0;
    double historyTimer_ = 0.0;
    double snapshotTimer_ = 0.0;
    double spectrumTimer_ = 0.0;
    double previousAudioInput_ = 0.0;
    double previousAudioOutput_ = 0.0;
    double previousAudioLowpass_ = 0.0;
    static constexpr std::size_t kFftSize = 512;
    std::array<float, kFftSize> audioHistory_{};
    std::size_t audioHistoryWrite_ = 0;
    std::array<float, kFftSize> fftReal_{};
    std::array<float, kFftSize> fftImag_{};

    std::atomic<float> requestedTension_{80.0f};
    std::atomic<float> requestedDamping_{0.8f};
    std::atomic<float> requestedHeatingFraction_{0.85f};
    std::atomic<float> requestedHeatLoss_{0.035f};
    std::atomic<float> requestedPickupPosition_{0.5f};
    std::atomic<float> requestedPluckAmplitude_{0.008f};
    std::atomic<float> requestedAmbientTemperature_{20.0f};
    std::atomic<uint32_t> pendingCommands_{kCommandReset | kCommandPluck};
    std::atomic<bool> active_{false};
    std::atomic<bool> paused_{false};
    std::atomic<bool> muted_{false};

    float tension_ = 80.0f;
    float dampingRate_ = 0.8f;
    float heatingFraction_ = 0.85f;
    float heatLossCoefficient_ = 0.035f;
    float pickupPosition_ = 0.5f;
    float pluckAmplitude_ = 0.008f;
    float ambientTemperature_ = 20.0f;
    uint32_t clippingCount_ = 0;

    std::array<std::atomic<float>, StringSnapshot::kDisplayNodes> snapshotDisplacement_{};
    std::array<std::atomic<float>, StringSnapshot::kHistoryNodes> snapshotTemperatureHistory_{};
    std::array<std::atomic<float>, StringSnapshot::kSpectrumBins> snapshotSpectrumDb_{};
    std::atomic<float> snapshotSpectrumPeakFrequency_{0.0f};
    std::atomic<float> snapshotSpectrumPeakDb_{-80.0f};
    std::atomic<float> snapshotTemperature_{20.0f};
    std::atomic<float> snapshotHeatingPower_{0.0f};
    std::atomic<float> snapshotCoolingPower_{0.0f};
    std::atomic<float> snapshotMechanicalEnergy_{0.0f};
    std::atomic<float> snapshotThermalExcessEnergy_{0.0f};
    std::atomic<float> snapshotGeneratedHeat_{0.0f};
    std::atomic<float> snapshotHeatToEnvironment_{0.0f};
    std::atomic<float> snapshotResidual_{0.0f};
    std::atomic<float> snapshotTension_{80.0f};
    std::atomic<float> snapshotDamping_{0.8f};
    std::atomic<float> snapshotHeatingFraction_{0.85f};
    std::atomic<float> snapshotHeatLoss_{0.035f};
    std::atomic<float> snapshotPickup_{0.5f};
    std::atomic<float> snapshotPluckAmplitude_{0.008f};
    std::atomic<float> snapshotWaveSpeed_{0.0f};
    std::atomic<float> snapshotFundamental_{0.0f};
    std::atomic<float> snapshotSimulatedTime_{0.0f};
    std::atomic<float> snapshotAmbient_{20.0f};
    std::atomic<int> snapshotHistoryCount_{0};
    std::atomic<uint32_t> snapshotClippingCount_{0};
    std::atomic<bool> snapshotAudioReady_{false};
    std::atomic<bool> snapshotMuted_{false};
    std::atomic<bool> snapshotPaused_{false};
};
