#pragma once

#include <cstddef>
#include <vector>

struct FlipVec2 {
    double x = 0.0;
    double y = 0.0;
};

struct FlipVec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct FlipParticle {
    FlipVec3 position{};
    FlipVec3 velocity{};
};

struct FlipSolidBox {
    FlipVec3 center{};
    FlipVec3 halfExtents{};
};

struct FlipFluidSnapshot {
    std::vector<FlipParticle> particles;
    std::vector<float> pressure;
    std::vector<unsigned char> fluidCells;
    float flipRatio = 0.0f;
    float gravity = -9.81f;
    float simulatedTime = 0.0f;
    float maxDivergence = 0.0f;
    float rmsDivergence = 0.0f;
    int pressureIterations = 0;
    int substeps = 1;
};

class FlipFluid {
public:
    // Deliberately modest dimensions: this is a CPU reference solver intended
    // to make the 3D transfers and pressure solve inspectable in real time.
    static constexpr int kGridWidth = 24;
    static constexpr int kGridHeight = 16;
    static constexpr int kGridDepth = 8;
    static constexpr double kCellSize = 0.5;
    static constexpr double kWorldWidth = kGridWidth * kCellSize;
    static constexpr double kWorldHeight = kGridHeight * kCellSize;
    static constexpr double kWorldDepth = kGridDepth * kCellSize;

    FlipFluid();

    void Reset();
    void Step(double dt);
    void Poke(FlipVec3 point, FlipVec3 impulse);
    void AddParticleLayer();
    void ToggleSolidObstacle();
    void MoveSolidObstacle(FlipVec3 velocity, double dt);
    void AdjustFlipRatio(float delta);
    void AdjustGravity(float delta);

    FlipFluidSnapshot ReadSnapshot() const;
    const std::vector<FlipParticle>& Particles() const { return particles_; }
    const std::vector<float>& Pressure() const { return pressure_; }
    const std::vector<unsigned char>& FluidCells() const { return fluidCells_; }
    const std::vector<unsigned char>& SolidCells() const { return solidCells_; }
    const FlipSolidBox& SolidObstacle() const { return solidObstacle_; }
    const FlipSolidBox& BuoyantBox() const { return buoyantBox_; }
    bool SolidObstacleEnabled() const { return solidObstacleEnabled_; }
    float GetBuoyantBoxSubmergedFraction() const { return static_cast<float>(buoyantBoxSubmergedFraction_); }
    float GetBuoyantBoxVerticalVelocity() const { return static_cast<float>(buoyantBoxVelocity_.y); }
    float GetFlipRatio() const { return static_cast<float>(flipRatio_); }
    float GetGravity() const { return static_cast<float>(gravity_); }
    float GetSimulatedTime() const { return static_cast<float>(simulatedTime_); }
    float GetMaxDivergence() const { return maxDivergence_; }
    float GetRmsDivergence() const { return rmsDivergence_; }
    int GetPressureIterations() const { return pressureIterations_; }
    int GetSubsteps() const { return substeps_; }

private:
    std::size_t CellIndex(int i, int j, int k) const;
    std::size_t UIndex(int i, int j, int k) const;
    std::size_t VIndex(int i, int j, int k) const;
    std::size_t WIndex(int i, int j, int k) const;
    bool IsInside(FlipVec3 point, double margin = 0.0) const;
    void SeedParticles();
    void SimulateSubstep(double dt);
    void BuildFluidCells();
    void BuildSolidCells();
    void UpdateBuoyantBox(double dt);
    double EstimateFluidSurface() const;
    void ResolveBuoyantBoxStaticCollision();
    void ScatterVelocities();
    void ApplyGravity(double dt);
    void EnforceWalls();
    void EnforceSolidWalls();
    void ComputeDivergence();
    void ProjectPressure(double dt);
    void CorrectVelocities(double dt);
    void AdvectParticles(double dt);
    void ResolveSolidParticleCollision(FlipParticle& particle) const;
    void ResolveBoxParticleCollision(FlipParticle& particle, const FlipSolidBox& box, FlipVec3 boxVelocity) const;
    double SampleComponent(const std::vector<double>& values, const std::vector<unsigned char>& valid,
                           int sizeX, int sizeY, int sizeZ, double offsetX, double offsetY, double offsetZ,
                           FlipVec3 point) const;
    double SampleU(FlipVec3 point) const;
    double SampleV(FlipVec3 point) const;
    double SampleW(FlipVec3 point) const;
    double SampleDeltaU(FlipVec3 point) const;
    double SampleDeltaV(FlipVec3 point) const;
    double SampleDeltaW(FlipVec3 point) const;
    void ComputeDiagnostics();

    std::vector<FlipParticle> particles_;
    std::vector<unsigned char> fluidCells_;
    std::vector<unsigned char> solidCells_;
    std::vector<float> pressure_;
    std::vector<float> nextPressure_;
    std::vector<float> divergence_;

    std::vector<double> u_;
    std::vector<double> v_;
    std::vector<double> w_;
    std::vector<double> oldU_;
    std::vector<double> oldV_;
    std::vector<double> oldW_;
    std::vector<double> deltaU_;
    std::vector<double> deltaV_;
    std::vector<double> deltaW_;
    std::vector<double> uWeights_;
    std::vector<double> vWeights_;
    std::vector<double> wWeights_;
    std::vector<unsigned char> uValid_;
    std::vector<unsigned char> vValid_;
    std::vector<unsigned char> wValid_;

    double gravity_ = -9.81;
    double flipRatio_ = 0.0;
    double simulatedTime_ = 0.0;
    float maxDivergence_ = 0.0f;
    float rmsDivergence_ = 0.0f;
    int pressureIterations_ = 0;
    int nextSpawnLayer_ = 0;
    int substeps_ = 1;
    FlipSolidBox solidObstacle_{{5.6, 1.9, 2.0}, {1.1, 1.4, 1.0}};
    FlipVec3 solidObstacleVelocity_{};
    bool solidObstacleEnabled_ = true;
    FlipSolidBox buoyantBox_{{8.6, 5.2, 2.0}, {0.8, 0.8, 0.8}};
    FlipVec3 buoyantBoxVelocity_{};
    double buoyantBoxSubmergedFraction_ = 0.0;
    double buoyantBoxMass_ = 1200.0;
};
