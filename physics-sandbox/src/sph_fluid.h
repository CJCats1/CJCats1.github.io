#pragma once

#include "flip_fluid.h"

#include <cstddef>
#include <vector>

struct SphFluidSnapshot {
    std::vector<FlipParticle> particles;
    float smoothingRadius = 0.65f;
    float stiffness = 1800.0f;
    float viscosity = 0.12f;
    float gravity = -9.81f;
    float simulatedTime = 0.0f;
    float maxDensityError = 0.0f;
    float maxSpeed = 0.0f;
    float averageNeighbors = 0.0f;
    int substeps = 1;
};

class SphFluid {
public:
    static constexpr int kGridWidth = 24;
    static constexpr int kGridHeight = 16;
    static constexpr int kGridDepth = 8;
    static constexpr double kCellSize = 0.5;
    static constexpr double kWorldWidth = kGridWidth * kCellSize;
    static constexpr double kWorldHeight = kGridHeight * kCellSize;
    static constexpr double kWorldDepth = kGridDepth * kCellSize;
    static constexpr double kSmoothingRadius = 0.65;

    SphFluid();

    void Reset();
    void Step(double dt);
    void Poke(FlipVec3 point, FlipVec3 impulse);
    void AddParticleLayer();
    void AdjustStiffness(float delta);
    void AdjustViscosity(float delta);
    void AdjustGravity(float delta);

    SphFluidSnapshot ReadSnapshot() const;
    const std::vector<FlipParticle>& Particles() const { return particles_; }
    float GetSmoothingRadius() const { return static_cast<float>(kSmoothingRadius); }
    float GetStiffness() const { return static_cast<float>(stiffness_); }
    float GetViscosity() const { return static_cast<float>(viscosity_); }
    float GetGravity() const { return static_cast<float>(gravity_); }
    float GetSimulatedTime() const { return static_cast<float>(simulatedTime_); }
    float GetMaxDensityError() const { return maxDensityError_; }
    float GetMaxSpeed() const { return maxSpeed_; }
    float GetAverageNeighbors() const { return averageNeighbors_; }
    int GetSubsteps() const { return substeps_; }

private:
    std::size_t HashIndex(int i, int j, int k) const;
    bool IsInside(FlipVec3 point, double margin = 0.0) const;
    void SeedParticles();
    void BuildSpatialHash();
    void ComputeDensities();
    void ComputeForces();
    void Integrate(double dt);
    void SimulateSubstep(double dt);

    std::vector<FlipParticle> particles_;
    std::vector<double> densities_;
    std::vector<double> pressures_;
    std::vector<FlipVec3> accelerations_;
    std::vector<int> cellHead_;
    std::vector<int> nextParticle_;
    std::vector<int> neighborIndices_;
    std::vector<std::size_t> neighborOffsets_;

    double stiffness_ = 1800.0;
    double viscosity_ = 0.12;
    double gravity_ = -9.81;
    double particleMass_ = 10.0;
    double simulatedTime_ = 0.0;
    float maxDensityError_ = 0.0f;
    float maxSpeed_ = 0.0f;
    float averageNeighbors_ = 0.0f;
    double maxAcceleration_ = 9.81;
    int nextSpawnLayer_ = 0;
    int substeps_ = 1;
};
