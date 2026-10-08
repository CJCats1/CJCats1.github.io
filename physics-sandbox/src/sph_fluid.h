#pragma once

#include "flip_fluid.h"

#include <algorithm>
#include <cmath>
#include <vector>

class SphFluid {
public:
    static constexpr int kGridWidth = FlipFluid::kGridWidth;
    static constexpr int kGridHeight = FlipFluid::kGridHeight;
    static constexpr int kGridDepth = FlipFluid::kGridDepth;
    static constexpr double kCellSize = FlipFluid::kCellSize;
    static constexpr double kWorldWidth = FlipFluid::kWorldWidth;
    static constexpr double kWorldHeight = FlipFluid::kWorldHeight;
    static constexpr double kWorldDepth = FlipFluid::kWorldDepth;

    SphFluid() : bins_(static_cast<std::size_t>(kGridWidth * kGridHeight * kGridDepth)) { Reset(); }

    void Reset() {
        particles_.clear();
        simulatedTime_ = 0.0f;
        gravity_ = -9.81f;
        stiffness_ = 1800.0f;
        viscosity_ = 0.08f;
        maxDensityError_ = 0.0f;
        substeps_ = 1;
        for (int y = 1; y < 7; ++y) for (int z = 2; z < 6; ++z) for (int x = 4; x < 18; ++x) {
            particles_.push_back({{(x + 0.5) * kCellSize, (y + 0.5) * kCellSize, (z + 0.5) * kCellSize}, {}});
        }
        densities_.assign(particles_.size(), kRestDensity);
        pressures_.assign(particles_.size(), 0.0f);
    }

    void Step(double dt) {
        const float clamped = static_cast<float>(std::clamp(dt, 0.0, 0.05));
        if (clamped <= 0.0f || particles_.empty()) return;
        float maxSpeed = 0.0f;
        for (const FlipParticle& particle : particles_) maxSpeed = std::max(maxSpeed, Speed(particle.velocity));
        substeps_ = std::clamp(static_cast<int>(std::ceil((maxSpeed * clamped + std::abs(gravity_) * clamped * clamped) / (0.35f * static_cast<float>(kCellSize)))), 1, 6);
        const float h = clamped / static_cast<float>(substeps_);
        for (int step = 0; step < substeps_; ++step) {
            BuildSpatialHash();
            ComputeDensityAndPressure();
            Integrate(h);
            simulatedTime_ += h;
        }
    }

    void Poke(FlipVec3 point, FlipVec3 impulse) {
        for (FlipParticle& particle : particles_) {
            const FlipVec3 offset = {particle.position.x - point.x, particle.position.y - point.y, particle.position.z - point.z};
            const double distance = std::sqrt(offset.x * offset.x + offset.y * offset.y + offset.z * offset.z);
            if (distance < 1.5) {
                const float falloff = static_cast<float>(1.0 - distance / 1.5);
                particle.velocity.x += static_cast<float>(impulse.x) * falloff;
                particle.velocity.y += static_cast<float>(impulse.y) * falloff;
                particle.velocity.z += static_cast<float>(impulse.z) * falloff;
            }
        }
    }

    void AddParticleLayer() {
        for (int z = 2; z < 6; ++z) for (int x = 4; x < 18; ++x) {
            particles_.push_back({{(x + 0.5) * kCellSize, 7.0, (z + 0.5) * kCellSize}, {}});
        }
        densities_.resize(particles_.size(), kRestDensity);
        pressures_.resize(particles_.size(), 0.0f);
    }

    void AdjustStiffness(float delta) { stiffness_ = std::clamp(stiffness_ + delta, 100.0f, 10000.0f); }
    void AdjustViscosity(float delta) { viscosity_ = std::clamp(viscosity_ + delta, 0.0f, 2.0f); }
    void AdjustGravity(float delta) { gravity_ = std::clamp(gravity_ + delta, -30.0f, 0.0f); }

    const std::vector<FlipParticle>& Particles() const { return particles_; }
    float GetSmoothingRadius() const { return static_cast<float>(kSmoothingRadius); }
    float GetAverageNeighbors() const { return averageNeighbors_; }
    float GetStiffness() const { return stiffness_; }
    float GetViscosity() const { return viscosity_; }
    float GetMaxDensityError() const { return maxDensityError_; }
    float GetMaxSpeed() const { float maximum = 0.0f; for (const FlipParticle& p : particles_) maximum = std::max(maximum, Speed(p.velocity)); return maximum; }
    float GetGravity() const { return gravity_; }
    float GetSimulatedTime() const { return simulatedTime_; }
    int GetSubsteps() const { return substeps_; }

private:
    static constexpr double kSmoothingRadius = 0.85;
    static constexpr double kRestDensity = 18.0;
    static constexpr double kPressureScale = 0.018;
    static constexpr double kParticleMass = 1.0;

    static float Speed(FlipVec3 value) { return static_cast<float>(std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z)); }
    static int BinIndex(int x, int y, int z) { return x + kGridWidth * (y + kGridHeight * z); }

    void BuildSpatialHash() {
        for (auto& bin : bins_) bin.clear();
        for (std::size_t i = 0; i < particles_.size(); ++i) {
            const FlipVec3& position = particles_[i].position;
            const int x = std::clamp(static_cast<int>(position.x / kCellSize), 0, kGridWidth - 1);
            const int y = std::clamp(static_cast<int>(position.y / kCellSize), 0, kGridHeight - 1);
            const int z = std::clamp(static_cast<int>(position.z / kCellSize), 0, kGridDepth - 1);
            bins_[static_cast<std::size_t>(BinIndex(x, y, z))].push_back(static_cast<int>(i));
        }
    }

    template <typename Function>
    void ForEachNeighbor(int particleIndex, Function function) const {
        const FlipVec3& position = particles_[static_cast<std::size_t>(particleIndex)].position;
        const int cellX = std::clamp(static_cast<int>(position.x / kCellSize), 0, kGridWidth - 1);
        const int cellY = std::clamp(static_cast<int>(position.y / kCellSize), 0, kGridHeight - 1);
        const int cellZ = std::clamp(static_cast<int>(position.z / kCellSize), 0, kGridDepth - 1);
        const int range = static_cast<int>(std::ceil(kSmoothingRadius / kCellSize));
        for (int z = std::max(0, cellZ - range); z <= std::min(kGridDepth - 1, cellZ + range); ++z)
            for (int y = std::max(0, cellY - range); y <= std::min(kGridHeight - 1, cellY + range); ++y)
                for (int x = std::max(0, cellX - range); x <= std::min(kGridWidth - 1, cellX + range); ++x)
                    for (int neighbor : bins_[static_cast<std::size_t>(BinIndex(x, y, z))]) function(neighbor);
    }

    static double Kernel(double distance) {
        if (distance >= kSmoothingRadius) return 0.0;
        const double q = 1.0 - distance / kSmoothingRadius;
        return q * q * q;
    }

    void ComputeDensityAndPressure() {
        double neighborTotal = 0.0;
        maxDensityError_ = 0.0f;
        for (std::size_t i = 0; i < particles_.size(); ++i) {
            double density = 0.0;
            int neighborCount = 0;
            ForEachNeighbor(static_cast<int>(i), [&](int j) {
                const FlipVec3 delta = {particles_[j].position.x - particles_[i].position.x, particles_[j].position.y - particles_[i].position.y, particles_[j].position.z - particles_[i].position.z};
                const double distance = std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
                const double weight = Kernel(distance);
                if (weight > 0.0) { density += kParticleMass * weight; ++neighborCount; }
            });
            densities_[i] = static_cast<float>(density);
            pressures_[i] = static_cast<float>(stiffness_ * kPressureScale * (density - kRestDensity) / kRestDensity);
            maxDensityError_ = std::max(maxDensityError_, static_cast<float>(std::abs(density / kRestDensity - 1.0)));
            neighborTotal += static_cast<double>(neighborCount);
        }
        averageNeighbors_ = particles_.empty() ? 0.0f : static_cast<float>(neighborTotal / static_cast<double>(particles_.size()));
    }

    void Integrate(float dt) {
        std::vector<FlipVec3> acceleration(particles_.size(), {0.0, 0.0, static_cast<double>(gravity_)});
        for (std::size_t i = 0; i < particles_.size(); ++i) {
            ForEachNeighbor(static_cast<int>(i), [&](int j) {
                if (j == static_cast<int>(i)) return;
                const FlipVec3 delta = {particles_[j].position.x - particles_[i].position.x, particles_[j].position.y - particles_[i].position.y, particles_[j].position.z - particles_[i].position.z};
                const double distance = std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
                if (distance < 1.0e-6 || distance >= kSmoothingRadius) return;
                const FlipVec3 direction = {delta.x / distance, delta.y / distance, delta.z / distance};
                const double gradient = -3.0 * std::pow(1.0 - distance / kSmoothingRadius, 2.0) / kSmoothingRadius;
                const double pressureForce = -(pressures_[i] + pressures_[static_cast<std::size_t>(j)]) * 0.5 * gradient / std::max(0.1f, densities_[static_cast<std::size_t>(j)]);
                acceleration[i].x += direction.x * pressureForce;
                acceleration[i].y += direction.y * pressureForce;
                acceleration[i].z += direction.z * pressureForce;
                const double viscosityForce = viscosity_ * (particles_[j].velocity.x - particles_[i].velocity.x) * Kernel(distance) / std::max(0.1f, densities_[static_cast<std::size_t>(j)]);
                acceleration[i].x += viscosityForce;
                acceleration[i].y += viscosity_ * (particles_[j].velocity.y - particles_[i].velocity.y) * Kernel(distance) / std::max(0.1f, densities_[static_cast<std::size_t>(j)]);
                acceleration[i].z += viscosity_ * (particles_[j].velocity.z - particles_[i].velocity.z) * Kernel(distance) / std::max(0.1f, densities_[static_cast<std::size_t>(j)]);
            });
        }
        for (std::size_t i = 0; i < particles_.size(); ++i) {
            FlipParticle& particle = particles_[i];
            particle.velocity.x += acceleration[i].x * dt;
            particle.velocity.y += acceleration[i].y * dt;
            particle.velocity.z += acceleration[i].z * dt;
            const float speed = Speed(particle.velocity);
            if (speed > 18.0f) {
                const double scale = 18.0 / speed;
                particle.velocity.x *= scale;
                particle.velocity.y *= scale;
                particle.velocity.z *= scale;
            }
            particle.position.x += particle.velocity.x * dt;
            particle.position.y += particle.velocity.y * dt;
            particle.position.z += particle.velocity.z * dt;
            Collide(particle.position.x, particle.velocity.x, 0.12, kWorldWidth - 0.12);
            Collide(particle.position.y, particle.velocity.y, 0.18, kWorldHeight - 0.18);
            Collide(particle.position.z, particle.velocity.z, 0.12, kWorldDepth - 0.12);
        }
    }

    static void Collide(double& position, double& velocity, double minimum, double maximum) {
        if (position < minimum) { position = minimum; velocity = std::abs(velocity) * 0.35; }
        if (position > maximum) { position = maximum; velocity = -std::abs(velocity) * 0.35; }
    }

    std::vector<FlipParticle> particles_;
    std::vector<float> densities_;
    std::vector<float> pressures_;
    std::vector<std::vector<int>> bins_;
    float gravity_ = -9.81f;
    float stiffness_ = 1800.0f;
    float viscosity_ = 0.08f;
    float simulatedTime_ = 0.0f;
    float maxDensityError_ = 0.0f;
    float averageNeighbors_ = 0.0f;
    int substeps_ = 1;
};
