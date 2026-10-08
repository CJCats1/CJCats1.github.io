#include "sph_fluid.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kRestDensity = 1000.0;
constexpr double kParticleRadius = 0.09;
constexpr double kPokeRadius = 1.25;
constexpr double kParticleSpacing = 0.25;
constexpr double kPi = 3.14159265358979323846;
const double kPoly6Coefficient = 315.0 / (64.0 * kPi * std::pow(SphFluid::kSmoothingRadius, 9.0));
const double kSpikyGradientCoefficient = -45.0 / (kPi * std::pow(SphFluid::kSmoothingRadius, 6.0));
const double kViscosityLaplacianCoefficient = 45.0 / (kPi * std::pow(SphFluid::kSmoothingRadius, 6.0));
constexpr int kHashWidth = 19;
constexpr int kHashHeight = 13;
constexpr int kHashDepth = 7;

double Length(FlipVec3 value) {
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

FlipVec3 Add(FlipVec3 a, FlipVec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
FlipVec3 Subtract(FlipVec3 a, FlipVec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
FlipVec3 Scale(FlipVec3 value, double scalar) { return {value.x * scalar, value.y * scalar, value.z * scalar}; }

double Poly6(double radiusSquared, double smoothingRadius) {
    const double h2 = smoothingRadius * smoothingRadius;
    if (radiusSquared >= h2) return 0.0;
    const double difference = h2 - radiusSquared;
    return kPoly6Coefficient * difference * difference * difference;
}

double SpikyGradientMagnitude(double radius, double smoothingRadius) {
    if (radius <= 0.0 || radius >= smoothingRadius) return 0.0;
    return kSpikyGradientCoefficient * (smoothingRadius - radius) * (smoothingRadius - radius);
}

double ViscosityLaplacian(double radius, double smoothingRadius) {
    if (radius >= smoothingRadius) return 0.0;
    return kViscosityLaplacianCoefficient * (smoothingRadius - radius);
}

} // namespace

SphFluid::SphFluid()
    : particles_(),
      densities_(),
      pressures_(),
      accelerations_(),
      cellHead_(kHashWidth * kHashHeight * kHashDepth),
      nextParticle_() {
    Reset();
}

std::size_t SphFluid::HashIndex(int i, int j, int k) const {
    return static_cast<std::size_t>(i + kHashWidth * (j + kHashHeight * k));
}

bool SphFluid::IsInside(FlipVec3 point, double margin) const {
    return point.x >= margin && point.x <= kWorldWidth - margin &&
           point.y >= margin && point.y <= kWorldHeight - margin &&
           point.z >= margin && point.z <= kWorldDepth - margin;
}

void SphFluid::SeedParticles() {
    particles_.clear();
    for (int cellZ = 1; cellZ < 5; ++cellZ) {
        for (int cellY = 1; cellY < 7; ++cellY) {
            for (int cellX = 2; cellX < 12; ++cellX) {
                for (int subZ = 0; subZ < 2; ++subZ) {
                    for (int subY = 0; subY < 2; ++subY) {
                        for (int subX = 0; subX < 2; ++subX) {
                            const double x = (static_cast<double>(cellX) + (subX == 0 ? 0.25 : 0.75)) * 0.5;
                            const double y = (static_cast<double>(cellY) + (subY == 0 ? 0.25 : 0.75)) * 0.5;
                            const double z = (static_cast<double>(cellZ) + (subZ == 0 ? 0.25 : 0.75)) * 0.5;
                            particles_.push_back({{x, y, z}, {0.0, 0.0, 0.0}});
                        }
                    }
                }
            }
        }
    }
    densities_.resize(particles_.size(), kRestDensity);
    pressures_.resize(particles_.size(), 0.0);
    accelerations_.resize(particles_.size(), {0.0, gravity_, 0.0});
    nextParticle_.resize(particles_.size(), -1);
    neighborIndices_.reserve(particles_.size() * 72);
    neighborOffsets_.resize(particles_.size() + 1, 0);
}

void SphFluid::Reset() {
    simulatedTime_ = 0.0;
    maxDensityError_ = 0.0f;
    maxSpeed_ = 0.0f;
    averageNeighbors_ = 0.0f;
    substeps_ = 1;
    nextSpawnLayer_ = 0;
    SeedParticles();
    std::fill(cellHead_.begin(), cellHead_.end(), -1);
}

void SphFluid::AdjustStiffness(float delta) {
    stiffness_ = std::clamp(stiffness_ + static_cast<double>(delta), 250.0, 10000.0);
}

void SphFluid::AdjustViscosity(float delta) {
    viscosity_ = std::clamp(viscosity_ + static_cast<double>(delta), 0.0, 2.0);
}

void SphFluid::AdjustGravity(float delta) {
    gravity_ = std::clamp(gravity_ + static_cast<double>(delta), -30.0, 0.0);
}

void SphFluid::Poke(FlipVec3 point, FlipVec3 impulse) {
    if (!IsInside(point)) return;
    for (auto& particle : particles_) {
        const double distance = Length(Subtract(particle.position, point));
        if (distance >= kPokeRadius) continue;
        const double falloff = 1.0 - distance / kPokeRadius;
        particle.velocity = Add(particle.velocity, Scale(impulse, falloff));
    }
}

void SphFluid::AddParticleLayer() {
    // Add a fresh 4 x 1 x 2 voxel layer, with eight subcell particles per
    // voxel. The source tiles through the tank indefinitely; memory and
    // runtime are the only practical limits.
    const int sequence = nextSpawnLayer_++;
    const int cellY = 7 + sequence % 9;
    const int cellX = 2 + (sequence / 9) % 5 * 4;
    const int cellZ = 1 + (sequence / 45) % 2 * 2;
    for (int gridZ = cellZ; gridZ < cellZ + 2; ++gridZ) {
        for (int gridX = cellX; gridX < cellX + 4; ++gridX) {
            for (int subZ = 0; subZ < 2; ++subZ) {
                for (int subY = 0; subY < 2; ++subY) {
                    for (int subX = 0; subX < 2; ++subX) {
                        const double x = (static_cast<double>(gridX) + (subX == 0 ? 0.25 : 0.75)) * 0.5;
                        const double y = (static_cast<double>(cellY) + (subY == 0 ? 0.25 : 0.75)) * 0.5;
                        const double zPosition = (static_cast<double>(gridZ) + (subZ == 0 ? 0.25 : 0.75)) * 0.5;
                        particles_.push_back({{x, y, zPosition}, {0.0, 0.0, 0.0}});
                    }
                }
            }
        }
    }
    densities_.resize(particles_.size(), kRestDensity);
    pressures_.resize(particles_.size(), 0.0);
    accelerations_.resize(particles_.size(), {0.0, gravity_, 0.0});
    nextParticle_.resize(particles_.size(), -1);
    neighborOffsets_.resize(particles_.size() + 1, 0);
    neighborIndices_.reserve(particles_.size() * 72);
}

void SphFluid::BuildSpatialHash() {
    std::fill(cellHead_.begin(), cellHead_.end(), -1);
    for (std::size_t index = 0; index < particles_.size(); ++index) {
        const FlipVec3 position = particles_[index].position;
        const int i = std::clamp(static_cast<int>(position.x / kSmoothingRadius), 0, kHashWidth - 1);
        const int j = std::clamp(static_cast<int>(position.y / kSmoothingRadius), 0, kHashHeight - 1);
        const int k = std::clamp(static_cast<int>(position.z / kSmoothingRadius), 0, kHashDepth - 1);
        const std::size_t cell = HashIndex(i, j, k);
        nextParticle_[index] = cellHead_[cell];
        cellHead_[cell] = static_cast<int>(index);
    }
}

void SphFluid::ComputeDensities() {
    neighborIndices_.clear();
    neighborOffsets_[0] = 0;
    const double h = kSmoothingRadius;
    const double hSquared = h * h;
    double maximumDensityError = 0.0;
    for (std::size_t index = 0; index < particles_.size(); ++index) {
        const FlipVec3 position = particles_[index].position;
        const int centerI = static_cast<int>(position.x / h);
        const int centerJ = static_cast<int>(position.y / h);
        const int centerK = static_cast<int>(position.z / h);
        double density = 0.0;
        for (int k = centerK - 1; k <= centerK + 1; ++k) {
            for (int j = centerJ - 1; j <= centerJ + 1; ++j) {
                for (int i = centerI - 1; i <= centerI + 1; ++i) {
                    if (i < 0 || i >= kHashWidth || j < 0 || j >= kHashHeight || k < 0 || k >= kHashDepth) continue;
                    for (int neighbor = cellHead_[HashIndex(i, j, k)]; neighbor >= 0; neighbor = nextParticle_[static_cast<std::size_t>(neighbor)]) {
                        const FlipVec3 offset = Subtract(position, particles_[static_cast<std::size_t>(neighbor)].position);
                        const double radiusSquared = offset.x * offset.x + offset.y * offset.y + offset.z * offset.z;
                        if (radiusSquared >= hSquared) continue;
                        neighborIndices_.push_back(neighbor);
                        density += particleMass_ * Poly6(radiusSquared, h);
                    }
                }
            }
        }
        neighborOffsets_[index + 1] = neighborIndices_.size();
        densities_[index] = std::max(1.0, density);
        const double normalizedDensity = densities_[index] / kRestDensity;
        // A bounded linear equation of state is intentionally used for this
        // explicit real-time demo. The high-power WCSPH equation is much more
        // sensitive to timestep and produces explosive pressure spikes here.
        pressures_[index] = stiffness_ * std::max(0.0, normalizedDensity - 1.0);
        maximumDensityError = std::max(maximumDensityError, std::abs(densities_[index] - kRestDensity) / kRestDensity);
    }
    maxDensityError_ = static_cast<float>(maximumDensityError);
}

void SphFluid::ComputeForces() {
    std::size_t totalNeighbors = 0;
    double maximumAcceleration = std::abs(gravity_);
    const double h = kSmoothingRadius;
    for (std::size_t index = 0; index < particles_.size(); ++index) {
        const FlipVec3 velocity = particles_[index].velocity;
        FlipVec3 acceleration = {0.0, gravity_, 0.0};
        const FlipVec3 position = particles_[index].position;
        const std::size_t begin = neighborOffsets_[index];
        const std::size_t end = neighborOffsets_[index + 1];
        for (std::size_t neighborOffset = begin; neighborOffset < end; ++neighborOffset) {
            const std::size_t neighborIndex = static_cast<std::size_t>(neighborIndices_[neighborOffset]);
            if (neighborIndex == index) continue;
            const FlipVec3 offset = Subtract(position, particles_[neighborIndex].position);
            const double radiusSquared = offset.x * offset.x + offset.y * offset.y + offset.z * offset.z;
            const double radius = std::sqrt(radiusSquared);
            const double neighborDensity = std::max(1.0, densities_[neighborIndex]);
            const double pressureTerm = -particleMass_ * (pressures_[index] + pressures_[neighborIndex]) / (2.0 * neighborDensity);
            const double gradientMagnitude = SpikyGradientMagnitude(radius, h);
            const FlipVec3 gradient = Scale(offset, gradientMagnitude / std::max(radius, 1.0e-9));
            acceleration = Add(acceleration, Scale(gradient, pressureTerm));
            const double viscosityTerm = viscosity_ * particleMass_ * ViscosityLaplacian(radius, h) / neighborDensity;
            acceleration = Add(acceleration, Scale(Subtract(particles_[neighborIndex].velocity, velocity), viscosityTerm));
        }
        totalNeighbors += end - begin > 0 ? end - begin - 1 : 0;
        maximumAcceleration = std::max(maximumAcceleration, Length(acceleration));
        accelerations_[index] = acceleration;
    }
    averageNeighbors_ = particles_.empty() ? 0.0f : static_cast<float>(totalNeighbors) / static_cast<float>(particles_.size());
    maxAcceleration_ = maximumAcceleration;
}

void SphFluid::Integrate(double dt) {
    double maximumSpeed = 0.0;
    constexpr double bounce = 0.08;
    constexpr double maximumParticleSpeed = 18.0;
    for (std::size_t index = 0; index < particles_.size(); ++index) {
        auto& particle = particles_[index];
        particle.velocity = Add(particle.velocity, Scale(accelerations_[index], dt));
        particle.position = Add(particle.position, Scale(particle.velocity, dt));

        if (particle.position.x < kParticleRadius) { particle.position.x = kParticleRadius; particle.velocity.x = std::abs(particle.velocity.x) * bounce; }
        if (particle.position.x > kWorldWidth - kParticleRadius) { particle.position.x = kWorldWidth - kParticleRadius; particle.velocity.x = -std::abs(particle.velocity.x) * bounce; }
        if (particle.position.y < kParticleRadius) { particle.position.y = kParticleRadius; particle.velocity.y = std::abs(particle.velocity.y) * bounce; }
        if (particle.position.y > kWorldHeight - kParticleRadius) { particle.position.y = kWorldHeight - kParticleRadius; particle.velocity.y = -std::abs(particle.velocity.y) * bounce; }
        if (particle.position.z < kParticleRadius) { particle.position.z = kParticleRadius; particle.velocity.z = std::abs(particle.velocity.z) * bounce; }
        if (particle.position.z > kWorldDepth - kParticleRadius) { particle.position.z = kWorldDepth - kParticleRadius; particle.velocity.z = -std::abs(particle.velocity.z) * bounce; }
        const double speed = Length(particle.velocity);
        if (speed > maximumParticleSpeed) particle.velocity = Scale(particle.velocity, maximumParticleSpeed / speed);
        maximumSpeed = std::max(maximumSpeed, Length(particle.velocity));
    }
    maxSpeed_ = static_cast<float>(maximumSpeed);
}

void SphFluid::SimulateSubstep(double dt) {
    BuildSpatialHash();
    ComputeDensities();
    ComputeForces();
    Integrate(dt);
}

void SphFluid::Step(double dt) {
    const double clampedDt = std::clamp(dt, 1.0e-5, 0.05);
    const double accelerationDistance = 0.5 * maxAcceleration_ * clampedDt * clampedDt;
    const double cflDistance = maxSpeed_ * clampedDt + accelerationDistance;
    substeps_ = std::clamp(static_cast<int>(std::ceil(cflDistance / (kSmoothingRadius * 0.15))), 2, 8);
    const double substepDt = clampedDt / static_cast<double>(substeps_);
    for (int index = 0; index < substeps_; ++index) {
        SimulateSubstep(substepDt);
        simulatedTime_ += substepDt;
    }
}

SphFluidSnapshot SphFluid::ReadSnapshot() const {
    SphFluidSnapshot result;
    result.particles = particles_;
    result.smoothingRadius = static_cast<float>(kSmoothingRadius);
    result.stiffness = static_cast<float>(stiffness_);
    result.viscosity = static_cast<float>(viscosity_);
    result.gravity = static_cast<float>(gravity_);
    result.simulatedTime = static_cast<float>(simulatedTime_);
    result.maxDensityError = maxDensityError_;
    result.maxSpeed = maxSpeed_;
    result.averageNeighbors = averageNeighbors_;
    result.substeps = substeps_;
    return result;
}
