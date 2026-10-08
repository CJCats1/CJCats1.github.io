#include "flip_fluid.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kDensity = 1000.0;
constexpr double kParticleRadius = 0.09;
constexpr double kPokeRadius = 1.25;
constexpr int kPressureIterations = 45;
constexpr int kMinimumPressureIterations = 6;
constexpr double kPressureRelativeTolerance = 5.0e-3;

double Length(FlipVec3 value) {
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

FlipVec3 Add(FlipVec3 a, FlipVec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
FlipVec3 Scale(FlipVec3 value, double scalar) { return {value.x * scalar, value.y * scalar, value.z * scalar}; }

bool ContainsBox(FlipVec3 point, const FlipSolidBox& box, double margin = 0.0) {
    return std::abs(point.x - box.center.x) <= box.halfExtents.x + margin &&
           std::abs(point.y - box.center.y) <= box.halfExtents.y + margin &&
           std::abs(point.z - box.center.z) <= box.halfExtents.z + margin;
}

} // namespace

FlipFluid::FlipFluid()
    : particles_(),
      fluidCells_(kGridWidth * kGridHeight * kGridDepth),
      solidCells_(kGridWidth * kGridHeight * kGridDepth),
      pressure_(fluidCells_.size()),
      nextPressure_(fluidCells_.size()),
      divergence_(fluidCells_.size()),
      u_((kGridWidth + 1) * kGridHeight * kGridDepth),
      v_(kGridWidth * (kGridHeight + 1) * kGridDepth),
      w_(kGridWidth * kGridHeight * (kGridDepth + 1)),
      oldU_(u_.size()),
      oldV_(v_.size()),
      oldW_(w_.size()),
      deltaU_(u_.size()),
      deltaV_(v_.size()),
      deltaW_(w_.size()),
      uWeights_(u_.size()),
      vWeights_(v_.size()),
      wWeights_(w_.size()),
      uValid_(u_.size()),
      vValid_(v_.size()),
      wValid_(w_.size()) {
    Reset();
}

std::size_t FlipFluid::CellIndex(int i, int j, int k) const {
    return static_cast<std::size_t>(i + kGridWidth * (j + kGridHeight * k));
}

std::size_t FlipFluid::UIndex(int i, int j, int k) const {
    return static_cast<std::size_t>(i + (kGridWidth + 1) * (j + kGridHeight * k));
}

std::size_t FlipFluid::VIndex(int i, int j, int k) const {
    return static_cast<std::size_t>(i + kGridWidth * (j + (kGridHeight + 1) * k));
}

std::size_t FlipFluid::WIndex(int i, int j, int k) const {
    return static_cast<std::size_t>(i + kGridWidth * (j + kGridHeight * k));
}

bool FlipFluid::IsInside(FlipVec3 point, double margin) const {
    return point.x >= margin && point.x <= kWorldWidth - margin &&
           point.y >= margin && point.y <= kWorldHeight - margin &&
           point.z >= margin && point.z <= kWorldDepth - margin;
}

void FlipFluid::SeedParticles() {
    particles_.clear();
    // A rectangular block with eight particles per initially occupied voxel.
    // The fixed pattern makes reset and headless regression checks reproducible.
    for (int cellZ = 1; cellZ < 5; ++cellZ) {
        for (int cellY = 1; cellY < 7; ++cellY) {
            for (int cellX = 2; cellX < 12; ++cellX) {
                for (int subZ = 0; subZ < 2; ++subZ) {
                    for (int subY = 0; subY < 2; ++subY) {
                        for (int subX = 0; subX < 2; ++subX) {
                            const double x = (static_cast<double>(cellX) + (subX == 0 ? 0.25 : 0.75)) * kCellSize;
                            const double y = (static_cast<double>(cellY) + (subY == 0 ? 0.25 : 0.75)) * kCellSize;
                            const double z = (static_cast<double>(cellZ) + (subZ == 0 ? 0.25 : 0.75)) * kCellSize;
                            particles_.push_back({{x, y, z}, {0.0, 0.0, 0.0}});
                        }
                    }
                }
            }
        }
    }
}

void FlipFluid::Reset() {
    std::fill(fluidCells_.begin(), fluidCells_.end(), 0);
    std::fill(solidCells_.begin(), solidCells_.end(), 0);
    std::fill(pressure_.begin(), pressure_.end(), 0.0f);
    std::fill(nextPressure_.begin(), nextPressure_.end(), 0.0f);
    std::fill(divergence_.begin(), divergence_.end(), 0.0f);
    std::fill(u_.begin(), u_.end(), 0.0);
    std::fill(v_.begin(), v_.end(), 0.0);
    std::fill(w_.begin(), w_.end(), 0.0);
    std::fill(oldU_.begin(), oldU_.end(), 0.0);
    std::fill(oldV_.begin(), oldV_.end(), 0.0);
    std::fill(oldW_.begin(), oldW_.end(), 0.0);
    std::fill(deltaU_.begin(), deltaU_.end(), 0.0);
    std::fill(deltaV_.begin(), deltaV_.end(), 0.0);
    std::fill(deltaW_.begin(), deltaW_.end(), 0.0);
    std::fill(uWeights_.begin(), uWeights_.end(), 0.0);
    std::fill(vWeights_.begin(), vWeights_.end(), 0.0);
    std::fill(wWeights_.begin(), wWeights_.end(), 0.0);
    std::fill(uValid_.begin(), uValid_.end(), 0);
    std::fill(vValid_.begin(), vValid_.end(), 0);
    std::fill(wValid_.begin(), wValid_.end(), 0);
    simulatedTime_ = 0.0;
    maxDivergence_ = 0.0f;
    rmsDivergence_ = 0.0f;
    pressureIterations_ = 0;
    nextSpawnLayer_ = 0;
    substeps_ = 1;
    gravity_ = -9.81;
    flipRatio_ = 0.0;
    solidObstacle_ = {{5.6, 1.9, 2.0}, {1.1, 1.4, 1.0}};
    solidObstacleEnabled_ = true;
    solidObstacleVelocity_ = {};
    buoyantBox_ = {{3.5, 5.2, 1.5}, {0.8, 0.8, 0.8}};
    buoyantBoxVelocity_ = {};
    buoyantBoxSubmergedFraction_ = 0.0;
    SeedParticles();
    BuildSolidCells();
    for (auto& particle : particles_) ResolveSolidParticleCollision(particle);
}

void FlipFluid::AdjustFlipRatio(float delta) {
    flipRatio_ = std::clamp(flipRatio_ + static_cast<double>(delta), 0.0, 1.0);
}

void FlipFluid::AdjustGravity(float delta) {
    gravity_ = std::clamp(gravity_ + static_cast<double>(delta), -30.0, 0.0);
}

void FlipFluid::Poke(FlipVec3 point, FlipVec3 impulse) {
    if (!IsInside(point)) return;
    for (auto& particle : particles_) {
        const FlipVec3 offset = {particle.position.x - point.x, particle.position.y - point.y, particle.position.z - point.z};
        const double distance = Length(offset);
        if (distance < kPokeRadius) {
            const double falloff = 1.0 - distance / kPokeRadius;
            particle.velocity = Add(particle.velocity, Scale(impulse, falloff));
        }
    }
}

void FlipFluid::AddParticleLayer() {
    // No hard particle-count limit: each press appends one 4 x 1 x 2 voxel
    // layer. The source tiles through the tank so repeated presses do not
    // always place every new particle at the same coordinates.
    const int sequence = nextSpawnLayer_++;
    const int cellY = 7 + sequence % 9;
    const int cellX = 2 + (sequence / 9) % 5 * 4;
    const int cellZ = 1 + (sequence / 45) % 2 * 2;
    for (int z = cellZ; z < cellZ + 2; ++z) {
        for (int x = cellX; x < cellX + 4; ++x) {
            for (int subZ = 0; subZ < 2; ++subZ) {
                for (int subY = 0; subY < 2; ++subY) {
                    for (int subX = 0; subX < 2; ++subX) {
                        const double px = (static_cast<double>(x) + (subX == 0 ? 0.25 : 0.75)) * kCellSize;
                        const double py = (static_cast<double>(cellY) + (subY == 0 ? 0.25 : 0.75)) * kCellSize;
                        const double pz = (static_cast<double>(z) + (subZ == 0 ? 0.25 : 0.75)) * kCellSize;
                        particles_.push_back({{px, py, pz}, {0.0, -0.5, 0.0}});
                    }
                }
            }
        }
    }
}

void FlipFluid::ToggleSolidObstacle() {
    solidObstacleEnabled_ = !solidObstacleEnabled_;
    solidObstacleVelocity_ = {};
    BuildSolidCells();
}

void FlipFluid::MoveSolidObstacle(FlipVec3 velocity, double dt) {
    if (!solidObstacleEnabled_) {
        solidObstacleVelocity_ = {};
        return;
    }
    const double clampedDt = std::clamp(dt, 0.0, 0.1);
    solidObstacleVelocity_ = velocity;
    solidObstacle_.center = Add(solidObstacle_.center, Scale(velocity, clampedDt));
    const FlipVec3 margin = {solidObstacle_.halfExtents.x + kParticleRadius,
                             solidObstacle_.halfExtents.y + kParticleRadius,
                             solidObstacle_.halfExtents.z + kParticleRadius};
    solidObstacle_.center.x = std::clamp(solidObstacle_.center.x, margin.x, kWorldWidth - margin.x);
    solidObstacle_.center.y = std::clamp(solidObstacle_.center.y, margin.y, kWorldHeight - margin.y);
    solidObstacle_.center.z = std::clamp(solidObstacle_.center.z, margin.z, kWorldDepth - margin.z);
    ResolveBuoyantBoxStaticCollision();
    BuildSolidCells();
}

void FlipFluid::BuildSolidCells() {
    std::fill(solidCells_.begin(), solidCells_.end(), 0);
    const auto markBox = [&](const FlipSolidBox& box) {
        for (int k = 0; k < kGridDepth; ++k) {
            for (int j = 0; j < kGridHeight; ++j) {
                for (int i = 0; i < kGridWidth; ++i) {
                    const FlipVec3 cellCenter = {(static_cast<double>(i) + 0.5) * kCellSize,
                                                 (static_cast<double>(j) + 0.5) * kCellSize,
                                                 (static_cast<double>(k) + 0.5) * kCellSize};
                    if (ContainsBox(cellCenter, box)) solidCells_[CellIndex(i, j, k)] = 1;
                }
            }
        }
    };
    if (solidObstacleEnabled_) markBox(solidObstacle_);
    markBox(buoyantBox_);
}

double FlipFluid::EstimateFluidSurface() const {
    std::vector<int> particlesPerLayer(static_cast<std::size_t>(kGridHeight), 0);
    for (const auto& particle : particles_) {
        const int j = std::clamp(static_cast<int>(particle.position.y / kCellSize), 0, kGridHeight - 1);
        ++particlesPerLayer[static_cast<std::size_t>(j)];
    }
    int highestOccupiedLayer = 0;
    for (int j = 0; j < kGridHeight; ++j) {
        if (particlesPerLayer[static_cast<std::size_t>(j)] >= 8) highestOccupiedLayer = j;
    }
    return static_cast<double>(highestOccupiedLayer + 1) * kCellSize;
}

void FlipFluid::UpdateBuoyantBox(double dt) {
    const double waterSurface = EstimateFluidSurface();
    const double boxBottom = buoyantBox_.center.y - buoyantBox_.halfExtents.y;
    const double boxHeight = 2.0 * buoyantBox_.halfExtents.y;
    const double submergedDepth = std::clamp(waterSurface - boxBottom, 0.0, boxHeight);
    buoyantBoxSubmergedFraction_ = boxHeight > 1.0e-9 ? submergedDepth / boxHeight : 0.0;

    const double volume = 8.0 * buoyantBox_.halfExtents.x * buoyantBox_.halfExtents.y * buoyantBox_.halfExtents.z;
    const double fluidDensity = 1000.0;
    FlipVec3 force = {0.0, buoyantBoxMass_ * gravity_, 0.0};
    force.y += -gravity_ * fluidDensity * volume * buoyantBoxSubmergedFraction_;

    // Linear drag keeps the kinematic box from gaining unbounded speed while
    // still allowing fluid displacement to push it around visibly.
    const double drag = 1400.0;
    force = Add(force, Scale(buoyantBoxVelocity_, -drag));
    buoyantBoxVelocity_ = Add(buoyantBoxVelocity_, Scale(force, dt / buoyantBoxMass_));
    buoyantBoxVelocity_.x = std::clamp(buoyantBoxVelocity_.x, -12.0, 12.0);
    buoyantBoxVelocity_.y = std::clamp(buoyantBoxVelocity_.y, -12.0, 12.0);
    buoyantBoxVelocity_.z = std::clamp(buoyantBoxVelocity_.z, -12.0, 12.0);
    buoyantBox_.center = Add(buoyantBox_.center, Scale(buoyantBoxVelocity_, dt));

    const FlipVec3 margin = {buoyantBox_.halfExtents.x + kParticleRadius,
                             buoyantBox_.halfExtents.y + kParticleRadius,
                             buoyantBox_.halfExtents.z + kParticleRadius};
    if (buoyantBox_.center.x < margin.x) { buoyantBox_.center.x = margin.x; buoyantBoxVelocity_.x = std::max(0.0, buoyantBoxVelocity_.x); }
    if (buoyantBox_.center.x > kWorldWidth - margin.x) { buoyantBox_.center.x = kWorldWidth - margin.x; buoyantBoxVelocity_.x = std::min(0.0, buoyantBoxVelocity_.x); }
    if (buoyantBox_.center.y < margin.y) { buoyantBox_.center.y = margin.y; buoyantBoxVelocity_.y = std::max(0.0, buoyantBoxVelocity_.y); }
    if (buoyantBox_.center.y > kWorldHeight - margin.y) { buoyantBox_.center.y = kWorldHeight - margin.y; buoyantBoxVelocity_.y = std::min(0.0, buoyantBoxVelocity_.y); }
    if (buoyantBox_.center.z < margin.z) { buoyantBox_.center.z = margin.z; buoyantBoxVelocity_.z = std::max(0.0, buoyantBoxVelocity_.z); }
    if (buoyantBox_.center.z > kWorldDepth - margin.z) { buoyantBox_.center.z = kWorldDepth - margin.z; buoyantBoxVelocity_.z = std::min(0.0, buoyantBoxVelocity_.z); }
    ResolveBuoyantBoxStaticCollision();
}

void FlipFluid::ResolveBuoyantBoxStaticCollision() {
    if (!solidObstacleEnabled_) return;
    const double dx = buoyantBox_.center.x - solidObstacle_.center.x;
    const double dy = buoyantBox_.center.y - solidObstacle_.center.y;
    const double dz = buoyantBox_.center.z - solidObstacle_.center.z;
    const double overlapX = solidObstacle_.halfExtents.x + buoyantBox_.halfExtents.x - std::abs(dx);
    const double overlapY = solidObstacle_.halfExtents.y + buoyantBox_.halfExtents.y - std::abs(dy);
    const double overlapZ = solidObstacle_.halfExtents.z + buoyantBox_.halfExtents.z - std::abs(dz);
    if (overlapX <= 0.0 || overlapY <= 0.0 || overlapZ <= 0.0) return;

    const double epsilon = 1.0e-4;
    if (overlapY <= overlapX && overlapY <= overlapZ) {
        const double sign = dy >= 0.0 ? 1.0 : -1.0;
        buoyantBox_.center.y = solidObstacle_.center.y + sign * (solidObstacle_.halfExtents.y + buoyantBox_.halfExtents.y + epsilon);
        const double relativeVelocity = buoyantBoxVelocity_.y - solidObstacleVelocity_.y;
        if (relativeVelocity * sign < 0.0) buoyantBoxVelocity_.y = solidObstacleVelocity_.y;
    } else if (overlapX <= overlapZ) {
        const double sign = dx >= 0.0 ? 1.0 : -1.0;
        buoyantBox_.center.x = solidObstacle_.center.x + sign * (solidObstacle_.halfExtents.x + buoyantBox_.halfExtents.x + epsilon);
        const double relativeVelocity = buoyantBoxVelocity_.x - solidObstacleVelocity_.x;
        if (relativeVelocity * sign < 0.0) buoyantBoxVelocity_.x = solidObstacleVelocity_.x;
    } else {
        const double sign = dz >= 0.0 ? 1.0 : -1.0;
        buoyantBox_.center.z = solidObstacle_.center.z + sign * (solidObstacle_.halfExtents.z + buoyantBox_.halfExtents.z + epsilon);
        const double relativeVelocity = buoyantBoxVelocity_.z - solidObstacleVelocity_.z;
        if (relativeVelocity * sign < 0.0) buoyantBoxVelocity_.z = solidObstacleVelocity_.z;
    }
}

void FlipFluid::BuildFluidCells() {
    std::fill(fluidCells_.begin(), fluidCells_.end(), 0);
    for (auto& particle : particles_) {
        ResolveSolidParticleCollision(particle);
        const int i = std::clamp(static_cast<int>(particle.position.x / kCellSize), 0, kGridWidth - 1);
        const int j = std::clamp(static_cast<int>(particle.position.y / kCellSize), 0, kGridHeight - 1);
        const int k = std::clamp(static_cast<int>(particle.position.z / kCellSize), 0, kGridDepth - 1);
        if (!solidCells_[CellIndex(i, j, k)]) fluidCells_[CellIndex(i, j, k)] = 1;
    }
}

void FlipFluid::ScatterVelocities() {
    std::fill(u_.begin(), u_.end(), 0.0);
    std::fill(v_.begin(), v_.end(), 0.0);
    std::fill(w_.begin(), w_.end(), 0.0);
    std::fill(uWeights_.begin(), uWeights_.end(), 0.0);
    std::fill(vWeights_.begin(), vWeights_.end(), 0.0);
    std::fill(wWeights_.begin(), wWeights_.end(), 0.0);
    std::fill(uValid_.begin(), uValid_.end(), 0);
    std::fill(vValid_.begin(), vValid_.end(), 0);
    std::fill(wValid_.begin(), wValid_.end(), 0);

    auto scatter = [&](std::vector<double>& values, std::vector<double>& weights, int sizeX, int sizeY, int sizeZ,
                       double offsetX, double offsetY, double offsetZ, FlipVec3 point, double value) {
        const double gx = point.x / kCellSize - offsetX;
        const double gy = point.y / kCellSize - offsetY;
        const double gz = point.z / kCellSize - offsetZ;
        const int baseX = static_cast<int>(std::floor(gx));
        const int baseY = static_cast<int>(std::floor(gy));
        const int baseZ = static_cast<int>(std::floor(gz));
        for (int dk = 0; dk <= 1; ++dk) {
            for (int dj = 0; dj <= 1; ++dj) {
                for (int di = 0; di <= 1; ++di) {
                    const int i = baseX + di;
                    const int j = baseY + dj;
                    const int k = baseZ + dk;
                    if (i < 0 || i >= sizeX || j < 0 || j >= sizeY || k < 0 || k >= sizeZ) continue;
                    const double weight = std::max(0.0, 1.0 - std::abs(gx - static_cast<double>(i))) *
                                          std::max(0.0, 1.0 - std::abs(gy - static_cast<double>(j))) *
                                          std::max(0.0, 1.0 - std::abs(gz - static_cast<double>(k)));
                    const std::size_t index = static_cast<std::size_t>(i + sizeX * (j + sizeY * k));
                    values[index] += value * weight;
                    weights[index] += weight;
                }
            }
        }
    };

    for (const auto& particle : particles_) {
        scatter(u_, uWeights_, kGridWidth + 1, kGridHeight, kGridDepth, 0.0, 0.5, 0.5, particle.position, particle.velocity.x);
        scatter(v_, vWeights_, kGridWidth, kGridHeight + 1, kGridDepth, 0.5, 0.0, 0.5, particle.position, particle.velocity.y);
        scatter(w_, wWeights_, kGridWidth, kGridHeight, kGridDepth + 1, 0.5, 0.5, 0.0, particle.position, particle.velocity.z);
    }
    for (std::size_t i = 0; i < u_.size(); ++i) {
        if (uWeights_[i] > 1.0e-9) { u_[i] /= uWeights_[i]; uValid_[i] = 1; }
    }
    for (std::size_t i = 0; i < v_.size(); ++i) {
        if (vWeights_[i] > 1.0e-9) { v_[i] /= vWeights_[i]; vValid_[i] = 1; }
    }
    for (std::size_t i = 0; i < w_.size(); ++i) {
        if (wWeights_[i] > 1.0e-9) { w_[i] /= wWeights_[i]; wValid_[i] = 1; }
    }
}

void FlipFluid::ApplyGravity(double dt) {
    for (std::size_t i = 0; i < v_.size(); ++i) {
        if (vValid_[i]) v_[i] += gravity_ * dt;
    }
}

void FlipFluid::EnforceWalls() {
    for (int k = 0; k < kGridDepth; ++k) {
        for (int j = 0; j < kGridHeight; ++j) {
            u_[UIndex(0, j, k)] = 0.0;
            u_[UIndex(kGridWidth, j, k)] = 0.0;
        }
        for (int i = 0; i < kGridWidth; ++i) {
            v_[VIndex(i, 0, k)] = 0.0;
            v_[VIndex(i, kGridHeight, k)] = 0.0;
        }
    }
    for (int j = 0; j < kGridHeight; ++j) {
        for (int i = 0; i < kGridWidth; ++i) {
            w_[WIndex(i, j, 0)] = 0.0;
            w_[WIndex(i, j, kGridDepth)] = 0.0;
        }
    }
}

void FlipFluid::EnforceSolidWalls() {
    for (int k = 0; k < kGridDepth; ++k) {
        for (int j = 0; j < kGridHeight; ++j) {
            for (int i = 0; i < kGridWidth; ++i) {
                if (!solidCells_[CellIndex(i, j, k)]) continue;
                const FlipVec3 cellCenter = {(static_cast<double>(i) + 0.5) * kCellSize,
                                             (static_cast<double>(j) + 0.5) * kCellSize,
                                             (static_cast<double>(k) + 0.5) * kCellSize};
                const FlipVec3 boundaryVelocity = ContainsBox(cellCenter, buoyantBox_) ? buoyantBoxVelocity_ : (solidObstacleEnabled_ ? solidObstacleVelocity_ : FlipVec3{});
                u_[UIndex(i, j, k)] = boundaryVelocity.x;
                u_[UIndex(i + 1, j, k)] = boundaryVelocity.x;
                v_[VIndex(i, j, k)] = boundaryVelocity.y;
                v_[VIndex(i, j + 1, k)] = boundaryVelocity.y;
                w_[WIndex(i, j, k)] = boundaryVelocity.z;
                w_[WIndex(i, j, k + 1)] = boundaryVelocity.z;
            }
        }
    }
}

void FlipFluid::ComputeDivergence() {
    std::fill(divergence_.begin(), divergence_.end(), 0.0f);
    for (int k = 0; k < kGridDepth; ++k) {
        for (int j = 0; j < kGridHeight; ++j) {
            for (int i = 0; i < kGridWidth; ++i) {
                const std::size_t cell = CellIndex(i, j, k);
                if (!fluidCells_[cell]) continue;
                divergence_[cell] = static_cast<float>((u_[UIndex(i + 1, j, k)] - u_[UIndex(i, j, k)] +
                                                        v_[VIndex(i, j + 1, k)] - v_[VIndex(i, j, k)] +
                                                        w_[WIndex(i, j, k + 1)] - w_[WIndex(i, j, k)]) / kCellSize);
            }
        }
    }
}

void FlipFluid::ProjectPressure(double dt) {
    std::fill(pressure_.begin(), pressure_.end(), 0.0f);
    pressureIterations_ = 0;
    for (int iteration = 0; iteration < kPressureIterations; ++iteration) {
        double maximumDelta = 0.0;
        double maximumPressure = 0.0;
        for (int k = 0; k < kGridDepth; ++k) {
            for (int j = 0; j < kGridHeight; ++j) {
                for (int i = 0; i < kGridWidth; ++i) {
                    const std::size_t cell = CellIndex(i, j, k);
                    if (!fluidCells_[cell]) { nextPressure_[cell] = 0.0f; continue; }
                    const int neighborI[6] = {i - 1, i + 1, i, i, i, i};
                    const int neighborJ[6] = {j, j, j - 1, j + 1, j, j};
                    const int neighborK[6] = {k, k, k, k, k - 1, k + 1};
                    double sum = 0.0;
                    int diagonal = 0;
                    for (int n = 0; n < 6; ++n) {
                        if (neighborI[n] < 0 || neighborI[n] >= kGridWidth || neighborJ[n] < 0 || neighborJ[n] >= kGridHeight || neighborK[n] < 0 || neighborK[n] >= kGridDepth) continue;
                        const std::size_t neighbor = CellIndex(neighborI[n], neighborJ[n], neighborK[n]);
                        if (solidCells_[neighbor]) continue;
                        ++diagonal;
                        if (fluidCells_[neighbor]) sum += pressure_[neighbor];
                    }
                    nextPressure_[cell] = diagonal > 0 ? static_cast<float>((sum - kDensity * kCellSize * kCellSize * static_cast<double>(divergence_[cell]) / dt) / static_cast<double>(diagonal)) : 0.0f;
                    maximumDelta = std::max(maximumDelta, std::abs(static_cast<double>(nextPressure_[cell] - pressure_[cell])));
                    maximumPressure = std::max(maximumPressure, std::abs(static_cast<double>(nextPressure_[cell])));
                }
            }
        }
        pressure_.swap(nextPressure_);
        pressureIterations_ = iteration + 1;
        const double scale = std::max(1.0, maximumPressure);
        if (pressureIterations_ >= kMinimumPressureIterations && maximumDelta <= kPressureRelativeTolerance * scale) break;
    }
}

void FlipFluid::CorrectVelocities(double dt) {
    for (int k = 0; k < kGridDepth; ++k) {
        for (int j = 0; j < kGridHeight; ++j) {
            for (int i = 1; i < kGridWidth; ++i) {
                const bool leftFluid = fluidCells_[CellIndex(i - 1, j, k)] != 0;
                const bool rightFluid = fluidCells_[CellIndex(i, j, k)] != 0;
                if (!leftFluid && !rightFluid) continue;
                const float leftPressure = leftFluid ? pressure_[CellIndex(i - 1, j, k)] : 0.0f;
                const float rightPressure = rightFluid ? pressure_[CellIndex(i, j, k)] : 0.0f;
                u_[UIndex(i, j, k)] -= dt / kDensity * (static_cast<double>(rightPressure) - leftPressure) / kCellSize;
            }
        }
    }
    for (int k = 0; k < kGridDepth; ++k) {
        for (int j = 1; j < kGridHeight; ++j) {
            for (int i = 0; i < kGridWidth; ++i) {
                const bool bottomFluid = fluidCells_[CellIndex(i, j - 1, k)] != 0;
                const bool topFluid = fluidCells_[CellIndex(i, j, k)] != 0;
                if (!bottomFluid && !topFluid) continue;
                const float bottomPressure = bottomFluid ? pressure_[CellIndex(i, j - 1, k)] : 0.0f;
                const float topPressure = topFluid ? pressure_[CellIndex(i, j, k)] : 0.0f;
                v_[VIndex(i, j, k)] -= dt / kDensity * (static_cast<double>(topPressure) - bottomPressure) / kCellSize;
            }
        }
    }
    for (int k = 1; k < kGridDepth; ++k) {
        for (int j = 0; j < kGridHeight; ++j) {
            for (int i = 0; i < kGridWidth; ++i) {
                const bool backFluid = fluidCells_[CellIndex(i, j, k - 1)] != 0;
                const bool frontFluid = fluidCells_[CellIndex(i, j, k)] != 0;
                if (!backFluid && !frontFluid) continue;
                const float backPressure = backFluid ? pressure_[CellIndex(i, j, k - 1)] : 0.0f;
                const float frontPressure = frontFluid ? pressure_[CellIndex(i, j, k)] : 0.0f;
                w_[WIndex(i, j, k)] -= dt / kDensity * (static_cast<double>(frontPressure) - backPressure) / kCellSize;
            }
        }
    }
}

double FlipFluid::SampleComponent(const std::vector<double>& values, const std::vector<unsigned char>& valid,
                                  int sizeX, int sizeY, int sizeZ, double offsetX, double offsetY, double offsetZ,
                                  FlipVec3 point) const {
    const double gx = point.x / kCellSize - offsetX;
    const double gy = point.y / kCellSize - offsetY;
    const double gz = point.z / kCellSize - offsetZ;
    const int baseX = static_cast<int>(std::floor(gx));
    const int baseY = static_cast<int>(std::floor(gy));
    const int baseZ = static_cast<int>(std::floor(gz));
    double result = 0.0;
    double totalWeight = 0.0;
    for (int dk = 0; dk <= 1; ++dk) {
        for (int dj = 0; dj <= 1; ++dj) {
            for (int di = 0; di <= 1; ++di) {
                const int i = baseX + di;
                const int j = baseY + dj;
                const int k = baseZ + dk;
                if (i < 0 || i >= sizeX || j < 0 || j >= sizeY || k < 0 || k >= sizeZ) continue;
                const double weight = std::max(0.0, 1.0 - std::abs(gx - static_cast<double>(i))) *
                                      std::max(0.0, 1.0 - std::abs(gy - static_cast<double>(j))) *
                                      std::max(0.0, 1.0 - std::abs(gz - static_cast<double>(k)));
                const std::size_t index = static_cast<std::size_t>(i + sizeX * (j + sizeY * k));
                if (!valid[index]) continue;
                result += values[index] * weight;
                totalWeight += weight;
            }
        }
    }
    return totalWeight > 1.0e-9 ? result / totalWeight : 0.0;
}

double FlipFluid::SampleU(FlipVec3 point) const { return SampleComponent(u_, uValid_, kGridWidth + 1, kGridHeight, kGridDepth, 0.0, 0.5, 0.5, point); }
double FlipFluid::SampleV(FlipVec3 point) const { return SampleComponent(v_, vValid_, kGridWidth, kGridHeight + 1, kGridDepth, 0.5, 0.0, 0.5, point); }
double FlipFluid::SampleW(FlipVec3 point) const { return SampleComponent(w_, wValid_, kGridWidth, kGridHeight, kGridDepth + 1, 0.5, 0.5, 0.0, point); }
double FlipFluid::SampleDeltaU(FlipVec3 point) const { return SampleComponent(deltaU_, uValid_, kGridWidth + 1, kGridHeight, kGridDepth, 0.0, 0.5, 0.5, point); }
double FlipFluid::SampleDeltaV(FlipVec3 point) const { return SampleComponent(deltaV_, vValid_, kGridWidth, kGridHeight + 1, kGridDepth, 0.5, 0.0, 0.5, point); }
double FlipFluid::SampleDeltaW(FlipVec3 point) const { return SampleComponent(deltaW_, wValid_, kGridWidth, kGridHeight, kGridDepth + 1, 0.5, 0.5, 0.0, point); }

void FlipFluid::AdvectParticles(double dt) {
    for (auto& particle : particles_) {
        const FlipVec3 oldPosition = particle.position;
        const FlipVec3 picVelocity = {SampleU(oldPosition), SampleV(oldPosition), SampleW(oldPosition)};
        const FlipVec3 flipVelocity = Add(particle.velocity, {SampleDeltaU(oldPosition), SampleDeltaV(oldPosition), SampleDeltaW(oldPosition)});
        const FlipVec3 blended = Add(Scale(picVelocity, 1.0 - flipRatio_), Scale(flipVelocity, flipRatio_));
        particle.velocity = blended;
        const FlipVec3 midpoint = Add(oldPosition, Scale(blended, 0.5 * dt));
        const FlipVec3 midpointVelocity = {SampleU(midpoint), SampleV(midpoint), SampleW(midpoint)};
        particle.position = Add(oldPosition, Scale(midpointVelocity, dt));

        if (particle.position.x < kParticleRadius) { particle.position.x = kParticleRadius; particle.velocity.x = std::max(0.0, particle.velocity.x); }
        if (particle.position.x > kWorldWidth - kParticleRadius) { particle.position.x = kWorldWidth - kParticleRadius; particle.velocity.x = std::min(0.0, particle.velocity.x); }
        if (particle.position.y < kParticleRadius) { particle.position.y = kParticleRadius; particle.velocity.y = std::max(0.0, particle.velocity.y); }
        if (particle.position.y > kWorldHeight - kParticleRadius) { particle.position.y = kWorldHeight - kParticleRadius; particle.velocity.y = std::min(0.0, particle.velocity.y); }
        if (particle.position.z < kParticleRadius) { particle.position.z = kParticleRadius; particle.velocity.z = std::max(0.0, particle.velocity.z); }
        if (particle.position.z > kWorldDepth - kParticleRadius) { particle.position.z = kWorldDepth - kParticleRadius; particle.velocity.z = std::min(0.0, particle.velocity.z); }
        ResolveSolidParticleCollision(particle);
    }
}

void FlipFluid::ResolveSolidParticleCollision(FlipParticle& particle) const {
    if (solidObstacleEnabled_) ResolveBoxParticleCollision(particle, solidObstacle_, solidObstacleVelocity_);
    ResolveBoxParticleCollision(particle, buoyantBox_, buoyantBoxVelocity_);
}

void FlipFluid::ResolveBoxParticleCollision(FlipParticle& particle, const FlipSolidBox& box, FlipVec3 boxVelocity) const {
    const double hx = box.halfExtents.x + kParticleRadius;
    const double hy = box.halfExtents.y + kParticleRadius;
    const double hz = box.halfExtents.z + kParticleRadius;
    const double dx = particle.position.x - box.center.x;
    const double dy = particle.position.y - box.center.y;
    const double dz = particle.position.z - box.center.z;
    if (std::abs(dx) > hx || std::abs(dy) > hy || std::abs(dz) > hz) return;

    const double pushX = hx - std::abs(dx);
    const double pushY = hy - std::abs(dy);
    const double pushZ = hz - std::abs(dz);
    if (pushY <= pushX && pushY <= pushZ) {
        const double sign = dy >= 0.0 ? 1.0 : -1.0;
        particle.position.y = box.center.y + sign * hy;
        if ((particle.velocity.y - boxVelocity.y) * sign < 0.0) particle.velocity.y = boxVelocity.y;
    } else if (pushX <= pushZ) {
        const double sign = dx >= 0.0 ? 1.0 : -1.0;
        particle.position.x = box.center.x + sign * hx;
        if ((particle.velocity.x - boxVelocity.x) * sign < 0.0) particle.velocity.x = boxVelocity.x;
    } else {
        const double sign = dz >= 0.0 ? 1.0 : -1.0;
        particle.position.z = box.center.z + sign * hz;
        if ((particle.velocity.z - boxVelocity.z) * sign < 0.0) particle.velocity.z = boxVelocity.z;
    }
}

void FlipFluid::ComputeDiagnostics() {
    ComputeDivergence();
    double sumSquared = 0.0;
    double maximum = 0.0;
    int count = 0;
    for (std::size_t i = 0; i < divergence_.size(); ++i) {
        if (!fluidCells_[i]) continue;
        const double absolute = std::abs(static_cast<double>(divergence_[i]));
        maximum = std::max(maximum, absolute);
        sumSquared += absolute * absolute;
        ++count;
    }
    maxDivergence_ = static_cast<float>(maximum);
    rmsDivergence_ = count > 0 ? static_cast<float>(std::sqrt(sumSquared / static_cast<double>(count))) : 0.0f;
}

void FlipFluid::SimulateSubstep(double dt) {
    UpdateBuoyantBox(dt);
    BuildSolidCells();
    BuildFluidCells();
    ScatterVelocities();
    oldU_ = u_;
    oldV_ = v_;
    oldW_ = w_;
    ApplyGravity(dt);
    EnforceWalls();
    EnforceSolidWalls();
    ComputeDivergence();
    ProjectPressure(dt);
    CorrectVelocities(dt);
    EnforceWalls();
    EnforceSolidWalls();
    for (std::size_t i = 0; i < u_.size(); ++i) deltaU_[i] = u_[i] - oldU_[i];
    for (std::size_t i = 0; i < v_.size(); ++i) deltaV_[i] = v_[i] - oldV_[i];
    for (std::size_t i = 0; i < w_.size(); ++i) deltaW_[i] = w_[i] - oldW_[i];
    ComputeDiagnostics();
    AdvectParticles(dt);
}

void FlipFluid::Step(double dt) {
    const double clampedDt = std::clamp(dt, 1.0e-5, 0.05);
    double maxSpeed = 0.0;
    for (const auto& particle : particles_) maxSpeed = std::max(maxSpeed, Length(particle.velocity));
    const double accelerationDistance = std::abs(gravity_) * clampedDt * clampedDt;
    substeps_ = std::clamp(static_cast<int>(std::ceil((maxSpeed * clampedDt + accelerationDistance) / (kCellSize * 0.45))), 1, 8);
    const double substepDt = clampedDt / static_cast<double>(substeps_);
    for (int i = 0; i < substeps_; ++i) {
        SimulateSubstep(substepDt);
        simulatedTime_ += substepDt;
    }
}

FlipFluidSnapshot FlipFluid::ReadSnapshot() const {
    FlipFluidSnapshot result;
    result.particles = particles_;
    result.pressure = pressure_;
    result.fluidCells = fluidCells_;
    result.flipRatio = static_cast<float>(flipRatio_);
    result.gravity = static_cast<float>(gravity_);
    result.simulatedTime = static_cast<float>(simulatedTime_);
    result.maxDivergence = maxDivergence_;
    result.rmsDivergence = rmsDivergence_;
    result.pressureIterations = pressureIterations_;
    result.substeps = substeps_;
    return result;
}
