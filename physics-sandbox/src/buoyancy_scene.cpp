#include "buoyancy_scene.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr double kEpsilon = 1.0e-9;

} // namespace

BuoyancyScene::BuoyancyScene() { Reset(); }

void BuoyancyScene::Reset() {
    position_ = {0.0, 5.35};
    velocity_ = {0.0, 0.0};
    angle_ = 0.10;
    angularVelocity_ = 0.0;
    mass_ = 1100.0;
    waterDensity_ = kWaterDensity;
    dragCoefficient_ = 420.0;
    simulatedTime_ = 0.0;
    dragEnabled_ = true;
    rotationEnabled_ = true;
    position3D_ = {0.0, 5.35, 0.0};
    velocity3D_ = {0.0, 0.0, 0.0};
    angularVelocity3D_ = {0.0, 0.0, 0.0};
    angularMomentum3D_ = {0.0, 0.0, 0.0};
    orientation3D_ = {0.985, 0.08, 0.12, -0.04};
    orientation3D_ = NormalizeQuat(orientation3D_);
}

BuoyancyVec2 BuoyancyScene::Add(BuoyancyVec2 a, BuoyancyVec2 b) { return {a.x + b.x, a.y + b.y}; }
BuoyancyVec2 BuoyancyScene::Subtract(BuoyancyVec2 a, BuoyancyVec2 b) { return {a.x - b.x, a.y - b.y}; }
BuoyancyVec2 BuoyancyScene::Scale(BuoyancyVec2 value, double scalar) { return {value.x * scalar, value.y * scalar}; }
double BuoyancyScene::Dot(BuoyancyVec2 a, BuoyancyVec2 b) { return a.x * b.x + a.y * b.y; }
double BuoyancyScene::Cross(BuoyancyVec2 a, BuoyancyVec2 b) { return a.x * b.y - a.y * b.x; }
BuoyancyVec2 BuoyancyScene::Rotate(BuoyancyVec2 value, double angle) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return {c * value.x - s * value.y, s * value.x + c * value.y};
}
BuoyancyVec2 BuoyancyScene::CrossScalar(double scalar, BuoyancyVec2 value) { return {-scalar * value.y, scalar * value.x}; }

BuoyancyVec3 BuoyancyScene::Add3(BuoyancyVec3 a, BuoyancyVec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
BuoyancyVec3 BuoyancyScene::Subtract3(BuoyancyVec3 a, BuoyancyVec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
BuoyancyVec3 BuoyancyScene::Scale3(BuoyancyVec3 value, double scalar) { return {value.x * scalar, value.y * scalar, value.z * scalar}; }
double BuoyancyScene::Dot3(BuoyancyVec3 a, BuoyancyVec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
BuoyancyVec3 BuoyancyScene::Cross3(BuoyancyVec3 a, BuoyancyVec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }

BuoyancyQuat BuoyancyScene::NormalizeQuat(BuoyancyQuat value) {
    const double length = std::sqrt(value.w * value.w + value.x * value.x + value.y * value.y + value.z * value.z);
    return length > kEpsilon ? BuoyancyQuat{value.w / length, value.x / length, value.y / length, value.z / length} : BuoyancyQuat{};
}

BuoyancyQuat BuoyancyScene::MultiplyQuat(BuoyancyQuat a, BuoyancyQuat b) {
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

BuoyancyVec3 BuoyancyScene::Rotate3(BuoyancyQuat orientation, BuoyancyVec3 value) {
    const BuoyancyQuat vector{0.0, value.x, value.y, value.z};
    const BuoyancyQuat conjugate{orientation.w, -orientation.x, -orientation.y, -orientation.z};
    const BuoyancyQuat rotated = MultiplyQuat(MultiplyQuat(orientation, vector), conjugate);
    return {rotated.x, rotated.y, rotated.z};
}

BuoyancyVec3 BuoyancyScene::InverseRotate3(BuoyancyQuat orientation, BuoyancyVec3 value) {
    const BuoyancyQuat conjugate{orientation.w, -orientation.x, -orientation.y, -orientation.z};
    return Rotate3(conjugate, value);
}

BuoyancyQuat BuoyancyScene::IntegrateQuat(BuoyancyQuat orientation, BuoyancyVec3 angularVelocity, double dt) {
    const BuoyancyQuat spin{0.0, angularVelocity.x, angularVelocity.y, angularVelocity.z};
    const BuoyancyQuat derivative = MultiplyQuat(orientation, spin);
    orientation.w += 0.5 * derivative.w * dt;
    orientation.x += 0.5 * derivative.x * dt;
    orientation.y += 0.5 * derivative.y * dt;
    orientation.z += 0.5 * derivative.z * dt;
    return NormalizeQuat(orientation);
}

std::array<BuoyancyVec2, 4> BuoyancyScene::BodyCorners() const {
    const std::array<BuoyancyVec2, 4> local = {{{-kBodyWidth * 0.5, -kBodyHeight * 0.5},
                                                 { kBodyWidth * 0.5, -kBodyHeight * 0.5},
                                                 { kBodyWidth * 0.5,  kBodyHeight * 0.5},
                                                 {-kBodyWidth * 0.5,  kBodyHeight * 0.5}}};
    std::array<BuoyancyVec2, 4> world{};
    for (std::size_t i = 0; i < local.size(); ++i) world[i] = Add(position_, Rotate(local[i], angle_));
    return world;
}

std::array<BuoyancyVec3, 8> BuoyancyScene::BodyCorners3D() const {
    const double halfWidth = kBodyWidth * 0.5;
    const double halfHeight = kBodyHeight * 0.5;
    const double halfDepth = kBodyDepth * 0.5;
    const std::array<BuoyancyVec3, 8> local = {{{-halfWidth, -halfHeight, -halfDepth},
                                                 { halfWidth, -halfHeight, -halfDepth},
                                                 { halfWidth,  halfHeight, -halfDepth},
                                                 {-halfWidth,  halfHeight, -halfDepth},
                                                 {-halfWidth, -halfHeight,  halfDepth},
                                                 { halfWidth, -halfHeight,  halfDepth},
                                                 { halfWidth,  halfHeight,  halfDepth},
                                                 {-halfWidth,  halfHeight,  halfDepth}}};
    std::array<BuoyancyVec3, 8> world{};
    for (std::size_t i = 0; i < local.size(); ++i) world[i] = Add3(position3D_, Rotate3(orientation3D_, local[i]));
    return world;
}

BuoyancyScene::Polygon BuoyancyScene::ClipBelowWater(const std::array<BuoyancyVec2, 4>& corners, int& count) const {
    Polygon output{};
    count = 0;
    for (int i = 0; i < 4; ++i) {
        const BuoyancyVec2 a = corners[static_cast<std::size_t>(i)];
        const BuoyancyVec2 b = corners[static_cast<std::size_t>((i + 1) % 4)];
        const bool insideA = a.y <= kWaterLevel;
        const bool insideB = b.y <= kWaterLevel;
        if (insideA && insideB) {
            output[static_cast<std::size_t>(count++)] = b;
        } else if (insideA != insideB) {
            const double denominator = b.y - a.y;
            const double t = std::abs(denominator) > kEpsilon ? (kWaterLevel - a.y) / denominator : 0.0;
            output[static_cast<std::size_t>(count++)] = Add(a, Scale(Subtract(b, a), std::clamp(t, 0.0, 1.0)));
            if (insideB) output[static_cast<std::size_t>(count++)] = b;
        }
    }
    return output;
}

void BuoyancyScene::ComputeSubmergedGeometry(const std::array<BuoyancyVec2, 4>& corners, double& area, BuoyancyVec2& centroid, Polygon& polygon, int& count) const {
    polygon = ClipBelowWater(corners, count);
    area = 0.0;
    centroid = position_;
    if (count < 3) {
        count = 0;
        polygon = {};
        return;
    }

    double twiceSignedArea = 0.0;
    BuoyancyVec2 weightedCentroid{};
    for (int i = 0; i < count; ++i) {
        const BuoyancyVec2 a = polygon[static_cast<std::size_t>(i)];
        const BuoyancyVec2 b = polygon[static_cast<std::size_t>((i + 1) % count)];
        const double cross = Cross(a, b);
        twiceSignedArea += cross;
        weightedCentroid = Add(weightedCentroid, Scale(Add(a, b), cross));
    }
    if (std::abs(twiceSignedArea) < kEpsilon) {
        count = 0;
        polygon = {};
        return;
    }
    area = std::abs(twiceSignedArea) * 0.5;
    centroid = Scale(weightedCentroid, 1.0 / (3.0 * twiceSignedArea));
}

void BuoyancyScene::Step(double dt) {
    if (threeDimensional_) {
        Step3D(dt);
        return;
    }
    const double step = std::clamp(dt, 1.0e-5, 0.02);
    const auto corners = BodyCorners();
    double submergedArea = 0.0;
    BuoyancyVec2 centerOfBuoyancy{};
    Polygon submergedPolygon{};
    int submergedCount = 0;
    ComputeSubmergedGeometry(corners, submergedArea, centerOfBuoyancy, submergedPolygon, submergedCount);

    const double volume = submergedArea * kBodyThickness;
    const BuoyancyVec2 gravityForce = {0.0, mass_ * kGravity};
    const BuoyancyVec2 buoyancyForce = {0.0, -waterDensity_ * volume * kGravity};
    BuoyancyVec2 dragForce{};
    double torque = Cross(Subtract(centerOfBuoyancy, position_), buoyancyForce);

    if (dragEnabled_ && submergedArea > 0.0) {
        const double submergedFraction = std::clamp(submergedArea / (kBodyWidth * kBodyHeight), 0.0, 1.0);
        dragForce = Scale(velocity_, -dragCoefficient_ * submergedFraction);
        torque -= angularVelocity_ * dampingTorque_ * submergedFraction;
    }

    const BuoyancyVec2 totalForce = Add(Add(gravityForce, buoyancyForce), dragForce);
    const double inertia = mass_ * (kBodyWidth * kBodyWidth + kBodyHeight * kBodyHeight) / 12.0;
    velocity_ = Add(velocity_, Scale(totalForce, step / mass_));
    if (rotationEnabled_) angularVelocity_ += torque * step / inertia;
    else angularVelocity_ = 0.0;
    position_ = Add(position_, Scale(velocity_, step));
    if (rotationEnabled_) angle_ += angularVelocity_ * step;
    ApplyTankContacts();
    simulatedTime_ += step;
}

void BuoyancyScene::ComputeSubmergedGeometry3D(const std::array<BuoyancyVec3, 8>& corners, double& volume, BuoyancyVec3& centroid) const {
    // Convergent body-space volume quadrature. This is intentionally explicit in
    // the reference scene; exact convex polyhedron clipping can replace it later.
    (void)corners;
    constexpr int samplesPerAxis = 16;
    const double sampleVolume = kBodyWidth * kBodyHeight * kBodyDepth / static_cast<double>(samplesPerAxis * samplesPerAxis * samplesPerAxis);
    volume = 0.0;
    centroid = position3D_;
    BuoyancyVec3 weighted{};
    for (int iz = 0; iz < samplesPerAxis; ++iz) {
        for (int iy = 0; iy < samplesPerAxis; ++iy) {
            for (int ix = 0; ix < samplesPerAxis; ++ix) {
                const BuoyancyVec3 local = {
                    (static_cast<double>(ix) + 0.5) / samplesPerAxis * kBodyWidth - kBodyWidth * 0.5,
                    (static_cast<double>(iy) + 0.5) / samplesPerAxis * kBodyHeight - kBodyHeight * 0.5,
                    (static_cast<double>(iz) + 0.5) / samplesPerAxis * kBodyDepth - kBodyDepth * 0.5};
                const BuoyancyVec3 world = Add3(position3D_, Rotate3(orientation3D_, local));
                if (world.y <= kWaterLevel) {
                    volume += sampleVolume;
                    weighted = Add3(weighted, Scale3(world, sampleVolume));
                }
            }
        }
    }
    if (volume > kEpsilon) centroid = Scale3(weighted, 1.0 / volume);
}

BuoyancyVec3 BuoyancyScene::InverseWorldInertiaMultiply(BuoyancyVec3 value) const {
    const double ix = mass_ * (kBodyHeight * kBodyHeight + kBodyDepth * kBodyDepth) / 12.0;
    const double iy = mass_ * (kBodyWidth * kBodyWidth + kBodyDepth * kBodyDepth) / 12.0;
    const double iz = mass_ * (kBodyWidth * kBodyWidth + kBodyHeight * kBodyHeight) / 12.0;
    const BuoyancyVec3 bodyValue = InverseRotate3(orientation3D_, value);
    return Rotate3(orientation3D_, {bodyValue.x / ix, bodyValue.y / iy, bodyValue.z / iz});
}

void BuoyancyScene::Step3D(double dt) {
    const double step = std::clamp(dt, 1.0e-5, 0.02);
    const auto corners = BodyCorners3D();
    double submergedVolume = 0.0;
    BuoyancyVec3 centerOfBuoyancy{};
    ComputeSubmergedGeometry3D(corners, submergedVolume, centerOfBuoyancy);

    const BuoyancyVec3 gravityForce = {0.0, mass_ * kGravity, 0.0};
    const BuoyancyVec3 buoyancyForce = {0.0, -waterDensity_ * submergedVolume * kGravity, 0.0};
    BuoyancyVec3 dragForce{};
    BuoyancyVec3 torque = Cross3(Subtract3(centerOfBuoyancy, position3D_), buoyancyForce);
    if (dragEnabled_ && submergedVolume > 0.0) {
        const double fraction = std::clamp(submergedVolume / (kBodyWidth * kBodyHeight * kBodyDepth), 0.0, 1.0);
        dragForce = Scale3(velocity3D_, -dragCoefficient_ * fraction);
        torque = Subtract3(torque, Scale3(angularVelocity3D_, dampingTorque_ * fraction));
    }

    const BuoyancyVec3 totalForce = Add3(Add3(gravityForce, buoyancyForce), dragForce);
    velocity3D_ = Add3(velocity3D_, Scale3(totalForce, step / mass_));
    if (rotationEnabled_) {
        const BuoyancyVec3 gyroscopic = Cross3(angularVelocity3D_, angularMomentum3D_);
        angularMomentum3D_ = Add3(angularMomentum3D_, Scale3(Subtract3(torque, gyroscopic), step));
        angularVelocity3D_ = InverseWorldInertiaMultiply(angularMomentum3D_);
    } else {
        angularMomentum3D_ = {};
        angularVelocity3D_ = {};
    }
    position3D_ = Add3(position3D_, Scale3(velocity3D_, step));
    if (rotationEnabled_) orientation3D_ = IntegrateQuat(orientation3D_, angularVelocity3D_, step);
    ApplyTankContacts3D();
    simulatedTime_ += step;
}

void BuoyancyScene::ApplyTankContacts3D() {
    auto corners = BodyCorners3D();
    auto applyContact = [&](BuoyancyVec3 normal, BuoyancyVec3 contact, double penetration) {
        if (penetration <= 0.0) return;
        position3D_ = Add3(position3D_, Scale3(normal, penetration));
        const BuoyancyVec3 r = Subtract3(contact, position3D_);
        const BuoyancyVec3 pointVelocity = Add3(velocity3D_, Cross3(angularVelocity3D_, r));
        const double normalVelocity = Dot3(pointVelocity, normal);
        const BuoyancyVec3 inverseInertiaLever = InverseWorldInertiaMultiply(Cross3(r, normal));
        const double effectiveMass = 1.0 / mass_ + Dot3(Cross3(inverseInertiaLever, r), normal);
        if (normalVelocity >= 0.0 || effectiveMass <= kEpsilon) return;
        const double impulseMagnitude = -(1.0 + restitution_) * normalVelocity / effectiveMass;
        const BuoyancyVec3 impulse = Scale3(normal, impulseMagnitude);
        velocity3D_ = Add3(velocity3D_, Scale3(impulse, 1.0 / mass_));
        angularMomentum3D_ = Add3(angularMomentum3D_, Cross3(r, impulse));
        angularVelocity3D_ = InverseWorldInertiaMultiply(angularMomentum3D_);
    };

    auto extreme = [&](int axis, bool minimum) {
        BuoyancyVec3 selected = corners[0];
        for (const BuoyancyVec3 corner : corners) {
            const double selectedValue = axis == 0 ? selected.x : (axis == 1 ? selected.y : selected.z);
            const double cornerValue = axis == 0 ? corner.x : (axis == 1 ? corner.y : corner.z);
            if ((minimum && cornerValue < selectedValue) || (!minimum && cornerValue > selectedValue)) selected = corner;
        }
        return selected;
    };
    const BuoyancyVec3 lowY = extreme(1, true);
    const BuoyancyVec3 lowX = extreme(0, true);
    const BuoyancyVec3 highX = extreme(0, false);
    const BuoyancyVec3 lowZ = extreme(2, true);
    const BuoyancyVec3 highZ = extreme(2, false);
    applyContact({0.0, 1.0, 0.0}, lowY, 0.12 - lowY.y);
    applyContact({1.0, 0.0, 0.0}, lowX, -kTankHalfWidth - lowX.x);
    applyContact({-1.0, 0.0, 0.0}, highX, highX.x - kTankHalfWidth);
    applyContact({0.0, 0.0, 1.0}, lowZ, -kTankHalfDepth - lowZ.z);
    applyContact({0.0, 0.0, -1.0}, highZ, highZ.z - kTankHalfDepth);
}

void BuoyancyScene::ApplyTankContacts() {
    auto corners = BodyCorners();
    double minimumY = corners[0].y;
    double minimumX = corners[0].x;
    double maximumX = corners[0].x;
    for (const BuoyancyVec2 corner : corners) {
        minimumY = std::min(minimumY, corner.y);
        minimumX = std::min(minimumX, corner.x);
        maximumX = std::max(maximumX, corner.x);
    }

    if (minimumY < 0.12) {
        position_.y += 0.12 - minimumY;
        corners = BodyCorners();
        BuoyancyVec2 contact = corners[0];
        for (const BuoyancyVec2 corner : corners) if (corner.y < contact.y) contact = corner;
        const BuoyancyVec2 r = Subtract(contact, position_);
        const BuoyancyVec2 pointVelocity = Add(velocity_, CrossScalar(angularVelocity_, r));
        const double normalVelocity = pointVelocity.y;
        const double inertia = mass_ * (kBodyWidth * kBodyWidth + kBodyHeight * kBodyHeight) / 12.0;
        const double inverseMass = 1.0 / mass_;
        const double inverseInertia = rotationEnabled_ ? 1.0 / inertia : 0.0;
        const double lever = r.x;
        const double effectiveMass = inverseMass + lever * lever * inverseInertia;
        if (normalVelocity < 0.0 && effectiveMass > kEpsilon) {
            const double impulse = -(1.0 + restitution_) * normalVelocity / effectiveMass;
            velocity_.y += impulse * inverseMass;
            if (rotationEnabled_) angularVelocity_ += lever * impulse * inverseInertia;
        }
    }
    if (minimumX < -kTankHalfWidth) {
        position_.x += -kTankHalfWidth - minimumX;
        velocity_.x = std::max(0.0, velocity_.x);
    }
    if (maximumX > kTankHalfWidth) {
        position_.x -= maximumX - kTankHalfWidth;
        velocity_.x = std::min(0.0, velocity_.x);
    }
}

void BuoyancyScene::ApplyImpulse(BuoyancyVec2 impulse) {
    if (threeDimensional_) {
        velocity3D_.y += impulse.y / mass_;
        velocity3D_.x += impulse.x / mass_;
        return;
    }
    velocity_ = Add(velocity_, Scale(impulse, 1.0 / mass_));
}
void BuoyancyScene::AdjustMass(double delta) { mass_ = std::clamp(mass_ + delta, 100.0, 5000.0); }
void BuoyancyScene::AdjustWaterDensity(double delta) { waterDensity_ = std::clamp(waterDensity_ + delta, 100.0, 2000.0); }
void BuoyancyScene::AdjustDrag(double delta) { dragCoefficient_ = std::clamp(dragCoefficient_ + delta, 0.0, 1600.0); }
void BuoyancyScene::ToggleDrag() { dragEnabled_ = !dragEnabled_; }
void BuoyancyScene::ToggleRotation() {
    rotationEnabled_ = !rotationEnabled_;
    if (!rotationEnabled_) {
        angularVelocity_ = 0.0;
        angularVelocity3D_ = {};
        angularMomentum3D_ = {};
    }
}

BuoyancySceneSnapshot BuoyancyScene::ReadSnapshot() const {
    BuoyancySceneSnapshot snapshot{};
    if (threeDimensional_) {
        snapshot.full3D = true;
        snapshot.corners3D = BodyCorners3D();
        ComputeSubmergedGeometry3D(snapshot.corners3D, snapshot.submergedVolume3D, snapshot.centerOfBuoyancy3D);
        snapshot.position3D = position3D_;
        snapshot.velocity3D = velocity3D_;
        snapshot.angularVelocity3D = angularVelocity3D_;
        snapshot.orientation3D = orientation3D_;
        snapshot.mass = mass_;
        snapshot.bodyDensity = mass_ / (kBodyWidth * kBodyHeight * kBodyDepth);
        snapshot.waterDensity = waterDensity_;
        snapshot.buoyancyForce3D = waterDensity_ * snapshot.submergedVolume3D * -kGravity;
        snapshot.torqueMagnitude3D = std::sqrt(Dot3(Cross3(Subtract3(snapshot.centerOfBuoyancy3D, position3D_), {0.0, snapshot.buoyancyForce3D, 0.0}), Cross3(Subtract3(snapshot.centerOfBuoyancy3D, position3D_), {0.0, snapshot.buoyancyForce3D, 0.0})));
        snapshot.position = {position3D_.x, position3D_.y};
        snapshot.velocity = {velocity3D_.x, velocity3D_.y};
        snapshot.submergedVolume = snapshot.submergedVolume3D;
        snapshot.buoyancyForce = snapshot.buoyancyForce3D;
        snapshot.dragForce = {0.0, 0.0};
        snapshot.mechanicalEnergy = 0.5 * mass_ * Dot3(velocity3D_, velocity3D_) + 0.5 * Dot3(angularVelocity3D_, angularMomentum3D_) - mass_ * kGravity * position3D_.y;
        snapshot.simulatedTime = simulatedTime_;
        snapshot.dragEnabled = dragEnabled_;
        return snapshot;
    }
    snapshot.corners = BodyCorners();
    ComputeSubmergedGeometry(snapshot.corners, snapshot.submergedArea, snapshot.centerOfBuoyancy, snapshot.submergedPolygon, snapshot.submergedCount);
    snapshot.position = position_;
    snapshot.velocity = velocity_;
    snapshot.angle = angle_;
    snapshot.angularVelocity = angularVelocity_;
    snapshot.mass = mass_;
    snapshot.bodyDensity = mass_ / (kBodyWidth * kBodyHeight * kBodyThickness);
    snapshot.waterDensity = waterDensity_;
    snapshot.submergedVolume = snapshot.submergedArea * kBodyThickness;
    snapshot.buoyancyForce = waterDensity_ * snapshot.submergedVolume * -kGravity;
    snapshot.dragForce = dragEnabled_ ? Scale(velocity_, -dragCoefficient_ * std::clamp(snapshot.submergedArea / (kBodyWidth * kBodyHeight), 0.0, 1.0)) : BuoyancyVec2{};
    snapshot.torque = Cross(Subtract(snapshot.centerOfBuoyancy, position_), {0.0, snapshot.buoyancyForce});
    snapshot.mechanicalEnergy = 0.5 * mass_ * Dot(velocity_, velocity_) + 0.5 * (mass_ * (kBodyWidth * kBodyWidth + kBodyHeight * kBodyHeight) / 12.0) * angularVelocity_ * angularVelocity_ - mass_ * kGravity * position_.y;
    snapshot.simulatedTime = simulatedTime_;
    snapshot.dragEnabled = dragEnabled_;
    return snapshot;
}
