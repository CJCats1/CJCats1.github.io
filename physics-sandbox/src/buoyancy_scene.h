#pragma once

#include <array>

struct BuoyancyVec2 {
    double x = 0.0;
    double y = 0.0;
};

struct BuoyancyVec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct BuoyancyQuat {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct BuoyancySceneSnapshot {
    std::array<BuoyancyVec2, 4> corners{};
    std::array<BuoyancyVec2, 8> submergedPolygon{};
    int submergedCount = 0;
    BuoyancyVec2 position{};
    BuoyancyVec2 velocity{};
    BuoyancyVec2 centerOfBuoyancy{};
    double angle = 0.0;
    double angularVelocity = 0.0;
    double mass = 0.0;
    double bodyDensity = 0.0;
    double waterDensity = 0.0;
    double submergedArea = 0.0;
    double submergedVolume = 0.0;
    double buoyancyForce = 0.0;
    BuoyancyVec2 dragForce{};
    double torque = 0.0;
    double mechanicalEnergy = 0.0;
    double simulatedTime = 0.0;
    bool dragEnabled = true;
    bool full3D = false;
    BuoyancyVec3 position3D{};
    BuoyancyVec3 velocity3D{};
    BuoyancyVec3 angularVelocity3D{};
    BuoyancyVec3 centerOfBuoyancy3D{};
    BuoyancyQuat orientation3D{};
    std::array<BuoyancyVec3, 8> corners3D{};
    double submergedVolume3D = 0.0;
    double buoyancyForce3D = 0.0;
    double torqueMagnitude3D = 0.0;
};

class BuoyancyScene {
public:
    static constexpr double kTankHalfWidth = 6.0;
    static constexpr double kTankHeight = 8.0;
    static constexpr double kWaterLevel = 3.0;
    static constexpr double kWaterDensity = 1000.0;
    static constexpr double kGravity = -9.81;
    static constexpr double kBodyWidth = 2.2;
    static constexpr double kBodyHeight = 1.2;
    static constexpr double kBodyThickness = 1.0;
    static constexpr double kBodyDepth = 1.4;
    static constexpr double kTankHalfDepth = 4.0;

    BuoyancyScene();

    void Reset();
    void Step(double dt);
    void SetThreeDimensional(bool enabled) { threeDimensional_ = enabled; }
    bool IsThreeDimensional() const { return threeDimensional_; }
    void ApplyImpulse(BuoyancyVec2 impulse);
    void AdjustMass(double delta);
    void AdjustWaterDensity(double delta);
    void AdjustDrag(double delta);
    void ToggleDrag();
    void ToggleRotation();

    BuoyancySceneSnapshot ReadSnapshot() const;
    double GetMass() const { return mass_; }
    double GetWaterDensity() const { return waterDensity_; }
    double GetDragCoefficient() const { return dragCoefficient_; }
    bool GetRotationEnabled() const { return rotationEnabled_; }

private:
    using Polygon = std::array<BuoyancyVec2, 8>;

    static BuoyancyVec2 Add(BuoyancyVec2 a, BuoyancyVec2 b);
    static BuoyancyVec2 Subtract(BuoyancyVec2 a, BuoyancyVec2 b);
    static BuoyancyVec2 Scale(BuoyancyVec2 value, double scalar);
    static double Dot(BuoyancyVec2 a, BuoyancyVec2 b);
    static double Cross(BuoyancyVec2 a, BuoyancyVec2 b);
    static BuoyancyVec2 Rotate(BuoyancyVec2 value, double angle);
    static BuoyancyVec2 CrossScalar(double scalar, BuoyancyVec2 value);
    static BuoyancyVec3 Add3(BuoyancyVec3 a, BuoyancyVec3 b);
    static BuoyancyVec3 Subtract3(BuoyancyVec3 a, BuoyancyVec3 b);
    static BuoyancyVec3 Scale3(BuoyancyVec3 value, double scalar);
    static double Dot3(BuoyancyVec3 a, BuoyancyVec3 b);
    static BuoyancyVec3 Cross3(BuoyancyVec3 a, BuoyancyVec3 b);
    static BuoyancyQuat NormalizeQuat(BuoyancyQuat value);
    static BuoyancyQuat MultiplyQuat(BuoyancyQuat a, BuoyancyQuat b);
    static BuoyancyVec3 Rotate3(BuoyancyQuat orientation, BuoyancyVec3 value);
    static BuoyancyVec3 InverseRotate3(BuoyancyQuat orientation, BuoyancyVec3 value);
    static BuoyancyQuat IntegrateQuat(BuoyancyQuat orientation, BuoyancyVec3 angularVelocity, double dt);

    std::array<BuoyancyVec2, 4> BodyCorners() const;
    std::array<BuoyancyVec3, 8> BodyCorners3D() const;
    Polygon ClipBelowWater(const std::array<BuoyancyVec2, 4>& corners, int& count) const;
    void ComputeSubmergedGeometry(const std::array<BuoyancyVec2, 4>& corners, double& area, BuoyancyVec2& centroid, Polygon& polygon, int& count) const;
    void ApplyTankContacts();
    void Step3D(double dt);
    void ComputeSubmergedGeometry3D(const std::array<BuoyancyVec3, 8>& corners, double& volume, BuoyancyVec3& centroid) const;
    void ApplyTankContacts3D();
    BuoyancyVec3 InverseWorldInertiaMultiply(BuoyancyVec3 value) const;

    BuoyancyVec2 position_{};
    BuoyancyVec2 velocity_{};
    double angle_ = 0.0;
    double angularVelocity_ = 0.0;
    double mass_ = 1100.0;
    double waterDensity_ = kWaterDensity;
    double dragCoefficient_ = 420.0;
    double dampingTorque_ = 240.0;
    double restitution_ = 0.12;
    double simulatedTime_ = 0.0;
    bool dragEnabled_ = true;
    bool rotationEnabled_ = true;
    bool threeDimensional_ = false;

    BuoyancyVec3 position3D_{};
    BuoyancyVec3 velocity3D_{};
    BuoyancyVec3 angularVelocity3D_{};
    BuoyancyVec3 angularMomentum3D_{};
    BuoyancyQuat orientation3D_{};
};
