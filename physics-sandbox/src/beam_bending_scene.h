#pragma once

#include "raylib.h"

#include <cstddef>
#include <vector>

// Linear Euler-Bernoulli beam gallery using two-node cubic Hermite elements.
// The displayed displacement may be exaggerated; diagnostics remain in SI
// units and identify the initial small-deflection model.
class BeamBendingScene {
public:
    BeamBendingScene();

    void Reset();
    void Step(float dt);
    void HandleInput(const Camera3D& camera, bool allowWorldInteraction = true);
    void Draw3D(const Camera3D& camera) const;
    void Draw2D() const;
    void DrawHud(bool paused) const;

    Vector3 FocusTarget() const;
    const char* SelectedStationName() const;
    float DriveFrequency() const { return driveFrequency_; }
    float TipMass() const { return tipMass_; }
    float SimulatedTime() const { return simulatedTime_; }
    std::size_t BeamCount() const { return beams_.size(); }

private:
    enum class Support { Cantilever, SimplySupported, ClampedClamped };
    enum class Kind { Cantilever, DivingBoard, Bridge, Material, Thickness, Driven, Clamped, ModeShape };

    struct Material {
        const char* name = "material";
        double youngsModulus = 1.0e9;
        double density = 700.0;
        Color color{120, 213, 255, 255};
    };

    struct Section {
        double width = 0.06;
        double height = 0.04;
        double Area() const { return width * height; }
        double Inertia() const { return width * height * height * height / 12.0; }
    };

    struct Beam {
        const char* name = "beam";
        Kind kind = Kind::Cantilever;
        int station = 0;
        Vector3 origin{};
        float length = 2.4f;
        int elements = 16;
        Support support = Support::Cantilever;
        Material material{};
        Section section{};
        double tipMass = 0.0;
        double load = 0.0;
        double loadPosition = 0.5;
        bool dynamic = true;
        std::vector<double> mass;
        std::vector<double> stiffness;
        std::vector<double> damping;
        std::vector<double> displacement;
        std::vector<double> velocity;
        std::vector<double> acceleration;
        std::vector<int> freeDofs;
        double analyticTipDeflection = 0.0;
        double maxDisplacement = 0.0;
    };

    static constexpr int kStationCount = 8;
    static constexpr float kDisplayScale = 6.0f;

    static double MatrixAt(const std::vector<double>& matrix, int size, int row, int column);
    static double& MatrixAt(std::vector<double>& matrix, int size, int row, int column);
    static Vector3 Add(Vector3 a, Vector3 b);
    static Vector3 Subtract(Vector3 a, Vector3 b);
    static Vector3 Scale(Vector3 value, float scalar);
    static float Dot(Vector3 a, Vector3 b);
    static float Length(Vector3 value);

    void BuildGallery();
    void AddBeam(const char* name, Kind kind, int station, Vector3 origin, Support support,
                 Material material, Section section, double load, double loadPosition,
                 bool dynamic, double tipMass = 0.0);
    void Assemble(Beam& beam);
    void ResetBeam(Beam& beam, bool useInitialLoad = true);
    void ResetStation(int station);
    void SolveStatic(Beam& beam, const std::vector<double>& force);
    bool SolveLinearSystem(std::vector<double> matrix, std::vector<double> rhs, std::vector<double>& solution) const;
    void StepBeam(Beam& beam, double dt);
    std::vector<double> ExternalForce(const Beam& beam, double time, bool initial) const;
    void AddPointLoad(const Beam& beam, std::vector<double>& force, double position, double value) const;
    void AddAttachmentForce(const Beam& beam, std::vector<double>& force) const;
    void SetModeShape(Beam& beam) const;
    double HermiteDisplacement(const Beam& beam, double x) const;
    Vector3 BeamPoint(const Beam& beam, double x, double displacement) const;
    void DrawBeam(const Beam& beam) const;
    void UpdateDiagnostics(Beam& beam);
    bool HandleStationClick(Vector2 mouse);
    Rectangle StationCardRectangle(int station) const;
    Rectangle StationResetRectangle(int station) const;
    int PickBeam(const Ray& ray, float& depth, double& localX) const;
    Vector3 PointOnRay(const Ray& ray, float depth) const;

    std::vector<Beam> beams_;
    int selectedStation_ = -1;
    int selectedBeam_ = -1;
    bool dragging_ = false;
    float dragDepth_ = 0.0f;
    double dragLocalX_ = 0.0;
    float dragTargetWorldY_ = 0.0f;
    int modeNumber_ = 1;
    float driveFrequency_ = 1.0f;
    float tipMass_ = 0.8f;
    float simulatedTime_ = 0.0f;
    double solverResidual_ = 0.0;
};

