#pragma once

#include "raylib.h"

#include <array>
#include <cstddef>
#include <string>
#include <vector>

// First deliverable from cloth_and_elastic_solids_plan.md.
// The scene deliberately labels the soft-body model as introductory: cloth uses
// XPBD distance constraints and the jelly uses XPBD edge + signed-volume
// constraints. It is not a calibrated continuum FEM material.
class ClothElasticSolidsScene {
public:
    ClothElasticSolidsScene();
    ~ClothElasticSolidsScene();

    ClothElasticSolidsScene(const ClothElasticSolidsScene&) = delete;
    ClothElasticSolidsScene& operator=(const ClothElasticSolidsScene&) = delete;

    void Reset();
    void Step(float dt);
    void HandleInput(const Camera3D& camera, bool allowWorldInteraction = true);
    void Draw3D(const Camera3D& camera) const;
    void Draw2D() const;
    void DrawHud(bool paused) const;

    void ToggleWind();
    void AdjustWind(float delta);
    void ToggleWireframe();

    bool WindEnabled() const { return windEnabled_; }
    bool WireframeEnabled() const { return wireframe_; }
    float WindStrength() const { return windStrength_; }
    float SimulatedTime() const { return simulatedTime_; }
    std::size_t ParticleCount() const { return nodes_.size(); }
    std::size_t ConstraintCount() const { return constraintCount_; }
    float MaxStretch() const { return maxStretch_; }
    float JellyVolumeRatio() const { return jellyVolumeRatio_; }
    bool Dragging() const { return dragging_; }
    Vector3 FocusTarget() const;
    const char* SelectedStationName() const;

private:
    enum class Collider { None, Sphere };

    struct Node {
        Vector3 rest{};
        Vector3 position{};
        Vector3 previous{};
        Vector3 velocity{};
        float inverseMass = 1.0f;
        bool pinned = false;
    };

    struct DistanceConstraint {
        int a = 0;
        int b = 0;
        float restLength = 0.0f;
        float compliance = 0.0f;
        float lambda = 0.0f;
    };

    struct VolumeConstraint {
        std::array<int, 4> nodes{};
        float restSignedVolume = 0.0f;
        float compliance = 0.0f;
        float lambda = 0.0f;
    };

    struct Cloth {
        const char* name = "cloth";
        int columns = 0;
        int rows = 0;
        std::vector<int> nodes;
        std::vector<DistanceConstraint> constraints;
        Vector3 origin{};
        Vector3 axisU{};
        Vector3 axisV{};
        Color color{};
        Collider collider = Collider::None;
    };

    struct Jelly {
        std::array<int, 8> corners{};
        std::vector<DistanceConstraint> constraints;
        std::vector<VolumeConstraint> volumes;
        float restVolume = 0.0f;
    };

    int AddNode(Vector3 position, float mass, bool pinned = false);
    void AddDistance(std::vector<DistanceConstraint>& constraints, int a, int b, float compliance);
    Cloth& AddCloth(const char* name, Vector3 origin, Vector3 axisU, Vector3 axisV,
                    int columns, int rows, float mass, float stretchCompliance,
                    float bendCompliance, Color color, Collider collider = Collider::None);
    void BuildScene();
    void BuildJelly();
    void ResetConstraintMultipliers();
    void Integrate(float dt);
    void ApplyWind(std::vector<Vector3>& forces) const;
    void SolveDistance(DistanceConstraint& constraint, float dt);
    void SolveVolume(VolumeConstraint& constraint, float dt);
    void SolveCollisions();
    void UpdateDiagnostics();
    bool HandleStationClick(Vector2 mouse);
    void ResetStation(int station);
    Rectangle StationCardRectangle(int station) const;
    Rectangle StationResetRectangle(int station) const;
    void DrawCloth(const Cloth& cloth) const;
    void DrawJelly() const;
    void DrawStationMarkers() const;
    int PickNode(const Ray& ray, float& depth) const;
    Vector3 MousePointOnRay(const Ray& ray, float depth) const;

    static Vector3 Add(Vector3 a, Vector3 b);
    static Vector3 Subtract(Vector3 a, Vector3 b);
    static Vector3 Scale(Vector3 value, float scalar);
    static float Dot(Vector3 a, Vector3 b);
    static Vector3 Cross(Vector3 a, Vector3 b);
    static float Length(Vector3 value);
    static Vector3 Normalize(Vector3 value);
    static float SignedVolume(Vector3 a, Vector3 b, Vector3 c, Vector3 d);
    static Color StrainColor(float strain, Color base);

    std::vector<Node> nodes_;
    std::vector<Cloth> cloths_;
    Jelly* jellyPtr_ = nullptr;
    std::size_t constraintCount_ = 0;
    float simulatedTime_ = 0.0f;
    float windStrength_ = 3.0f;
    float maxStretch_ = 0.0f;
    float jellyVolumeRatio_ = 1.0f;
    bool windEnabled_ = true;
    bool wireframe_ = false;
    bool dragging_ = false;
    int draggedNode_ = -1;
    float dragDepth_ = 0.0f;
    float draggedInverseMass_ = 0.0f;
    int selectedStation_ = -1;
};
