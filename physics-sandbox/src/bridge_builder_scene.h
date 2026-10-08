#pragma once

#include "raylib.h"

#include <vector>

// A small construction-oriented bridge scene. The edit view is planar; the
// test view extrudes the same authored truss into 3D and drives a vehicle with
// gravity, motor force, and two-way deck contact through the spring network.
class BridgeBuilderScene {
public:
    BridgeBuilderScene();

    void Reset();
    void Step(float dt);
    void HandleInput(const Camera3D& camera, bool allowWorldInteraction = true);
    void Draw3D(const Camera3D& camera) const;
    void Draw2D() const;
    void DrawHud(bool paused) const;

    Vector3 FocusTarget() const { return {5.0f, 2.6f, 0.0f}; }
    bool TestMode() const { return testMode_; }
    bool BuildMode() const { return buildMode_; }
    int NodeCount() const { return static_cast<int>(nodes_.size()); }
    int EdgeCount() const { return static_cast<int>(edges_.size()); }
    float CartPosition() const { return cartPosition_; }

private:
    enum class LinkMaterial { Steel, Road, Wood, Rope };

    struct LinkProperties {
        const char* name = "STEEL";
        float stiffness = 420.0f;
        float damping = 4.0f;
        Color color{126, 183, 255, 255};
    };

    struct Node {
        Vector3 rest{};
        Vector3 position{};
        Vector3 velocity{};
        bool fixed = false;
    };

    struct Edge {
        int a = 0;
        int b = 0;
        float restLength = 0.0f;
        float stiffness = 420.0f;
        float damping = 4.0f;
        LinkMaterial material = LinkMaterial::Steel;
    };

    static Vector3 Add(Vector3 a, Vector3 b);
    static Vector3 Subtract(Vector3 a, Vector3 b);
    static Vector3 Scale(Vector3 value, float scalar);
    static float Dot(Vector3 a, Vector3 b);
    static float Length(Vector3 value);
    static Vector3 Normalize(Vector3 value);
    static float DistanceToSegment(Vector2 point, Vector2 a, Vector2 b, float& along);
    static LinkProperties Properties(LinkMaterial material);

    void BuildStarterBridge();
    void ClearAll();
    void RemoveNode(int nodeIndex);
    bool HandleBuildMenuClick(Vector2 mouse);
    int AddNode(Vector3 position, bool fixed);
    bool AddEdge(int a, int b);
    void RemoveNearestEdge(Vector2 mouse);
    int NearestNode(Vector2 mouse, float radius) const;
    Vector2 WorldToScreen(Vector3 position) const;
    Vector3 ScreenToWorld(Vector2 point) const;
    Vector3 PointOnRay(const Ray& ray, float depth) const;
    void ApplyVehicleContact(std::vector<Vector3>& forces, float& contactForce);
    bool DeckHeightAt(float x, float& height, float& velocity, int& leftNode, float& fraction) const;
    void ResetPhysics();
    void ToggleTestMode();
    void DrawEdge2D(const Edge& edge) const;
    void DrawEdge3D(const Edge& edge) const;

    std::vector<Node> nodes_;
    std::vector<Edge> edges_;
    bool buildMode_ = true;
    bool testMode_ = false;
    float cartPosition_ = 0.8f;
    float cartHeight_ = 1.63f;
    float cartVelocityX_ = 1.15f;
    float cartVelocityY_ = 0.0f;
    float cartMass_ = 1.2f;
    float cartDriveForce_ = 1.8f;
    float cartContactForce_ = 0.0f;
    LinkMaterial selectedMaterial_ = LinkMaterial::Steel;
    float simulatedTime_ = 0.0f;
    int linkStart_ = -1;
    int selectedNode_ = -1;
    int dragNode_ = -1;
    float dragDepth_ = 0.0f;
};
