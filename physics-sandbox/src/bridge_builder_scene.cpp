#include "bridge_builder_scene.h"
#include "web_mouse.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr float kGravity = -9.81f;
constexpr float kBridgeDepth = 1.35f;
constexpr float kDeckY = 1.45f;
constexpr float kNodeRadius = 0.10f;
constexpr float kCartHalfHeight = 0.18f;
constexpr float kGroundY = 0.35f;
}

Vector3 BridgeBuilderScene::Add(Vector3 a, Vector3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vector3 BridgeBuilderScene::Subtract(Vector3 a, Vector3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vector3 BridgeBuilderScene::Scale(Vector3 value, float scalar) { return {value.x * scalar, value.y * scalar, value.z * scalar}; }
float BridgeBuilderScene::Dot(Vector3 a, Vector3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
float BridgeBuilderScene::Length(Vector3 value) { return std::sqrt(Dot(value, value)); }
Vector3 BridgeBuilderScene::Normalize(Vector3 value) { const float length = Length(value); return length > 1.0e-6f ? Scale(value, 1.0f / length) : Vector3{0.0f, 1.0f, 0.0f}; }

float BridgeBuilderScene::DistanceToSegment(Vector2 point, Vector2 a, Vector2 b, float& along) {
    const Vector2 ab = {b.x - a.x, b.y - a.y};
    const Vector2 ap = {point.x - a.x, point.y - a.y};
    const float lengthSquared = ab.x * ab.x + ab.y * ab.y;
    along = lengthSquared > 1.0e-6f ? std::clamp((ap.x * ab.x + ap.y * ab.y) / lengthSquared, 0.0f, 1.0f) : 0.0f;
    const Vector2 closest = {a.x + ab.x * along, a.y + ab.y * along};
    const float dx = point.x - closest.x;
    const float dy = point.y - closest.y;
    return std::sqrt(dx * dx + dy * dy);
}

BridgeBuilderScene::LinkProperties BridgeBuilderScene::Properties(LinkMaterial material) {
    switch (material) {
        case LinkMaterial::Road: return {"ROAD", 700.0f, 10.0f, Color{255, 188, 92, 255}};
        case LinkMaterial::Wood: return {"WOOD", 260.0f, 6.0f, Color{196, 132, 78, 255}};
        case LinkMaterial::Rope: return {"ROPE", 90.0f, 2.0f, Color{213, 92, 161, 255}};
        case LinkMaterial::Steel: return {"STEEL", 420.0f, 4.0f, Color{126, 183, 255, 255}};
    }
    return {};
}

BridgeBuilderScene::BridgeBuilderScene() { Reset(); }

int BridgeBuilderScene::AddNode(Vector3 position, bool fixed) {
    nodes_.push_back({position, position, {}, fixed});
    return static_cast<int>(nodes_.size() - 1);
}

bool BridgeBuilderScene::AddEdge(int a, int b) {
    if (a == b || a < 0 || b < 0 || a >= static_cast<int>(nodes_.size()) || b >= static_cast<int>(nodes_.size())) return false;
    if (a > b) std::swap(a, b);
    for (const Edge& edge : edges_) if (edge.a == a && edge.b == b) return false;
    const float rest = Length(Subtract(nodes_[b].rest, nodes_[a].rest));
    if (rest < 0.05f) return false;
    const LinkProperties properties = Properties(selectedMaterial_);
    edges_.push_back({a, b, rest, properties.stiffness, properties.damping, selectedMaterial_});
    return true;
}

void BridgeBuilderScene::BuildStarterBridge() {
    nodes_.clear();
    edges_.clear();
    // A two-support deck with an upper truss. The same node/link graph is
    // used by the planar editor and the extruded 3D test view.
    for (int i = 0; i <= 10; ++i) AddNode({static_cast<float>(i), kDeckY, 0.0f}, i == 0 || i == 10);
    for (int i = 0; i <= 5; ++i) AddNode({static_cast<float>(i * 2), 3.25f, 0.0f}, false);
    selectedMaterial_ = LinkMaterial::Road;
    for (int i = 0; i < 10; ++i) AddEdge(i, i + 1);
    selectedMaterial_ = LinkMaterial::Steel;
    for (int i = 0; i < 5; ++i) {
        const int top = 11 + i;
        const int nextTop = top + 1;
        const int left = i * 2;
        const int right = left + 2;
        AddEdge(top, nextTop);
        AddEdge(top, left);
        AddEdge(top, left + 1);
        AddEdge(top, right);
        AddEdge(nextTop, left + 1);
        AddEdge(nextTop, right);
    }
    AddEdge(0, 11);
    AddEdge(10, 16);
    selectedMaterial_ = LinkMaterial::Steel;
    ResetPhysics();
}

void BridgeBuilderScene::ClearAll() {
    nodes_.clear();
    edges_.clear();
    buildMode_ = true;
    testMode_ = false;
    linkStart_ = -1;
    ResetPhysics();
}

void BridgeBuilderScene::RemoveNode(int nodeIndex) {
    if (nodeIndex < 0 || nodeIndex >= static_cast<int>(nodes_.size())) return;
    edges_.erase(std::remove_if(edges_.begin(), edges_.end(), [nodeIndex](const Edge& edge) {
        return edge.a == nodeIndex || edge.b == nodeIndex;
    }), edges_.end());
    nodes_.erase(nodes_.begin() + nodeIndex);
    for (Edge& edge : edges_) {
        if (edge.a > nodeIndex) --edge.a;
        if (edge.b > nodeIndex) --edge.b;
    }
    if (linkStart_ == nodeIndex) linkStart_ = -1;
    else if (linkStart_ > nodeIndex) --linkStart_;
    ResetPhysics();
}

bool BridgeBuilderScene::HandleBuildMenuClick(Vector2 mouse) {
    const float panelX = static_cast<float>(GetScreenWidth()) - 285.0f;
    const float panelY = 20.0f;
    for (int i = 0; i < 4; ++i) {
        const Rectangle button{panelX + 12.0f, panelY + 48.0f + i * 32.0f, 231.0f, 26.0f};
        if (CheckCollisionPointRec(mouse, button)) {
            selectedMaterial_ = static_cast<LinkMaterial>(i);
            return true;
        }
    }
    const Rectangle clearButton{panelX + 12.0f, panelY + 184.0f, 231.0f, 32.0f};
    if (CheckCollisionPointRec(mouse, clearButton)) {
        ClearAll();
        return true;
    }
    return false;
}

void BridgeBuilderScene::ResetPhysics() {
    for (Node& node : nodes_) { node.position = node.rest; node.velocity = {}; }
    simulatedTime_ = 0.0f;
    cartPosition_ = 0.8f;
    cartHeight_ = kDeckY + kCartHalfHeight;
    cartVelocityX_ = 1.15f;
    cartVelocityY_ = 0.0f;
    cartContactForce_ = 0.0f;
    selectedNode_ = -1;
    dragNode_ = -1;
}

void BridgeBuilderScene::Reset() {
    buildMode_ = true;
    testMode_ = false;
    linkStart_ = -1;
    BuildStarterBridge();
}

bool BridgeBuilderScene::DeckHeightAt(float x, float& height, float& velocity, int& leftNode, float& fraction) const {
    if (nodes_.size() < 11 || x < 0.0f || x > 10.0f) return false;
    const float clampedX = std::clamp(x, 0.0f, 10.0f);
    leftNode = std::clamp(static_cast<int>(std::floor(clampedX)), 0, 9);
    fraction = clampedX - static_cast<float>(leftNode);
    const Node& left = nodes_[static_cast<std::size_t>(leftNode)];
    const Node& right = nodes_[static_cast<std::size_t>(leftNode + 1)];
    height = left.position.y * (1.0f - fraction) + right.position.y * fraction;
    velocity = left.velocity.y * (1.0f - fraction) + right.velocity.y * fraction;
    return true;
}

void BridgeBuilderScene::ApplyVehicleContact(std::vector<Vector3>& forces, float& contactForce) {
    contactForce = 0.0f;
    float deckHeight = 0.0f;
    float deckVelocity = 0.0f;
    int leftNode = 0;
    float fraction = 0.0f;
    if (!DeckHeightAt(cartPosition_, deckHeight, deckVelocity, leftNode, fraction)) return;

    const float penetration = deckHeight - (cartHeight_ - kCartHalfHeight);
    if (penetration <= 0.0f) return;

    const float relativeVelocity = cartVelocityY_ - deckVelocity;
    contactForce = std::max(0.0f, 2600.0f * penetration - 45.0f * relativeVelocity);
    forces[static_cast<std::size_t>(leftNode)].y -= contactForce * (1.0f - fraction);
    forces[static_cast<std::size_t>(leftNode + 1)].y -= contactForce * fraction;
}

void BridgeBuilderScene::Step(float dt) {
    const float h = std::clamp(dt, 0.0f, 0.05f);
    if (h <= 0.0f || !testMode_) return;
    cartVelocityX_ += (cartDriveForce_ / cartMass_) * h;
    cartVelocityX_ *= 0.999f;
    cartPosition_ += cartVelocityX_ * h;
    if (cartPosition_ > 9.8f) { cartPosition_ = 9.8f; cartVelocityX_ = -std::abs(cartVelocityX_); }
    if (cartPosition_ < 0.2f) { cartPosition_ = 0.2f; cartVelocityX_ = std::abs(cartVelocityX_); }
    cartVelocityY_ += kGravity * h;
    std::vector<Vector3> forces(nodes_.size(), Vector3{});
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
        if (!nodes_[i].fixed) forces[i].y += 0.25f * kGravity;
    }
    ApplyVehicleContact(forces, cartContactForce_);
    cartVelocityY_ += (cartContactForce_ / cartMass_) * h;
    for (const Edge& edge : edges_) {
        Node& a = nodes_[static_cast<std::size_t>(edge.a)];
        Node& b = nodes_[static_cast<std::size_t>(edge.b)];
        const Vector3 delta = Subtract(b.position, a.position);
        const float length = Length(delta);
        if (length < 1.0e-5f) continue;
        const Vector3 direction = Scale(delta, 1.0f / length);
        const Vector3 relativeVelocity = Subtract(b.velocity, a.velocity);
        const float speed = Dot(relativeVelocity, direction);
        const float magnitude = edge.stiffness * (length - edge.restLength) + edge.damping * speed;
        const Vector3 springForce = Scale(direction, magnitude);
        if (!a.fixed) forces[static_cast<std::size_t>(edge.a)] = Add(forces[static_cast<std::size_t>(edge.a)], springForce);
        if (!b.fixed) forces[static_cast<std::size_t>(edge.b)] = Subtract(forces[static_cast<std::size_t>(edge.b)], springForce);
    }
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
        Node& node = nodes_[i];
        if (node.fixed) { node.position = node.rest; node.velocity = {}; continue; }
        node.velocity = Add(node.velocity, Scale(forces[i], h));
        node.velocity = Scale(node.velocity, 0.996f);
        node.position = Add(node.position, Scale(node.velocity, h));
        if (node.position.y < kGroundY) { node.position.y = kGroundY; if (node.velocity.y < 0.0f) node.velocity.y = 0.0f; }
    }
    cartHeight_ += cartVelocityY_ * h;
    float deckHeight = 0.0f;
    float deckVelocity = 0.0f;
    int leftNode = 0;
    float fraction = 0.0f;
    if (DeckHeightAt(cartPosition_, deckHeight, deckVelocity, leftNode, fraction) && cartHeight_ - kCartHalfHeight < deckHeight) {
        cartHeight_ = deckHeight + kCartHalfHeight;
        if (cartVelocityY_ < deckVelocity) cartVelocityY_ = deckVelocity;
    }
    if (cartHeight_ - kCartHalfHeight < kGroundY) {
        cartHeight_ = kGroundY + kCartHalfHeight;
        if (cartVelocityY_ < 0.0f) cartVelocityY_ = 0.0f;
    }
    cartVelocityY_ *= 0.999f;
    simulatedTime_ += h;
}

void BridgeBuilderScene::ToggleTestMode() {
    testMode_ = !testMode_;
    buildMode_ = !testMode_;
    linkStart_ = -1;
    ResetPhysics();
}

Vector2 BridgeBuilderScene::WorldToScreen(Vector3 position) const {
    const float scale = std::min(GetScreenWidth() * 0.72f / 12.0f, GetScreenHeight() * 0.72f / 6.0f);
    return {80.0f + position.x * scale, static_cast<float>(GetScreenHeight()) - 105.0f - position.y * scale};
}

Vector3 BridgeBuilderScene::ScreenToWorld(Vector2 point) const {
    const float scale = std::min(GetScreenWidth() * 0.72f / 12.0f, GetScreenHeight() * 0.72f / 6.0f);
    return {(point.x - 80.0f) / scale, (static_cast<float>(GetScreenHeight()) - 105.0f - point.y) / scale, 0.0f};
}

int BridgeBuilderScene::NearestNode(Vector2 mouse, float radius) const {
    int nearest = -1;
    float best = radius;
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
        const Vector2 screen = WorldToScreen(nodes_[i].position);
        const float dx = screen.x - mouse.x;
        const float dy = screen.y - mouse.y;
        const float distance = std::sqrt(dx * dx + dy * dy);
        if (distance < best) { best = distance; nearest = static_cast<int>(i); }
    }
    return nearest;
}

void BridgeBuilderScene::RemoveNearestEdge(Vector2 mouse) {
    int nearest = -1;
    float best = 14.0f;
    for (std::size_t i = 0; i < edges_.size(); ++i) {
        float along = 0.0f;
        const float distance = DistanceToSegment(mouse, WorldToScreen(nodes_[static_cast<std::size_t>(edges_[i].a)].position), WorldToScreen(nodes_[static_cast<std::size_t>(edges_[i].b)].position), along);
        if (distance < best) { best = distance; nearest = static_cast<int>(i); }
    }
    if (nearest >= 0) edges_.erase(edges_.begin() + nearest);
}

Vector3 BridgeBuilderScene::PointOnRay(const Ray& ray, float depth) const { return Add(ray.position, Scale(ray.direction, depth)); }

void BridgeBuilderScene::HandleInput(const Camera3D& camera, bool allowWorldInteraction) {
    if (IsKeyPressed(KEY_R)) Reset();
    if (IsKeyPressed(KEY_B)) ToggleTestMode();
    if (IsKeyPressed(KEY_SPACE)) ToggleTestMode();
    if (buildMode_ && IsKeyPressed(KEY_Z)) {
        const int next = (static_cast<int>(selectedMaterial_) + 1) % 4;
        selectedMaterial_ = static_cast<LinkMaterial>(next);
    }
    if (IsKeyPressed(KEY_C)) ClearAll();
    if (!allowWorldInteraction) {
        if (buildMode_ && PhysicsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            const Vector2 mouse = PhysicsMousePosition();
            if (HandleBuildMenuClick(mouse)) return;
            const int node = NearestNode(mouse, 17.0f);
            if (node >= 0) { if (linkStart_ < 0) linkStart_ = node; else { AddEdge(linkStart_, node); linkStart_ = -1; } }
            else if (nodes_.size() < 64) { const Vector3 point = ScreenToWorld(mouse); AddNode({std::round(point.x * 2.0f) * 0.5f, std::max(0.45f, std::round(point.y * 2.0f) * 0.5f), 0.0f}, false); }
        }
        if (buildMode_ && PhysicsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
            const Vector2 mouse = PhysicsMousePosition();
            const int node = NearestNode(mouse, 17.0f);
            if (node >= 0) RemoveNode(node);
            else RemoveNearestEdge(mouse);
        }
        return;
    }
    if (buildMode_) return;
    if (PhysicsMouseButtonPressed(MOUSE_BUTTON_LEFT) && dragNode_ < 0) {
        const Ray ray = GetMouseRay(PhysicsMousePosition(), camera);
        float bestDepth = 0.0f;
        float bestDistance = 0.22f;
        for (std::size_t i = 0; i < nodes_.size(); ++i) {
            const Vector3 toNode = Subtract(nodes_[i].position, ray.position);
            const float depth = Dot(toNode, ray.direction);
            if (depth <= 0.0f) continue;
            const float distance = Length(Subtract(nodes_[i].position, PointOnRay(ray, depth)));
            if (distance < bestDistance) { bestDistance = distance; bestDepth = depth; dragNode_ = static_cast<int>(i); }
        }
        if (dragNode_ >= 0) dragDepth_ = bestDepth;
    }
    if (dragNode_ >= 0 && PhysicsMouseButtonDown(MOUSE_BUTTON_LEFT) && !nodes_[static_cast<std::size_t>(dragNode_)].fixed) {
        const Vector3 point = PointOnRay(GetMouseRay(PhysicsMousePosition(), camera), dragDepth_);
        nodes_[static_cast<std::size_t>(dragNode_)].position = point;
        nodes_[static_cast<std::size_t>(dragNode_)].velocity = {};
    }
    if (dragNode_ >= 0 && PhysicsMouseButtonReleased(MOUSE_BUTTON_LEFT)) dragNode_ = -1;
}

void BridgeBuilderScene::DrawEdge2D(const Edge& edge) const {
    const Node& a = nodes_[static_cast<std::size_t>(edge.a)];
    const Node& b = nodes_[static_cast<std::size_t>(edge.b)];
    const float strain = Length(Subtract(b.position, a.position)) / edge.restLength - 1.0f;
    const float heat = std::clamp(std::abs(strain) * 22.0f, 0.0f, 1.0f);
    const Color base = Properties(edge.material).color;
    const Color color = {static_cast<unsigned char>(base.r + (255 - base.r) * heat), static_cast<unsigned char>(base.g * (1.0f - heat)), static_cast<unsigned char>(base.b * (1.0f - heat)), 255};
    DrawLineEx(WorldToScreen(a.position), WorldToScreen(b.position), 4.0f, color);
}

void BridgeBuilderScene::DrawEdge3D(const Edge& edge) const {
    const Node& a = nodes_[static_cast<std::size_t>(edge.a)];
    const Node& b = nodes_[static_cast<std::size_t>(edge.b)];
    const float strain = Length(Subtract(b.position, a.position)) / edge.restLength - 1.0f;
    const float heat = std::clamp(std::abs(strain) * 22.0f, 0.0f, 1.0f);
    const Color base = Properties(edge.material).color;
    const Color color = {static_cast<unsigned char>(base.r + (255 - base.r) * heat), static_cast<unsigned char>(base.g * (1.0f - heat)), static_cast<unsigned char>(base.b * (1.0f - heat)), 255};
    for (const float z : {-kBridgeDepth, kBridgeDepth}) DrawLine3D({a.position.x, a.position.y, z}, {b.position.x, b.position.y, z}, color);
    DrawLine3D(a.position, b.position, Fade(color, 0.5f));
}

void BridgeBuilderScene::Draw2D() const {
    DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), Color{10, 15, 27, 255});
    DrawText("2D BRIDGE BUILDER / DESIGN VIEW", 34, 32, 22, Color{232, 238, 248, 255});
    const float scale = std::min(GetScreenWidth() * 0.72f / 12.0f, GetScreenHeight() * 0.72f / 6.0f);
    for (int x = 0; x <= 12; ++x) DrawLine(static_cast<int>(80 + x * scale), 78, static_cast<int>(80 + x * scale), GetScreenHeight() - 104, Color{40, 52, 71, 255});
    for (int y = 0; y <= 6; ++y) DrawLine(80, static_cast<int>(GetScreenHeight() - 105 - y * scale), static_cast<int>(80 + 12 * scale), static_cast<int>(GetScreenHeight() - 105 - y * scale), Color{40, 52, 71, 255});
    for (const Edge& edge : edges_) DrawEdge2D(edge);
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
        const Vector2 point = WorldToScreen(nodes_[i].position);
        DrawCircleV(point, i == static_cast<std::size_t>(linkStart_) ? 10.0f : 7.0f, nodes_[i].fixed ? Color{255, 188, 92, 255} : Color{126, 235, 167, 255});
    }
    if (testMode_) {
        const Vector2 cart = WorldToScreen({cartPosition_, cartHeight_, 0.0f});
        DrawRectangle(static_cast<int>(cart.x - 15), static_cast<int>(cart.y - 10), 30, 20, Color{255, 112, 66, 255});
    }
}

void BridgeBuilderScene::Draw3D(const Camera3D& camera) const {
    (void)camera;
    DrawPlane({5.0f, 0.0f, 0.0f}, {18.0f, 10.0f}, Color{18, 26, 40, 255});
    for (int i = -2; i <= 12; ++i) DrawLine3D({static_cast<float>(i), 0.012f, -4.0f}, {static_cast<float>(i), 0.012f, 4.0f}, Color{40, 52, 71, 255});
    for (const Edge& edge : edges_) DrawEdge3D(edge);
    for (const Node& node : nodes_) {
        for (const float z : {-kBridgeDepth, kBridgeDepth}) DrawSphere({node.position.x, node.position.y, z}, kNodeRadius, node.fixed ? Color{255, 188, 92, 255} : Color{126, 235, 167, 255});
        DrawLine3D({node.position.x, node.position.y, -kBridgeDepth}, {node.position.x, node.position.y, kBridgeDepth}, Color{126, 183, 255, 220});
    }
    if (testMode_) {
        const int left = std::clamp(static_cast<int>(std::floor(cartPosition_)), 0, 9);
        const float fraction = cartPosition_ - static_cast<float>(left);
        const Vector3 a = nodes_[static_cast<std::size_t>(left)].position;
        const Vector3 b = nodes_[static_cast<std::size_t>(left + 1)].position;
        const Vector3 center = Add(Scale(a, 1.0f - fraction), Scale(b, fraction));
        DrawCube({center.x, cartHeight_, 0.0f}, 0.48f, 0.36f, 0.64f, Color{255, 112, 66, 255});
        DrawCubeWires({center.x, cartHeight_, 0.0f}, 0.48f, 0.36f, 0.64f, Color{255, 226, 180, 255});
    }
}

void BridgeBuilderScene::DrawHud(bool paused) const {
    DrawRectangle(20, 20, 640, 214, Color{15, 21, 34, 232});
    DrawRectangleLines(20, 20, 640, 214, Color{76, 91, 117, 255});
    DrawText("BRIDGE BUILDER", 38, 35, 22, Color{232, 238, 248, 255});
    DrawText(buildMode_ ? "2D DESIGN MODE" : "3D TEST MODE", 38, 68, 15, buildMode_ ? Color{120, 213, 255, 255} : Color{255, 188, 92, 255});
    DrawText(TextFormat("nodes %d   links %d   cart %s   %s", NodeCount(), EdgeCount(), testMode_ ? "DRIVING" : "OFF", paused ? "PAUSED" : "RUNNING"), 38, 98, 15, Color{196, 204, 219, 255});
    DrawText(TextFormat("material %s   height %.2f   contact %s", Properties(selectedMaterial_).name, cartHeight_, cartContactForce_ > 0.0f ? "ACTIVE" : "FREE"), 38, 122, 14, Color{255, 188, 92, 255});
    DrawText("2D: left-click nodes to link / empty grid to add", 38, 148, 13, Color{171, 183, 201, 255});
    DrawText("right-click node delete / link remove   C clear", 38, 173, 13, Color{171, 183, 201, 255});
    DrawText("Z cycle material   B/SPACE test   V 2D/3D   R reset", 38, 198, 13, Color{147, 160, 182, 255});

    if (buildMode_) {
        const float panelX = static_cast<float>(GetScreenWidth()) - 285.0f;
        const float panelY = 20.0f;
        DrawRectangle(static_cast<int>(panelX), static_cast<int>(panelY), 255, 230, Color{15, 21, 34, 242});
        DrawRectangleLines(static_cast<int>(panelX), static_cast<int>(panelY), 255, 230, Color{76, 91, 117, 255});
        DrawText("BUILD MATERIALS", static_cast<int>(panelX + 12.0f), static_cast<int>(panelY + 14.0f), 17, Color{232, 238, 248, 255});
        DrawText("select material for new links", static_cast<int>(panelX + 12.0f), static_cast<int>(panelY + 34.0f), 11, Color{171, 183, 201, 255});
        for (int i = 0; i < 4; ++i) {
            const LinkMaterial material = static_cast<LinkMaterial>(i);
            const Rectangle button{panelX + 12.0f, panelY + 48.0f + i * 32.0f, 231.0f, 26.0f};
            const LinkProperties properties = Properties(material);
            DrawRectangleRec(button, material == selectedMaterial_ ? Color{54, 70, 97, 255} : Color{25, 34, 51, 255});
            DrawRectangleLinesEx(button, material == selectedMaterial_ ? 2.0f : 1.0f, material == selectedMaterial_ ? Color{120, 213, 255, 255} : Color{62, 76, 99, 255});
            DrawCircle(static_cast<int>(button.x + 13.0f), static_cast<int>(button.y + 13.0f), 6.0f, properties.color);
            DrawText(properties.name, static_cast<int>(button.x + 27.0f), static_cast<int>(button.y + 6.0f), 13, Color{232, 238, 248, 255});
            DrawText(TextFormat("k %.0f", properties.stiffness), static_cast<int>(button.x + 150.0f), static_cast<int>(button.y + 6.0f), 12, Color{171, 183, 201, 255});
        }
        const Rectangle clearButton{panelX + 12.0f, panelY + 184.0f, 231.0f, 32.0f};
        DrawRectangleRec(clearButton, Color{97, 45, 57, 255});
        DrawRectangleLinesEx(clearButton, 1.0f, Color{255, 130, 130, 255});
        DrawText("CLEAR ALL NODES + LINKS", static_cast<int>(clearButton.x + 24.0f), static_cast<int>(clearButton.y + 9.0f), 12, Color{255, 226, 226, 255});
    }
}
