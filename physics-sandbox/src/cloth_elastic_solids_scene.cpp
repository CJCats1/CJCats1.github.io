#include "cloth_elastic_solids_scene.h"
#include "web_mouse.h"
#include "rlgl.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace {
constexpr float kGravity = -9.81f;
constexpr float kFixedSubstep = 1.0f / 120.0f;
constexpr int kSolverIterations = 7;
constexpr float kSphereRadius = 0.92f;
const Vector3 kSphereCenter = {3.55f, 1.08f, -1.15f};

struct Edge { int a = 0; int b = 0; };
}

Vector3 ClothElasticSolidsScene::Add(Vector3 a, Vector3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vector3 ClothElasticSolidsScene::Subtract(Vector3 a, Vector3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vector3 ClothElasticSolidsScene::Scale(Vector3 value, float scalar) { return {value.x * scalar, value.y * scalar, value.z * scalar}; }
float ClothElasticSolidsScene::Dot(Vector3 a, Vector3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vector3 ClothElasticSolidsScene::Cross(Vector3 a, Vector3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
float ClothElasticSolidsScene::Length(Vector3 value) { return std::sqrt(Dot(value, value)); }
Vector3 ClothElasticSolidsScene::Normalize(Vector3 value) {
    const float length = Length(value);
    return length > 1.0e-6f ? Scale(value, 1.0f / length) : Vector3{0.0f, 1.0f, 0.0f};
}
float ClothElasticSolidsScene::SignedVolume(Vector3 a, Vector3 b, Vector3 c, Vector3 d) {
    return Dot(Subtract(b, a), Cross(Subtract(c, a), Subtract(d, a))) / 6.0f;
}

Color ClothElasticSolidsScene::StrainColor(float strain, Color base) {
    const float amount = std::clamp(std::abs(strain) * 18.0f, 0.0f, 1.0f);
    const Color hot = {255, 112, 66, 255};
    return {
        static_cast<unsigned char>(base.r + (hot.r - base.r) * amount),
        static_cast<unsigned char>(base.g + (hot.g - base.g) * amount),
        static_cast<unsigned char>(base.b + (hot.b - base.b) * amount), 255
    };
}

void DrawDoubleSidedTriangle(Vector3 a, Vector3 b, Vector3 c, Color color) {
    // Keep both windings in the immediate-mode path. This makes the cloth
    // double-sided even if a backend or render-state transition restores face
    // culling between batched raylib draw calls.
    DrawTriangle3D(a, b, c, color);
    DrawTriangle3D(c, b, a, color);
}

ClothElasticSolidsScene::ClothElasticSolidsScene() { Reset(); }
ClothElasticSolidsScene::~ClothElasticSolidsScene() { delete jellyPtr_; }

int ClothElasticSolidsScene::AddNode(Vector3 position, float mass, bool pinned) {
    Node node;
    node.rest = position;
    node.position = position;
    node.previous = position;
    node.inverseMass = pinned ? 0.0f : 1.0f / std::max(0.001f, mass);
    node.pinned = pinned;
    nodes_.push_back(node);
    return static_cast<int>(nodes_.size() - 1);
}

void ClothElasticSolidsScene::AddDistance(std::vector<DistanceConstraint>& constraints, int a, int b, float compliance) {
    DistanceConstraint constraint;
    constraint.a = a;
    constraint.b = b;
    constraint.restLength = Length(Subtract(nodes_[b].rest, nodes_[a].rest));
    constraint.compliance = compliance;
    constraints.push_back(constraint);
}

ClothElasticSolidsScene::Cloth& ClothElasticSolidsScene::AddCloth(
    const char* name, Vector3 origin, Vector3 axisU, Vector3 axisV, int columns, int rows,
    float mass, float stretchCompliance, float bendCompliance, Color color, Collider collider) {
    cloths_.push_back({});
    Cloth& cloth = cloths_.back();
    cloth.name = name;
    cloth.columns = columns;
    cloth.rows = rows;
    cloth.origin = origin;
    cloth.axisU = axisU;
    cloth.axisV = axisV;
    cloth.color = color;
    cloth.collider = collider;
    cloth.nodes.resize(static_cast<std::size_t>(columns * rows));

    const float nodeMass = mass / static_cast<float>(columns * rows);
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < columns; ++x) {
            const float u = columns > 1 ? static_cast<float>(x) / static_cast<float>(columns - 1) : 0.0f;
            const float v = rows > 1 ? static_cast<float>(y) / static_cast<float>(rows - 1) : 0.0f;
            const Vector3 position = Add(origin, Add(Scale(axisU, u), Scale(axisV, v)));
            const bool pin = (std::string(name) == "FLAG" || std::string(name) == "CURTAIN") && x == 0;
            cloth.nodes[static_cast<std::size_t>(x + columns * y)] = AddNode(position, nodeMass, pin);
        }
    }
    const auto index = [columns](int x, int y) { return x + columns * y; };
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < columns; ++x) {
            if (x + 1 < columns) AddDistance(cloth.constraints, cloth.nodes[index(x, y)], cloth.nodes[index(x + 1, y)], stretchCompliance);
            if (y + 1 < rows) AddDistance(cloth.constraints, cloth.nodes[index(x, y)], cloth.nodes[index(x, y + 1)], stretchCompliance);
            if (x + 1 < columns && y + 1 < rows) {
                AddDistance(cloth.constraints, cloth.nodes[index(x, y)], cloth.nodes[index(x + 1, y + 1)], stretchCompliance);
                AddDistance(cloth.constraints, cloth.nodes[index(x + 1, y)], cloth.nodes[index(x, y + 1)], stretchCompliance);
            }
            if (x + 2 < columns) AddDistance(cloth.constraints, cloth.nodes[index(x, y)], cloth.nodes[index(x + 2, y)], bendCompliance);
            if (y + 2 < rows) AddDistance(cloth.constraints, cloth.nodes[index(x, y)], cloth.nodes[index(x, y + 2)], bendCompliance);
        }
    }
    constraintCount_ += cloth.constraints.size();
    return cloth;
}

void ClothElasticSolidsScene::BuildJelly() {
    jellyPtr_ = new Jelly();
    Jelly& jelly = *jellyPtr_;
    const Vector3 center = {4.85f, 1.35f, 1.25f};
    const Vector3 half = {0.68f, 0.68f, 0.68f};
    const std::array<Vector3, 8> offsets = {{
        {-half.x, -half.y, -half.z}, {half.x, -half.y, -half.z}, {half.x, half.y, -half.z}, {-half.x, half.y, -half.z},
        {-half.x, -half.y, half.z}, {half.x, -half.y, half.z}, {half.x, half.y, half.z}, {-half.x, half.y, half.z}
    }};
    for (std::size_t i = 0; i < offsets.size(); ++i) jelly.corners[i] = AddNode(Add(center, offsets[i]), 0.16f, false);

    const std::array<Edge, 20> edges = {{
        {0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4},
        {0, 4}, {1, 5}, {2, 6}, {3, 7}, {0, 2}, {1, 3}, {4, 6}, {5, 7},
        {0, 6}, {1, 7}, {2, 4}, {3, 5}
    }};
    for (const Edge edge : edges) AddDistance(jelly.constraints, jelly.corners[edge.a], jelly.corners[edge.b], 0.00008f);

    // Five positively oriented tetrahedra cover the cube. Volume constraints
    // retain a useful introductory volume-retention signal during impact.
    const std::array<std::array<int, 4>, 5> tetrahedra = {{
        {{0, 1, 3, 4}}, {{1, 2, 3, 6}}, {{1, 3, 4, 6}}, {{1, 4, 5, 6}}, {{3, 4, 6, 7}}
    }};
    for (const auto& tet : tetrahedra) {
        VolumeConstraint volume;
        for (int i = 0; i < 4; ++i) volume.nodes[static_cast<std::size_t>(i)] = jelly.corners[static_cast<std::size_t>(tet[static_cast<std::size_t>(i)])];
        volume.restSignedVolume = SignedVolume(nodes_[volume.nodes[0]].rest, nodes_[volume.nodes[1]].rest, nodes_[volume.nodes[2]].rest, nodes_[volume.nodes[3]].rest);
        volume.compliance = 0.000002f;
        jelly.volumes.push_back(volume);
        jelly.restVolume += std::abs(volume.restSignedVolume);
    }
    constraintCount_ += jelly.constraints.size() + jelly.volumes.size();
}

void ClothElasticSolidsScene::BuildScene() {
    nodes_.clear();
    cloths_.clear();
    if (jellyPtr_ != nullptr) { delete jellyPtr_; jellyPtr_ = nullptr; }
    constraintCount_ = 0;

    AddCloth("FLAG", {-5.5f, 5.55f, -1.7f}, {2.9f, 0.0f, 0.0f}, {0.0f, -2.05f, 0.0f}, 13, 9, 1.1f, 0.00004f, 0.0008f, {54, 148, 255, 255});
    AddCloth("CURTAIN", {-2.0f, 4.75f, -1.85f}, {2.0f, 0.0f, 0.0f}, {0.0f, -3.55f, 0.0f}, 11, 13, 1.4f, 0.00010f, 0.0012f, {213, 92, 161, 255});
    AddCloth("HAMMOCK", {-0.4f, 3.25f, -2.55f}, {2.65f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.55f}, 11, 7, 1.6f, 0.00009f, 0.0014f, {58, 204, 166, 255});

    const std::array<Color, 3> materialColors = {{{92, 186, 255, 255}, {242, 187, 75, 255}, {152, 112, 231, 255}}};
    const std::array<const char*, 3> materialNames = {{"SOFT SHEET", "STIFF SHEET", "STIFF BEND"}};
    for (int i = 0; i < 3; ++i) {
        const float x = -4.15f + static_cast<float>(i) * 1.72f;
        const float stretch = i == 0 ? 0.00015f : 0.000018f;
        const float bend = i == 2 ? 0.00012f : 0.0012f;
        AddCloth(materialNames[static_cast<std::size_t>(i)], {x, 3.95f, 2.15f}, {1.25f, 0.0f, 0.0f}, {0.0f, -1.6f, 0.0f}, 7, 6, 0.55f, stretch, bend, materialColors[static_cast<std::size_t>(i)]);
    }
    AddCloth("SPHERE DRAPE", {2.45f, 2.65f, -1.85f}, {2.15f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.7f}, 11, 9, 1.3f, 0.00008f, 0.0010f, {224, 117, 69, 255}, Collider::Sphere);
    BuildJelly();
}

void ClothElasticSolidsScene::Reset() {
    simulatedTime_ = 0.0f;
    maxStretch_ = 0.0f;
    jellyVolumeRatio_ = 1.0f;
    dragging_ = false;
    draggedNode_ = -1;
    selectedStation_ = -1;
    BuildScene();
}

Vector3 ClothElasticSolidsScene::FocusTarget() const {
    switch (selectedStation_) {
        case 0: return {-1.9f, 3.25f, -1.65f};
        case 1: return {-1.7f, 3.35f, 2.15f};
        case 2: return {3.55f, 1.55f, -1.15f};
        case 3: return {4.85f, 1.45f, 1.25f};
        default: return {0.0f, 2.75f, 0.0f};
    }
}

const char* ClothElasticSolidsScene::SelectedStationName() const {
    switch (selectedStation_) {
        case 0: return "CLOTH WALL";
        case 1: return "MATERIAL TESTS";
        case 2: return "SPHERE DRAPE";
        case 3: return "JELLY CUBE";
        default: return "GALLERY OVERVIEW";
    }
}

Rectangle ClothElasticSolidsScene::StationCardRectangle(int station) const {
    return {static_cast<float>(GetScreenWidth() - 295), 58.0f + static_cast<float>(station) * 29.0f, 275.0f, 24.0f};
}

Rectangle ClothElasticSolidsScene::StationResetRectangle(int station) const {
    const Rectangle card = StationCardRectangle(station);
    return {card.x + card.width - 62.0f, card.y + 2.0f, 58.0f, card.height - 4.0f};
}

void ClothElasticSolidsScene::ResetStation(int station) {
    if (station == 3) {
        if (jellyPtr_ == nullptr) return;
        for (int index : jellyPtr_->corners) {
            Node& node = nodes_[static_cast<std::size_t>(index)];
            node.position = node.rest;
            node.previous = node.rest;
            node.velocity = {};
        }
        for (DistanceConstraint& constraint : jellyPtr_->constraints) constraint.lambda = 0.0f;
        for (VolumeConstraint& constraint : jellyPtr_->volumes) constraint.lambda = 0.0f;
    } else {
        const int firstCloth = station == 0 ? 0 : (station == 1 ? 3 : 6);
        const int lastCloth = station == 0 ? 3 : (station == 1 ? 6 : 7);
        for (int clothIndex = firstCloth; clothIndex < lastCloth; ++clothIndex) {
            Cloth& cloth = cloths_[static_cast<std::size_t>(clothIndex)];
            for (int index : cloth.nodes) {
                Node& node = nodes_[static_cast<std::size_t>(index)];
                node.position = node.rest;
                node.previous = node.rest;
                node.velocity = {};
            }
            for (DistanceConstraint& constraint : cloth.constraints) constraint.lambda = 0.0f;
        }
    }
    UpdateDiagnostics();
}

bool ClothElasticSolidsScene::HandleStationClick(Vector2 mouse) {
    for (int station = 0; station < 4; ++station) {
        if (!CheckCollisionPointRec(mouse, StationCardRectangle(station))) continue;
        selectedStation_ = station;
        if (CheckCollisionPointRec(mouse, StationResetRectangle(station))) ResetStation(station);
        return true;
    }
    return false;
}

void ClothElasticSolidsScene::ResetConstraintMultipliers() {
    for (Cloth& cloth : cloths_) for (DistanceConstraint& constraint : cloth.constraints) constraint.lambda = 0.0f;
    if (jellyPtr_ == nullptr) return;
    for (DistanceConstraint& constraint : jellyPtr_->constraints) constraint.lambda = 0.0f;
    for (VolumeConstraint& constraint : jellyPtr_->volumes) constraint.lambda = 0.0f;
}

void ClothElasticSolidsScene::ApplyWind(std::vector<Vector3>& forces) const {
    if (!windEnabled_) return;
    const Vector3 airVelocity = {windStrength_ * (0.72f + 0.28f * std::sin(simulatedTime_ * 1.7f)), 0.0f, windStrength_ * 0.20f * std::cos(simulatedTime_ * 1.1f)};
    for (const Cloth& cloth : cloths_) {
        if (cloth.nodes.size() < 4) continue;
        for (int y = 0; y + 1 < cloth.rows; ++y) {
            for (int x = 0; x + 1 < cloth.columns; ++x) {
                const auto index = [columns = cloth.columns](int px, int py) { return px + columns * py; };
                const int ia = cloth.nodes[index(x, y)];
                const int ib = cloth.nodes[index(x + 1, y)];
                const int ic = cloth.nodes[index(x, y + 1)];
                const int id = cloth.nodes[index(x + 1, y + 1)];
                const std::array<std::array<int, 3>, 2> triangles = {{{{ia, ib, ic}}, {{ib, id, ic}}}};
                for (const auto& triangle : triangles) {
                    const Vector3 a = nodes_[triangle[0]].position;
                    const Vector3 b = nodes_[triangle[1]].position;
                    const Vector3 c = nodes_[triangle[2]].position;
                    const Vector3 cross = Cross(Subtract(b, a), Subtract(c, a));
                    const float doubleArea = Length(cross);
                    if (doubleArea < 1.0e-6f) continue;
                    const Vector3 normal = Scale(cross, 1.0f / doubleArea);
                    const float area = doubleArea * 0.5f;
                    const Vector3 relative = Subtract(airVelocity, Scale(Add(Add(nodes_[triangle[0]].velocity, nodes_[triangle[1]].velocity), nodes_[triangle[2]].velocity), 1.0f / 3.0f));
                    const float normalSpeed = Dot(relative, normal);
                    const float pressure = 0.5f * 1.225f * 1.15f * area * normalSpeed * std::abs(normalSpeed);
                    const Vector3 force = Scale(normal, pressure);
                    for (int node : triangle) forces[static_cast<std::size_t>(node)] = Add(forces[static_cast<std::size_t>(node)], Scale(force, 1.0f / 3.0f));
                }
            }
        }
    }
}

void ClothElasticSolidsScene::Integrate(float dt) {
    std::vector<Vector3> forces(nodes_.size(), Vector3{0.0f, 0.0f, 0.0f});
    for (const Node& node : nodes_) {
        if (node.inverseMass > 0.0f) forces[static_cast<std::size_t>(&node - nodes_.data())].y += kGravity / node.inverseMass;
    }
    ApplyWind(forces);
    for (Node& node : nodes_) {
        node.previous = node.position;
        if (node.inverseMass <= 0.0f) continue;
        node.velocity = Scale(node.velocity, std::pow(0.992f, dt / kFixedSubstep));
        node.velocity = Add(node.velocity, Scale(forces[static_cast<std::size_t>(&node - nodes_.data())], node.inverseMass * dt));
        node.position = Add(node.position, Scale(node.velocity, dt));
    }
}

void ClothElasticSolidsScene::SolveDistance(DistanceConstraint& constraint, float dt) {
    Node& a = nodes_[static_cast<std::size_t>(constraint.a)];
    Node& b = nodes_[static_cast<std::size_t>(constraint.b)];
    const Vector3 delta = Subtract(b.position, a.position);
    const float length = Length(delta);
    if (length < 1.0e-6f) return;
    const Vector3 direction = Scale(delta, 1.0f / length);
    const float c = length - constraint.restLength;
    const float alpha = constraint.compliance / (dt * dt);
    const float denominator = a.inverseMass + b.inverseMass + alpha;
    if (denominator < 1.0e-8f) return;
    const float deltaLambda = (-c - alpha * constraint.lambda) / denominator;
    constraint.lambda += deltaLambda;
    a.position = Add(a.position, Scale(direction, -a.inverseMass * deltaLambda));
    b.position = Add(b.position, Scale(direction, b.inverseMass * deltaLambda));
}

void ClothElasticSolidsScene::SolveVolume(VolumeConstraint& constraint, float dt) {
    Node& a = nodes_[static_cast<std::size_t>(constraint.nodes[0])];
    Node& b = nodes_[static_cast<std::size_t>(constraint.nodes[1])];
    Node& c = nodes_[static_cast<std::size_t>(constraint.nodes[2])];
    Node& d = nodes_[static_cast<std::size_t>(constraint.nodes[3])];
    const float volume = SignedVolume(a.position, b.position, c.position, d.position);
    const float constraintValue = volume - constraint.restSignedVolume;
    // The four volume gradients sum to zero, so rigid translation does not
    // create a corrective force. The opposite-vertex form for ga is useful
    // here because the signed-volume constraint must remain translation-safe.
    const Vector3 ga = Scale(Cross(Subtract(b.position, c.position), Subtract(d.position, c.position)), 1.0f / 6.0f);
    const Vector3 gb = Scale(Cross(Subtract(c.position, a.position), Subtract(d.position, a.position)), 1.0f / 6.0f);
    const Vector3 gc = Scale(Cross(Subtract(d.position, a.position), Subtract(b.position, a.position)), 1.0f / 6.0f);
    const Vector3 gd = Scale(Cross(Subtract(b.position, a.position), Subtract(c.position, a.position)), 1.0f / 6.0f);
    const float alpha = constraint.compliance / (dt * dt);
    const float denominator = a.inverseMass * Dot(ga, ga) + b.inverseMass * Dot(gb, gb) + c.inverseMass * Dot(gc, gc) + d.inverseMass * Dot(gd, gd) + alpha;
    if (denominator < 1.0e-10f) return;
    const float deltaLambda = (-constraintValue - alpha * constraint.lambda) / denominator;
    constraint.lambda += deltaLambda;
    a.position = Add(a.position, Scale(ga, a.inverseMass * deltaLambda));
    b.position = Add(b.position, Scale(gb, b.inverseMass * deltaLambda));
    c.position = Add(c.position, Scale(gc, c.inverseMass * deltaLambda));
    d.position = Add(d.position, Scale(gd, d.inverseMass * deltaLambda));
}

void ClothElasticSolidsScene::SolveCollisions() {
    for (Node& node : nodes_) {
        if (node.inverseMass <= 0.0f) continue;
        if (node.position.y < 0.035f) node.position.y = 0.035f;
    }
    for (const Cloth& cloth : cloths_) {
        if (cloth.collider != Collider::Sphere) continue;
        for (int index : cloth.nodes) {
            Node& node = nodes_[static_cast<std::size_t>(index)];
            const Vector3 offset = Subtract(node.position, kSphereCenter);
            const float distance = Length(offset);
            const float contactRadius = kSphereRadius + 0.035f;
            if (distance > 1.0e-6f && distance < contactRadius) node.position = Add(kSphereCenter, Scale(offset, contactRadius / distance));
        }
    }
}

void ClothElasticSolidsScene::Step(float dt) {
    const float clampedDt = std::clamp(dt, 0.0f, 0.05f);
    if (clampedDt <= 0.0f) return;
    const int substeps = 2;
    const float h = clampedDt / static_cast<float>(substeps);
    for (int substep = 0; substep < substeps; ++substep) {
        ResetConstraintMultipliers();
        Integrate(h);
        for (int iteration = 0; iteration < kSolverIterations; ++iteration) {
            for (Cloth& cloth : cloths_) for (DistanceConstraint& constraint : cloth.constraints) SolveDistance(constraint, h);
            if (jellyPtr_ != nullptr) {
                for (DistanceConstraint& constraint : jellyPtr_->constraints) SolveDistance(constraint, h);
                for (VolumeConstraint& constraint : jellyPtr_->volumes) SolveVolume(constraint, h);
            }
            SolveCollisions();
        }
        for (Node& node : nodes_) {
            if (node.inverseMass <= 0.0f) { node.velocity = {}; continue; }
            node.velocity = Scale(Subtract(node.position, node.previous), 1.0f / h);
            if (node.position.y <= 0.036f && node.velocity.y < 0.0f) node.velocity.y = 0.0f;
        }
        simulatedTime_ += h;
    }
    UpdateDiagnostics();
}

void ClothElasticSolidsScene::UpdateDiagnostics() {
    maxStretch_ = 0.0f;
    auto inspect = [this](const std::vector<DistanceConstraint>& constraints) {
        for (const DistanceConstraint& constraint : constraints) {
            const float rest = std::max(1.0e-6f, constraint.restLength);
            const float length = Length(Subtract(nodes_[static_cast<std::size_t>(constraint.b)].position, nodes_[static_cast<std::size_t>(constraint.a)].position));
            maxStretch_ = std::max(maxStretch_, std::abs(length / rest - 1.0f));
        }
    };
    for (const Cloth& cloth : cloths_) inspect(cloth.constraints);
    if (jellyPtr_ != nullptr) {
        inspect(jellyPtr_->constraints);
        float volume = 0.0f;
        for (const VolumeConstraint& constraint : jellyPtr_->volumes) {
            const auto& ids = constraint.nodes;
            volume += std::abs(SignedVolume(nodes_[ids[0]].position, nodes_[ids[1]].position, nodes_[ids[2]].position, nodes_[ids[3]].position));
        }
        jellyVolumeRatio_ = jellyPtr_->restVolume > 1.0e-8f ? volume / jellyPtr_->restVolume : 1.0f;
    }
}

Vector3 ClothElasticSolidsScene::MousePointOnRay(const Ray& ray, float depth) const { return Add(ray.position, Scale(ray.direction, depth)); }

int ClothElasticSolidsScene::PickNode(const Ray& ray, float& depth) const {
    int picked = -1;
    float bestDistance = 0.20f;
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
        if (nodes_[i].inverseMass <= 0.0f) continue;
        const Vector3 toNode = Subtract(nodes_[i].position, ray.position);
        const float along = Dot(toNode, ray.direction);
        if (along <= 0.0f) continue;
        const Vector3 closest = MousePointOnRay(ray, along);
        const float distance = Length(Subtract(nodes_[i].position, closest));
        if (distance < bestDistance) { bestDistance = distance; picked = static_cast<int>(i); depth = along; }
    }
    return picked;
}

void ClothElasticSolidsScene::HandleInput(const Camera3D& camera, bool allowWorldInteraction) {
    if (IsKeyPressed(KEY_R)) Reset();
    if (IsKeyPressed(KEY_W)) ToggleWind();
    if (IsKeyPressed(KEY_F)) ToggleWireframe();
    if (IsKeyPressed(KEY_LEFT_BRACKET)) AdjustWind(-0.5f);
    if (IsKeyPressed(KEY_RIGHT_BRACKET)) AdjustWind(0.5f);

    if (PhysicsMouseButtonPressed(MOUSE_BUTTON_LEFT) && HandleStationClick(PhysicsMousePosition())) return;
    if (!allowWorldInteraction) return;

    const Ray ray = GetMouseRay(PhysicsMousePosition(), camera);
    if (PhysicsMouseButtonPressed(MOUSE_BUTTON_LEFT) && !dragging_) {
        draggedNode_ = PickNode(ray, dragDepth_);
        if (draggedNode_ >= 0) {
            dragging_ = true;
            draggedInverseMass_ = nodes_[static_cast<std::size_t>(draggedNode_)].inverseMass;
            nodes_[static_cast<std::size_t>(draggedNode_)].inverseMass = 0.0f;
        }
    }
    if (dragging_ && PhysicsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        Node& node = nodes_[static_cast<std::size_t>(draggedNode_)];
        node.position = MousePointOnRay(ray, dragDepth_);
        node.velocity = {};
    }
    if (dragging_ && PhysicsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
        Node& node = nodes_[static_cast<std::size_t>(draggedNode_)];
        node.inverseMass = draggedInverseMass_;
        node.pinned = draggedInverseMass_ <= 0.0f;
        dragging_ = false;
        draggedNode_ = -1;
    }
}

void ClothElasticSolidsScene::ToggleWind() { windEnabled_ = !windEnabled_; }
void ClothElasticSolidsScene::AdjustWind(float delta) { windStrength_ = std::clamp(windStrength_ + delta, 0.0f, 12.0f); }
void ClothElasticSolidsScene::ToggleWireframe() { wireframe_ = !wireframe_; }

void ClothElasticSolidsScene::DrawCloth(const Cloth& cloth) const {
    const auto index = [columns = cloth.columns](int x, int y) { return x + columns * y; };
    for (int y = 0; y + 1 < cloth.rows; ++y) {
        for (int x = 0; x + 1 < cloth.columns; ++x) {
            const int ia = cloth.nodes[index(x, y)];
            const int ib = cloth.nodes[index(x + 1, y)];
            const int ic = cloth.nodes[index(x, y + 1)];
            const int id = cloth.nodes[index(x + 1, y + 1)];
            const Vector3 a = nodes_[static_cast<std::size_t>(ia)].position;
            const Vector3 b = nodes_[static_cast<std::size_t>(ib)].position;
            const Vector3 c = nodes_[static_cast<std::size_t>(ic)].position;
            const Vector3 d = nodes_[static_cast<std::size_t>(id)].position;
            const float stretch = std::max({
                std::abs(Length(Subtract(b, a)) / std::max(1.0e-5f, Length(Subtract(nodes_[static_cast<std::size_t>(ib)].rest, nodes_[static_cast<std::size_t>(ia)].rest))) - 1.0f),
                std::abs(Length(Subtract(c, a)) / std::max(1.0e-5f, Length(Subtract(nodes_[static_cast<std::size_t>(ic)].rest, nodes_[static_cast<std::size_t>(ia)].rest))) - 1.0f),
                std::abs(Length(Subtract(d, b)) / std::max(1.0e-5f, Length(Subtract(nodes_[static_cast<std::size_t>(id)].rest, nodes_[static_cast<std::size_t>(ib)].rest))) - 1.0f)
            });
            const Color color = StrainColor(stretch, cloth.color);
            DrawDoubleSidedTriangle(a, b, c, color);
            DrawDoubleSidedTriangle(b, d, c, color);
            if (wireframe_) {
                DrawLine3D(a, b, Fade(WHITE, 0.55f));
                DrawLine3D(a, c, Fade(WHITE, 0.55f));
                DrawLine3D(b, d, Fade(WHITE, 0.55f));
                DrawLine3D(c, d, Fade(WHITE, 0.55f));
            }
        }
    }
    for (int y = 0; y < cloth.rows; ++y) for (int x = 0; x < cloth.columns; ++x) {
        const Node& node = nodes_[static_cast<std::size_t>(cloth.nodes[index(x, y)])];
        if (node.pinned) DrawSphere(node.position, 0.055f, Color{126, 235, 167, 255});
    }
}

void ClothElasticSolidsScene::DrawJelly() const {
    if (jellyPtr_ == nullptr) return;
    const auto p = [this](int index) { return nodes_[static_cast<std::size_t>(jellyPtr_->corners[static_cast<std::size_t>(index)])].position; };
    const Color fill = jellyVolumeRatio_ > 0.85f ? Color{83, 210, 177, 100} : Color{238, 138, 74, 120};
    const std::array<std::array<int, 3>, 12> triangles = {{
        {{0, 1, 2}}, {{0, 2, 3}}, {{4, 6, 5}}, {{4, 7, 6}}, {{0, 4, 5}}, {{0, 5, 1}},
        {{3, 2, 6}}, {{3, 6, 7}}, {{0, 3, 7}}, {{0, 7, 4}}, {{1, 5, 6}}, {{1, 6, 2}}
    }};
    for (const auto& triangle : triangles) DrawDoubleSidedTriangle(p(triangle[0]), p(triangle[1]), p(triangle[2]), fill);
    const std::array<Edge, 12> edges = {{{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}}};
    for (const Edge edge : edges) DrawLine3D(p(edge.a), p(edge.b), Color{226, 250, 239, 255});
    for (int i = 0; i < 8; ++i) DrawSphere(p(i), 0.07f, Color{126, 235, 167, 255});
}

void ClothElasticSolidsScene::DrawStationMarkers() const {
    DrawCylinder({-5.5f, 2.8f, -1.7f}, 0.035f, 0.035f, 5.6f, 8, Color{153, 166, 188, 255});
    DrawSphere(kSphereCenter, kSphereRadius, Color{76, 90, 116, 255});
    DrawSphereWires(kSphereCenter, kSphereRadius, 18, 12, Color{183, 203, 232, 255});
    DrawCylinder({4.85f, 0.05f, 1.25f}, 0.82f, 0.82f, 0.10f, 24, Color{47, 59, 79, 255});
}

void ClothElasticSolidsScene::Draw3D(const Camera3D& camera) const {
    (void)camera;
    // Cloth is intentionally double-sided: folds and the back of the hanging
    // surfaces must remain visible from the orbit camera.
    rlDisableBackfaceCulling();
    DrawPlane({0.0f, 0.0f, 0.0f}, {15.0f, 10.0f}, Color{18, 26, 40, 255});
    for (int i = -7; i <= 7; ++i) {
        DrawLine3D({static_cast<float>(i), 0.012f, -5.0f}, {static_cast<float>(i), 0.012f, 5.0f}, Color{40, 52, 71, 255});
        DrawLine3D({-7.0f, 0.012f, static_cast<float>(i) * 0.72f}, {7.0f, 0.012f, static_cast<float>(i) * 0.72f}, Color{40, 52, 71, 255});
    }
    DrawStationMarkers();
    for (const Cloth& cloth : cloths_) DrawCloth(cloth);
    DrawJelly();
    rlEnableBackfaceCulling();
}

void ClothElasticSolidsScene::Draw2D() const {
    const int width = GetScreenWidth();
    const int height = GetScreenHeight();
    DrawRectangle(0, 0, width, height, Color{10, 15, 27, 255});
    DrawText("PLANAR INSPECTION / X-Y PROJECTION", 34, 32, 22, Color{232, 238, 248, 255});
    const float scale = std::min(width * 0.78f / 14.0f, height * 0.78f / 7.0f);
    const Vector2 center = {width * 0.50f, height * 0.78f};
    auto project = [scale, center](Vector3 p) { return Vector2{center.x + p.x * scale, center.y - p.y * scale}; };
    for (int i = -7; i <= 7; ++i) DrawLine(static_cast<int>(center.x + i * scale), 85, static_cast<int>(center.x + i * scale), static_cast<int>(center.y), Color{40, 52, 71, 255});
    for (int i = 0; i <= 7; ++i) DrawLine(30, static_cast<int>(center.y - i * scale), width - 30, static_cast<int>(center.y - i * scale), Color{40, 52, 71, 255});
    for (const Cloth& cloth : cloths_) {
        const auto index = [columns = cloth.columns](int x, int y) { return x + columns * y; };
        for (int y = 0; y + 1 < cloth.rows; ++y) for (int x = 0; x + 1 < cloth.columns; ++x) {
            const Vector2 a = project(nodes_[static_cast<std::size_t>(cloth.nodes[index(x, y)])].position);
            const Vector2 b = project(nodes_[static_cast<std::size_t>(cloth.nodes[index(x + 1, y)])].position);
            const Vector2 c = project(nodes_[static_cast<std::size_t>(cloth.nodes[index(x + 1, y + 1)])].position);
            const Vector2 d = project(nodes_[static_cast<std::size_t>(cloth.nodes[index(x, y + 1)])].position);
            DrawTriangle(a, b, c, Fade(cloth.color, 0.72f));
            DrawTriangle(a, c, d, Fade(cloth.color, 0.72f));
            if (wireframe_) { DrawLineV(a, b, WHITE); DrawLineV(a, d, WHITE); }
        }
    }
    if (jellyPtr_ != nullptr) {
        const auto projectJelly = [this, &project](int i) { return project(nodes_[static_cast<std::size_t>(jellyPtr_->corners[static_cast<std::size_t>(i)])].position); };
        const std::array<Edge, 12> edges = {{{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}}};
        for (const Edge edge : edges) DrawLineV(projectJelly(edge.a), projectJelly(edge.b), Color{126, 235, 167, 255});
    }
    DrawLine(30, static_cast<int>(center.y), width - 30, static_cast<int>(center.y), Color{220, 91, 99, 180});
}

void ClothElasticSolidsScene::DrawHud(bool paused) const {
    const int width = GetScreenWidth();
    DrawRectangle(20, 20, 610, 210, Color{15, 21, 34, 232});
    DrawRectangleLines(20, 20, 610, 210, Color{76, 91, 117, 255});
    DrawText("CLOTH + ELASTIC SOLIDS LAB", 38, 35, 22, Color{232, 238, 248, 255});
    DrawText("FIRST DELIVERABLE / XPBD REFERENCE MODELS", 38, 68, 14, Color{120, 213, 255, 255});
    DrawText(TextFormat("particles %4d   constraints %4d   t %6.2f s", static_cast<int>(ParticleCount()), static_cast<int>(ConstraintCount()), simulatedTime_), 38, 98, 15, Color{196, 204, 219, 255});
    DrawText(TextFormat("max strain %6.3f   jelly volume %5.3f", maxStretch_, jellyVolumeRatio_), 38, 124, 15, maxStretch_ > 0.15f ? Color{255, 188, 92, 255} : Color{126, 235, 167, 255});
    DrawText(TextFormat("wind %s   strength %4.1f m/s   %s", windEnabled_ ? "ON " : "OFF", windStrength_, paused ? "PAUSED" : "RUNNING"), 38, 150, 15, Color{196, 204, 219, 255});
    DrawText("intro model: distance XPBD + tetra volume; not calibrated FEM", 38, 178, 13, Color{171, 183, 201, 255});
    DrawText("R reset  P pause  N step  V 2D/3D  F wire  W wind  [ ] wind", 38, 202, 13, Color{147, 160, 182, 255});

    const int legendX = width - 285;
    DrawRectangle( width - 305, 20, 285, 215, Color{15, 21, 34, 220});
    DrawRectangleLines(width - 305, 20, 285, 215, Color{76, 91, 117, 255});
    DrawText("STATIONS / CLICK TO FOCUS", legendX, 37, 17, Color{232, 238, 248, 255});
    const std::array<const char*, 4> stationNames = {{"flag / curtain / hammock", "soft vs stiff fabric", "cloth over sphere", "tetrahedral jelly cube"}};
    const std::array<Color, 4> stationColors = {{{126, 183, 255, 255}, {242, 187, 75, 255}, {224, 117, 69, 255}, {83, 210, 177, 255}}};
    for (int station = 0; station < 4; ++station) {
        const Rectangle card = StationCardRectangle(station);
        const Rectangle reset = StationResetRectangle(station);
        const bool selected = selectedStation_ == station;
        DrawRectangleRec(card, selected ? Color{28, 42, 62, 255} : Color{15, 21, 34, 255});
        DrawRectangleLinesEx(card, selected ? 2.0f : 1.0f, selected ? stationColors[static_cast<std::size_t>(station)] : Color{52, 61, 77, 255});
        DrawText(stationNames[static_cast<std::size_t>(station)], static_cast<int>(card.x + 8), static_cast<int>(card.y + 5), 13, stationColors[static_cast<std::size_t>(station)]);
        DrawRectangleRec(reset, Color{34, 45, 61, 255});
        DrawText("RESET", static_cast<int>(reset.x + 7), static_cast<int>(reset.y + 4), 11, Color{196, 204, 219, 255});
    }
    DrawText(TextFormat("focused: %s", SelectedStationName()), legendX, 187, 13, Color{171, 183, 201, 255});
    DrawText("click RESET for local state", legendX, 207, 13, Color{171, 183, 201, 255});
}
