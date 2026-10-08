#include "raylib.h"
#include "vibrating_string.h"
#include "flip_fluid.h"
#include "sph_fluid.h"
#include "powder_scene.h"
#include "buoyancy_scene.h"
#include "cloth_elastic_solids_scene.h"
#include "beam_bending_scene.h"
#include "bridge_builder_scene.h"
#include "web_mouse.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>

EM_JS(void, InstallPhysicsMouseBridge, (), {
    const canvas = document.querySelector('canvas');
    if (!canvas || canvas.__physicsMouseBridgeInstalled) return;
    canvas.__physicsMouseBridgeInstalled = true;
    const state = Module.physicsMouse = {x: 0, y: 0, buttons: 0, pressed: 0, released: 0, wheel: 0, deltaX: 0, deltaY: 0};
    const buttonBit = (button) => 1 << (button === 2 ? 1 : (button === 1 ? 2 : 0));
    const updatePosition = (event) => {
        const bounds = canvas.getBoundingClientRect();
        const nextX = (event.clientX - bounds.left) * canvas.width / Math.max(1, bounds.width);
        const nextY = (event.clientY - bounds.top) * canvas.height / Math.max(1, bounds.height);
        state.deltaX += nextX - state.x;
        state.deltaY += nextY - state.y;
        state.x = nextX;
        state.y = nextY;
    };
    canvas.addEventListener('pointermove', (event) => { updatePosition(event); event.preventDefault(); }, {passive: false});
    canvas.addEventListener('pointerdown', (event) => {
        updatePosition(event);
        const bit = buttonBit(event.button);
        state.buttons |= bit;
        state.pressed |= bit;
        if (canvas.setPointerCapture) canvas.setPointerCapture(event.pointerId);
        event.preventDefault();
    }, {passive: false});
    canvas.addEventListener('pointerup', (event) => {
        updatePosition(event);
        const bit = buttonBit(event.button);
        state.buttons &= ~bit;
        state.released |= bit;
        if (canvas.releasePointerCapture) canvas.releasePointerCapture(event.pointerId);
        event.preventDefault();
    }, {passive: false});
    canvas.addEventListener('wheel', (event) => { state.wheel += event.deltaY < 0 ? 1 : -1; event.preventDefault(); }, {passive: false});
    canvas.addEventListener('contextmenu', (event) => event.preventDefault());
});

EM_JS(int, PhysicsWebMousePressed, (int button), {
    const state = Module.physicsMouse;
    if (!state) return 0;
    const bit = 1 << button;
    if ((state.pressed & bit) === 0) return 0;
    state.pressed &= ~bit;
    return 1;
});
EM_JS(int, PhysicsWebMouseDown, (int button), { const state = Module.physicsMouse; return state && (state.buttons & (1 << button)) ? 1 : 0; });
EM_JS(int, PhysicsWebMouseReleased, (int button), {
    const state = Module.physicsMouse;
    if (!state) return 0;
    const bit = 1 << button;
    if ((state.released & bit) === 0) return 0;
    state.released &= ~bit;
    return 1;
});
EM_JS(float, PhysicsWebMouseX, (), { return Module.physicsMouse ? Module.physicsMouse.x : 0; });
EM_JS(float, PhysicsWebMouseY, (), { return Module.physicsMouse ? Module.physicsMouse.y : 0; });
EM_JS(float, PhysicsWebMouseWheel, (), {
    const state = Module.physicsMouse;
    if (!state) return 0;
    const wheel = state.wheel;
    state.wheel = 0;
    return wheel;
});
EM_JS(float, PhysicsWebMouseDeltaX, (), { return Module.physicsMouse ? Module.physicsMouse.deltaX : 0; });
EM_JS(float, PhysicsWebMouseDeltaY, (), {
    const state = Module.physicsMouse;
    if (!state) return 0;
    const delta = state.deltaY;
    state.deltaX = 0;
    state.deltaY = 0;
    return delta;
});

Vector2 PhysicsMousePosition() { return {PhysicsWebMouseX(), PhysicsWebMouseY()}; }
bool PhysicsMouseButtonPressed(int button) { return PhysicsWebMousePressed(button) != 0; }
bool PhysicsMouseButtonDown(int button) { return PhysicsWebMouseDown(button) != 0; }
bool PhysicsMouseButtonReleased(int button) { return PhysicsWebMouseReleased(button) != 0; }
Vector2 PhysicsMouseDelta() { return {PhysicsWebMouseDeltaX(), PhysicsWebMouseDeltaY()}; }
float PhysicsMouseWheelMove() { return PhysicsWebMouseWheel(); }
#endif

namespace {

constexpr float kGridStep = 0.5f;
constexpr float kRestLength = 3.0f;
constexpr float kGravity = -9.81f;
constexpr float kFixedTimeStep = 1.0f / 120.0f;

enum class SimulationKind { SpringMass, BouncingBall, NewtonCooling, VibratingString, FlipFluid, SphFluid, Powder, Buoyancy, ClothElasticSolids, BeamBending, BridgeBuilder };
enum class ViewMode { TwoD, ThreeD, Spectrum, Split };

struct SpringMass {
    Vector3 position{0.0f, 2.8f, 0.0f};
    Vector3 velocity{0.0f, 0.0f, 0.0f};
    Vector3 anchor{0.0f, 5.8f, 0.0f};
    float springConstant = 22.0f;
    float mass = 1.0f;
    float dampingCoefficient = 1.15f;
};

struct BouncingBall {
    Vector3 position{0.0f, 5.0f, 0.0f};
    Vector3 velocity{2.4f, 0.0f, 1.5f};
    float radius = 0.55f;
    float gravity = 9.81f;
    float restitution = 0.78f;
};

struct NewtonCooling {
    float temperature = 100.0f;
    float ambientTemperature = 20.0f;
    float coolingConstant = 0.18f;
    float elapsed = 0.0f;
};

struct OrbitCamera { float yaw = 0.72f; float pitch = 0.38f; float distance = 13.0f; };

Vector3 Add(Vector3 a, Vector3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vector3 Subtract(Vector3 a, Vector3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vector3 Scale(Vector3 value, float scalar) { return {value.x * scalar, value.y * scalar, value.z * scalar}; }
float Length(Vector3 value) { return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z); }
Vector3 Normalize(Vector3 value) { const float length = Length(value); return length > 0.0001f ? Scale(value, 1.0f / length) : Vector3{0.0f, 1.0f, 0.0f}; }
float Lerp(float a, float b, float amount) { return a + (b - a) * amount; }

struct SimulationMenuItem {
    SimulationKind kind;
    const char* title;
    const char* description;
    Color accent;
};

const SimulationMenuItem kSimulationMenu[] = {
    {SimulationKind::SpringMass, "SPRING / MASS", "Movable anchor, gravity, damping", Color{120, 213, 255, 255}},
    {SimulationKind::BouncingBall, "BOUNCING BALL", "Gravity, walls and restitution", Color{126, 183, 255, 255}},
    {SimulationKind::NewtonCooling, "NEWTON COOLING", "Temperature approaches ambient", Color{255, 188, 92, 255}},
    {SimulationKind::VibratingString, "VIBRATING STRING", "Wave motion, audio and DFT", Color{126, 235, 167, 255}},
    {SimulationKind::FlipFluid, "FLIP FLUID", "3D grid / PIC fluid solver", Color{42, 164, 239, 255}},
    {SimulationKind::SphFluid, "SPH FLUID", "3D particle fluid solver", Color{50, 196, 224, 255}},
    {SimulationKind::Powder, "POWDER", "Sand, liquids, gases and fire", Color{224, 185, 103, 255}},
    {SimulationKind::Buoyancy, "BUOYANCY", "Floating rigid body reference", Color{106, 211, 229, 255}},
    {SimulationKind::ClothElasticSolids, "CLOTH / ELASTIC SOLIDS", "XPBD cloth and tetrahedral jelly gallery", Color{213, 92, 161, 255}},
    {SimulationKind::BeamBending, "BEAM BENDING", "Euler-Bernoulli beam FEM gallery", Color{126, 235, 167, 255}},
    {SimulationKind::BridgeBuilder, "BRIDGE BUILDER", "2D truss design and 3D load test", Color{255, 188, 92, 255}}
};

int SimulationMenuIndex(SimulationKind simulation) {
    for (int i = 0; i < static_cast<int>(sizeof(kSimulationMenu) / sizeof(kSimulationMenu[0])); ++i) {
        if (kSimulationMenu[i].kind == simulation) return i;
    }
    return 0;
}

Color TemperatureColor(float temperature) {
    const float normalized = std::clamp((temperature + 20.0f) / 140.0f, 0.0f, 1.0f);
    const Color cold = {56, 126, 255, 255};
    const Color warm = {255, 214, 82, 255};
    const Color hot = {244, 75, 65, 255};
    const Color from = normalized < 0.5f ? cold : warm;
    const Color to = normalized < 0.5f ? warm : hot;
    const float local = normalized < 0.5f ? normalized * 2.0f : (normalized - 0.5f) * 2.0f;
    return {static_cast<unsigned char>(Lerp(from.r, to.r, local)), static_cast<unsigned char>(Lerp(from.g, to.g, local)), static_cast<unsigned char>(Lerp(from.b, to.b, local)), 255};
}

void ResetSpringMass(SpringMass& s) { s.anchor = {0.0f, 5.8f, 0.0f}; s.position = {0.0f, s.anchor.y - kRestLength, 0.0f}; s.velocity = {0.0f, 0.0f, 0.0f}; }
void ResetBall(BouncingBall& b) { b.position = {0.0f, 5.0f, 0.0f}; b.velocity = {2.4f, 0.0f, 1.5f}; }
void ResetCooling(NewtonCooling& c) { c.temperature = 100.0f; c.ambientTemperature = 20.0f; c.coolingConstant = 0.18f; c.elapsed = 0.0f; }

void StepSpringMass(SpringMass& s, float dt) {
    const Vector3 springVector = Subtract(s.position, s.anchor);
    const float extension = Length(springVector) - kRestLength;
    const Vector3 springForce = Scale(Normalize(springVector), -s.springConstant * extension);
    const Vector3 dampingForce = Scale(s.velocity, -s.dampingCoefficient);
    const Vector3 gravityForce = {0.0f, s.mass * kGravity, 0.0f};
    const Vector3 acceleration = Scale(Add(Add(springForce, dampingForce), gravityForce), 1.0f / s.mass);
    s.velocity = Add(s.velocity, Scale(acceleration, dt));
    s.position = Add(s.position, Scale(s.velocity, dt));
}

void StepBall(BouncingBall& b, float dt) {
    b.velocity.y -= b.gravity * dt;
    b.position = Add(b.position, Scale(b.velocity, dt));
    if (b.position.y < b.radius) { b.position.y = b.radius; b.velocity.y = std::abs(b.velocity.y) * b.restitution; }
    const float boundary = 6.5f - b.radius;
    if (std::abs(b.position.x) > boundary) { b.position.x = std::clamp(b.position.x, -boundary, boundary); b.velocity.x *= -b.restitution; }
    if (std::abs(b.position.z) > boundary) { b.position.z = std::clamp(b.position.z, -boundary, boundary); b.velocity.z *= -b.restitution; }
}

void StepCooling(NewtonCooling& c, float dt) {
    c.temperature += -c.coolingConstant * (c.temperature - c.ambientTemperature) * dt;
    c.elapsed += dt;
}

void MoveAnchor(SpringMass& s, Vector3 direction) {
    s.anchor = Add(s.anchor, Scale(direction, kGridStep));
    s.anchor.x = std::clamp(s.anchor.x, -5.0f, 5.0f);
    s.anchor.y = std::clamp(s.anchor.y, 1.0f, 9.0f);
    s.anchor.z = std::clamp(s.anchor.z, -5.0f, 5.0f);
}

void HandleCameraInput(OrbitCamera& c, ViewMode view) {
    if (view == ViewMode::ThreeD && PhysicsMouseButtonDown(MOUSE_BUTTON_RIGHT)) { const Vector2 d = PhysicsMouseDelta(); c.yaw -= d.x * 0.006f; c.pitch = std::clamp(c.pitch - d.y * 0.006f, -1.2f, 1.2f); }
    c.distance = std::clamp(c.distance - PhysicsMouseWheelMove() * 0.8f, 5.0f, 26.0f);
}

Camera3D MakeCamera(const OrbitCamera& orbit, Vector3 target);

void HandleInput(SpringMass& spring, BouncingBall& ball, NewtonCooling& cooling, VibratingStringEngine& string, FlipFluid& fluid, SphFluid& sph, PowderScene& powder, BuoyancyScene& buoyancy, ClothElasticSolidsScene& cloth, BeamBendingScene& beam, BridgeBuilderScene& bridge,
                 SimulationKind& simulation, ViewMode& view, bool& paused, bool& muted, OrbitCamera& camera, bool& flipFrontView) {
    const bool bridgeHotkeySelection = IsKeyPressed(KEY_B) && simulation != SimulationKind::FlipFluid && simulation != SimulationKind::BridgeBuilder;
    if (IsKeyPressed(KEY_ONE)) simulation = SimulationKind::SpringMass;
    if (IsKeyPressed(KEY_TWO)) simulation = SimulationKind::BouncingBall;
    if (IsKeyPressed(KEY_THREE)) simulation = SimulationKind::NewtonCooling;
    if (IsKeyPressed(KEY_FOUR)) simulation = SimulationKind::VibratingString;
    if (IsKeyPressed(KEY_FIVE)) simulation = SimulationKind::FlipFluid;
    if (IsKeyPressed(KEY_SIX)) simulation = SimulationKind::SphFluid;
    if (IsKeyPressed(KEY_SEVEN)) simulation = SimulationKind::Powder;
    if (IsKeyPressed(KEY_EIGHT)) simulation = SimulationKind::Buoyancy;
    if (IsKeyPressed(KEY_NINE)) simulation = SimulationKind::ClothElasticSolids;
    if (IsKeyPressed(KEY_ZERO)) simulation = SimulationKind::BeamBending;
    if (bridgeHotkeySelection) simulation = SimulationKind::BridgeBuilder;
    if (IsKeyPressed(KEY_TAB)) {
        if (simulation == SimulationKind::SpringMass) simulation = SimulationKind::BouncingBall;
        else if (simulation == SimulationKind::BouncingBall) simulation = SimulationKind::NewtonCooling;
        else if (simulation == SimulationKind::NewtonCooling) simulation = SimulationKind::VibratingString;
        else if (simulation == SimulationKind::VibratingString) simulation = SimulationKind::FlipFluid;
        else if (simulation == SimulationKind::FlipFluid) simulation = SimulationKind::SphFluid;
        else if (simulation == SimulationKind::SphFluid) simulation = SimulationKind::Powder;
        else if (simulation == SimulationKind::Powder) simulation = SimulationKind::Buoyancy;
        else if (simulation == SimulationKind::Buoyancy) simulation = SimulationKind::ClothElasticSolids;
        else if (simulation == SimulationKind::ClothElasticSolids) simulation = SimulationKind::BeamBending;
        else if (simulation == SimulationKind::BeamBending) simulation = SimulationKind::BridgeBuilder;
        else simulation = SimulationKind::SpringMass;
    }
    if (IsKeyPressed(KEY_V)) {
        if (simulation == SimulationKind::VibratingString) {
            if (view == ViewMode::TwoD) view = ViewMode::ThreeD;
            else if (view == ViewMode::ThreeD) view = ViewMode::Spectrum;
            else if (view == ViewMode::Spectrum) view = ViewMode::Split;
            else view = ViewMode::TwoD;
        } else {
            view = view == ViewMode::ThreeD ? ViewMode::TwoD : ViewMode::ThreeD;
        }
    }
    if (IsKeyPressed(KEY_P)) { paused = !paused; string.SetPaused(paused); }

    if (simulation == SimulationKind::SpringMass) {
        if (IsKeyPressed(KEY_W)) MoveAnchor(spring, {0.0f, 0.0f, -1.0f});
        if (IsKeyPressed(KEY_S)) MoveAnchor(spring, {0.0f, 0.0f, 1.0f});
        if (IsKeyPressed(KEY_A)) MoveAnchor(spring, {-1.0f, 0.0f, 0.0f});
        if (IsKeyPressed(KEY_D)) MoveAnchor(spring, {1.0f, 0.0f, 0.0f});
        if (IsKeyPressed(KEY_SPACE)) MoveAnchor(spring, {0.0f, 1.0f, 0.0f});
        if (IsKeyPressed(KEY_LEFT_CONTROL)) MoveAnchor(spring, {0.0f, -1.0f, 0.0f});
        if (IsKeyPressed(KEY_LEFT_BRACKET)) spring.springConstant = std::max(1.0f, spring.springConstant - 1.0f);
        if (IsKeyPressed(KEY_RIGHT_BRACKET)) spring.springConstant = std::min(80.0f, spring.springConstant + 1.0f);
        if (IsKeyPressed(KEY_MINUS)) spring.mass = std::max(0.25f, spring.mass - 0.25f);
        if (IsKeyPressed(KEY_EQUAL)) spring.mass = std::min(8.0f, spring.mass + 0.25f);
        if (IsKeyPressed(KEY_COMMA)) spring.dampingCoefficient = std::max(0.05f, spring.dampingCoefficient - 0.05f);
        if (IsKeyPressed(KEY_PERIOD)) spring.dampingCoefficient = std::min(8.0f, spring.dampingCoefficient + 0.05f);
        if (IsKeyPressed(KEY_BACKSPACE)) { spring.springConstant = 22.0f; spring.mass = 1.0f; spring.dampingCoefficient = 1.15f; }
        if (IsKeyPressed(KEY_R)) ResetSpringMass(spring);
    } else if (simulation == SimulationKind::BouncingBall) {
        if (IsKeyPressed(KEY_LEFT_BRACKET)) ball.gravity = std::max(0.0f, ball.gravity - 0.5f);
        if (IsKeyPressed(KEY_RIGHT_BRACKET)) ball.gravity = std::min(30.0f, ball.gravity + 0.5f);
        if (IsKeyPressed(KEY_MINUS)) ball.radius = std::max(0.2f, ball.radius - 0.05f);
        if (IsKeyPressed(KEY_EQUAL)) ball.radius = std::min(1.5f, ball.radius + 0.05f);
        if (IsKeyPressed(KEY_BACKSPACE)) { ball.gravity = 9.81f; ball.radius = 0.55f; ball.restitution = 0.78f; }
        if (IsKeyPressed(KEY_R)) ResetBall(ball);
    } else if (simulation == SimulationKind::NewtonCooling) {
        if (IsKeyPressed(KEY_LEFT_BRACKET)) cooling.coolingConstant = std::max(0.01f, cooling.coolingConstant - 0.02f);
        if (IsKeyPressed(KEY_RIGHT_BRACKET)) cooling.coolingConstant = std::min(2.0f, cooling.coolingConstant + 0.02f);
        if (IsKeyPressed(KEY_MINUS)) cooling.ambientTemperature = std::max(-20.0f, cooling.ambientTemperature - 5.0f);
        if (IsKeyPressed(KEY_EQUAL)) cooling.ambientTemperature = std::min(100.0f, cooling.ambientTemperature + 5.0f);
        if (IsKeyPressed(KEY_UP)) cooling.temperature = std::min(200.0f, cooling.temperature + 5.0f);
        if (IsKeyPressed(KEY_DOWN)) cooling.temperature = std::max(-20.0f, cooling.temperature - 5.0f);
        if (IsKeyPressed(KEY_BACKSPACE)) { cooling.ambientTemperature = 20.0f; cooling.coolingConstant = 0.18f; }
        if (IsKeyPressed(KEY_R)) ResetCooling(cooling);
    } else if (simulation == SimulationKind::VibratingString) {
        if (IsKeyPressed(KEY_SPACE)) string.RequestPluck();
        if (IsKeyPressed(KEY_T)) string.RequestHotStart();
        if (IsKeyPressed(KEY_M)) { muted = !muted; string.SetMuted(muted); }
        if (IsKeyPressed(KEY_LEFT_BRACKET)) string.AdjustTension(-5.0f);
        if (IsKeyPressed(KEY_RIGHT_BRACKET)) string.AdjustTension(5.0f);
        if (IsKeyPressed(KEY_MINUS)) string.AdjustDamping(-0.05f);
        if (IsKeyPressed(KEY_EQUAL)) string.AdjustDamping(0.05f);
        if (IsKeyPressed(KEY_COMMA)) string.AdjustPluckAmplitude(-0.001f);
        if (IsKeyPressed(KEY_PERIOD)) string.AdjustPluckAmplitude(0.001f);
        if (IsKeyPressed(KEY_H)) string.AdjustHeatingFraction(-0.05f);
        if (IsKeyPressed(KEY_J)) string.AdjustHeatingFraction(0.05f);
        if (IsKeyPressed(KEY_K)) string.AdjustHeatLoss(-0.005f);
        if (IsKeyPressed(KEY_L)) string.AdjustHeatLoss(0.005f);
        if (IsKeyPressed(KEY_Q)) string.AdjustPickupPosition(-0.05f);
        if (IsKeyPressed(KEY_E)) string.AdjustPickupPosition(0.05f);
        if (IsKeyPressed(KEY_BACKSPACE)) string.RequestReset();
        if (IsKeyPressed(KEY_R)) string.RequestReset();
    } else if (simulation == SimulationKind::FlipFluid) {
        if (IsKeyPressed(KEY_LEFT_BRACKET)) fluid.AdjustFlipRatio(-0.1f);
        if (IsKeyPressed(KEY_RIGHT_BRACKET)) fluid.AdjustFlipRatio(0.1f);
        if (IsKeyPressed(KEY_MINUS)) fluid.AdjustGravity(-0.5f);
        if (IsKeyPressed(KEY_EQUAL)) fluid.AdjustGravity(0.5f);
        if (IsKeyPressed(KEY_B)) fluid.ToggleSolidObstacle();
        if (IsKeyPressed(KEY_F)) flipFrontView = !flipFrontView;
        FlipVec3 obstacleVelocity{};
        if (IsKeyDown(KEY_J)) obstacleVelocity.x -= 3.0;
        if (IsKeyDown(KEY_L)) obstacleVelocity.x += 3.0;
        if (IsKeyDown(KEY_I)) obstacleVelocity.z -= 3.0;
        if (IsKeyDown(KEY_K)) obstacleVelocity.z += 3.0;
        if (IsKeyDown(KEY_U)) obstacleVelocity.y += 3.0;
        if (IsKeyDown(KEY_O)) obstacleVelocity.y -= 3.0;
        fluid.MoveSolidObstacle(obstacleVelocity, GetFrameTime());
        if (IsKeyPressed(KEY_SPACE)) fluid.Poke({FlipFluid::kWorldWidth * 0.5, 2.0, FlipFluid::kWorldDepth * 0.5}, {0.0, 4.0, 0.0});
        if (IsKeyPressed(KEY_N)) fluid.AddParticleLayer();
        if (IsKeyPressed(KEY_BACKSPACE) || IsKeyPressed(KEY_R)) fluid.Reset();
    } else if (simulation == SimulationKind::SphFluid) {
        if (IsKeyPressed(KEY_LEFT_BRACKET)) sph.AdjustStiffness(-250.0f);
        if (IsKeyPressed(KEY_RIGHT_BRACKET)) sph.AdjustStiffness(250.0f);
        if (IsKeyPressed(KEY_MINUS)) sph.AdjustViscosity(-0.01f);
        if (IsKeyPressed(KEY_EQUAL)) sph.AdjustViscosity(0.01f);
        if (IsKeyPressed(KEY_COMMA)) sph.AdjustGravity(-0.5f);
        if (IsKeyPressed(KEY_PERIOD)) sph.AdjustGravity(0.5f);
        if (IsKeyPressed(KEY_SPACE)) sph.Poke({SphFluid::kWorldWidth * 0.5, 2.0, SphFluid::kWorldDepth * 0.5}, {0.0, 4.0, 0.0});
        if (IsKeyPressed(KEY_N)) sph.AddParticleLayer();
        if (IsKeyPressed(KEY_BACKSPACE) || IsKeyPressed(KEY_R)) sph.Reset();
    } else if (simulation == SimulationKind::Powder) {
        if (IsKeyPressed(KEY_LEFT_BRACKET)) powder.CycleMaterial(-1);
        if (IsKeyPressed(KEY_RIGHT_BRACKET)) powder.CycleMaterial(1);
        if (IsKeyPressed(KEY_Q)) powder.SetSelectedMaterial(PowderScene::Material::Sand);
        if (IsKeyPressed(KEY_W)) powder.SetSelectedMaterial(PowderScene::Material::Water);
        if (IsKeyPressed(KEY_E)) powder.SetSelectedMaterial(PowderScene::Material::Stone);
        if (IsKeyPressed(KEY_T)) powder.SetSelectedMaterial(PowderScene::Material::Wood);
        if (IsKeyPressed(KEY_Y)) powder.SetSelectedMaterial(PowderScene::Material::Fire);
        if (IsKeyPressed(KEY_U)) powder.SetSelectedMaterial(PowderScene::Material::Smoke);
        if (IsKeyPressed(KEY_I)) powder.SetSelectedMaterial(PowderScene::Material::Oil);
        if (IsKeyPressed(KEY_O)) powder.SetSelectedMaterial(PowderScene::Material::Acid);
        if (IsKeyPressed(KEY_BACKSPACE) || IsKeyPressed(KEY_R)) powder.Reset();
        if (IsKeyPressed(KEY_N)) powder.AddVolumeSource();
        if (IsKeyPressed(KEY_SPACE)) {
            if (powder.IsThreeDimensional()) powder.AddVolumeSource();
            else powder.AddImpulse(PowderScene::kGridWidth / 2, 40, 18);
        }
    } else if (simulation == SimulationKind::ClothElasticSolids) {
        cloth.HandleInput(MakeCamera(camera, cloth.FocusTarget()), view == ViewMode::ThreeD);
    } else if (simulation == SimulationKind::BeamBending) {
        beam.HandleInput(MakeCamera(camera, beam.FocusTarget()), view == ViewMode::ThreeD);
    } else if (simulation == SimulationKind::BridgeBuilder) {
        if (!bridgeHotkeySelection) bridge.HandleInput(MakeCamera(camera, bridge.FocusTarget()), view == ViewMode::ThreeD);
    } else {
        if (IsKeyPressed(KEY_LEFT_BRACKET)) buoyancy.AdjustWaterDensity(-50.0);
        if (IsKeyPressed(KEY_RIGHT_BRACKET)) buoyancy.AdjustWaterDensity(50.0);
        if (IsKeyPressed(KEY_MINUS)) buoyancy.AdjustMass(-50.0);
        if (IsKeyPressed(KEY_EQUAL)) buoyancy.AdjustMass(50.0);
        if (IsKeyPressed(KEY_COMMA)) buoyancy.AdjustDrag(-40.0);
        if (IsKeyPressed(KEY_PERIOD)) buoyancy.AdjustDrag(40.0);
        if (IsKeyPressed(KEY_D)) buoyancy.ToggleDrag();
        if (IsKeyPressed(KEY_G)) buoyancy.ToggleRotation();
        if (IsKeyPressed(KEY_SPACE)) buoyancy.ApplyImpulse({0.0, 260.0});
        if (IsKeyPressed(KEY_BACKSPACE) || IsKeyPressed(KEY_R)) buoyancy.Reset();
    }
    string.SetActive(simulation == SimulationKind::VibratingString);
    string.SetPaused(paused);
    string.SetMuted(muted);
    if (simulation != SimulationKind::VibratingString && (view == ViewMode::Spectrum || view == ViewMode::Split)) view = ViewMode::TwoD;
    HandleCameraInput(camera, view);
}

Camera3D MakeCamera(const OrbitCamera& orbit, Vector3 target) {
    const float cp = std::cos(orbit.pitch); Camera3D c{};
    c.position = {target.x + orbit.distance * cp * std::sin(orbit.yaw), target.y + orbit.distance * std::sin(orbit.pitch), target.z + orbit.distance * cp * std::cos(orbit.yaw)};
    c.target = target; c.up = {0.0f, 1.0f, 0.0f}; c.fovy = 45.0f; c.projection = CAMERA_PERSPECTIVE; return c;
}

Camera3D MakeCamera(const OrbitCamera& orbit) { return MakeCamera(orbit, {0.0f, 3.0f, 0.0f}); }

Camera3D MakeFlipCamera(const OrbitCamera& orbit, bool frontView) {
    if (!frontView) return MakeCamera(orbit);
    Camera3D camera{};
    camera.position = {0.0f, 4.0f, 18.0f};
    camera.target = {0.0f, 4.0f, 0.0f};
    camera.up = {0.0f, 1.0f, 0.0f};
    camera.fovy = 10.0f;
    camera.projection = CAMERA_ORTHOGRAPHIC;
    return camera;
}

void DrawGrid3D(float size, float spacing) {
    const int lines = static_cast<int>(size / spacing);
    for (int i = -lines; i <= lines; ++i) { const float o = static_cast<float>(i) * spacing; DrawLine3D({-size, 0.0f, o}, {size, 0.0f, o}, i == 0 ? Color{95, 105, 123, 255} : Color{52, 61, 77, 255}); DrawLine3D({o, 0.0f, -size}, {o, 0.0f, size}, i == 0 ? Color{95, 105, 123, 255} : Color{52, 61, 77, 255}); }
}

struct VoxelMeshBuilder {
    std::vector<float> vertices;
    std::vector<float> normals;
    std::vector<unsigned char> colors;

    void AddVertex(Vector3 position, Vector3 normal, Color color) {
        vertices.insert(vertices.end(), {position.x, position.y, position.z});
        normals.insert(normals.end(), {normal.x, normal.y, normal.z});
        colors.insert(colors.end(), {color.r, color.g, color.b, color.a});
    }

    void AddFace(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Vector3 normal, Color color) {
        AddVertex(a, normal, color); AddVertex(b, normal, color); AddVertex(c, normal, color);
        AddVertex(c, normal, color); AddVertex(d, normal, color); AddVertex(a, normal, color);
    }
};

void DrawVoxelFluid3D(const std::vector<FlipParticle>& particles, int gridWidth, int gridHeight, int gridDepth,
                     float occupancyCellSize, float renderCellSize, float worldWidth, float worldDepth, Color color) {
    const std::size_t cellCount = static_cast<std::size_t>(gridWidth * gridHeight * gridDepth);
    std::vector<unsigned char> occupied(cellCount, 0);
    const auto cellIndex = [gridWidth, gridHeight](int i, int j, int k) {
        return static_cast<std::size_t>(i + gridWidth * (j + gridHeight * k));
    };
    for (const auto& particle : particles) {
        const int i = std::clamp(static_cast<int>(particle.position.x / occupancyCellSize), 0, gridWidth - 1);
        const int j = std::clamp(static_cast<int>(particle.position.y / occupancyCellSize), 0, gridHeight - 1);
        const int k = std::clamp(static_cast<int>(particle.position.z / occupancyCellSize), 0, gridDepth - 1);
        occupied[cellIndex(i, j, k)] = 1;
    }

    VoxelMeshBuilder builder;
    const float halfWidth = worldWidth * 0.5f;
    const float halfDepth = worldDepth * 0.5f;
    const auto isOccupied = [&](int i, int j, int k) {
        return i >= 0 && i < gridWidth && j >= 0 && j < gridHeight && k >= 0 && k < gridDepth && occupied[cellIndex(i, j, k)] != 0;
    };
    for (int k = 0; k < gridDepth; ++k) {
        for (int j = 0; j < gridHeight; ++j) {
            for (int i = 0; i < gridWidth; ++i) {
                if (!isOccupied(i, j, k)) continue;
                const float xCenter = (static_cast<float>(i) + 0.5f) * occupancyCellSize - halfWidth;
                const float yCenter = (static_cast<float>(j) + 0.5f) * occupancyCellSize;
                const float zCenter = (static_cast<float>(k) + 0.5f) * occupancyCellSize - halfDepth;
                const float x0 = xCenter - renderCellSize * 0.5f;
                const float x1 = xCenter + renderCellSize * 0.5f;
                const float y0 = yCenter - renderCellSize * 0.5f;
                const float y1 = yCenter + renderCellSize * 0.5f;
                const float z0 = zCenter - renderCellSize * 0.5f;
                const float z1 = zCenter + renderCellSize * 0.5f;
                if (!isOccupied(i - 1, j, k)) builder.AddFace({x0, y0, z0}, {x0, y0, z1}, {x0, y1, z1}, {x0, y1, z0}, {-1.0f, 0.0f, 0.0f}, color);
                if (!isOccupied(i + 1, j, k)) builder.AddFace({x1, y0, z0}, {x1, y1, z0}, {x1, y1, z1}, {x1, y0, z1}, {1.0f, 0.0f, 0.0f}, color);
                if (!isOccupied(i, j - 1, k)) builder.AddFace({x0, y0, z0}, {x1, y0, z0}, {x1, y0, z1}, {x0, y0, z1}, {0.0f, -1.0f, 0.0f}, color);
                if (!isOccupied(i, j + 1, k)) builder.AddFace({x0, y1, z0}, {x0, y1, z1}, {x1, y1, z1}, {x1, y1, z0}, {0.0f, 1.0f, 0.0f}, color);
                if (!isOccupied(i, j, k - 1)) builder.AddFace({x0, y0, z0}, {x0, y1, z0}, {x1, y1, z0}, {x1, y0, z0}, {0.0f, 0.0f, -1.0f}, color);
                if (!isOccupied(i, j, k + 1)) builder.AddFace({x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1}, {0.0f, 0.0f, 1.0f}, color);
            }
        }
    }
    if (builder.vertices.empty()) return;

    Mesh mesh{};
    mesh.vertexCount = static_cast<int>(builder.vertices.size() / 3);
    mesh.triangleCount = mesh.vertexCount / 3;
    mesh.vertices = static_cast<float*>(MemAlloc(builder.vertices.size() * sizeof(float)));
    mesh.normals = static_cast<float*>(MemAlloc(builder.normals.size() * sizeof(float)));
    mesh.colors = static_cast<unsigned char*>(MemAlloc(builder.colors.size() * sizeof(unsigned char)));
    std::memcpy(mesh.vertices, builder.vertices.data(), builder.vertices.size() * sizeof(float));
    std::memcpy(mesh.normals, builder.normals.data(), builder.normals.size() * sizeof(float));
    std::memcpy(mesh.colors, builder.colors.data(), builder.colors.size() * sizeof(unsigned char));
    UploadMesh(&mesh, false);
    static Material material = LoadMaterialDefault();
    material.maps[MATERIAL_MAP_DIFFUSE].color = WHITE;
    const Matrix identity = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                             0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    DrawMesh(mesh, material, identity);
    UnloadMesh(mesh);
}

Vector2 WorldToScreen(Vector3 world, float scale, Vector2 center) { return {center.x + world.x * scale, center.y - world.y * scale}; }

void DrawGrid2D(float scale, Vector2 center) {
    for (int i = -14; i <= 14; ++i) { const float x = center.x + static_cast<float>(i) * scale * kGridStep; DrawLine(static_cast<int>(x), 40, static_cast<int>(x), GetScreenHeight() - 45, i == 0 ? Color{95, 105, 123, 255} : Color{52, 61, 77, 255}); }
    for (int i = 0; i <= 18; ++i) { const float y = center.y - static_cast<float>(i) * scale * kGridStep; DrawLine(30, static_cast<int>(y), GetScreenWidth() - 30, static_cast<int>(y), i == 0 ? Color{95, 105, 123, 255} : Color{52, 61, 77, 255}); }
    DrawLine(30, static_cast<int>(center.y), GetScreenWidth() - 30, static_cast<int>(center.y), Color{174, 72, 93, 180});
}

void DrawSpring3D(Vector3 start, Vector3 end, Color color) { DrawLine3D(start, end, color); DrawLine3D(Add(start, {0.025f, 0.0f, 0.0f}), Add(end, {0.025f, 0.0f, 0.0f}), Fade(color, 0.5f)); DrawLine3D(Add(start, {-0.025f, 0.0f, 0.0f}), Add(end, {-0.025f, 0.0f, 0.0f}), Fade(color, 0.5f)); }

void DrawSpring3D(const SpringMass& s) { const float size = 0.55f * std::cbrt(s.mass); DrawGrid3D(7.0f, kGridStep); DrawPlane({0.0f, -0.015f, 0.0f}, {14.0f, 14.0f}, Color{19, 26, 41, 255}); DrawSpring3D(s.anchor, s.position, Color{120, 213, 255, 255}); DrawSphere(s.anchor, 0.14f, Color{126, 235, 167, 255}); DrawCube(s.position, size, size, size, Color{255, 170, 72, 255}); DrawCubeWires(s.position, size, size, size, Color{255, 227, 170, 255}); }
void DrawSpring2D(const SpringMass& s) { const float scale = std::min(GetScreenWidth(), GetScreenHeight()) / 13.0f; const Vector2 center = {GetScreenWidth() * 0.5f, GetScreenHeight() * 0.72f}; const float size = 0.55f * std::cbrt(s.mass) * scale; const Vector2 a = WorldToScreen(s.anchor, scale, center); const Vector2 p = WorldToScreen(s.position, scale, center); DrawGrid2D(scale, center); DrawLineEx(a, p, 4.0f, Color{120, 213, 255, 255}); DrawCircleV(a, 0.14f * scale, Color{126, 235, 167, 255}); DrawRectangle(static_cast<int>(p.x - size * 0.5f), static_cast<int>(p.y - size * 0.5f), static_cast<int>(size), static_cast<int>(size), Color{255, 170, 72, 255}); DrawRectangleLines(static_cast<int>(p.x - size * 0.5f), static_cast<int>(p.y - size * 0.5f), static_cast<int>(size), static_cast<int>(size), Color{255, 227, 170, 255}); }
void DrawBall3D(const BouncingBall& b) { DrawGrid3D(7.0f, kGridStep); DrawPlane({0.0f, -0.015f, 0.0f}, {14.0f, 14.0f}, Color{19, 26, 41, 255}); DrawSphere(b.position, b.radius, Color{126, 183, 255, 255}); DrawSphereWires(b.position, b.radius, 16, 16, Color{216, 235, 255, 255}); }
void DrawBall2D(const BouncingBall& b) { const float scale = std::min(GetScreenWidth(), GetScreenHeight()) / 13.0f; const Vector2 center = {GetScreenWidth() * 0.5f, GetScreenHeight() * 0.72f}; const Vector2 p = WorldToScreen(b.position, scale, center); DrawGrid2D(scale, center); DrawCircleV(p, b.radius * scale, Color{126, 183, 255, 255}); DrawCircleLines(static_cast<int>(p.x), static_cast<int>(p.y), b.radius * scale, Color{216, 235, 255, 255}); }
void DrawCooling3D(const NewtonCooling& c) { DrawGrid3D(7.0f, kGridStep); DrawPlane({0.0f, -0.015f, 0.0f}, {14.0f, 14.0f}, Color{19, 26, 41, 255}); DrawCube({0.0f, 1.5f, 0.0f}, 2.5f, 2.5f, 2.5f, TemperatureColor(c.temperature)); DrawCubeWires({0.0f, 1.5f, 0.0f}, 2.5f, 2.5f, 2.5f, WHITE); }
void DrawCooling2D(const NewtonCooling& c) { const float scale = std::min(GetScreenWidth(), GetScreenHeight()) / 13.0f; const Vector2 center = {GetScreenWidth() * 0.5f, GetScreenHeight() * 0.72f}; const float size = 2.5f * scale; const Vector2 p = WorldToScreen({0.0f, 1.5f, 0.0f}, scale, center); DrawGrid2D(scale, center); DrawRectangle(static_cast<int>(p.x - size * 0.5f), static_cast<int>(p.y - size * 0.5f), static_cast<int>(size), static_cast<int>(size), TemperatureColor(c.temperature)); DrawRectangleLines(static_cast<int>(p.x - size * 0.5f), static_cast<int>(p.y - size * 0.5f), static_cast<int>(size), static_cast<int>(size), WHITE); }

float BuoyancyRenderScale() {
    return std::min((static_cast<float>(GetScreenWidth()) - 140.0f) / static_cast<float>(2.0 * BuoyancyScene::kTankHalfWidth),
                    (static_cast<float>(GetScreenHeight()) - 150.0f) / static_cast<float>(BuoyancyScene::kTankHeight));
}

Vector2 BuoyancyWorldToScreen(BuoyancyVec2 point) {
    const float scale = BuoyancyRenderScale();
    return {70.0f + static_cast<float>((point.x + BuoyancyScene::kTankHalfWidth) * scale),
            static_cast<float>(GetScreenHeight()) - 100.0f - static_cast<float>(point.y * scale)};
}

void DrawForceArrow2D(Vector2 from, Vector2 to, Color color) {
    DrawLineEx(from, to, 3.0f, color);
    const Vector2 difference = {to.x - from.x, to.y - from.y};
    const float length = std::sqrt(difference.x * difference.x + difference.y * difference.y);
    if (length < 0.001f) return;
    const Vector2 direction = {difference.x / length, difference.y / length};
    const Vector2 perpendicular = {-direction.y, direction.x};
    const Vector2 first = {to.x - direction.x * 12.0f - perpendicular.x * 6.0f, to.y - direction.y * 12.0f - perpendicular.y * 6.0f};
    const Vector2 second = {to.x - direction.x * 12.0f + perpendicular.x * 6.0f, to.y - direction.y * 12.0f + perpendicular.y * 6.0f};
    DrawLineEx(to, first, 3.0f, color);
    DrawLineEx(to, second, 3.0f, color);
}

void DrawBuoyancy2D(const BuoyancySceneSnapshot& snapshot) {
    const float scale = BuoyancyRenderScale();
    const int left = 70;
    const int right = left + static_cast<int>(2.0 * BuoyancyScene::kTankHalfWidth * scale);
    const int bottom = GetScreenHeight() - 100;
    const int waterY = bottom - static_cast<int>(BuoyancyScene::kWaterLevel * scale);
    DrawRectangle(left, bottom - static_cast<int>(BuoyancyScene::kTankHeight * scale), right - left, static_cast<int>(BuoyancyScene::kTankHeight * scale), Color{12, 20, 31, 255});
    DrawRectangle(left, waterY, right - left, bottom - waterY, Color{35, 119, 176, 150});
    DrawLine(left, waterY, right, waterY, Color{120, 213, 255, 255});
    DrawRectangleLines(left, bottom - static_cast<int>(BuoyancyScene::kTankHeight * scale), right - left, static_cast<int>(BuoyancyScene::kTankHeight * scale), Color{143, 169, 194, 255});

    const Vector2 bodyCenter = BuoyancyWorldToScreen(snapshot.position);
    const Color bodyColor = {228, 164, 76, 255};
    for (int i = 0; i < 4; ++i) {
        const Vector2 a = BuoyancyWorldToScreen(snapshot.corners[static_cast<std::size_t>(i)]);
        const Vector2 b = BuoyancyWorldToScreen(snapshot.corners[static_cast<std::size_t>((i + 1) % 4)]);
        DrawTriangle(bodyCenter, a, b, bodyColor);
        DrawLineEx(a, b, 3.0f, Color{255, 226, 163, 255});
    }
    DrawCircleV(bodyCenter, 6.0f, Color{255, 238, 187, 255});
    DrawLineEx(bodyCenter, {bodyCenter.x + std::cos(static_cast<float>(snapshot.angle)) * 35.0f, bodyCenter.y - std::sin(static_cast<float>(snapshot.angle)) * 35.0f}, 2.0f, Color{255, 238, 187, 255});
    if (snapshot.submergedCount > 0) {
        for (int i = 0; i < snapshot.submergedCount; ++i) {
            const Vector2 a = BuoyancyWorldToScreen(snapshot.submergedPolygon[static_cast<std::size_t>(i)]);
            const Vector2 b = BuoyancyWorldToScreen(snapshot.submergedPolygon[static_cast<std::size_t>((i + 1) % snapshot.submergedCount)]);
            DrawLineEx(a, b, 3.0f, Color{109, 230, 244, 255});
        }
        const Vector2 cob = BuoyancyWorldToScreen(snapshot.centerOfBuoyancy);
        DrawCircleV(cob, 7.0f, Color{126, 235, 167, 255});
        const float forceScale = 1.0f / 3000.0f;
        DrawForceArrow2D(bodyCenter, {bodyCenter.x, bodyCenter.y + static_cast<float>(snapshot.mass * -BuoyancyScene::kGravity * forceScale * scale)}, Color{255, 108, 100, 255});
        DrawForceArrow2D(cob, {cob.x, cob.y - static_cast<float>(snapshot.buoyancyForce * forceScale * scale)}, Color{126, 235, 167, 255});
    }
    DrawText("ANALYTIC WATER / FLOATING BODY", left + 12, bottom - static_cast<int>(BuoyancyScene::kTankHeight * scale) + 12, 18, Color{232, 238, 248, 255});
    DrawText("green: center of buoyancy   red: weight   cyan/green: buoyancy", left + 12, bottom - static_cast<int>(BuoyancyScene::kTankHeight * scale) + 37, 13, Color{168, 199, 221, 255});
}

void DrawBuoyancy3D(const BuoyancySceneSnapshot& snapshot) {
    DrawGrid3D(7.0f, 0.5f);
    DrawPlane({0.0f, -0.015f, 0.0f}, {14.0f, 14.0f}, Color{19, 26, 41, 255});
    DrawCubeWires({0.0f, static_cast<float>(BuoyancyScene::kTankHeight * 0.5), 0.0f}, static_cast<float>(2.0 * BuoyancyScene::kTankHalfWidth), static_cast<float>(BuoyancyScene::kTankHeight), static_cast<float>(2.0 * BuoyancyScene::kTankHalfDepth), Color{143, 169, 194, 255});
    DrawPlane({0.0f, static_cast<float>(BuoyancyScene::kWaterLevel), 0.0f}, {static_cast<float>(2.0 * BuoyancyScene::kTankHalfWidth), static_cast<float>(2.0 * BuoyancyScene::kTankHalfDepth)}, Color{35, 119, 176, 105});
    constexpr int edges[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    for (const auto& edge : edges) {
        const BuoyancyVec3 a = snapshot.corners3D[static_cast<std::size_t>(edge[0])];
        const BuoyancyVec3 b = snapshot.corners3D[static_cast<std::size_t>(edge[1])];
        DrawLine3D({static_cast<float>(a.x), static_cast<float>(a.y), static_cast<float>(a.z)}, {static_cast<float>(b.x), static_cast<float>(b.y), static_cast<float>(b.z)}, Color{255, 226, 163, 255});
    }
    const Vector3 center = {static_cast<float>(snapshot.position3D.x), static_cast<float>(snapshot.position3D.y), static_cast<float>(snapshot.position3D.z)};
    DrawSphere(center, 0.10f, Color{255, 238, 187, 255});
    const Vector3 cob = {static_cast<float>(snapshot.centerOfBuoyancy3D.x), static_cast<float>(snapshot.centerOfBuoyancy3D.y), static_cast<float>(snapshot.centerOfBuoyancy3D.z)};
    if (snapshot.submergedVolume3D > 0.0) {
        DrawSphere(cob, 0.13f, Color{126, 235, 167, 255});
        DrawLine3D(center, {center.x, center.y - static_cast<float>(snapshot.mass * -BuoyancyScene::kGravity / 3000.0), center.z}, Color{255, 108, 100, 255});
        DrawLine3D(cob, {cob.x, cob.y + static_cast<float>(snapshot.buoyancyForce3D / 3000.0), cob.z}, Color{126, 235, 167, 255});
    }
}

void DrawVibratingString3D(const StringSnapshot& s) { DrawGrid3D(7.0f, 0.5f); DrawPlane({0.0f, -0.015f, 0.0f}, {14.0f, 14.0f}, Color{19, 26, 41, 255}); const Color color = TemperatureColor(s.temperature); const float x0 = -s.length * 0.5f; for (std::size_t i = 0; i + 1 < s.kDisplayNodes; ++i) { const float x1 = x0 + s.length * static_cast<float>(i) / 64.0f; const float x2 = x0 + s.length * static_cast<float>(i + 1) / 64.0f; DrawLine3D({x1, s.displacement[i] * s.displacementScale + 3.5f, 0.0f}, {x2, s.displacement[i + 1] * s.displacementScale + 3.5f, 0.0f}, color); } DrawCube({x0, 3.5f, 0.0f}, 0.25f, 3.0f, 0.25f, Color{126, 235, 167, 255}); DrawCube({-x0, 3.5f, 0.0f}, 0.25f, 3.0f, 0.25f, Color{126, 235, 167, 255}); }
void DrawVibratingString2D(const StringSnapshot& s) { const float scale = std::min(GetScreenWidth(), GetScreenHeight()) / 8.0f; const Vector2 center = {GetScreenWidth() * 0.5f, GetScreenHeight() * 0.55f}; DrawGrid2D(scale, center); const Color color = TemperatureColor(s.temperature); for (std::size_t i = 0; i + 1 < s.kDisplayNodes; ++i) { const Vector2 p1 = {center.x - s.length * scale * 0.5f + s.length * scale * static_cast<float>(i) / 64.0f, center.y - s.displacement[i] * scale * s.displacementScale}; const Vector2 p2 = {center.x - s.length * scale * 0.5f + s.length * scale * static_cast<float>(i + 1) / 64.0f, center.y - s.displacement[i + 1] * scale * s.displacementScale}; DrawLineEx(p1, p2, 3.0f, color); } }
float FlipRenderScale() {
    return std::min((static_cast<float>(GetScreenWidth()) - 140.0f) / static_cast<float>(FlipFluid::kWorldWidth), (static_cast<float>(GetScreenHeight()) - 150.0f) / static_cast<float>(FlipFluid::kWorldHeight));
}

Vector2 FlipWorldToScreen(FlipVec2 point) {
    const float scale = FlipRenderScale();
    return {70.0f + static_cast<float>(point.x) * scale, static_cast<float>(GetScreenHeight()) - 100.0f - static_cast<float>(point.y) * scale};
}

FlipVec2 FlipScreenToWorld(Vector2 point) {
    const float scale = FlipRenderScale();
    return {(static_cast<double>(point.x) - 70.0) / scale, (static_cast<double>(GetScreenHeight()) - 100.0 - point.y) / scale};
}

Color FlipParticleColor(const FlipParticle& particle) {
    const float speed = std::clamp(static_cast<float>(std::sqrt(particle.velocity.x * particle.velocity.x + particle.velocity.y * particle.velocity.y + particle.velocity.z * particle.velocity.z)), 0.0f, 8.0f);
    return {static_cast<unsigned char>(40.0f + 24.0f * speed), static_cast<unsigned char>(150.0f + 10.0f * speed), static_cast<unsigned char>(235.0f - 14.0f * speed), 235};
}

void DrawFlip2D(const FlipFluid& fluid) {
    const float scale = FlipRenderScale();
    const Vector2 topLeft = FlipWorldToScreen({0.0, FlipFluid::kWorldHeight});
    const int tankWidth = static_cast<int>(FlipFluid::kWorldWidth * scale);
    const int tankHeight = static_cast<int>(FlipFluid::kWorldHeight * scale);
    DrawRectangle(static_cast<int>(topLeft.x), static_cast<int>(topLeft.y), tankWidth, tankHeight, Color{11, 26, 48, 255});
    if (fluid.SolidObstacleEnabled()) {
        const FlipSolidBox& obstacle = fluid.SolidObstacle();
        const Vector2 obstacleTopLeft = FlipWorldToScreen({obstacle.center.x - obstacle.halfExtents.x, obstacle.center.y + obstacle.halfExtents.y});
        DrawRectangle(static_cast<int>(obstacleTopLeft.x), static_cast<int>(obstacleTopLeft.y), static_cast<int>(2.0 * obstacle.halfExtents.x * scale), static_cast<int>(2.0 * obstacle.halfExtents.y * scale), Color{139, 148, 164, 180});
        DrawRectangleLines(static_cast<int>(obstacleTopLeft.x), static_cast<int>(obstacleTopLeft.y), static_cast<int>(2.0 * obstacle.halfExtents.x * scale), static_cast<int>(2.0 * obstacle.halfExtents.y * scale), Color{232, 205, 141, 255});
    }
    {
        const FlipSolidBox& buoyant = fluid.BuoyantBox();
        const Vector2 boxTopLeft = FlipWorldToScreen({buoyant.center.x - buoyant.halfExtents.x, buoyant.center.y + buoyant.halfExtents.y});
        DrawRectangle(static_cast<int>(boxTopLeft.x), static_cast<int>(boxTopLeft.y), static_cast<int>(2.0 * buoyant.halfExtents.x * scale), static_cast<int>(2.0 * buoyant.halfExtents.y * scale), Color{235, 155, 72, 180});
        DrawRectangleLines(static_cast<int>(boxTopLeft.x), static_cast<int>(boxTopLeft.y), static_cast<int>(2.0 * buoyant.halfExtents.x * scale), static_cast<int>(2.0 * buoyant.halfExtents.y * scale), Color{126, 235, 167, 255});
    }
    for (int j = 0; j < FlipFluid::kGridHeight; ++j) {
        for (int i = 0; i < FlipFluid::kGridWidth; ++i) {
            bool occupied = false;
            float pressureMagnitude = 0.0f;
            for (int k = 0; k < FlipFluid::kGridDepth; ++k) {
                const std::size_t index = static_cast<std::size_t>(i + FlipFluid::kGridWidth * (j + FlipFluid::kGridHeight * k));
                if (!fluid.FluidCells()[index]) continue;
                occupied = true;
                pressureMagnitude = std::max(pressureMagnitude, std::abs(fluid.Pressure()[index]));
            }
            if (!occupied) continue;
            const float pressure = std::clamp(pressureMagnitude / 5000.0f, 0.0f, 1.0f);
            const Vector2 cell = FlipWorldToScreen({i * FlipFluid::kCellSize, (j + 1) * FlipFluid::kCellSize});
            DrawRectangle(static_cast<int>(cell.x), static_cast<int>(cell.y), static_cast<int>(std::ceil(scale * FlipFluid::kCellSize)) + 1, static_cast<int>(std::ceil(scale * FlipFluid::kCellSize)) + 1, Color{20, static_cast<unsigned char>(70.0f + 60.0f * pressure), static_cast<unsigned char>(120.0f + 80.0f * pressure), 115});
        }
    }
    for (const auto& particle : fluid.Particles()) DrawCircleV(FlipWorldToScreen({particle.position.x, particle.position.y}), std::max(2.0f, static_cast<float>(0.09f * scale)), FlipParticleColor(particle));
    DrawRectangleLines(static_cast<int>(topLeft.x), static_cast<int>(topLeft.y), tankWidth, tankHeight, Color{125, 181, 232, 255});
    DrawText("3D FLIP / X-Y PROJECTION", static_cast<int>(topLeft.x) + 12, static_cast<int>(topLeft.y) + 12, 18, Color{232, 238, 248, 255});
    DrawText("left click: inject upward impulse   SPACE: center splash", static_cast<int>(topLeft.x) + 12, static_cast<int>(topLeft.y) + 37, 13, Color{147, 192, 230, 255});
}

void DrawFlip3D(const FlipFluid& fluid) {
    DrawGrid3D(9.0f, 0.5f);
    DrawPlane({0.0f, -0.015f, 0.0f}, {18.0f, 18.0f}, Color{19, 26, 41, 255});
    DrawCubeWires({0.0f, static_cast<float>(FlipFluid::kWorldHeight * 0.5), 0.0f}, static_cast<float>(FlipFluid::kWorldWidth), static_cast<float>(FlipFluid::kWorldHeight), static_cast<float>(FlipFluid::kWorldDepth), Color{125, 181, 232, 255});
    if (fluid.SolidObstacleEnabled()) {
        const FlipSolidBox& obstacle = fluid.SolidObstacle();
        // Simulation coordinates are [0, worldSize], while the 3D tank and
        // voxel mesh are centered around x/z = 0 for the orbit camera.
        const Vector3 renderCenter = {
            static_cast<float>(obstacle.center.x - FlipFluid::kWorldWidth * 0.5),
            static_cast<float>(obstacle.center.y),
            static_cast<float>(obstacle.center.z - FlipFluid::kWorldDepth * 0.5)
        };
        DrawCube(renderCenter, static_cast<float>(2.0 * obstacle.halfExtents.x), static_cast<float>(2.0 * obstacle.halfExtents.y), static_cast<float>(2.0 * obstacle.halfExtents.z), Color{139, 148, 164, 170});
        DrawCubeWires(renderCenter, static_cast<float>(2.0 * obstacle.halfExtents.x), static_cast<float>(2.0 * obstacle.halfExtents.y), static_cast<float>(2.0 * obstacle.halfExtents.z), Color{232, 205, 141, 255});
    }
    {
        const FlipSolidBox& buoyant = fluid.BuoyantBox();
        const Vector3 renderCenter = {
            static_cast<float>(buoyant.center.x - FlipFluid::kWorldWidth * 0.5),
            static_cast<float>(buoyant.center.y),
            static_cast<float>(buoyant.center.z - FlipFluid::kWorldDepth * 0.5)
        };
        DrawCube(renderCenter, static_cast<float>(2.0 * buoyant.halfExtents.x), static_cast<float>(2.0 * buoyant.halfExtents.y), static_cast<float>(2.0 * buoyant.halfExtents.z), Color{235, 155, 72, 180});
        DrawCubeWires(renderCenter, static_cast<float>(2.0 * buoyant.halfExtents.x), static_cast<float>(2.0 * buoyant.halfExtents.y), static_cast<float>(2.0 * buoyant.halfExtents.z), Color{126, 235, 167, 255});
    }
    DrawVoxelFluid3D(fluid.Particles(), FlipFluid::kGridWidth, FlipFluid::kGridHeight, FlipFluid::kGridDepth,
                     static_cast<float>(FlipFluid::kCellSize), static_cast<float>(FlipFluid::kCellSize), static_cast<float>(FlipFluid::kWorldWidth),
                     static_cast<float>(FlipFluid::kWorldDepth), Color{42, 164, 239, 220});
}

float SphRenderScale() {
    return std::min((static_cast<float>(GetScreenWidth()) - 140.0f) / static_cast<float>(SphFluid::kWorldWidth),
                    (static_cast<float>(GetScreenHeight()) - 150.0f) / static_cast<float>(SphFluid::kWorldHeight));
}

Vector2 SphWorldToScreen(FlipVec3 point) {
    const float scale = SphRenderScale();
    return {70.0f + static_cast<float>(point.x) * scale,
            static_cast<float>(GetScreenHeight()) - 100.0f - static_cast<float>(point.y) * scale};
}

FlipVec3 SphScreenToWorld(Vector2 point) {
    const float scale = SphRenderScale();
    return {(static_cast<double>(point.x) - 70.0) / scale,
            (static_cast<double>(GetScreenHeight()) - 100.0 - point.y) / scale,
            SphFluid::kWorldDepth * 0.5};
}

void DrawSph2D(const SphFluid& fluid) {
    const float scale = SphRenderScale();
    const Vector2 topLeft = SphWorldToScreen({0.0, SphFluid::kWorldHeight, 0.0});
    const int tankWidth = static_cast<int>(SphFluid::kWorldWidth * scale);
    const int tankHeight = static_cast<int>(SphFluid::kWorldHeight * scale);
    DrawRectangle(static_cast<int>(topLeft.x), static_cast<int>(topLeft.y), tankWidth, tankHeight, Color{11, 26, 48, 255});
    const auto& particles = fluid.Particles();
    const std::size_t renderStride = particles.size() > 1400 ? 2 : 1;
    for (std::size_t index = 0; index < particles.size(); index += renderStride) {
        const auto& particle = particles[index];
        DrawCircleV(SphWorldToScreen(particle.position), std::max(2.0f, static_cast<float>(0.13f * scale)), FlipParticleColor(particle));
    }
    DrawRectangleLines(static_cast<int>(topLeft.x), static_cast<int>(topLeft.y), tankWidth, tankHeight, Color{125, 181, 232, 255});
    DrawText("SPH / 3D PARTICLE PROJECTION", static_cast<int>(topLeft.x) + 12, static_cast<int>(topLeft.y) + 12, 18, Color{232, 238, 248, 255});
    DrawText("left click: inject upward impulse   SPACE: center splash", static_cast<int>(topLeft.x) + 12, static_cast<int>(topLeft.y) + 37, 13, Color{147, 192, 230, 255});
}

void DrawSph3D(const SphFluid& fluid) {
    DrawGrid3D(9.0f, 0.5f);
    DrawPlane({0.0f, -0.015f, 0.0f}, {18.0f, 18.0f}, Color{19, 26, 41, 255});
    DrawCubeWires({0.0f, static_cast<float>(SphFluid::kWorldHeight * 0.5), 0.0f}, static_cast<float>(SphFluid::kWorldWidth), static_cast<float>(SphFluid::kWorldHeight), static_cast<float>(SphFluid::kWorldDepth), Color{125, 181, 232, 255});
    DrawVoxelFluid3D(fluid.Particles(), SphFluid::kGridWidth, SphFluid::kGridHeight, SphFluid::kGridDepth,
                     static_cast<float>(SphFluid::kCellSize), 0.62f, static_cast<float>(SphFluid::kWorldWidth),
                     static_cast<float>(SphFluid::kWorldDepth), Color{50, 196, 224, 220});
}

constexpr PowderScene::Material kPowderPalette[] = {
    PowderScene::Material::Sand,
    PowderScene::Material::Water,
    PowderScene::Material::Stone,
    PowderScene::Material::Wood,
    PowderScene::Material::Fire,
    PowderScene::Material::Smoke,
    PowderScene::Material::Steam,
    PowderScene::Material::Oil,
    PowderScene::Material::Acid,
    PowderScene::Material::Empty
};

Rectangle PowderPalettePanel() {
    const float width = std::min(270.0f, static_cast<float>(GetScreenWidth() - 40));
    return {static_cast<float>(GetScreenWidth()) - width - 24.0f, 28.0f, width, 350.0f};
}

Rectangle PowderPaletteItem(int index) {
    const Rectangle panel = PowderPalettePanel();
    const float gap = 8.0f;
    const float itemWidth = (panel.width - 3.0f * gap) * 0.5f;
    const int column = index % 2;
    const int row = index / 2;
    return {panel.x + gap + static_cast<float>(column) * (itemWidth + gap), panel.y + 54.0f + static_cast<float>(row) * 34.0f, itemWidth, 28.0f};
}

Rectangle PowderBrushItem(int index) {
    const Rectangle panel = PowderPalettePanel();
    const float gap = 8.0f;
    const float itemWidth = (panel.width - 4.0f * gap) / 3.0f;
    return {panel.x + gap + static_cast<float>(index) * (itemWidth + gap), panel.y + panel.height - 31.0f, itemWidth, 23.0f};
}

int PowderPaletteHit(Vector2 point) {
    const int count = static_cast<int>(sizeof(kPowderPalette) / sizeof(kPowderPalette[0]));
    for (int i = 0; i < count; ++i) {
        if (CheckCollisionPointRec(point, PowderPaletteItem(i))) return i;
    }
    return -1;
}

void DrawPowderPalette(const PowderScene& powder, int brushRadius) {
    const Rectangle panel = PowderPalettePanel();
    DrawRectangleRec(panel, Color{15, 21, 34, 242});
    DrawRectangleLinesEx(panel, 1.0f, Color{76, 91, 117, 255});
    DrawText("POWDER TOOLS", static_cast<int>(panel.x + 14), static_cast<int>(panel.y + 12), 17, Color{232, 238, 248, 255});
    DrawText("MATERIAL", static_cast<int>(panel.x + 14), static_cast<int>(panel.y + 35), 11, Color{147, 160, 182, 255});

    const int count = static_cast<int>(sizeof(kPowderPalette) / sizeof(kPowderPalette[0]));
    for (int i = 0; i < count; ++i) {
        const Rectangle item = PowderPaletteItem(i);
        const PowderScene::Material material = kPowderPalette[i];
        const bool selected = material == powder.SelectedMaterial();
        const Color fill = selected ? Color{40, 58, 78, 255} : Color{24, 31, 46, 255};
        DrawRectangleRec(item, fill);
        DrawRectangleLinesEx(item, selected ? 2.0f : 1.0f, selected ? Color{126, 235, 167, 255} : Color{76, 91, 117, 255});
        if (material != PowderScene::Material::Empty) {
            DrawRectangle(static_cast<int>(item.x + 7), static_cast<int>(item.y + 7), 14, 14, PowderScene::MaterialColor(material));
        } else {
            DrawLine(static_cast<int>(item.x + 7), static_cast<int>(item.y + 21), static_cast<int>(item.x + 21), static_cast<int>(item.y + 7), Color{255, 122, 122, 255});
        }
        DrawText(material == PowderScene::Material::Empty ? "ERASER" : PowderScene::MaterialName(material), static_cast<int>(item.x + 29), static_cast<int>(item.y + 7), 11, Color{218, 225, 237, 255});
    }

    const int controlsY = static_cast<int>(panel.y + panel.height - 67.0f);
    DrawText(TextFormat("BRUSH SIZE  %d", brushRadius), static_cast<int>(panel.x + 14), controlsY, 12, Color{196, 204, 219, 255});
    DrawText("[ / ] changes material", static_cast<int>(panel.x + 14), controlsY + 16, 11, Color{147, 160, 182, 255});
    constexpr int brushSizes[3] = {2, 4, 7};
    for (int i = 0; i < 3; ++i) {
        const Rectangle item = PowderBrushItem(i);
        const bool selected = brushRadius == brushSizes[i];
        DrawRectangleRec(item, selected ? Color{40, 58, 78, 255} : Color{24, 31, 46, 255});
        DrawRectangleLinesEx(item, selected ? 2.0f : 1.0f, selected ? Color{126, 235, 167, 255} : Color{76, 91, 117, 255});
        DrawText(i == 0 ? "SMALL" : (i == 1 ? "MED" : "LARGE"), static_cast<int>(item.x + 8), static_cast<int>(item.y + 6), 10, Color{218, 225, 237, 255});
    }
}

float PowderRenderScale() {
    const float worldWidth = PowderScene::kGridWidth * PowderScene::kCellSize;
    const float worldHeight = PowderScene::kGridHeight * PowderScene::kCellSize;
    return std::min((static_cast<float>(GetScreenWidth()) - 140.0f) / worldWidth,
                    (static_cast<float>(GetScreenHeight()) - 150.0f) / worldHeight);
}

Vector2 PowderCellToScreen(int x, int y) {
    const float scale = PowderRenderScale();
    const float left = 70.0f;
    const float bottom = static_cast<float>(GetScreenHeight()) - 100.0f;
    return {left + static_cast<float>(x) * PowderScene::kCellSize * scale,
            bottom - static_cast<float>(y + 1) * PowderScene::kCellSize * scale};
}

bool PowderScreenToCell(Vector2 point, int& x, int& y) {
    const float scale = PowderRenderScale();
    const float left = 70.0f;
    const float bottom = static_cast<float>(GetScreenHeight()) - 100.0f;
    x = static_cast<int>((point.x - left) / (PowderScene::kCellSize * scale));
    y = static_cast<int>((bottom - point.y) / (PowderScene::kCellSize * scale));
    return x >= 0 && x < PowderScene::kGridWidth && y >= 0 && y < PowderScene::kGridHeight;
}

bool PowderRayBoxHit(const Ray& ray, Vector3 minimum, Vector3 maximum, float& entry, float& exit) {
    entry = 0.0f;
    exit = 100000.0f;
    const float origins[3] = {ray.position.x, ray.position.y, ray.position.z};
    const float directions[3] = {ray.direction.x, ray.direction.y, ray.direction.z};
    const float minimums[3] = {minimum.x, minimum.y, minimum.z};
    const float maximums[3] = {maximum.x, maximum.y, maximum.z};
    for (int axis = 0; axis < 3; ++axis) {
        if (std::abs(directions[axis]) < 0.000001f) {
            if (origins[axis] < minimums[axis] || origins[axis] > maximums[axis]) return false;
            continue;
        }
        float nearDistance = (minimums[axis] - origins[axis]) / directions[axis];
        float farDistance = (maximums[axis] - origins[axis]) / directions[axis];
        if (nearDistance > farDistance) std::swap(nearDistance, farDistance);
        entry = std::max(entry, nearDistance);
        exit = std::min(exit, farDistance);
        if (entry > exit) return false;
    }
    return exit >= 0.0f;
}

bool PowderRayToVolumeCell(Vector2 mouse, const OrbitCamera& orbit, int& x, int& y, int& z) {
    const float worldWidth = PowderScene::kVolumeWidth * PowderScene::kVolumeCellSize;
    const float worldHeight = PowderScene::kVolumeHeight * PowderScene::kVolumeCellSize;
    const float worldDepth = PowderScene::kVolumeDepth * PowderScene::kVolumeCellSize;
    const Camera3D camera = MakeCamera(orbit);
    const Ray ray = GetMouseRay(mouse, camera);
    float entry = 0.0f;
    float exit = 0.0f;
    if (!PowderRayBoxHit(ray, {-worldWidth * 0.5f, 0.0f, -worldDepth * 0.5f}, {worldWidth * 0.5f, worldHeight, worldDepth * 0.5f}, entry, exit)) return false;
    const float hitDistance = std::max(entry, 0.0f) + 0.02f;
    const Vector3 hit = {ray.position.x + ray.direction.x * hitDistance, ray.position.y + ray.direction.y * hitDistance, ray.position.z + ray.direction.z * hitDistance};
    x = static_cast<int>((hit.x + worldWidth * 0.5f) / PowderScene::kVolumeCellSize);
    y = static_cast<int>(hit.y / PowderScene::kVolumeCellSize);
    z = static_cast<int>((hit.z + worldDepth * 0.5f) / PowderScene::kVolumeCellSize);
    return x >= 0 && x < PowderScene::kVolumeWidth && y >= 0 && y < PowderScene::kVolumeHeight && z >= 0 && z < PowderScene::kVolumeDepth;
}

void DrawPowder2D(const PowderScene& powder) {
    const float scale = PowderRenderScale();
    const int width = static_cast<int>(PowderScene::kGridWidth * PowderScene::kCellSize * scale);
    const int height = static_cast<int>(PowderScene::kGridHeight * PowderScene::kCellSize * scale);
    const int left = 70;
    const int top = GetScreenHeight() - 100 - height;
    DrawRectangle(left, top, width, height, Color{12, 20, 31, 255});

    const int cellPixels = std::max(1, static_cast<int>(std::ceil(PowderScene::kCellSize * scale)));
    for (int y = 0; y < PowderScene::kGridHeight; ++y) {
        for (int x = 0; x < PowderScene::kGridWidth; ++x) {
            const PowderScene::Cell& cell = powder.CellAt(x, y);
            if (cell.material == PowderScene::Material::Empty) continue;
            const Vector2 position = PowderCellToScreen(x, y);
            DrawRectangle(static_cast<int>(position.x), static_cast<int>(position.y), cellPixels + 1, cellPixels + 1,
                          PowderScene::MaterialColor(cell.material, cell.temperature));
        }
    }
    DrawRectangleLines(left, top, width, height, Color{143, 169, 194, 255});
    DrawText("POWDER / CELLULAR AUTOMATON", left + 12, top + 12, 18, Color{232, 238, 248, 255});
    DrawText("left drag: paint   right drag: erase   SPACE: impulse", left + 12, top + 37, 13, Color{168, 199, 221, 255});
}

void DrawPowder3D(const PowderScene& powder) {
    const float worldWidth = PowderScene::kVolumeWidth * PowderScene::kVolumeCellSize;
    const float worldHeight = PowderScene::kVolumeHeight * PowderScene::kVolumeCellSize;
    const float worldDepth = PowderScene::kVolumeDepth * PowderScene::kVolumeCellSize;
    DrawGrid3D(7.0f, 0.5f);
    DrawPlane({0.0f, -0.015f, 0.0f}, {14.0f, 14.0f}, Color{19, 26, 41, 255});
    DrawCubeWires({0.0f, worldHeight * 0.5f, 0.0f}, worldWidth, worldHeight, worldDepth, Color{143, 169, 194, 255});

    for (int z = 0; z < PowderScene::kVolumeDepth; ++z) {
        for (int y = 0; y < PowderScene::kVolumeHeight; ++y) {
            for (int x = 0; x < PowderScene::kVolumeWidth; ++x) {
                const PowderScene::Cell& cell = powder.VolumeCellAt(x, y, z);
                if (cell.material == PowderScene::Material::Empty) continue;
                const float px = (static_cast<float>(x) + 0.5f) * PowderScene::kVolumeCellSize - worldWidth * 0.5f;
                const float py = (static_cast<float>(y) + 0.5f) * PowderScene::kVolumeCellSize;
                const float pz = (static_cast<float>(z) + 0.5f) * PowderScene::kVolumeCellSize - worldDepth * 0.5f;
                DrawCube({px, py, pz}, PowderScene::kVolumeCellSize * 1.02f, PowderScene::kVolumeCellSize * 1.02f, PowderScene::kVolumeCellSize * 1.02f,
                         PowderScene::MaterialColor(cell.material, cell.temperature));
            }
        }
    }
}

void HandlePowderMouseInput(PowderScene& powder, SimulationKind simulation, ViewMode view, int& brushRadius, const OrbitCamera& orbit) {
    if (simulation != SimulationKind::Powder) return;
    const Vector2 mouse = PhysicsMousePosition();
    if (PhysicsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        const int materialIndex = PowderPaletteHit(mouse);
        if (materialIndex >= 0) {
            powder.SetSelectedMaterial(kPowderPalette[materialIndex]);
            return;
        }
        constexpr int brushSizes[3] = {2, 4, 7};
        for (int i = 0; i < 3; ++i) {
            if (CheckCollisionPointRec(mouse, PowderBrushItem(i))) {
                brushRadius = brushSizes[i];
                return;
            }
        }
    }
    if (CheckCollisionPointRec(mouse, PowderPalettePanel())) return;
    if (view == ViewMode::ThreeD) {
        int x = 0;
        int y = 0;
        int z = 0;
        if (!PowderRayToVolumeCell(mouse, orbit, x, y, z)) return;
        if (PhysicsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) powder.EraseVolume(x, y, z, brushRadius);
            else powder.PaintVolume(x, y, z, brushRadius, powder.SelectedMaterial());
        }
        return;
    }
    if (view != ViewMode::TwoD) return;
    int x = 0;
    int y = 0;
    if (!PowderScreenToCell(mouse, x, y)) return;
    if (PhysicsMouseButtonDown(MOUSE_BUTTON_LEFT)) powder.Paint(x, y, brushRadius, powder.SelectedMaterial());
    if (PhysicsMouseButtonDown(MOUSE_BUTTON_RIGHT)) powder.Erase(x, y, brushRadius);
}

Rectangle SimulationMenuCard(int index) {
    const int count = static_cast<int>(sizeof(kSimulationMenu) / sizeof(kSimulationMenu[0]));
    const float menuWidth = std::min(1100.0f, static_cast<float>(GetScreenWidth() - 80));
    const float cardWidth = (menuWidth - 24.0f) * 0.5f;
    const int rows = (count + 1) / 2;
    const float top = 130.0f;
    const float gap = rows > 4 ? 10.0f : 16.0f;
    const float cardHeight = std::clamp((static_cast<float>(GetScreenHeight()) - top - 70.0f - gap * static_cast<float>(rows - 1)) / static_cast<float>(rows), 58.0f, 92.0f);
    const float left = (static_cast<float>(GetScreenWidth()) - menuWidth) * 0.5f;
    const int column = index % 2;
    const int row = index / 2;
    (void)count;
    return {left + static_cast<float>(column) * (cardWidth + 24.0f), top + static_cast<float>(row) * (cardHeight + gap), cardWidth, cardHeight};
}

void HandleSimulationMenuInput(SimulationKind& simulation, bool& menuOpen, int& cursor) {
    const int count = static_cast<int>(sizeof(kSimulationMenu) / sizeof(kSimulationMenu[0]));
    if (IsKeyPressed(KEY_ESCAPE)) {
        menuOpen = false;
        return;
    }
    if (IsKeyPressed(KEY_LEFT)) cursor = std::max(0, cursor - 1);
    if (IsKeyPressed(KEY_RIGHT)) cursor = std::min(count - 1, cursor + 1);
    if (IsKeyPressed(KEY_UP)) cursor = std::max(0, cursor - 2);
    if (IsKeyPressed(KEY_DOWN)) cursor = std::min(count - 1, cursor + 2);
    if (IsKeyPressed(KEY_ENTER)) {
        simulation = kSimulationMenu[cursor].kind;
        menuOpen = false;
        return;
    }
    if (PhysicsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        const Vector2 mouse = PhysicsMousePosition();
        for (int i = 0; i < count; ++i) {
            if (CheckCollisionPointRec(mouse, SimulationMenuCard(i))) {
                cursor = i;
                simulation = kSimulationMenu[i].kind;
                menuOpen = false;
                return;
            }
        }
    }
}

void DrawSimulationMenu(SimulationKind simulation, int cursor) {
    const int count = static_cast<int>(sizeof(kSimulationMenu) / sizeof(kSimulationMenu[0]));
    DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), Color{5, 9, 18, 245});
    DrawText("PHYSICS SANDBOX", 60, 48, 32, Color{232, 238, 248, 255});
    DrawText("SELECT A SIMULATION", 62, 90, 18, Color{120, 213, 255, 255});
    DrawText("Click a card or use the arrow keys and Enter. Press F1 to reopen this menu.", 62, 112, 14, Color{147, 160, 182, 255});

    for (int i = 0; i < count; ++i) {
        const Rectangle card = SimulationMenuCard(i);
        const bool selected = i == cursor;
        const bool active = kSimulationMenu[i].kind == simulation;
        const Color fill = selected ? Color{28, 42, 62, 255} : Color{15, 21, 34, 255};
        DrawRectangleRec(card, fill);
        DrawRectangleLinesEx(card, selected ? 3.0f : 1.0f, selected ? kSimulationMenu[i].accent : Color{76, 91, 117, 255});
        DrawRectangle(static_cast<int>(card.x), static_cast<int>(card.y), 7, static_cast<int>(card.height), kSimulationMenu[i].accent);
        const int shortcut = i == 9 ? 0 : i + 1;
        DrawText(TextFormat("%d", shortcut), static_cast<int>(card.x + 24), static_cast<int>(card.y + 18), 16, kSimulationMenu[i].accent);
        DrawText(kSimulationMenu[i].title, static_cast<int>(card.x + 60), static_cast<int>(card.y + 16), 20, Color{232, 238, 248, 255});
        DrawText(kSimulationMenu[i].description, static_cast<int>(card.x + 60), static_cast<int>(card.y + 49), 14, Color{171, 183, 201, 255});
        if (active) DrawText("ACTIVE", static_cast<int>(card.x + card.width - 74), static_cast<int>(card.y + 18), 11, Color{126, 235, 167, 255});
    }
    DrawText("F1 menu", 62, GetScreenHeight() - 42, 14, Color{147, 160, 182, 255});
}

void HandleFlipMouseInput(FlipFluid& fluid, SimulationKind simulation, ViewMode view) {
    if (simulation != SimulationKind::FlipFluid || view != ViewMode::TwoD || !PhysicsMouseButtonPressed(MOUSE_BUTTON_LEFT)) return;
    const FlipVec2 point = FlipScreenToWorld(PhysicsMousePosition());
    fluid.Poke({point.x, point.y, FlipFluid::kWorldDepth * 0.5}, {0.0, 5.0, 0.0});
}

void HandleSphMouseInput(SphFluid& fluid, SimulationKind simulation, ViewMode view) {
    if (simulation != SimulationKind::SphFluid || view != ViewMode::TwoD || !PhysicsMouseButtonPressed(MOUSE_BUTTON_LEFT)) return;
    fluid.Poke(SphScreenToWorld(PhysicsMousePosition()), {0.0, 5.0, 0.0});
}


void DrawCoolingGraph(const NewtonCooling& c) { DrawRectangle(GetScreenWidth() - 300, 220, 280, 150, Color{15, 21, 34, 232}); DrawText(TextFormat("T(t)  %5.1f C", c.temperature), GetScreenWidth() - 280, 240, 16, TemperatureColor(c.temperature)); DrawText(TextFormat("ambient %5.1f C", c.ambientTemperature), GetScreenWidth() - 280, 270, 16, Color{126, 235, 167, 255}); }
void DrawStringGraph(const StringSnapshot& s) { const int x = GetScreenWidth() - 300; const int y = 220; const int w = 280; const int h = 150; DrawRectangle(x, y, w, h, Color{15, 21, 34, 232}); DrawRectangleLines(x, y, w, h, Color{76, 91, 117, 255}); DrawText("string temperature", x + 12, y + 10, 15, Color{232, 238, 248, 255}); const float low = std::min(0.0f, s.ambientTemperature - 10.0f); const float high = std::max(100.0f, s.temperature + 10.0f); for (int i = 1; i < s.historyCount && i < static_cast<int>(s.kHistoryNodes); ++i) { const float x1 = x + 12 + static_cast<float>(i - 1) / 89.0f * (w - 24); const float x2 = x + 12 + static_cast<float>(i) / 89.0f * (w - 24); const float y1 = y + h - 20 - std::clamp((s.temperatureHistory[i - 1] - low) / (high - low), 0.0f, 1.0f) * (h - 55); const float y2 = y + h - 20 - std::clamp((s.temperatureHistory[i] - low) / (high - low), 0.0f, 1.0f) * (h - 55); DrawLineEx({x1, y1}, {x2, y2}, 2.0f, TemperatureColor(s.temperatureHistory[i])); } DrawText(TextFormat("%5.2f C   dQ %5.2f W", s.temperature, s.heatingPower), x + 12, y + h - 18, 12, TemperatureColor(s.temperature)); }
struct PianoLayout {
    int x = 40;
    int y = 0;
    int width = 0;
    int keyHeight = 65;
    float whiteWidth = 0.0f;
    float blackWidth = 0.0f;
    int blackHeight = 40;
};

constexpr int kPianoNoteCount = 13;
constexpr float kPianoFrequencies[kPianoNoteCount] = {
    261.63f, 277.18f, 293.66f, 311.13f, 329.63f, 349.23f, 369.99f,
    392.00f, 415.30f, 440.00f, 466.16f, 493.88f, 523.25f
};
constexpr const char* kPianoLabels[kPianoNoteCount] = {
    "C4", "C#4", "D4", "D#4", "E4", "F4", "F#4", "G4", "G#4", "A4", "A#4", "B4", "C5"
};
constexpr int kWhiteNoteIndices[8] = {0, 2, 4, 5, 7, 9, 11, 12};
constexpr int kBlackNoteIndices[5] = {1, 3, 6, 8, 10};
constexpr int kBlackAfterWhite[5] = {0, 1, 3, 4, 5};

PianoLayout GetPianoLayout() {
    PianoLayout layout;
    layout.y = GetScreenHeight() - 150;
    layout.width = std::max(160, GetScreenWidth() - 80);
    layout.whiteWidth = static_cast<float>(layout.width) / 8.0f;
    layout.blackWidth = layout.whiteWidth * 0.58f;
    return layout;
}

int PianoKeyAtPoint(Vector2 point) {
    const PianoLayout layout = GetPianoLayout();
    if (point.x < layout.x || point.x > layout.x + layout.width || point.y < layout.y || point.y > layout.y + layout.keyHeight) return -1;
    for (int i = 0; i < 5; ++i) {
        const float center = static_cast<float>(layout.x) + (static_cast<float>(kBlackAfterWhite[i]) + 1.0f) * layout.whiteWidth;
        const float left = center - layout.blackWidth * 0.5f;
        if (point.x >= left && point.x <= left + layout.blackWidth && point.y <= layout.y + layout.blackHeight) return kBlackNoteIndices[i];
    }
    const int white = std::clamp(static_cast<int>((point.x - static_cast<float>(layout.x)) / layout.whiteWidth), 0, 7);
    return kWhiteNoteIndices[white];
}

void HandlePianoInput(VibratingStringEngine& string, SimulationKind simulation, ViewMode view, int& selectedKey) {
    if (simulation != SimulationKind::VibratingString || view != ViewMode::Split) return;
    if (PhysicsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        const int key = PianoKeyAtPoint(PhysicsMousePosition());
        if (key >= 0) { selectedKey = key; string.PlayFrequency(kPianoFrequencies[key]); }
    }
    constexpr KeyboardKey whiteKeys[8] = {KEY_Z, KEY_X, KEY_C, KEY_V, KEY_B, KEY_N, KEY_M, KEY_COMMA};
    for (int i = 0; i < 8; ++i) {
        if (IsKeyPressed(whiteKeys[i])) { selectedKey = kWhiteNoteIndices[i]; string.PlayFrequency(kPianoFrequencies[selectedKey]); }
    }
}

void DrawPiano(int selectedKey) {
    const PianoLayout layout = GetPianoLayout();
    DrawText("PIANO / PLUCK", layout.x, layout.y - 19, 14, Color{196, 204, 219, 255});
    const Color white = Color{235, 238, 242, 255};
    const Color selectedWhite = Color{255, 211, 121, 255};
    for (int i = 0; i < 8; ++i) {
        const int keyX = static_cast<int>(layout.x + static_cast<float>(i) * layout.whiteWidth);
        const int keyWidth = static_cast<int>(std::ceil(layout.whiteWidth));
        const Color fill = kWhiteNoteIndices[i] == selectedKey ? selectedWhite : white;
        DrawRectangle(keyX, layout.y, keyWidth, layout.keyHeight, fill);
        DrawRectangleLines(keyX, layout.y, keyWidth, layout.keyHeight, Color{42, 50, 64, 255});
        DrawText(kPianoLabels[kWhiteNoteIndices[i]], keyX + keyWidth / 2 - 10, layout.y + layout.keyHeight - 20, 12, Color{22, 29, 42, 255});
    }
    for (int i = 0; i < 5; ++i) {
        const float center = static_cast<float>(layout.x) + (static_cast<float>(kBlackAfterWhite[i]) + 1.0f) * layout.whiteWidth;
        const int keyX = static_cast<int>(center - layout.blackWidth * 0.5f);
        const int keyWidth = static_cast<int>(std::ceil(layout.blackWidth));
        const Color fill = kBlackNoteIndices[i] == selectedKey ? Color{239, 142, 74, 255} : Color{28, 34, 48, 255};
        DrawRectangle(keyX, layout.y, keyWidth, layout.blackHeight, fill);
        DrawRectangleLines(keyX, layout.y, keyWidth, layout.blackHeight, Color{122, 132, 151, 255});
        DrawText(kPianoLabels[kBlackNoteIndices[i]], keyX + keyWidth / 2 - 12, layout.y + layout.blackHeight - 16, 11, Color{232, 238, 248, 255});
    }
    DrawText("mouse: all keys   keyboard: Z X C V B N M ,", layout.x + layout.width - 290, layout.y - 19, 12, Color{147, 160, 182, 255});
}

void DrawCombinedStringView(const StringSnapshot& s, int selectedKey) {
    const int width = GetScreenWidth();
    const int height = GetScreenHeight();
    const int divider = width / 2;
    const int top = 92;
    const int bottom = height - 175;
    const Color panel = {15, 21, 34, 232};
    const Color axis = {95, 105, 123, 255};
    DrawRectangle(25, 45, divider - 40, bottom - 25, panel);
    DrawRectangle(divider + 15, 45, width - divider - 40, bottom - 25, panel);
    DrawText("2D STRING MOTION", 45, 58, 18, Color{232, 238, 248, 255});
    DrawText("DFT / PICKUP SPECTRUM", divider + 30, 58, 18, Color{232, 238, 248, 255});

    const float stringLeft = 50.0f;
    const float stringRight = static_cast<float>(divider - 55);
    const float stringCenterY = (top + bottom) * 0.5f;
    const float normalizedAmplitude = std::max(0.001f, s.pluckAmplitude);
    for (std::size_t i = 0; i + 1 < s.kDisplayNodes; ++i) {
        const float x1 = stringLeft + (stringRight - stringLeft) * static_cast<float>(i) / 64.0f;
        const float x2 = stringLeft + (stringRight - stringLeft) * static_cast<float>(i + 1) / 64.0f;
        const float y1 = stringCenterY - (s.displacement[i] / normalizedAmplitude) * (bottom - top) * 0.32f;
        const float y2 = stringCenterY - (s.displacement[i + 1] / normalizedAmplitude) * (bottom - top) * 0.32f;
        DrawLineEx({x1, y1}, {x2, y2}, 3.0f, TemperatureColor(s.temperature));
    }
    DrawLine(static_cast<int>(stringLeft), static_cast<int>(stringCenterY), static_cast<int>(stringRight), static_cast<int>(stringCenterY), axis);
    DrawCircleV({stringLeft, stringCenterY}, 7.0f, Color{126, 235, 167, 255});
    DrawCircleV({stringRight, stringCenterY}, 7.0f, Color{126, 235, 167, 255});
    DrawText(TextFormat("visual scale %.1fx", s.displacementScale), 45, bottom - 22, 12, Color{147, 160, 182, 255});

    const int spectrumLeft = divider + 35;
    const int spectrumRight = width - 35;
    const int spectrumTop = top + 20;
    const int spectrumBottom = bottom - 20;
    DrawLine(spectrumLeft, spectrumBottom, spectrumRight, spectrumBottom, axis);
    DrawLine(spectrumLeft, spectrumTop, spectrumLeft, spectrumBottom, axis);
    const float binWidth = 48000.0f / 512.0f;
    for (std::size_t i = 1; i < s.kSpectrumBins; ++i) {
        const float magnitude = std::clamp((s.spectrumDb[i] + 80.0f) / 80.0f, 0.0f, 1.0f);
        const float x = spectrumLeft + static_cast<float>(i) / static_cast<float>(s.kSpectrumBins - 1) * static_cast<float>(spectrumRight - spectrumLeft);
        const Color color = i * binWidth < s.fundamentalFrequency * 2.0f ? Color{255, 188, 92, 255} : Color{120, 213, 255, 255};
        DrawLineEx({x, static_cast<float>(spectrumBottom)}, {x, spectrumBottom - magnitude * static_cast<float>(spectrumBottom - spectrumTop)}, 4.0f, color);
    }
    DrawText(TextFormat("peak %6.1f Hz   f1 %6.1f Hz", s.spectrumPeakFrequency, s.fundamentalFrequency), spectrumLeft, spectrumBottom + 12, 13, Color{120, 213, 255, 255});
    DrawPiano(selectedKey);
}
void DrawSpectrum(const StringSnapshot& s) {
    const int left = 70;
    const int top = 90;
    const int right = GetScreenWidth() - 70;
    const int bottom = GetScreenHeight() - 115;
    DrawRectangle(left - 22, top - 42, right - left + 44, bottom - top + 78, Color{15, 21, 34, 232});
    DrawRectangleLines(left - 22, top - 42, right - left + 44, bottom - top + 78, Color{76, 91, 117, 255});
    DrawText("DFT OF PLUCK PICKUP", left, top - 28, 22, Color{232, 238, 248, 255});
    DrawText(TextFormat("peak %6.1f Hz   %.1f dB    predicted f1 %6.1f Hz", s.spectrumPeakFrequency, s.spectrumPeakDb, s.fundamentalFrequency), left, top + 4, 16, Color{120, 213, 255, 255});
    DrawLine(left, bottom, right, bottom, Color{95, 105, 123, 255});
    DrawLine(left, top + 25, left, bottom, Color{95, 105, 123, 255});
    for (int mark = 0; mark <= 4; ++mark) {
        const float db = -20.0f * static_cast<float>(mark);
        const float y = bottom - (1.0f - static_cast<float>(mark) / 4.0f) * static_cast<float>(bottom - top - 25);
        DrawLine(left, static_cast<int>(y), right, static_cast<int>(y), Color{52, 61, 77, 255});
        DrawText(TextFormat("%+.0f dB", db), 10, static_cast<int>(y - 7), 12, Color{147, 160, 182, 255});
    }
    const float binWidth = 48000.0f / 512.0f;
    for (std::size_t i = 1; i < s.kSpectrumBins; ++i) {
        const float magnitude = std::clamp((s.spectrumDb[i] + 80.0f) / 80.0f, 0.0f, 1.0f);
        const float x = left + static_cast<float>(i) / static_cast<float>(s.kSpectrumBins - 1) * static_cast<float>(right - left);
        const Color bar = i * binWidth < s.fundamentalFrequency * 2.0f ? Color{255, 188, 92, 255} : Color{120, 213, 255, 255};
        DrawLineEx({x, static_cast<float>(bottom)}, {x, bottom - magnitude * static_cast<float>(bottom - top - 25)}, 5.0f, bar);
    }
    DrawText("0 Hz", left - 12, bottom + 12, 12, Color{147, 160, 182, 255});
    DrawText(TextFormat("%.1f kHz", 63.0f * binWidth / 1000.0f), right - 35, bottom + 12, 12, Color{147, 160, 182, 255});
}

void DrawHud(const SpringMass& spring, const BouncingBall& ball, const NewtonCooling& cooling, const StringSnapshot& string, const FlipFluid& fluid, const SphFluid& sph, const PowderScene& powder, const BuoyancySceneSnapshot& buoyancy, SimulationKind simulation, ViewMode view, bool paused, bool muted) {
    const bool isSpring = simulation == SimulationKind::SpringMass;
    const bool isBall = simulation == SimulationKind::BouncingBall;
    const bool isCooling = simulation == SimulationKind::NewtonCooling;
    const bool isString = simulation == SimulationKind::VibratingString;
    const bool isFlip = simulation == SimulationKind::FlipFluid;
    const bool isSph = simulation == SimulationKind::SphFluid;
    const bool isPowder = simulation == SimulationKind::Powder;
    const bool isBuoyancy = simulation == SimulationKind::Buoyancy;
    const int number = isSpring ? 1 : (isBall ? 2 : (isCooling ? 3 : (isString ? 4 : (isFlip ? 5 : (isSph ? 6 : (isPowder ? 7 : 8))))));
    const char* name = isSpring ? "SPRING / MASS" : (isBall ? "BOUNCING BALL" : (isCooling ? "NEWTON COOLING" : (isString ? "VIBRATING STRING" : (isFlip ? "FLIP FLUID" : (isSph ? "SPH FLUID" : (isPowder ? "POWDER" : "BUOYANCY"))))));
    const int panelHeight = 270;
    DrawRectangle(20, 20, 560, panelHeight, Color{15, 21, 34, 232});
    DrawRectangleLines(20, 20, 560, panelHeight, Color{76, 91, 117, 255});
    DrawText("PHYSICS SANDBOX", 38, 35, 22, Color{232, 238, 248, 255});
    DrawText(TextFormat("SIM %d  %s", number, name), 38, 70, 18, Color{120, 213, 255, 255});
    DrawText(view == ViewMode::ThreeD ? "3D" : (view == ViewMode::Spectrum ? "DFT" : (view == ViewMode::Split ? "SPLIT" : "2D")), 490, 70, 18, Color{196, 204, 219, 255});

    int statusY = 231;
    if (isSpring) {
        const float speed = Length(spring.velocity);
        DrawText(TextFormat("k %5.1f   mass %4.2f   b %4.2f", spring.springConstant, spring.mass, spring.dampingCoefficient), 38, 103, 17, Color{120, 213, 255, 255});
        DrawText(TextFormat("position %+.2f %+.2f %+.2f", spring.position.x, spring.position.y, spring.position.z), 38, 131, 16, Color{196, 204, 219, 255});
        DrawText(TextFormat("speed %5.2f m/s", speed), 38, 159, 15, Color{196, 204, 219, 255});
        DrawText("basic spring-mass oscillator", 38, 187, 14, Color{196, 204, 219, 255});
    } else if (isBall) {
        statusY = 231;
        DrawText(TextFormat("gravity %4.1f m/s2   radius %4.2f m", ball.gravity, ball.radius), 38, 103, 17, Color{120, 213, 255, 255});
        DrawText(TextFormat("height %4.2f m", ball.position.y), 38, 131, 16, Color{196, 204, 219, 255});
    } else if (isCooling) {
        statusY = 179;
        DrawText(TextFormat("T %5.1f C   ambient %5.1f C", cooling.temperature, cooling.ambientTemperature), 38, 103, 17, TemperatureColor(cooling.temperature));
        DrawText(TextFormat("k %4.2f /s   time %4.1f s", cooling.coolingConstant, cooling.elapsed), 38, 131, 16, Color{196, 204, 219, 255});
    } else if (isFlip) {
        statusY = 239;
        DrawText(TextFormat("particles %4d   alpha %4.2f   grid %dx%dx%d", static_cast<int>(fluid.Particles().size()), fluid.GetFlipRatio(), FlipFluid::kGridWidth, FlipFluid::kGridHeight, FlipFluid::kGridDepth), 38, 103, 16, Color{120, 213, 255, 255});
        DrawText(TextFormat("max div %8.3f   RMS div %8.3f", fluid.GetMaxDivergence(), fluid.GetRmsDivergence()), 38, 131, 15, Color{196, 204, 219, 255});
        DrawText(TextFormat("gravity %5.2f   obstacle %s   pressure iters %d", fluid.GetGravity(), fluid.SolidObstacleEnabled() ? "ON" : "OFF", fluid.GetPressureIterations()), 38, 159, 14, Color{196, 204, 219, 255});
        DrawText(TextFormat("buoyant y %5.2f   vy %+5.2f   submerged %3.0f%%", fluid.BuoyantBox().center.y, fluid.GetBuoyantBoxVerticalVelocity(), 100.0f * fluid.GetBuoyantBoxSubmergedFraction()), 38, 187, 14, Color{126, 235, 167, 255});
        DrawText(TextFormat("t %6.2f s   internal substeps %d", fluid.GetSimulatedTime(), fluid.GetSubsteps()), 38, 215, 14, Color{196, 204, 219, 255});
    } else if (isSph) {
        statusY = 179;
        DrawText(TextFormat("particles %4d   h %4.2f   neighbors %4.1f", static_cast<int>(sph.Particles().size()), sph.GetSmoothingRadius(), sph.GetAverageNeighbors()), 38, 103, 16, Color{120, 213, 255, 255});
        DrawText(TextFormat("stiffness %6.0f   viscosity %4.2f", sph.GetStiffness(), sph.GetViscosity()), 38, 131, 15, Color{196, 204, 219, 255});
        DrawText(TextFormat("density error %6.3f   max speed %5.2f", sph.GetMaxDensityError(), sph.GetMaxSpeed()), 38, 159, 15, Color{196, 204, 219, 255});
        DrawText(TextFormat("gravity %5.2f   t %6.2f s   substeps %d", sph.GetGravity(), sph.GetSimulatedTime(), sph.GetSubsteps()), 38, 187, 15, Color{196, 204, 219, 255});
    } else if (isPowder) {
        statusY = 179;
        DrawText(TextFormat("cells %5d   selected %s", powder.OccupiedCount(), powder.SelectedMaterialName()), 38, 103, 16, Color{120, 213, 255, 255});
        DrawText(TextFormat("sand %4d   water %4d   oil %4d", powder.Count(PowderScene::Material::Sand), powder.Count(PowderScene::Material::Water), powder.Count(PowderScene::Material::Oil)), 38, 131, 15, Color{196, 204, 219, 255});
        DrawText(TextFormat("fire %4d   smoke %4d   steam %4d", powder.Count(PowderScene::Material::Fire), powder.Count(PowderScene::Material::Smoke), powder.Count(PowderScene::Material::Steam)), 38, 159, 15, Color{238, 196, 128, 255});
        DrawText("grid-based powder / liquid / gas sandbox", 38, 187, 15, Color{196, 204, 219, 255});
    } else if (isBuoyancy) {
        statusY = 231;
        DrawText(TextFormat("mass %6.1f kg   body rho %6.1f   water rho %6.1f", buoyancy.mass, buoyancy.bodyDensity, buoyancy.waterDensity), 38, 103, 15, Color{120, 213, 255, 255});
        if (buoyancy.full3D) {
            const double speed = std::sqrt(buoyancy.velocity3D.x * buoyancy.velocity3D.x + buoyancy.velocity3D.y * buoyancy.velocity3D.y + buoyancy.velocity3D.z * buoyancy.velocity3D.z);
            const double angularSpeed = std::sqrt(buoyancy.angularVelocity3D.x * buoyancy.angularVelocity3D.x + buoyancy.angularVelocity3D.y * buoyancy.angularVelocity3D.y + buoyancy.angularVelocity3D.z * buoyancy.angularVelocity3D.z);
            DrawText(TextFormat("3D V %5.3f m3   buoyancy %7.1f N", buoyancy.submergedVolume3D, buoyancy.buoyancyForce3D), 38, 131, 15, Color{126, 235, 167, 255});
            DrawText(TextFormat("|torque| %7.2f Nm   |omega| %6.2f rad/s", buoyancy.torqueMagnitude3D, angularSpeed), 38, 159, 14, Color{196, 204, 219, 255});
            DrawText(TextFormat("speed %5.2f m/s   energy %7.1f J   drag %s", speed, buoyancy.mechanicalEnergy, buoyancy.dragEnabled ? "ON" : "OFF"), 38, 187, 14, Color{196, 204, 219, 255});
        } else {
            DrawText(TextFormat("submerged V %5.3f m3   buoyancy %7.1f N", buoyancy.submergedVolume, buoyancy.buoyancyForce), 38, 131, 15, Color{126, 235, 167, 255});
            DrawText(TextFormat("torque %+7.2f Nm   angle %+6.2f rad   omega %+6.2f", buoyancy.torque, buoyancy.angle, buoyancy.angularVelocity), 38, 159, 14, Color{196, 204, 219, 255});
            DrawText(TextFormat("speed %5.2f m/s   energy %7.1f J   drag %s", std::sqrt(buoyancy.velocity.x * buoyancy.velocity.x + buoyancy.velocity.y * buoyancy.velocity.y), buoyancy.mechanicalEnergy, buoyancy.dragEnabled ? "ON" : "OFF"), 38, 187, 14, Color{196, 204, 219, 255});
        }
    } else {
        DrawText(TextFormat("T %5.2f C   ambient %5.2f C   dT %+5.2f", string.temperature, string.ambientTemperature, string.temperature - string.ambientTemperature), 38, 103, 16, TemperatureColor(string.temperature));
        DrawText(TextFormat("f1 %6.1f Hz   wave speed %6.1f m/s", string.fundamentalFrequency, string.waveSpeed), 38, 131, 16, Color{120, 213, 255, 255});
        DrawText(TextFormat("heat %5.3f W   cool %5.3f W   Q %6.3f J", string.heatingPower, string.coolingPower, string.generatedHeat), 38, 159, 15, Color{238, 196, 128, 255});
        DrawText(TextFormat("Emech %6.4f J   residual %+7.4f J", string.mechanicalEnergy, string.numericalResidual), 38, 187, 15, Color{196, 204, 219, 255});
    }
    DrawText(paused ? "PAUSED" : (isString && muted ? "RUNNING / MUTED" : "RUNNING"), 38, statusY, 16, paused ? Color{255, 194, 92, 255} : Color{126, 235, 167, 255});

    const int footerY = GetScreenHeight() - 76;
    DrawRectangle(20, footerY, GetScreenWidth() - 40, 56, Color{15, 21, 34, 232});
    if (isSpring) {
        DrawText("1-6/TAB simulations   V 2D/3D   WASD + SPACE/CTRL anchor", 36, footerY + 9, 15, Color{218, 225, 237, 255});
        DrawText("[ ] k  -/= mass  ,/. damping  R reset  P pause", 36, footerY + 31, 14, Color{147, 160, 182, 255});
    } else if (isBall) {
        DrawText("1-6/TAB simulations   V 2D/3D   [ ] gravity   -/= radius", 36, footerY + 9, 15, Color{218, 225, 237, 255});
        DrawText("R reset   P pause   BACKSPACE restore constants   right-drag orbit", 36, footerY + 31, 14, Color{147, 160, 182, 255});
    } else if (isCooling) {
        DrawText("1-6/TAB simulations   V 2D/3D   UP/DOWN object temperature", 36, footerY + 9, 15, Color{218, 225, 237, 255});
        DrawText("[ ] cooling k   -/= ambient temperature   R reset   P pause", 36, footerY + 31, 14, Color{147, 160, 182, 255});
        DrawCoolingGraph(cooling);
    } else if (isFlip) {
        DrawText("1-6/TAB simulations   V 2D/3D   left click impulse   SPACE splash", 36, footerY + 9, 15, Color{218, 225, 237, 255});
        DrawText("[ ] FLIP alpha   -/= gravity   B obstacle   IJKL/UO move box   F front/orbit   N add 64   R reset", 36, footerY + 31, 14, Color{147, 160, 182, 255});
    } else if (isSph) {
        DrawText("1-6/TAB simulations   V 2D/3D   left click impulse   SPACE splash", 36, footerY + 9, 15, Color{218, 225, 237, 255});
        DrawText("[ ] stiffness   -/= viscosity   ,/. gravity   N add 64   R reset   P pause", 36, footerY + 31, 14, Color{147, 160, 182, 255});
    } else if (isPowder) {
        if (view == ViewMode::ThreeD) {
            DrawText("1-7/TAB simulations   V 2D/3D   click POWDER TOOLS palette", 36, footerY + 9, 15, Color{218, 225, 237, 255});
            DrawText("left drag paint   SHIFT+left erase   right drag orbit   wheel zoom   N source", 36, footerY + 31, 14, Color{147, 160, 182, 255});
        } else {
            DrawText("1-7/TAB simulations   V 2D/3D   click POWDER TOOLS palette", 36, footerY + 9, 15, Color{218, 225, 237, 255});
            DrawText("left drag paint   right drag erase   N add 3D source   SPACE impulse   R reset   P pause", 36, footerY + 31, 14, Color{147, 160, 182, 255});
        }
    } else if (isBuoyancy) {
        DrawText("1-8/TAB simulations   V 2D/3D   SPACE impulse   right-drag orbit", 36, footerY + 9, 15, Color{218, 225, 237, 255});
        DrawText("[ ] water rho   -/= mass   ,/. drag   D toggle drag   G rotation   R reset   P pause", 36, footerY + 31, 14, Color{147, 160, 182, 255});
    } else {
        DrawText("1-6/TAB simulations   V 2D/3D/DFT/SPLIT   SPACE pluck   T hot start   M mute", 36, footerY + 9, 15, Color{218, 225, 237, 255});
        DrawText("[ ] tension  -/= damping  ,/. pluck  H/J eta  K/L hA  Q/E pickup  R reset  click piano or Z/X/C/V/B/N/M/,", 36, footerY + 31, 14, Color{147, 160, 182, 255});
        if (view == ViewMode::Spectrum) DrawText(TextFormat("DFT peak %6.1f Hz  predicted f1 %6.1f Hz", string.spectrumPeakFrequency, string.fundamentalFrequency), 36, footerY + 31, 14, Color{147, 160, 182, 255});
        else if (view != ViewMode::Split) DrawStringGraph(string);
    }
}

struct AppState {
    SpringMass spring;
    BouncingBall ball;
    NewtonCooling cooling;
    VibratingStringEngine string;
    FlipFluid fluid;
    SphFluid sph;
    PowderScene powder;
    BuoyancyScene buoyancy;
    ClothElasticSolidsScene cloth;
    BeamBendingScene beam;
    BridgeBuilderScene bridge;
    OrbitCamera camera;
    bool paused = false;
    bool muted = false;
    bool menuOpen = true;
    bool flipFrontView = false;
    SimulationKind simulation = SimulationKind::SpringMass;
    SimulationKind previousSimulation = SimulationKind::SpringMass;
    ViewMode view = ViewMode::ThreeD;
    int menuCursor = 0;
    int powderBrushRadius = 4;
    int selectedPianoKey = -1;
    float accumulator = 0.0f;
    bool shutdownRequested = false;
};
} // namespace

void RunFrame(AppState& app) {
    if (IsKeyPressed(KEY_F1)) {
        app.menuOpen = !app.menuOpen;
        if (app.menuOpen) app.menuCursor = SimulationMenuIndex(app.simulation);
    }
    if (app.menuOpen) {
        HandleSimulationMenuInput(app.simulation, app.menuOpen, app.menuCursor);
    } else {
        HandleInput(app.spring, app.ball, app.cooling, app.string, app.fluid, app.sph, app.powder, app.buoyancy,
                    app.cloth, app.beam, app.bridge, app.simulation, app.view, app.paused, app.muted, app.camera, app.flipFrontView);
        app.powder.SetThreeDimensional(app.simulation == SimulationKind::Powder && app.view == ViewMode::ThreeD);
        app.buoyancy.SetThreeDimensional(app.simulation == SimulationKind::Buoyancy && app.view == ViewMode::ThreeD);
        HandlePianoInput(app.string, app.simulation, app.view, app.selectedPianoKey);
        HandleFlipMouseInput(app.fluid, app.simulation, app.view);
        HandleSphMouseInput(app.sph, app.simulation, app.view);
        HandlePowderMouseInput(app.powder, app.simulation, app.view, app.powderBrushRadius, app.camera);
        if (IsKeyPressed(KEY_N) && app.paused) {
            if (app.simulation == SimulationKind::ClothElasticSolids) app.cloth.Step(kFixedTimeStep);
            else if (app.simulation == SimulationKind::BeamBending) app.beam.Step(kFixedTimeStep);
            else if (app.simulation == SimulationKind::BridgeBuilder) app.bridge.Step(kFixedTimeStep);
        }
        if (!app.paused) {
            app.accumulator = std::min(app.accumulator + GetFrameTime(), 0.1f);
            while (app.accumulator >= kFixedTimeStep) {
                if (app.simulation == SimulationKind::SpringMass) StepSpringMass(app.spring, kFixedTimeStep);
                else if (app.simulation == SimulationKind::BouncingBall) StepBall(app.ball, kFixedTimeStep);
                else if (app.simulation == SimulationKind::NewtonCooling) StepCooling(app.cooling, kFixedTimeStep);
                else if (app.simulation == SimulationKind::VibratingString) app.string.Step(kFixedTimeStep);
                else if (app.simulation == SimulationKind::FlipFluid) app.fluid.Step(kFixedTimeStep);
                else if (app.simulation == SimulationKind::SphFluid) app.sph.Step(kFixedTimeStep);
                else if (app.simulation == SimulationKind::Powder) app.powder.Step(kFixedTimeStep);
                else if (app.simulation == SimulationKind::Buoyancy) app.buoyancy.Step(kFixedTimeStep);
                else if (app.simulation == SimulationKind::ClothElasticSolids) app.cloth.Step(kFixedTimeStep);
                else if (app.simulation == SimulationKind::BeamBending) app.beam.Step(kFixedTimeStep);
                else app.bridge.Step(kFixedTimeStep);
                app.accumulator -= kFixedTimeStep;
            }
        }
    }
    if (app.simulation != app.previousSimulation) {
        app.accumulator = 0.0f;
        if (app.simulation == SimulationKind::BridgeBuilder) app.view = ViewMode::TwoD;
        app.previousSimulation = app.simulation;
    }

    const StringSnapshot stringSnapshot = app.string.ReadSnapshot();
    BeginDrawing();
    ClearBackground(Color{10, 15, 27, 255});
    if (app.view == ViewMode::Split && app.simulation == SimulationKind::VibratingString) DrawCombinedStringView(stringSnapshot, app.selectedPianoKey);
    else if (app.view == ViewMode::Spectrum && app.simulation == SimulationKind::VibratingString) DrawSpectrum(stringSnapshot);
    else if (app.view == ViewMode::ThreeD) {
        const Camera3D cam = app.simulation == SimulationKind::FlipFluid ? MakeFlipCamera(app.camera, app.flipFrontView) :
            (app.simulation == SimulationKind::ClothElasticSolids ? MakeCamera(app.camera, app.cloth.FocusTarget()) :
            (app.simulation == SimulationKind::BeamBending ? MakeCamera(app.camera, app.beam.FocusTarget()) :
            (app.simulation == SimulationKind::BridgeBuilder ? MakeCamera(app.camera, app.bridge.FocusTarget()) : MakeCamera(app.camera))));
        BeginMode3D(cam);
        if (app.simulation == SimulationKind::SpringMass) DrawSpring3D(app.spring);
        else if (app.simulation == SimulationKind::BouncingBall) DrawBall3D(app.ball);
        else if (app.simulation == SimulationKind::NewtonCooling) DrawCooling3D(app.cooling);
        else if (app.simulation == SimulationKind::FlipFluid) DrawFlip3D(app.fluid);
        else if (app.simulation == SimulationKind::SphFluid) DrawSph3D(app.sph);
        else if (app.simulation == SimulationKind::Powder) DrawPowder3D(app.powder);
        else if (app.simulation == SimulationKind::Buoyancy) DrawBuoyancy3D(app.buoyancy.ReadSnapshot());
        else if (app.simulation == SimulationKind::ClothElasticSolids) app.cloth.Draw3D(cam);
        else if (app.simulation == SimulationKind::BeamBending) app.beam.Draw3D(cam);
        else if (app.simulation == SimulationKind::BridgeBuilder) app.bridge.Draw3D(cam);
        else DrawVibratingString3D(stringSnapshot);
        EndMode3D();
    } else if (app.simulation == SimulationKind::SpringMass) DrawSpring2D(app.spring);
    else if (app.simulation == SimulationKind::BouncingBall) DrawBall2D(app.ball);
    else if (app.simulation == SimulationKind::NewtonCooling) DrawCooling2D(app.cooling);
    else if (app.simulation == SimulationKind::FlipFluid) DrawFlip2D(app.fluid);
    else if (app.simulation == SimulationKind::SphFluid) DrawSph2D(app.sph);
    else if (app.simulation == SimulationKind::Powder) DrawPowder2D(app.powder);
    else if (app.simulation == SimulationKind::Buoyancy) DrawBuoyancy2D(app.buoyancy.ReadSnapshot());
    else if (app.simulation == SimulationKind::ClothElasticSolids) app.cloth.Draw2D();
    else if (app.simulation == SimulationKind::BeamBending) app.beam.Draw2D();
    else if (app.simulation == SimulationKind::BridgeBuilder) app.bridge.Draw2D();
    else DrawVibratingString2D(stringSnapshot);

    const BuoyancySceneSnapshot buoyancySnapshot = app.buoyancy.ReadSnapshot();
    if (app.simulation == SimulationKind::ClothElasticSolids) app.cloth.DrawHud(app.paused);
    else if (app.simulation == SimulationKind::BeamBending) app.beam.DrawHud(app.paused);
    else if (app.simulation == SimulationKind::BridgeBuilder) app.bridge.DrawHud(app.paused);
    else DrawHud(app.spring, app.ball, app.cooling, stringSnapshot, app.fluid, app.sph, app.powder, buoyancySnapshot,
                 app.simulation, app.view, app.paused, app.muted);
    if (app.simulation == SimulationKind::Powder && !app.menuOpen) DrawPowderPalette(app.powder, app.powderBrushRadius);
    if (app.menuOpen) DrawSimulationMenu(app.simulation, app.menuCursor);
    EndDrawing();
    app.shutdownRequested = WindowShouldClose();
}

#ifdef __EMSCRIPTEN__
AppState* gPhysicsApp = nullptr;
void RunPhysicsWebFrame() {
    if (gPhysicsApp == nullptr || gPhysicsApp->shutdownRequested) {
        emscripten_cancel_main_loop();
        return;
    }
    RunFrame(*gPhysicsApp);
}
#endif

int main() {
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(1280, 800, "Physics Sandbox");
#ifdef __EMSCRIPTEN__
    InstallPhysicsMouseBridge();
    SetTargetFPS(0);
#else
    SetTargetFPS(60);
#endif
    static AppState app;
#ifdef __EMSCRIPTEN__
    gPhysicsApp = &app;
    emscripten_set_main_loop(RunPhysicsWebFrame, 0, 1);
#else
    app.string.StartAudio();
    while (!app.shutdownRequested) RunFrame(app);
    app.string.StopAudio();
    CloseWindow();
#endif
    return 0;
}
