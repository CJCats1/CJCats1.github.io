#pragma once

#include "raylib.h"

#ifdef __EMSCRIPTEN__
Vector2 PhysicsMousePosition();
bool PhysicsMouseButtonPressed(int button);
bool PhysicsMouseButtonDown(int button);
bool PhysicsMouseButtonReleased(int button);
Vector2 PhysicsMouseDelta();
float PhysicsMouseWheelMove();
#else
inline Vector2 PhysicsMousePosition() { return GetMousePosition(); }
inline bool PhysicsMouseButtonPressed(int button) { return IsMouseButtonPressed(button); }
inline bool PhysicsMouseButtonDown(int button) { return IsMouseButtonDown(button); }
inline bool PhysicsMouseButtonReleased(int button) { return IsMouseButtonReleased(button); }
inline Vector2 PhysicsMouseDelta() { return GetMouseDelta(); }
inline float PhysicsMouseWheelMove() { return GetMouseWheelMove(); }
#endif
