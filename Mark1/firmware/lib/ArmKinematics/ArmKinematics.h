#pragma once
#include <Arduino.h>

// Kinematics for the 3-DoF arm.


// Inverse kinematics given a Cartesian target (x, y, z), writes the three joint angles into
// t1, t2, t3 (radians). Link lengths come from config.h (L1, L2, L3).

bool ik(float x, float y, float z, float& t1, float& t2, float& t3);
// Forward kinematics: joint angles (rad) -> Cartesian hand position (m). Inverse of ik().
bool fk(float& x, float& y, float& z, float t1, float t2, float t3);