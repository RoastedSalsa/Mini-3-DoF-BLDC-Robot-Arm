#pragma once
#include <Arduino.h>

// Inverse kinematics for the 3-DoF arm.
//
// Given a Cartesian target (x, y, z), writes the three joint angles into
// t1, t2, t3 (radians). Link lengths come from config.h (L1, L2, L3).
//
// The expressions and signs are copied verbatim from the original full.cpp loop
// math; behavior is unchanged. Returns true.
bool ik(float x, float y, float z, float& t1, float& t2, float& t3);
