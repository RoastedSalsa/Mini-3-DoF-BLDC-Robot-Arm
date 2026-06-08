#pragma once
//
// config.h — centralized hardware/control constants for the Mini 3-DoF BLDC arm.
//
// Values are EXTRACTED verbatim from the authoritative sketch (src/full.cpp) so
// that every symbol resolves to exactly the same value as the literal it
// replaces. Floats mirror the original (float-typed) assignments; pin tokens use
// `constexpr auto` to preserve whatever type the core gives PA8/PB7/... .
//
// NOTE: sensor_direction (Direction::CW for all three joints in full.cpp) is left
// inline in the sketches on purpose — it is a SimpleFOC enum, and keeping this
// header free of <SimpleFOC.h> lets as5600.cpp share the I2C pins without pulling
// the motor library in.
//
#include <Arduino.h>   // so PB7, PA15, ... resolve here

// ======================= I2C / AS5600 magnetic sensors =======================
// Each sensor sits on its own hardware I2C bus (SDA, SCL).
constexpr auto S1_SDA_PIN = PB7;   // sensor 1 — I2C1 HW
constexpr auto S1_SCL_PIN = PA15;
constexpr auto S2_SDA_PIN = PC9;   // sensor 2 — I2C3 HW
constexpr auto S2_SCL_PIN = PC8;
constexpr auto S3_SDA_PIN = PC7;   // sensor 3 — I2C4 HW
constexpr auto S3_SCL_PIN = PC6;
constexpr uint32_t I2C_CLOCK_HZ = 400000;   // try 100000 if issues

// ============================= BLDC driver pins ==============================
// 3-PWM phase pins (A, B, C) + enable, per joint.
constexpr auto M1_PH_A = PA8;   // driver1 — TIM1
constexpr auto M1_PH_B = PA9;
constexpr auto M1_PH_C = PA10;
constexpr auto M1_EN   = PC5;

constexpr auto M2_PH_A = PA0;   // driver2 — TIM2
constexpr auto M2_PH_B = PA1;
constexpr auto M2_PH_C = PB10;
constexpr auto M2_EN   = PB12;

constexpr auto M3_PH_A = PA6;   // driver3 — TIM3
constexpr auto M3_PH_B = PB5;
constexpr auto M3_PH_C = PB0;
constexpr auto M3_EN   = PB13;

// ============================== Motor pole pairs =============================
// NOTE: joint 3 differs from joints 1 & 2.
constexpr int M1_POLE_PAIRS = 11;
constexpr int M2_POLE_PAIRS = 11;
constexpr int M3_POLE_PAIRS = 7;

// ================================ Power / limits =============================
constexpr float SUPPLY_VOLTAGE = 12.0;   // battery / PSU voltage
constexpr float VOLTAGE_LIMIT  = 12.0;   // FOC voltage limit

// ============================ Per-motor control gains ========================
// Kept per-motor (values can differ between joints).
constexpr float M1_VEL_P       = 0.5;
constexpr float M1_VEL_I       = 0.1;
constexpr float M1_ANGLE_P     = 20;
constexpr float M1_VEL_LIMIT   = 10;
constexpr float M1_LPF_TF      = 0.01;
constexpr float M1_OUTPUT_RAMP = 1000;

constexpr float M2_VEL_P       = 0.3;
constexpr float M2_VEL_I       = 0.1;
constexpr float M2_ANGLE_P     = 20;
constexpr float M2_VEL_LIMIT   = 10;
constexpr float M2_LPF_TF      = 0.01;
constexpr float M2_OUTPUT_RAMP = 1000;

constexpr float M3_VEL_P       = 0.5;
constexpr float M3_VEL_I       = 0.1;
constexpr float M3_ANGLE_P     = 20;
constexpr float M3_VEL_LIMIT   = 10;
constexpr float M3_LPF_TF      = 0.01;
constexpr float M3_OUTPUT_RAMP = 1000;

// ===================== Per-motor calibration (electrical) ====================
// zero_electric_angle from full.cpp. (sensor_direction is Direction::CW for all
// three; left inline in the sketches — see header note.)
constexpr float M1_ZERO_ELEC_ANGLE = 6.002470;
constexpr float M2_ZERO_ELEC_ANGLE = 3.529690;
constexpr float M3_ZERO_ELEC_ANGLE = 2.21200;

// ============================== Arm kinematics ===============================
// Link lengths [m]. Typed float to match the original float l1/l2/l3 (preserves
// the float arithmetic in the IK expressions).
constexpr float L1 = 0.0488;
constexpr float L2 = 0.2;
constexpr float L3 = 0.2;

// Joint-2 gear ratio: the inline `* 3` applied to theta2 in full.cpp.
constexpr int J2_GEAR_RATIO = 3;

// ===================== Crude homing reference offsets ========================
// PLACEHOLDERS — these are the hand-tuned `zeroed*` constants from full.cpp's
// startup block. Centralized as-is; to be replaced when real homing lands.
constexpr float HOMING_OFFSET_J1_UPPER = 6.487;   // sensor1 angle in (3.14, 6.28)
constexpr float HOMING_OFFSET_J1_LOWER = 0.172;   // sensor1 angle in (-3.14, 3.14]
constexpr float HOMING_OFFSET_J2_NEG   = -6.7;    // sensor2 angle < 1
constexpr float HOMING_OFFSET_J2_POS   = 4.34;    // sensor2 angle > 1
constexpr float HOMING_OFFSET_J3       = 3.5;
