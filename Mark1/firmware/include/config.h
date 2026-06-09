#pragma once
//
// config.h — centralized hardware/control constants for the Mini 3-DoF BLDC arm.
//
// NOTE: sensor_direction (Direction::CW for all three joints in full.cpp) is left out of here to account for AS5600.cpp
//
#include <Arduino.h>   // pin names are here

// ======================= I2C / AS5600 magnetic sensors =======================
// Each sensor sits on its own hardware I2C bus (SDA, SCL).
constexpr auto S1_SDA_PIN = PB7;   // sensor 1 — I2C1 HW
constexpr auto S1_SCL_PIN = PA15;
constexpr auto S2_SDA_PIN = PC9;   // sensor 2 — I2C3 HW
constexpr auto S2_SCL_PIN = PC8;
constexpr auto S3_SDA_PIN = PC7;   // sensor 3 — I2C4 HW
constexpr auto S3_SCL_PIN = PC6;
constexpr uint32_t I2C_CLOCK_HZ = 400000;

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
constexpr int M1_POLE_PAIRS = 11; //G3506 Ipower
constexpr int M2_POLE_PAIRS = 11; //G3506 Ipower  
constexpr int M3_POLE_PAIRS = 7;    //GM2804 random Aliexpress

// ================================ Power / limits =============================
constexpr float SUPPLY_VOLTAGE = 12.0;   // battery / PSU voltage
constexpr float VOLTAGE_LIMIT  = 12.0;   // FOC voltage limit

// ============================ Per-motor control gains ========================
// Cascaded velocity + angle PID, per joint. Can be tunned live using the `pid_tuner` tool

constexpr float M1_VEL_P       = 0.3;
constexpr float M1_VEL_I       = 0.1;
constexpr float M1_ANGLE_P     = 10;
constexpr float M1_ANGLE_I     = 1.0;
constexpr float M1_ANGLE_D     = 0.05;
constexpr float M1_VEL_LIMIT   = 100;
constexpr float M1_LPF_TF      = 0.01;
constexpr float M1_OUTPUT_RAMP = 1000;

constexpr float M2_VEL_P       = 0.3;
constexpr float M2_VEL_I       = 0.1;
constexpr float M2_ANGLE_P     = 10;
constexpr float M2_ANGLE_I     = 1.0;
constexpr float M2_ANGLE_D     = 0.05;
constexpr float M2_VEL_LIMIT   = 100;
constexpr float M2_LPF_TF      = 0.01;
constexpr float M2_OUTPUT_RAMP = 1000;

constexpr float M3_VEL_P       = 0.3;
constexpr float M3_VEL_I       = 0.1;
constexpr float M3_ANGLE_P     = 30;
constexpr float M3_ANGLE_I     = 5.0;
constexpr float M3_ANGLE_D     = 0.01;
constexpr float M3_VEL_LIMIT   = 100;
constexpr float M3_LPF_TF      = 0.01;
constexpr float M3_OUTPUT_RAMP = 1000;

// ===================== Per-motor calibration (electrical) ====================
// REDO each time motors/sensors are reassembled.
constexpr float M1_ZERO_ELEC_ANGLE = 1.2732;
constexpr float M2_ZERO_ELEC_ANGLE = 0.61210;
constexpr float M3_ZERO_ELEC_ANGLE = 2.2273;

// ============================== Arm kinematics ===============================
// Link lengths [m].
constexpr float L1 = 0.0488;
constexpr float L2 = 0.2;
constexpr float L3 = 0.2;

// Joint-2 gear ratio
constexpr int J2_GEAR_RATIO = 3;

// ====================== Per-joint software travel limits =====================
// Applied to the FINAL motor target (sensor frame, radians) before move(). 
// Keeps IK / serial commands from driving a joint into a mechanical hard-stop.
constexpr float J1_MIN = 2.0,  J1_MAX = 5.3;
constexpr float J2_MIN = -4.0, J2_MAX = 1.5;
constexpr float J3_MIN = 0.8,  J3_MAX = 6.0;

// =============================== Trajectory ==================================
// Continuous Cartesian waypoint loop (see lib/Trajectory). The arm cycles
// through these waypoints, interpolating position over a fixed time per segment.
constexpr float   TRAJ_PX[]            = {0.2, 0.2,  0.2,  0.2};
constexpr float   TRAJ_PY[]            = {0.1, 0.1, -0.1, -0.1};
constexpr float   TRAJ_PZ[]            = {0.15, 0.25, 0.25, 0.15};
constexpr uint8_t TRAJ_COUNT           = 4;
constexpr float   TRAJ_SEGMENT_TIME_S  = 2.0;   // seconds per segment

// =============================== Telemetry ===================================
// Periodic serial monitoring (see lib/Telemetry). One CSV sample per interval.
constexpr unsigned long TELEMETRY_PERIOD_MS = 200;
