#pragma once
#include <numbers>

namespace pendulum {
constexpr float PI_VAL = 3.14159265358979323846f;

// ==================== ARM GEOMETRY ==================== 
constexpr float SHOULDER_TO_ELBOW = 0.35f;
constexpr float ELBOW_TO_WRIST = 0.3f;
constexpr float WRIST_TO_CLAW = 0.10f;

// Joint offsets: perpendicular distance from link axis to next link attachment (at +PI/2)
// This models the real-world offset where the next link is not centered on the joint
constexpr float ELBOW_JOINT_OFFSET = 0.04f;  // meters, offset at elbow joint
constexpr float WRIST_JOINT_OFFSET = 0.04f;  // meters, offset at wrist joint

// ==================== JOINT LIMITS ==================== 
// Turntable: full rotation
constexpr float TURNTABLE_MIN_ANGLE = -PI_VAL;
constexpr float TURNTABLE_MAX_ANGLE = PI_VAL;

// ==================== Shoulder ==================== 
// 0 = horizontal (floor level), π/2 = straight up, π = back over behind
// Min of 0 prevents shoulder from pointing below floor
constexpr float SHOULDER_MIN_ANGLE = 0.0f;
constexpr float SHOULDER_MAX_ANGLE = PI_VAL;

// ==================== Elbow ==================== 
// Full range: allows complete fold in either direction
constexpr float ELBOW_MIN_ANGLE = -PI_VAL;
constexpr float ELBOW_MAX_ANGLE = PI_VAL;

// ==================== Wrist ==================== 
// Full range
constexpr float WRIST_MIN_ANGLE = -PI_VAL;
constexpr float WRIST_MAX_ANGLE = PI_VAL;

// ==================== INITIAL POSE ==================== 
// Starting configuration: shoulder up, forearm down, wrist up (Z-shape)
constexpr float INIT_TURNTABLE = 0.0f;
constexpr float INIT_SHOULDER  = PI_VAL / 2.0f;    // straight up
constexpr float INIT_ELBOW     = -PI_VAL;           // folds straight down
constexpr float INIT_WRIST     = PI_VAL;             // back up
constexpr float INIT_CLAW      = 0.0f;

// ==================== MOTOR SPECS ==================== 
struct MotorConfig {
    float max_torque;      // N·m
    float max_speed;       // rad/s
    float gear_ratio;      // motor:joint
    float rotor_inertia;   // kg·m²
    float damping;         // N·m·s/rad (friction/damping)
};
constexpr MotorConfig TURNTABLE_MOTOR = {50.0f, 3.0f, 100.0f, 0.01f, 0.5f};
constexpr MotorConfig SHOULDER_MOTOR = {30.0f, 2.5f, 50.0f, 0.008f, 0.3f};
constexpr MotorConfig ELBOW_MOTOR = {25.0f, 2.5f, 40.0f, 0.006f, 0.25f};
constexpr MotorConfig WRIST_MOTOR = {10.0f, 3.0f, 30.0f, 0.003f, 0.1f};

// Claw
constexpr MotorConfig CLAW_MOTOR = {5.0f, 4.0f, 20.0f, 0.001f, 0.05f};

// ==================== ARM MASS PROPERTIES ==================== 
constexpr float SHOULDER_MASS = 2.0f;
constexpr float ELBOW_MASS = 1.5f;
constexpr float WRIST_MASS = 0.8f;
constexpr float CLAW_MASS = 0.5f;

constexpr float COM_OFFSET = 0.5f; // For simplicity sake, assume center of gravity is centerd

// ==================== SIMULATION SETTINGS ==================== 
constexpr float SIMULATION_DT = 0.001f;     // 1ms steps
constexpr float REAL_TIME_FACTOR = 1.0f;    // 1.0 = real time
constexpr float CONTROL_UPDATE_RATE = 100;  // Hz (Update IK every Nth step)

// ==================== IK SOLVER SETTING ==================== 
constexpr int MAX_IK_ITERATIONS = 100;
constexpr float IK_TOLERANCE = 0.001f;      // 1mm tolerance
constexpr float IK_DAMPING = 0.01f;         // damping for numerical stability

// ==================== VISION/OBSTACLE AVOIDANCE (Future implementation) ==================== 
constexpr float VISION_FOV = 60.0f;         // degrees
constexpr float VISION_RANGE = 2.0f;        // meters

// ==================== PID CONFIGURATION ====================
struct PIDConfig {
    float Kp;
    float Ki;
    float Kd;
    float integral_max;
};

constexpr PIDConfig TURNTABLE_PID = {15.0f, 0.5f, 2.0f, 5.0f};
constexpr PIDConfig SHOULDER_PID = {20.0f, 1.0f, 3.0f, 8.0f};
constexpr PIDConfig ELBOW_PID = {18.0f, 0.8f, 2.5f, 7.0f};
constexpr PIDConfig WRIST_PID = {12.0f, 0.3f, 1.5f, 4.0f};
constexpr PIDConfig CLAW_PID = {8.0f, 0.2f, 1.0f, 2.0f};
}