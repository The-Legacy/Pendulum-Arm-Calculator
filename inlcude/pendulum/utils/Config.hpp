#pragma once
#include <numbers>

namespace pendulum {
constexpr float PI = 3.14159265358979323846f;

// ==================== ARM GEOMETRY ==================== 
constexpr float SHOULDER_TO_ELBOW = 0.35f;
constexpr float ELBOW_TO_WRIST = 0.3f;
constexpr float WRIST_TO_CLAW = 0.10f;

// ==================== JOINT LIMITS ==================== 
// Turntable is assumed to be continuous rotation
constexpr float TURNTABLE_MIN_ANGLE = -PI;
constexpr float TURNTABLE_MAX_ANGLE = PI;

// ==================== Shoulder ==================== 
// From 90 ot 45 degrees
constexpr float SHOULDER_MIN_ANGLE = -PI/2.0f;
constexpr float SHOULDER_MAX_ANGLE = PI/4.0f;

// ==================== Elbow ==================== 
// From -120 to 0 degrees (cannot fold backwards)
constexpr float ELBOW_MIN_ANGLE = -2.09f;
constexpr float ELBOW_MAX_ANGLE = 0.0f;

// ==================== Wrist ==================== 
// Between -45 and 45 degrees
constexpr float WRIST_MIN_ANGLE = -PI/4.0f;
constexpr float WRIST_MAX_ANGLE = PI/4.0f;

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


}