#pragma once
#include <array>

namespace pendulum {
    struct SystemState {
        // Joint angles for turntable, shoudler, elbow, wrist, and claw
        std::array<float, 5> theta = {0.0f};
        std::array<float, 5> theta_dot = {0.0f}; // Angular velocities
        std::array<float, 5> tau = {0.0f};       // Joint Torque

        struct EndEffector {
            float x, y, z;      // Claw Position
            float wrist_pitch;  // Actual wrist angle from z axis
        } end_effector;
    };
}