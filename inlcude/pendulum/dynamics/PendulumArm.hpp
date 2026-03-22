#pragma once
#include "SystemState.hpp"
#include <cmath>

namespace pendulum {
    class PendulumArm {
        public:
            // Forward kinematics: angle to end position
            void forwardKinematics(SystemState& state);

            // Inverse Kinematics: target pos and wrist angle to joint angles
            // Returns false if unreachable
            bool inverseKinematics(float target_x, float target_y, float target_z,
                                float target_wrist_pitch, SystemState& state);

        private:
            // Calc position of each link
            struct CylindricalEE {
                float r, z, wrist_pitch;
            };

            CylindricalEE computeEndEffector(float theta_shoulder, float theta_elbow, 
                                             float theta_wrist);

            // Analytical (closed-form geometric) IK solver
            bool solveAnalytical(float target_r, float target_z,
                                 float target_wrist_pitch, SystemState& state);
    };
}