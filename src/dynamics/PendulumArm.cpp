#include "PendulumArm.hpp"
#include "../utils/Config.hpp"
#include <cmath>

namespace pendulum {

    PendulumArm::CylindricalEE PendulumArm::computeEndEffector(float theta_shoulder,
                                                               float theta_elbow,
                                                               float theta_wrist){
        // Cumulative angles
        float arm_pitch_shoulder = theta_shoulder;
        float arm_pitch_elbow = theta_shoulder + theta_elbow;
        float arm_pitch_wrist = theta_shoulder + theta_elbow + theta_wrist;

        // Shoulder link at origin
        float r1 = SHOULDER_TO_ELBOW * std::cos(arm_pitch_shoulder);
        float z1 = SHOULDER_TO_ELBOW * std::sin(arm_pitch_shoulder);

        // Elbow link
        float r2 = r1 + ELBOW_TO_WRIST * std::cos(arm_pitch_elbow);
        float z2 = z1 + ELBOW_TO_WRIST * std::sin(arm_pitch_elbow);

        // Wrist link
        float r_claw = r2 + WRIST_TO_CLAW * std::cos(arm_pitch_wrist);
        float z_claw = z2 + WRIST_TO_CLAW * std::sin(arm_pitch_wrist);

        return {r_claw, z_claw, arm_pitch_wrist};
    }

    void PendulumArm::forwardKinematics(SystemState& state) {
        // Get current joint angles
        float theta_turntable = state.theta[0];  // About z axis
        float theta_shoulder = state.theta[1];   // pitch from horizontal
        float theta_elbow = state.theta[2];      // pitch offset at elbow
        float theta_wrist = state.theta[3];      // pitch offset at wrist

        // Compute EE in cylindrical coords
        CylindricalEE ee = computeEndEffector(theta_shoulder, theta_elbow, theta_wrist);

        // Convert Coords from cylindrical too cartesian
        state.end_effector.x = ee.r * std::cos(theta_turntable);
        state.end_effector.y = ee.r * std::sin(theta_turntable);
        state.end_effector.z = ee.z;
        state.end_effector.wrist_pitch = ee.wrist_pitch;
    }

    bool PendulumArm::inverseKinematics(float target_x, float target_y, float target_z,
                                        float target_wrist_pitch, SystemState& state) {
        // Find turntable angle from target xy position
        float theta_turntable = std::atan2(target_y, target_x);

        // Convert target to 2D cylindrical coords
        float r_target = std::sqrt(target_x * target_x + target_y * target_y);

        // Use analytical geometric solver for the 3-DOF IK problem
        bool success = solveAnalytical(r_target, target_z, target_wrist_pitch, state);

        if (success) {
            state.theta[0] = theta_turntable;
        }

        return success;
    }

    bool PendulumArm::solveAnalytical(float target_r, float target_z,
                                     float target_wrist_pitch, SystemState& state) {
        const float L1 = SHOULDER_TO_ELBOW;
        const float L2 = ELBOW_TO_WRIST;

        // Step 1: Back-propagate the wrist link to find the wrist joint position
        float r_wrist = target_r - WRIST_TO_CLAW * std::cos(target_wrist_pitch);
        float z_wrist = target_z  - WRIST_TO_CLAW * std::sin(target_wrist_pitch);

        // Step 2: Check reachability for the 2-link (shoulder + elbow) chain
        float d = std::sqrt(r_wrist * r_wrist + z_wrist * z_wrist);
        if (d > L1 + L2 || d < std::abs(L1 - L2)) {
            return false;  // Target out of reach
        }

        // Step 3: Solve for shoulder angle via law of cosines
        // alpha = angle at shoulder between bearing to wrist joint and L1
        float phi = std::atan2(z_wrist, r_wrist);
        float cos_alpha = (L1 * L1 + d * d - L2 * L2) / (2.0f * L1 * d);
        cos_alpha = std::max(-1.0f, std::min(1.0f, cos_alpha));  // Clamp for numerical safety
        float alpha = std::acos(cos_alpha);

        // Two solutions: elbow-up (phi + alpha) and elbow-down (phi - alpha)
        float theta_s[2] = {phi + alpha, phi - alpha};
        float theta_e[2], theta_w[2];

        for (int i = 0; i < 2; ++i) {
            // Elbow joint position from shoulder angle
            float r_elbow = L1 * std::cos(theta_s[i]);
            float z_elbow = L1 * std::sin(theta_s[i]);

            // Relative elbow angle = absolute bearing from elbow to wrist joint minus shoulder angle
            float abs_elbow_pitch = std::atan2(z_wrist - z_elbow, r_wrist - r_elbow);
            theta_e[i] = abs_elbow_pitch - theta_s[i];

            // Wrist angle is whatever remains to match the desired wrist pitch
            theta_w[i] = target_wrist_pitch - (theta_s[i] + theta_e[i]);
        }

        // Pick first solution (prefer elbow-up) that satisfies joint limits
        for (int i = 0; i < 2; ++i) {
            if (theta_s[i] >= SHOULDER_MIN_ANGLE && theta_s[i] <= SHOULDER_MAX_ANGLE &&
                theta_e[i] >= ELBOW_MIN_ANGLE    && theta_e[i] <= ELBOW_MAX_ANGLE    &&
                theta_w[i] >= WRIST_MIN_ANGLE    && theta_w[i] <= WRIST_MAX_ANGLE) {
                state.theta[1] = theta_s[i];
                state.theta[2] = theta_e[i];
                state.theta[3] = theta_w[i];
                return true;
            }
        }

        return false;  // No solution within joint limits
    }
}