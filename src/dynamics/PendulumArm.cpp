#include "../../inlcude/pendulum/dynamics/PendulumArm.hpp"
#include "../../inlcude/pendulum/utils/Config.hpp"
#include <cmath>
#include <iostream>

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

        // Offsets are tangential (around Z axis) — they don't affect the (r,z) plane
        // Elbow link from end of shoulder
        float r2 = r1 + ELBOW_TO_WRIST * std::cos(arm_pitch_elbow);
        float z2 = z1 + ELBOW_TO_WRIST * std::sin(arm_pitch_elbow);

        // Wrist link from end of forearm
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

        // Total tangential offset from joint offsets sticking out the side
        float d_off = ELBOW_JOINT_OFFSET + WRIST_JOINT_OFFSET;

        // Convert from cylindrical to cartesian, including tangential offset
        // Tangential direction at turntable angle θ is (-sin(θ), cos(θ))
        state.end_effector.x = ee.r * std::cos(theta_turntable) - d_off * std::sin(theta_turntable);
        state.end_effector.y = ee.r * std::sin(theta_turntable) + d_off * std::cos(theta_turntable);
        state.end_effector.z = ee.z;
        state.end_effector.wrist_pitch = ee.wrist_pitch;
    }

    bool PendulumArm::inverseKinematics(float target_x, float target_y, float target_z,
                                        float target_wrist_pitch, SystemState& state) {
        // Total tangential offset from joint offsets
        float d_off = ELBOW_JOINT_OFFSET + WRIST_JOINT_OFFSET;

        // The end effector is at (r*cos(θ) - d_off*sin(θ), r*sin(θ) + d_off*cos(θ))
        // Given target (tx, ty): tx² + ty² = r² + d_off²
        float dist_sq = target_x * target_x + target_y * target_y;
        float r_target = std::sqrt(std::max(0.0f, dist_sq - d_off * d_off));

        // Turntable angle: target_angle = θ + atan2(d_off, r)
        float target_angle = std::atan2(target_y, target_x);
        float theta_turntable = target_angle - std::atan2(d_off, r_target);

        // Try the requested wrist pitch first
        if (solveAnalytical(r_target, target_z, target_wrist_pitch, state)) {
            state.theta[0] = theta_turntable;
            return true;
        }

        // If requested pitch fails, sweep outward from the requested angle to find
        // the closest reachable wrist pitch (tries ±5°, ±10°, ... ±180°)
        std::cerr << "IK: Requested wrist_pitch=" << target_wrist_pitch 
                  << " rad unreachable, searching for closest..." << std::endl;
        for (int deg = 5; deg <= 180; deg += 5) {
            float delta = deg * (PI_VAL / 180.0f);
            // Try both directions from the requested pitch
            float tries[] = {target_wrist_pitch + delta, target_wrist_pitch - delta};
            for (float wp : tries) {
                if (solveAnalytical(r_target, target_z, wp, state)) {
                    std::cerr << "IK: Using fallback wrist_pitch=" << wp 
                              << " rad (" << (wp * 180.0f / PI_VAL) << " deg)" << std::endl;
                    state.theta[0] = theta_turntable;
                    return true;
                }
            }
        }

        std::cerr << "IK FAIL: No reachable wrist pitch found" << std::endl;
        return false;
    }

    bool PendulumArm::solveAnalytical(float target_r, float target_z,
                                     float target_wrist_pitch, SystemState& state) {
        // Offsets are tangential (around Z axis) so they don't affect the planar IK
        // Use raw link lengths for the 2-link chain
        const float L1 = SHOULDER_TO_ELBOW;
        const float L2 = ELBOW_TO_WRIST;

        // Step 1: Back-propagate the wrist link to find the wrist joint position
        float r_wrist = target_r - WRIST_TO_CLAW * std::cos(target_wrist_pitch);
        float z_wrist = target_z - WRIST_TO_CLAW * std::sin(target_wrist_pitch);

        // Step 2: Check reachability for the 2-link chain
        float d = std::sqrt(r_wrist * r_wrist + z_wrist * z_wrist);
        if (d > L1 + L2 || d < std::abs(L1 - L2)) {
            std::cerr << "IK FAIL: Out of reach. d=" << d << ", L1+L2=" << (L1+L2) 
                      << ", |L1-L2|=" << std::abs(L1-L2) << std::endl;
            return false;
        }

        // Step 3: Solve for shoulder angle via law of cosines
        float phi = std::atan2(z_wrist, r_wrist);
        float cos_alpha = (L1 * L1 + d * d - L2 * L2) / (2.0f * L1 * d);
        cos_alpha = std::max(-1.0f, std::min(1.0f, cos_alpha));
        float alpha = std::acos(cos_alpha);

        // Two solutions: elbow-up and elbow-down
        float theta_s[2] = {phi + alpha, phi - alpha};
        float theta_e[2], theta_w[2];

        for (int i = 0; i < 2; ++i) {
            // Elbow position from shoulder angle
            float r_elbow = L1 * std::cos(theta_s[i]);
            float z_elbow = L1 * std::sin(theta_s[i]);

            // Bearing from elbow to wrist
            float abs_elbow_pitch = std::atan2(z_wrist - z_elbow, r_wrist - r_elbow);
            theta_e[i] = abs_elbow_pitch - theta_s[i];

            // Wrist angle is whatever remains to match the desired wrist pitch
            theta_w[i] = target_wrist_pitch - (theta_s[i] + theta_e[i]);
            
            std::cerr << "IK Solution " << i << ": theta_s=" << theta_s[i] 
                      << " (limits: " << SHOULDER_MIN_ANGLE << " to " << SHOULDER_MAX_ANGLE << ")"
                      << ", theta_e=" << theta_e[i] 
                      << " (limits: " << ELBOW_MIN_ANGLE << " to " << ELBOW_MAX_ANGLE << ")"
                      << ", theta_w=" << theta_w[i]
                      << " (limits: " << WRIST_MIN_ANGLE << " to " << WRIST_MAX_ANGLE << ")" << std::endl;
        }

        // Pick first solution that satisfies joint limits
        for (int i = 0; i < 2; ++i) {
            if (theta_s[i] >= SHOULDER_MIN_ANGLE && theta_s[i] <= SHOULDER_MAX_ANGLE &&
                theta_e[i] >= ELBOW_MIN_ANGLE    && theta_e[i] <= ELBOW_MAX_ANGLE    &&
                theta_w[i] >= WRIST_MIN_ANGLE    && theta_w[i] <= WRIST_MAX_ANGLE) {
                std::cerr << "IK SUCCESS: Using solution " << i << std::endl;
                state.theta[1] = theta_s[i];
                state.theta[2] = theta_e[i];
                state.theta[3] = theta_w[i];
                return true;
            }
        }

        std::cerr << "IK FAIL: No solution within joint limits" << std::endl;
        return false;
    }
}