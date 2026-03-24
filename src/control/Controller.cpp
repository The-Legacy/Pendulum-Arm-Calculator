#include "../../inlcude/pendulum/control/Controller.hpp"
#include "../../inlcude/pendulum/utils/Config.hpp"

namespace pendulum {

    Controller::Controller() : pid_controllers{}, motors{} {
        initializeControllers();
        setInitialPose();
    }

    void Controller::initializeControllers() {
        // Initialize PIDs with config gains
        pid_controllers[0] = PID(TURNTABLE_PID.Kp, TURNTABLE_PID.Ki, TURNTABLE_PID.Kd,
                        TURNTABLE_PID.integral_max, TURNTABLE_MOTOR.max_torque);
        pid_controllers[1] = PID(SHOULDER_PID.Kp, SHOULDER_PID.Ki, SHOULDER_PID.Kd,
                        SHOULDER_PID.integral_max, SHOULDER_MOTOR.max_torque);
        pid_controllers[2] = PID(ELBOW_PID.Kp, ELBOW_PID.Ki, ELBOW_PID.Kd,
                        ELBOW_PID.integral_max, ELBOW_MOTOR.max_torque);
        pid_controllers[3] = PID(WRIST_PID.Kp, WRIST_PID.Ki, WRIST_PID.Kd,
                        WRIST_PID.integral_max, WRIST_MOTOR.max_torque);
        pid_controllers[4] = PID(CLAW_PID.Kp, CLAW_PID.Ki, CLAW_PID.Kd,
                        CLAW_PID.integral_max, CLAW_MOTOR.max_torque);

        // Initialize motors
        motors[0] = Motor(TURNTABLE_MOTOR.max_torque, TURNTABLE_MOTOR.max_speed,
                    TURNTABLE_MOTOR.gear_ratio, TURNTABLE_MOTOR.rotor_inertia,
                    TURNTABLE_MOTOR.damping);
        motors[1] = Motor(SHOULDER_MOTOR.max_torque, SHOULDER_MOTOR.max_speed,
                    SHOULDER_MOTOR.gear_ratio, SHOULDER_MOTOR.rotor_inertia,
                    SHOULDER_MOTOR.damping);
        motors[2] = Motor(ELBOW_MOTOR.max_torque, ELBOW_MOTOR.max_speed,
                    ELBOW_MOTOR.gear_ratio, ELBOW_MOTOR.rotor_inertia,
                    ELBOW_MOTOR.damping);
        motors[3] = Motor(WRIST_MOTOR.max_torque, WRIST_MOTOR.max_speed,
                    WRIST_MOTOR.gear_ratio, WRIST_MOTOR.rotor_inertia,
                    WRIST_MOTOR.damping);
        motors[4] = Motor(CLAW_MOTOR.max_torque, CLAW_MOTOR.max_speed,
                    CLAW_MOTOR.gear_ratio, CLAW_MOTOR.rotor_inertia,
                    CLAW_MOTOR.damping);
    }

    void Controller::setInitialPose() {
        // Set initial joint angles from config
        state.theta[0] = INIT_TURNTABLE;
        state.theta[1] = INIT_SHOULDER;
        state.theta[2] = INIT_ELBOW;
        state.theta[3] = INIT_WRIST;
        state.theta[4] = INIT_CLAW;

        // Sync motors to initial angles
        for (int i = 0; i < 5; ++i) {
            motors[i].setAngle(state.theta[i]);
            motors[i].setVelocity(0.0f);
        }

        // Set initial target to current pose (hold position until setTarget called)
        target_state = state;

        // Compute initial FK
        arm.forwardKinematics(state);
    }

    void Controller::setTarget(float target_x, float target_y, float target_z,
                                float target_wrist_pitch) {
        // Solve IK for the target
        if (arm.inverseKinematics(target_x, target_y, target_z, target_wrist_pitch, target_state)) {
            // Target is reachable will be used
        }
        // Target is unreachable
    }

    void Controller::update(float dt) {
        // Compute errors
        std::array<float, 5> errors;
        for (int i = 0; i < 5; ++i) {
            errors[i] = target_state.theta[i] - state.theta[i];
        }

        // Run PID for each joint
        std::array<float, 5> torque_commands;
        for (int i = 0; i < 5; ++i) {
            torque_commands[i] = pid_controllers[i].update(errors[i], dt);
        }

        // Apply torques to motors
        for (int i = 0; i < 5; ++i){
            motors[i].update(torque_commands[i], dt);
            state.theta[i] = motors[i].getAngle();
            state.theta_dot[i] = motors[i].getVelocity();
        }

        // cmopute forward kinematics to update
        arm.forwardKinematics(state);
    }

    void Controller::reset() {
        for (int i = 0; i < 5; ++i) {
            pid_controllers[i].reset();
        }
        state = SystemState();
        target_state = SystemState();
        setInitialPose();
    }
}