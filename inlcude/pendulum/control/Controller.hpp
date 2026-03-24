#pragma once
#include "PID.hpp"
#include "../dynamics/PendulumArm.hpp"
#include "../dynamics/Motor.hpp"
#include "../dynamics/SystemState.hpp"
#include <array>

namespace pendulum {

    class Controller {
        public:
            Controller();

            // Set target end effector pos
            void setTarget(float target_x, float target_y, float target_z, float target_wrist_pitch);

            // Main control loop
            void update(float dt);

            // Get current state
            const SystemState& getState() const { return state; }

            // Reset all controllers
            void reset();
        
        private: 
            // Arm Kinetics
            PendulumArm arm;

            // Target state
            SystemState target_state;

            // Control: 5 PIDs
            std::array<PID, 5> pid_controllers;

            // Actuators: 5 motors
            std::array<Motor, 5> motors;

            // Current state
            SystemState state;

            // Helper to initalize PID and Motor arrays
            void initializeControllers();

            // Set the initial arm pose from Config constants
            void setInitialPose();
    };
}