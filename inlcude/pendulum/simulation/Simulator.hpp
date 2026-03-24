#pragma once
#include "../control/Controller.hpp"
#include "../dynamics/SystemState.hpp"
#include <vector>
#include <chrono>

namespace pendulum {

    class Simulator {
        public:
            Simulator();

            // Set target pos
            void setTarget(float target_x, float target_y, float target_z, float target_wrist_pitch);

            // Run simulation for one control timestep
            void step();

            // Run sum for duration of time
            void run(float duration_seconds);

            // Get current state
            const SystemState& getState() const { return controller.getState(); }

            // Reset simulation
            void reset();

            // Accessors
            float getSimulationTime() const { return sim_time; }
            int getStepCount() const { return step_count; }

        private:
            Controller controller;

            // Sim timing
            float sim_time = 0.0f;
            int step_count = 0;

            // Wall-clock tracking for real-time simulation
            std::chrono::high_resolution_clock::time_point last_wall_time;
            bool is_running = false;
    };
}