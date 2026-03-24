#include "../../inlcude/pendulum/simulation/Simulator.hpp"
#include "../../inlcude/pendulum/utils/Config.hpp"
#include <thread>

namespace pendulum {

    Simulator::Simulator() : controller(), sim_time(0.0f), step_count(0) {
        last_wall_time = std::chrono::high_resolution_clock::now();
    }

    void Simulator::setTarget(float target_x, float target_y, float target_z,
                              float target_wrist_pitch) {
        controller.setTarget(target_x, target_y, target_z, target_wrist_pitch);
    }

    void Simulator::step() {
        // Execute one control timestep
        controller.update(SIMULATION_DT);

        sim_time += SIMULATION_DT;
        step_count++;
    }

    void Simulator::run(float duration_seconds) {
        is_running = true;
        float elapsed = 0.0f;
        last_wall_time = std::chrono::high_resolution_clock::now();

        while (elapsed < duration_seconds && is_running){
            // Execute simulation step
            step();

            // Handle real-time pacing
            if (REAL_TIME_FACTOR > 0.0f) {
                float sim_dt_wall_time = SIMULATION_DT / REAL_TIME_FACTOR;
                auto now = std::chrono::high_resolution_clock::now();
                float wall_elapsed = std::chrono::duration<float>(now - last_wall_time).count();

                // Sleep if we're ahead of wall clock
                if (wall_elapsed < sim_dt_wall_time) {
                    float sleep_time = sim_dt_wall_time - wall_elapsed;
                    std::this_thread::sleep_for(
                        std::chrono::duration<float>(sleep_time)
                    );
                }
            }

            elapsed += SIMULATION_DT;
        }
        is_running = false;
    }

    void Simulator::reset() {
        controller.reset();
        sim_time = 0.0f;
        step_count = 0;
    }
}