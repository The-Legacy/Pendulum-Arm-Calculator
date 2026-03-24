#include "../inlcude/pendulum/simulation/Simulator.hpp"
#include "../inlcude/pendulum/simulation/Renderer.hpp"
#include "raylib.h"
#include <iostream>
#include <cmath>
#include <string>
#include <sstream>

int main() {
    std::cerr << "=== Simulator Started ===" << std::endl;

    try {
        pendulum::Simulator sim;
        pendulum::Renderer renderer;

        if (!renderer.init()) {
            std::cerr << "Failed to initialize renderer" << std::endl;
            return 1;
        }

        float target_x = 0.35f, target_y = 0.35f, target_z = 0.2f;
        float wrist_pitch = 0.0f;
        sim.setTarget(target_x, target_y, target_z, wrist_pitch);

        int frame_count = 0;
        float distance_tolerance = 0.01f;
        bool target_reached = false;

        // Speed control: 1.0x = real-time (~17 sim steps per 60fps frame)
        float speed_multiplier = 1.0f;
        const int BASE_STEPS_PER_FRAME = 17;  // ~real-time at 60fps with 1ms DT

        // In-window text input state
        bool input_mode = false;
        std::string input_text;

        while (!renderer.shouldClose()) {
            // --- Handle keyboard input ---

            // Toggle target input mode with T key (only when not already in input mode)
            if (!input_mode && IsKeyPressed(KEY_T)) {
                input_mode = true;
                input_text.clear();
            }

            if (input_mode) {
                // Capture typed characters
                int ch = GetCharPressed();
                while (ch > 0) {
                    if (ch >= 32 && ch <= 126 && input_text.size() < 60) {
                        input_text += static_cast<char>(ch);
                    }
                    ch = GetCharPressed();
                }

                // Backspace
                if (IsKeyPressed(KEY_BACKSPACE) && !input_text.empty()) {
                    input_text.pop_back();
                }

                // Submit with Enter
                if (IsKeyPressed(KEY_ENTER)) {
                    std::istringstream iss(input_text);
                    float nx, ny, nz;
                    if (iss >> nx >> ny >> nz) {
                        float wp_deg = 0.0f;
                        iss >> wp_deg;  // optional wrist pitch in degrees
                        float wp = wp_deg * (3.14159265f / 180.0f);
                        target_x = nx; target_y = ny; target_z = nz; wrist_pitch = wp;
                        std::cerr << "New target: (" << target_x << ", " << target_y << ", " << target_z
                                  << ") wrist_pitch=" << wp_deg << "deg (" << wrist_pitch << " rad)" << std::endl;
                        sim.setTarget(target_x, target_y, target_z, wrist_pitch);
                        target_reached = false;
                    }
                    input_mode = false;
                    input_text.clear();
                }

                // Cancel with Escape
                if (IsKeyPressed(KEY_ESCAPE)) {
                    input_mode = false;
                    input_text.clear();
                }
            } else {
                // Speed controls (only when not typing)
                if (IsKeyPressed(KEY_EQUAL) || IsKeyPressed(KEY_KP_ADD)) {
                    speed_multiplier *= 2.0f;
                    if (speed_multiplier > 32.0f) speed_multiplier = 32.0f;
                }
                if (IsKeyPressed(KEY_MINUS) || IsKeyPressed(KEY_KP_SUBTRACT)) {
                    speed_multiplier *= 0.5f;
                    if (speed_multiplier < 0.125f) speed_multiplier = 0.125f;
                }

                // Reset sim with R
                if (IsKeyPressed(KEY_R)) {
                    sim.reset();
                    sim.setTarget(target_x, target_y, target_z, wrist_pitch);
                    target_reached = false;
                    std::cerr << "Simulation reset" << std::endl;
                }
            }

            // --- Simulation stepping (scaled by speed) ---
            int steps = static_cast<int>(BASE_STEPS_PER_FRAME * speed_multiplier);
            if (steps < 1) steps = 1;
            for (int i = 0; i < steps; i++) {
                sim.step();
            }

            // --- Render ---
            renderer.render(sim.getState(), target_x, target_y, target_z,
                           sim.getSimulationTime(), sim.getStepCount(),
                           speed_multiplier, input_mode, input_text);

            // --- Target reach detection ---
            const auto& state = sim.getState();
            if (!target_reached) {
                float dx = state.end_effector.x - target_x;
                float dy = state.end_effector.y - target_y;
                float dz = state.end_effector.z - target_z;
                float distance = std::sqrt(dx*dx + dy*dy + dz*dz);
                if (distance < distance_tolerance) {
                    target_reached = true;
                    std::cerr << "TARGET REACHED at sim time " << sim.getSimulationTime() << "s" << std::endl;
                }
            }

            // Diagnostics every 120 frames
            if (frame_count++ % 120 == 0) {
                std::cerr << "Time: " << sim.getSimulationTime() << "s"
                          << " | Speed: " << speed_multiplier << "x"
                          << " | EE: (" << state.end_effector.x
                          << ", " << state.end_effector.y
                          << ", " << state.end_effector.z << ")"
                          << " | Target: (" << target_x << ", " << target_y << ", " << target_z << ")"
                          << (target_reached ? " [REACHED]" : "") << std::endl;
            }
        }

        renderer.shutdown();
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown exception caught" << std::endl;
        return 1;
    }

    std::cerr << "=== Simulator Ended ===" << std::endl;
    return 0;
}