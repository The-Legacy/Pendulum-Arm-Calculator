#pragma once
#include "../dynamics/SystemState.hpp"
#include <string>

namespace pendulum {

    class Renderer {
        public:
            Renderer(int window_width = 1200, int window_height = 800);
            ~Renderer();

            // Initialize Window
            bool init();

            // Main render loop
            void render(const SystemState& state, float target_x, float target_y, float target_z,
                        float sim_time, int step_count, float speed_multiplier,
                        bool input_mode, const std::string& input_text);

            // Check if window should close
            bool shouldClose() const;

            // Cleanup
            void shutdown();

        private:
            int window_width, window_height;
            bool initialized = false;

            // Camera orbit state
            float cam_angle_h = 0.8f;   // horizontal orbit angle (radians)
            float cam_angle_v = 0.5f;   // vertical angle (radians)
            float cam_distance = 1.5f;  // distance from target

            // Helper drawing function
            void drawArm(const SystemState& state);
            void drawLink(float r_start, float z_start, float r_end, float z_end,
                        float radius, float turntable_angle, float hue);
            void drawSphere(float x, float y, float z, float radius, float hue);
            void drawClaw(const SystemState& state);
            void drawTarget(float target_x, float target_y, float target_z);
            void drawCoordinateFrame();
            void drawHUD(float sim_time, int step_count, float speed_multiplier);
            void drawInputOverlay(const std::string& input_text);
            void drawControlsHelp();
    };
}