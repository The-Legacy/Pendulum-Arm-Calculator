#pragma once

namespace pendulum {

    class Motor {
        public:
            // Default constructor
            Motor(); 

            // Initialize motor
            Motor(float max_torque, float max_speed, float gear_ratio,
                float rotor_inertia, float damping);

            // Apply torque command and update it
            void update(float torque_command, float dt);

            // Accessors
            float getAngle() const { return angle; }
            float getVelocity() const { return velocity; }

            // Setters
            void setAngle(float theta) { angle = theta; }
            void setVelocity(float omega) { velocity = omega; }

        private:
            // Motor params
            float max_torque;
            float max_speed;
            float gear_ratio;
            float rotor_inertia;
            float damping;

            // Motor state (Measured using radians)
            float angle = 0.0f;
            float velocity = 0.0f;
    };
}