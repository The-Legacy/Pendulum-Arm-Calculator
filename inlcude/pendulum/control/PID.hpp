#pragma

namespace pendulum {

    class PID {
        public:
            // Default cntr
            PID();

            // Initialize PID controller
            PID(float kp, float ki, float kd, float integral_max, float max_torque);

            // Compute output torque given error and timestep
            float update(float error, float dt);

            // Reset internal state
            void reset();

        private:
            float Kp, Ki, Kd;
            float integral_max;
            float max_torque;

            // Internal state
            float integral_term = 0.0f;
            float prev_error = 0.0f;
    };
}