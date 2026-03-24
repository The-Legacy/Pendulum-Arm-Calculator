#include "../../inlcude/pendulum/control/PID.hpp"
#include <algorithm>
#include <cmath>

namespace pendulum {

    PID::PID() : Kp(0.0f), Ki(0.0f), Kd(0.0f), integral_max(0.0f),
                max_torque(0.0f), integral_term(0.0f), prev_error(0.0f) {}

    PID::PID(float kp, float ki, float kd, float integral_max_val, float max_torque_val)
        : Kp(kp), Ki(ki), Kd(kd), integral_max(integral_max_val), max_torque(max_torque_val),
        integral_term(0.0f), prev_error(0.0f) {}


    float PID::update(float error, float dt) {
        // Proportional term
        float P = Kp * error;

        // Integral term
        integral_term += Ki * error * dt;
        integral_term = std::max(-integral_max, std::min(integral_max, integral_term));
        float I = integral_term;

        // Derivative term
        float derivative = (prev_error == 0.0f) ? 0.0f : (error - prev_error) / dt;
        float D = Kd * derivative;
        prev_error = error;

        float torque = P + I + D;

        torque = std::max(-max_torque, std::min(max_torque, torque));

        return torque;
    }

    void PID::reset() {
        integral_term = 0.0f;
        prev_error = 0.0f;
    }
}