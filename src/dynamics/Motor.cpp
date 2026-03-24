#include "../../inlcude/pendulum/dynamics/Motor.hpp"
#include <algorithm>
#include <cmath>

namespace pendulum {

  Motor::Motor() : max_torque(0.0f), max_speed(0.0f), gear_ratio(0.0f),
                  rotor_inertia(0.0f), damping(0.0f), angle(0.0f), velocity(0.0f) {}

  Motor::Motor(float max_torque_val, float max_speed_val, float gear_ratio_val,
              float rotor_inertia_val, float damping_val)
      : max_torque(max_torque_val), max_speed(max_speed_val), gear_ratio(gear_ratio_val),
        rotor_inertia(rotor_inertia_val), damping(damping_val), angle(0.0f), velocity(0.0f) {}
  
  void Motor::update(float torque_command, float dt) {
    // Clamp torque to motor limits
    torque_command = std::max(-max_torque, std::min(max_torque, torque_command));

    // Dynamics: tau = I*a + b*w
    // Solve for accerleration a
    float acceleration = (torque_command - damping * velocity) / rotor_inertia;

    // Update velocity with acceleration limits
    velocity += acceleration * dt;
    velocity = std::max(-max_speed, std::min(max_speed, velocity));

    // Update anfle with wilocity
    angle += velocity * dt;
  }
}