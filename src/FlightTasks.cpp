#include <FlightTasks.hpp>

namespace mc_pos_control {

FlightTasks::FlightTasks() {
  _constraints.yawspeed = 200; // deg/s
  _constraints.speed_xy = 12; // m/s
  _constraints.speed_up = 2; // m/s
  _constraints.speed_down = 1; // m/s
  _constraints.tilt = 0.122164049; // rad
}

bool FlightTasks::update() {
  return true;
}

bool FlightTasks::activate() {
  _resetSetpoints();
  return true;
}

bool FlightTasks::updateInitialize() {
  return true;
}

void FlightTasks::updateSticks(const mavros_msgs::ManualControlConstPtr& manual) {}

const Constraints& FlightTasks::getConstraints() { 
  return _constraints; 
}

const mavros_msgs::PositionTarget FlightTasks::getPositionSetpoint() {
  /* fill position setpoint message */
  mavros_msgs::PositionTarget vehicle_local_position_setpoint;

  vehicle_local_position_setpoint.position.x = _position_setpoint(0);
  vehicle_local_position_setpoint.position.y = _position_setpoint(1);
  vehicle_local_position_setpoint.position.z = _position_setpoint(2);

  vehicle_local_position_setpoint.velocity.x = _velocity_setpoint(0);
  vehicle_local_position_setpoint.velocity.y = _velocity_setpoint(1);
  vehicle_local_position_setpoint.velocity.z = _velocity_setpoint(2);

  vehicle_local_position_setpoint.acceleration_or_force.x = _thrust_setpoint(0);
  vehicle_local_position_setpoint.acceleration_or_force.y = _thrust_setpoint(1);
  vehicle_local_position_setpoint.acceleration_or_force.z = _thrust_setpoint(2);

  vehicle_local_position_setpoint.yaw = _yaw_setpoint;
  vehicle_local_position_setpoint.yaw_rate = _yawspeed_setpoint;

  return vehicle_local_position_setpoint;
}

void FlightTasks::_resetSetpoints() {
  _position_setpoint.fill(NAN);
  _velocity_setpoint.fill(NAN);
  _thrust_setpoint.fill(NAN);
  _yaw_setpoint = _yawspeed_setpoint = NAN;
}

void FlightTasks::updateState(const PositionControlStates& states) {
  _position = q_ng.inverse().toRotationMatrix() * states.position;
  _velocity = q_ng.inverse().toRotationMatrix() * states.velocity;
  quat q = q_ng * from_euler(vec3(0, 0, states.yaw)) * q_rb;
  _yaw = dcm2vec(quat2dcm(q))[2];
}


}