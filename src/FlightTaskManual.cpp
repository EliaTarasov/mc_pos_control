#include <FlightTaskManual.hpp>

namespace mc_pos_control {

bool FlightTaskManual::update() {
  if(FlightTasks::update())
    return _evaluateSticks();
  return false;
}

bool FlightTaskManual::activate() {
  return FlightTasks::activate();
}
  
bool FlightTaskManual::updateInitialize() {
  bool ret = FlightTasks::updateInitialize();
  const bool sticks_available = _evaluateSticks();

  if (_sticks_data_required) {
    ret = ret && sticks_available;
  }

  return ret;
}

void FlightTaskManual::updateSticks(const mavros_msgs::ManualControlConstPtr& manual) {
  /* Linear scale  */
  _sticks(0) = manual->x; /* NED x, "pitch" [-1,1] */
  _sticks(1) = manual->y; /* NED y, "roll" [-1,1] */
  _sticks(2) = -(manual->z - 0.5f) * 2.f; /* NED z, "thrust" resacaled from [0,1] to [-1,1] */
  _sticks(3) = manual->r; /* "yaw" [-1,1] */
}

bool FlightTaskManual::_evaluateSticks() {
  /* Exponential scale */
  _sticks_expo(0) = expo_deadzone(_sticks(0), MPC_XY_MAN_EXPO, MPC_HOLD_DZ);
  _sticks_expo(1) = expo_deadzone(_sticks(1), MPC_XY_MAN_EXPO, MPC_HOLD_DZ);
  _sticks_expo(2) = expo_deadzone(_sticks(2), MPC_Z_MAN_EXPO, MPC_HOLD_DZ);
  _sticks_expo(3) = expo_deadzone(_sticks(3), MPC_YAW_EXPO, MPC_HOLD_DZ);

  // valid stick inputs are required
  return PX4_ISFINITE(_sticks(0)) && PX4_ISFINITE(_sticks(1)) && PX4_ISFINITE(_sticks(2)) && PX4_ISFINITE(_sticks(3));
}

}