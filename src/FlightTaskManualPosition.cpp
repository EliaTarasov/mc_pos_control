#include <FlightTaskManualPosition.hpp>

namespace mc_pos_control {

bool FlightTaskManualPosition::updateInitialize() {
  bool ret = FlightTaskManualAltitude::updateInitialize();
  // require valid position / velocity in xy
  return ret && PX4_ISFINITE(_position(0)) && PX4_ISFINITE(_position(1)) && PX4_ISFINITE(_velocity(0)) && PX4_ISFINITE(_velocity(1));
}

bool FlightTaskManualPosition::activate() {
  // all requirements from altitude-mode still have to hold
  bool ret = FlightTaskManualAltitude::activate();

  // set task specific constraint
  if (_constraints.speed_xy >= MPC_VEL_MANUAL) {
    _constraints.speed_xy = MPC_VEL_MANUAL;
  }

  _position_setpoint(0) = _position(0);
  _position_setpoint(1) = _position(1);
  _velocity_setpoint(0) = _velocity_setpoint(1) = 0.0f;
  _velocity_scale = _constraints.speed_xy;

  // for position-controlled mode, we need a valid position and velocity state in NE-direction 
  return ret;
}

bool FlightTaskManualPosition::update() {
  if(FlightTaskManualAltitude::update()) {
    _scaleSticks();
    _updateSetpoints();
    return true;
  }
  return false;
}

void FlightTaskManualPosition::_scaleSticks() {
  /* Use same scaling as for FlightTaskManualAltitude */
  FlightTaskManualAltitude::_scaleSticks();

  /* Constrain length of stick inputs to 1 for xy*/
  vec2 stick_xy(&_sticks_expo(0));

  scalar_t mag = constrain(stick_xy.norm(), 0.0, 1.0);

  if (mag > FLT_EPSILON) {
    stick_xy = stick_xy.normalized() * mag;
  }

  scalar_t _deltatime = 0.0;
  ros::Time now = ros::Time::now();
  if (prevStamp_.sec != 0) {
    _deltatime = (now - prevStamp_).toSec();
  }
  prevStamp_ = now;

  if (stick_xy.norm() > 0.5f) {
    // raise the limit at a constant rate up to the user specified value

    if (_velocity_scale < _constraints.speed_xy) {
      _velocity_scale += _deltatime * MPC_ACC_HOR_ESTM;

    } else {
      _velocity_scale = _constraints.speed_xy;

    }
  }

  // scale velocity to its maximum limits
  vec2 vel_sp_xy = stick_xy * _velocity_scale;

  vec2 sp;
  sp(0) = vel_sp_xy(0);
  sp(1) = vel_sp_xy(1);

  /* Rotate setpoint into local frame. */
  _rotateIntoHeadingFrame(sp);

  _velocity_setpoint(0) = sp(0);
  _velocity_setpoint(1) = sp(1);
}

void FlightTaskManualPosition::_updateXYlock() {
  /* If position lock is not active, position setpoint is set to NAN.*/
  const scalar_t vel_xy_norm = vec2(_velocity(0), _velocity(1)).norm();
  const bool apply_brake = vec2(_velocity_setpoint(0), _velocity_setpoint(1)).norm() < FLT_EPSILON;
  const bool stopped = (MPC_HOLD_MAX_XY < FLT_EPSILON || vel_xy_norm < MPC_HOLD_MAX_XY);

  if (apply_brake && stopped && !PX4_ISFINITE(_position_setpoint(0))) {
    _position_setpoint(0) = _position(0);
    _position_setpoint(1) = _position(1);

  } else if (PX4_ISFINITE(_position_setpoint(0)) && apply_brake) {
    // Position is locked but check if a reset event has happened.
    // We will shift the setpoints.
    //_position_setpoint(0) = _position(0);
    //_position_setpoint(1) = _position(1);
  } else {
    /* don't lock*/
    _position_setpoint(0) = NAN;
    _position_setpoint(1) = NAN;
  }
}

void FlightTaskManualPosition::_updateSetpoints() {
  FlightTaskManualAltitude::_updateSetpoints(); // needed to get yaw and setpoints in z-direction

  _thrust_setpoint.fill(NAN); // don't require any thrust setpoints
  _updateXYlock(); // check for position lock
}

}