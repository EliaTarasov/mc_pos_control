#include <FlightTaskManualAltitude.hpp>

namespace mc_pos_control {

bool FlightTaskManualAltitude::updateInitialize() {
  bool ret = FlightTaskManual::updateInitialize();
  // in addition to manual require valid position and velocity in D-direction and valid yaw
  return ret && PX4_ISFINITE(_position(2)) && PX4_ISFINITE(_velocity(2)) && PX4_ISFINITE(_yaw);
}

bool FlightTaskManualAltitude::activate() {
  bool ret = FlightTaskManual::activate();
  _yaw_setpoint = NAN;
  _yawspeed_setpoint = 0.0;
  _thrust_setpoint = vec3(0.0f, 0.0f, NAN); // altitude is controlled from position/velocity
  _position_setpoint(2) = _position(2);
  _velocity_setpoint(2) = 0.0;

  _max_speed_up = _constraints.speed_up;
  _min_speed_down = _constraints.speed_down;

  return ret;
}

bool FlightTaskManualAltitude::update() {
  if(FlightTaskManual::update()) {
    _scaleSticks();
    _updateSetpoints();
    return true;
  }

  return false;
}

void FlightTaskManualAltitude::_scaleSticks() {
  // Use sticks input with deadzone and exponential curve for vertical velocity and yawspeed
  _yawspeed_setpoint = _sticks_expo(3) * radians(MPC_MAN_Y_MAX);

  const scalar_t vel_max_z = (_sticks(2) > 0.0) ? _constraints.speed_down : _constraints.speed_up;
  _velocity_setpoint(2) = vel_max_z * _sticks_expo(2);
}

void FlightTaskManualAltitude::_updateAltitudeLock() {
  // Depending on stick inputs and velocity, position is locked.
  // If not locked, altitude setpoint is set to NAN.

  // Check if user wants to break
  const bool apply_brake = fabsf(_sticks_expo(2)) <= FLT_EPSILON;

  // Check if vehicle has stopped
  const bool stopped = (MPC_HOLD_MAX_Z < FLT_EPSILON || fabsf(_velocity(2)) < MPC_HOLD_MAX_Z);

  // Manage transition between use of distance to ground and distance to local origin
  // when terrain hold behaviour has been selected.
  // normal mode where height is dependent on local frame

  if (apply_brake && stopped && !PX4_ISFINITE(_position_setpoint(2))) {
    // lock position
    _position_setpoint(2) = _position(2);
  } else if (PX4_ISFINITE(_position_setpoint(2)) && apply_brake) {
    // Position is locked but check if a reset event has happened.
    // We will shift the setpoints.
    //_position_setpoint(2) = _position(2);
  } else {
      // user demands velocity change
      _position_setpoint(2) = NAN;
      // ensure that maximum altitude is respected
  }
}

void FlightTaskManualAltitude::_updateSetpoints() {
  _updateHeadingSetpoints(); // get yaw setpoint

  // Thrust in xy are extracted directly from stick inputs. A magnitude of
  // 1 means that maximum thrust along xy is demanded. A magnitude of 0 means no
  // thrust along xy is demanded. The maximum thrust along xy depends on the thrust
  // setpoint along z-direction, which is computed in PositionControl.cpp.

  vec2 sp;
  sp(0) = _sticks(0);
  sp(1) = _sticks(1);
  _rotateIntoHeadingFrame(sp);

  if (sp.norm() > 1.0) {
    sp.normalize();
  }

  _thrust_setpoint(0) = sp(0);
  _thrust_setpoint(1) = sp(1);
  _thrust_setpoint(2) = NAN;

  _updateAltitudeLock();
}

void FlightTaskManualAltitude::_rotateIntoHeadingFrame(vec2 &v) {
  scalar_t yaw_rotate = PX4_ISFINITE(_yaw_setpoint) ? _yaw_setpoint : _yaw;
  vec3 v_r = euler2dcm(vec3(0.0, 0.0, yaw_rotate)) * vec3(v(0), v(1), 0.0);
  v(0) = v_r(0);
  v(1) = v_r(1);
}

void FlightTaskManualAltitude::_updateHeadingSetpoints() {
  /* Yaw-lock depends on stick input. If not locked, yaw_sp is set to NAN. TODO: add yawspeed to get threshold.*/
  if (fabsf(_yawspeed_setpoint) > FLT_EPSILON) {
    // no fixed heading when rotating around yaw by stick
    _yaw_setpoint = NAN;

  } else {
    // hold the current heading when no more rotation commanded
    if (!PX4_ISFINITE(_yaw_setpoint)) {
      _yaw_setpoint = _yaw;

    }
  }
}

}