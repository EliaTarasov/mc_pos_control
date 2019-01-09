#include <PositionControl.hpp>

namespace mc_pos_control {

void PositionControl::updateState(const PositionControlStates& states) {
  _pos = q_ng.inverse().toRotationMatrix() * states.position;
  _vel = q_ng.inverse().toRotationMatrix() * states.velocity;
  quat q = q_ng * from_euler(vec3(0, 0, states.yaw)) * q_rb;
  _yaw = dcm2vec(quat2dcm(q))[2];
  _vel_dot(0) = _vel_x_deriv.update(-_vel(0));
  _vel_dot(1) = _vel_y_deriv.update(-_vel(1));
  _vel_dot(2) = _vel_z_deriv.update(-_vel(2));
}

bool PositionControl::updateSetpoint(const mavros_msgs::PositionTarget& setpoint) {
  _pos_sp = vec3(setpoint.position.x, setpoint.position.y, setpoint.position.z);
  _vel_sp = vec3(setpoint.velocity.x, setpoint.velocity.y, setpoint.velocity.z);
  _thr_sp = vec3(setpoint.acceleration_or_force.x, setpoint.acceleration_or_force.y, setpoint.acceleration_or_force.z);
  _yaw_sp = setpoint.yaw;
  _yawspeed_sp = setpoint.yaw_rate;

  bool mapping_succeeded = _interfaceMapping();

  // If full manual is required (thrust already generated), don't run position/velocity controller and just return thrust.
  _skip_controller = PX4_ISFINITE(setpoint.acceleration_or_force.x) && PX4_ISFINITE(setpoint.acceleration_or_force.y) && PX4_ISFINITE(setpoint.acceleration_or_force.z);

  return mapping_succeeded;
}

void PositionControl::generateThrustYawSetpoint(const scalar_t dt) {
  if (_skip_controller) {
    // Already received a valid thrust set-point.
    // Limit the thrust vector.
    scalar_t thr_mag = _thr_sp.norm();

    if (thr_mag > MPC_THR_MAX) {
      _thr_sp = _thr_sp.normalized() * MPC_THR_MAX;
    } else if (thr_mag < MPC_MANTHR_MIN && thr_mag > FLT_EPSILON) {
      _thr_sp = _thr_sp.normalized() * MPC_MANTHR_MIN;
    }

    // Just set the set-points equal to the current vehicle state.
    _pos_sp = _pos;
    _vel_sp = _vel;

  } else {
    _positionController();
    _velocityController(dt);
  }
}

bool PositionControl::_interfaceMapping() {
  // if nothing is valid, then apply failsafe landing
  bool failsafe = false;

  // Respects FlightTask interface, where NAN-set-points are of no interest
  // and do not require control. A valid position and velocity setpoint will
  // be mapped to a desired position setpoint with a feed-forward term.
  // States and setpoints which are integrals of the reference setpoint are set to 0.
  // For instance: reference is velocity-setpoint -> position and position-setpoint = 0
  //               reference is thrust-setpoint -> position, velocity, position-/velocity-setpoint = 0
  for (int i = 0; i <= 2; i++) {

    if (PX4_ISFINITE(_pos_sp(i))) {

      // Position control is required

      if (!PX4_ISFINITE(_vel_sp(i))) {
        // Velocity is not used as feedforward term.
        _vel_sp(i) = 0.0f;
      }

      // thrust setpoint is not supported in position control
      _thr_sp(i) = NAN;

      // to run position control, we require valid position and velocity
      if (!PX4_ISFINITE(_pos(i)) || !PX4_ISFINITE(_vel(i))) {
        failsafe = true;
      }

    } else if (PX4_ISFINITE(_vel_sp(i))) {

      // Velocity controller is active without position control.
      // Set integral states and setpoints to 0

      _pos_sp(i) = _pos(i) = 0.0f;

      // thrust setpoint is not supported in velocity control
      _thr_sp(i) = NAN;

      // to run velocity control, we require valid velocity
      if (!PX4_ISFINITE(_vel(i))) {
        failsafe = true;
      }

    } else if (PX4_ISFINITE(_thr_sp(i))) {
      
      // Thrust setpoint was generated from sticks directly.
      // Set all integral states and setpoints to 0
      
      _pos_sp(i) = _pos(i) = 0.0f;
      _vel_sp(i) = _vel(i) = 0.0f;

      // Reset the Integral term.
      _thr_int(i) = 0.0f;
      // Don't require velocity derivative.
      _vel_dot(i) = 0.0f;

    } else {
      // nothing is valid. do failsafe
      failsafe = true;
    }
  }

  // ensure that vel_dot is finite, otherwise set to 0
  if (!PX4_ISFINITE(_vel_dot(0)) || !PX4_ISFINITE(_vel_dot(1))) {
    _vel_dot(0) = _vel_dot(1) = 0.0f;
  }

  if (!PX4_ISFINITE(_vel_dot(2))) {
    _vel_dot(2) = 0.0f;
  }

  if (!PX4_ISFINITE(_yawspeed_sp)) {
    // Set the yawspeed to 0 since not used.
    _yawspeed_sp = 0.0f;
  }

  if (!PX4_ISFINITE(_yaw_sp)) {
    // Set the yaw-sp equal the current yaw.
    // That is the best we can do and it also
    // agrees with FlightTask-interface definition.
    if (PX4_ISFINITE(_yaw)) {
      _yaw_sp = _yaw;
    } else {
      failsafe = true;
    }
  }

  // check failsafe
  if (failsafe) {
    // point the thrust upwards
    _thr_sp(0) = _thr_sp(1) = 0.0f;
   // throttle down such that vehicle goes down with
   // 70% of throttle range between min and hover
   _thr_sp(2) = -(MPC_THR_MIN + (MPC_THR_HOVER - MPC_THR_MIN) * 0.7f);
  }

  return !(failsafe);
}

void PositionControl::_positionController() {
  // P-position controller
  const vec3 vel_sp_position = (_pos_sp - _pos).cwiseProduct(vec3(MPC_XY_P, MPC_XY_P, MPC_Z_P));
  _vel_sp = vel_sp_position + _vel_sp;

  // Constrain horizontal velocity by prioritizing the velocity component along the desired position setpoint over the feed-forward term.
  const vec2 vel_sp_xy = constrainXY(vec2(vel_sp_position(0), vel_sp_position(1)), vec2(_vel_sp(0) - vel_sp_position(0), _vel_sp(1) - vel_sp_position(1)), _constraints.speed_xy);
  _vel_sp(0) = vel_sp_xy(0);
  _vel_sp(1) = vel_sp_xy(1);

  // Constrain velocity in z-direction.
  _vel_sp(2) = constrain(_vel_sp(2), -_constraints.speed_up, _constraints.speed_down);
}

void PositionControl::_velocityController(const scalar_t &dt) {
  // Generate desired thrust setpoint.
  // PID
  // u_des = P(vel_err) + D(vel_err_dot) + I(vel_integral)
  // Umin <= u_des <= Umax
  //
  // Anti-Windup:
  // u_des = _thr_sp; r = _vel_sp; y = _vel
  // u_des >= Umax and r - y >= 0 => Saturation = true
  // u_des >= Umax and r - y <= 0 => Saturation = false
  // u_des <= Umin and r - y <= 0 => Saturation = true
  // u_des <= Umin and r - y >= 0 => Saturation = false
  //
  // Notes:
  // - PID implementation is in NED-frame
  // - control output in D-direction has priority over NE-direction
  // - the equilibrium point for the PID is at hover-thrust
  // - the maximum tilt cannot exceed 90 degrees. This means that it is not possible to have a desired thrust direction pointing in the positive D-direction (= downward)
  // - the desired thrust in D-direction is limited by the thrust limits
  // - the desired thrust in NE-direction is limited by the thrust excess after consideration of the desired thrust in D-direction. In addition, the thrust in NE-direction is also limited by the maximum tilt.

  const vec3 vel_err = _vel_sp - _vel;

  // Consider thrust in D-direction.
  scalar_t thrust_desired_D = MPC_Z_VEL_P * vel_err(2) +  MPC_Z_VEL_D * _vel_dot(2) + _thr_int(2) - MPC_THR_HOVER;

  // The Thrust limits are negated and swapped due to NED-frame.
  scalar_t uMax = -MPC_THR_MIN;
  scalar_t uMin = -MPC_THR_MAX;

  // Apply Anti-Windup in D-direction.
  bool stop_integral_D = (thrust_desired_D >= uMax && vel_err(2) >= 0.0f) || (thrust_desired_D <= uMin && vel_err(2) <= 0.0f);

  if (!stop_integral_D) {
    _thr_int(2) += vel_err(2) * MPC_Z_VEL_I * dt;

    // limit thrust integral
    _thr_int(2) = min(fabs(_thr_int(2)), MPC_THR_MAX) * sign(_thr_int(2));
  }

  // Saturate thrust setpoint in D-direction.
  _thr_sp(2) = constrain(thrust_desired_D, uMin, uMax);

  if (PX4_ISFINITE(_thr_sp(0)) && PX4_ISFINITE(_thr_sp(1))) {
    // Thrust set-point in NE-direction is already provided. Only
    // scaling by the maximum tilt is required.
    scalar_t thr_xy_max = fabsf(_thr_sp(2)) * tanf(_constraints.tilt);
    _thr_sp(0) *= thr_xy_max;
    _thr_sp(1) *= thr_xy_max;

  } else {
    // PID-velocity controller for NE-direction.
    vec2 thrust_desired_NE;
    thrust_desired_NE(0) = MPC_XY_VEL_P * vel_err(0) + MPC_XY_VEL_D * _vel_dot(0) + _thr_int(0);
    thrust_desired_NE(1) = MPC_XY_VEL_P * vel_err(1) + MPC_XY_VEL_D * _vel_dot(1) + _thr_int(1);

    // Get maximum allowed thrust in NE based on tilt and excess thrust.
    scalar_t thrust_max_NE_tilt = fabsf(_thr_sp(2)) * tanf(_constraints.tilt);
    scalar_t thrust_max_NE = sqrtf(MPC_THR_MAX * MPC_THR_MAX - _thr_sp(2) * _thr_sp(2));
    thrust_max_NE = min(thrust_max_NE_tilt, thrust_max_NE);

    // Saturate thrust in NE-direction.
    _thr_sp(0) = thrust_desired_NE(0);
    _thr_sp(1) = thrust_desired_NE(1);

    if (thrust_desired_NE.dot(thrust_desired_NE) > thrust_max_NE * thrust_max_NE) {
      scalar_t mag = thrust_desired_NE.norm();
      _thr_sp(0) = thrust_desired_NE(0) / mag * thrust_max_NE;
      _thr_sp(1) = thrust_desired_NE(1) / mag * thrust_max_NE;
    }

    // Use tracking Anti-Windup for NE-direction: during saturation, the integrator is used to unsaturate the output see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
    scalar_t arw_gain = 2.f / MPC_XY_VEL_P;

    vec2 vel_err_lim;
    vel_err_lim(0) = vel_err(0) - (thrust_desired_NE(0) - _thr_sp(0)) * arw_gain;
    vel_err_lim(1) = vel_err(1) - (thrust_desired_NE(1) - _thr_sp(1)) * arw_gain;

    // Update integral
    _thr_int(0) += MPC_XY_VEL_I * vel_err_lim(0) * dt;
    _thr_int(1) += MPC_XY_VEL_I * vel_err_lim(1) * dt;
  }
}

void PositionControl::updateConstraints(const Constraints& constraints) {
  _constraints = constraints;

  // For safety check if adjustable constraints are below global constraints. If they are not stricter than global
  // constraints, then just use global constraints for the limits.

//   if (!PX4_ISFINITE(constraints.tilt)
//     || !(constraints.tilt < math::max(MPC_TILTMAX_AIR_rad.get(), MPC_MAN_TILT_MAX_rad.get()))) {
// 		_constraints.tilt = math::max(MPC_TILTMAX_AIR_rad.get(), MPC_MAN_TILT_MAX_rad.get());
// 	}
// 
// 	if (!PX4_ISFINITE(constraints.speed_up) || !(constraints.speed_up < MPC_Z_VEL_MAX_UP.get())) {
// 		_constraints.speed_up = MPC_Z_VEL_MAX_UP.get();
// 	}
// 
// 	if (!PX4_ISFINITE(constraints.speed_down) || !(constraints.speed_down < MPC_Z_VEL_MAX_DN.get())) {
// 		_constraints.speed_down = MPC_Z_VEL_MAX_DN.get();
// 	}
// 
// 	if (!PX4_ISFINITE(constraints.speed_xy) || !(constraints.speed_xy < MPC_XY_VEL_MAX.get())) {
// 		_constraints.speed_xy = MPC_XY_VEL_MAX.get();
// 	}
}

}