#ifndef POSITION_CONTROL_HPP_
#define POSITION_CONTROL_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>

#include <mavros_msgs/PositionTarget.h>
#include <common.h>

#include <BlockDerivative.hpp>

namespace mc_pos_control {

  /**
   * 	Core Position-Control for MC.
   * 	This class contains P-controller for position and
   * 	PID-controller for velocity.
   * 	Inputs:
   * 		vehicle position/velocity/yaw
   * 		desired set-point position/velocity/thrust/yaw/yaw-speed
   * 		constraints that are stricter than global limits
   * 	Output
   * 		thrust vector and a yaw-setpoint
   *
   * 	If there is a position and a velocity set-point present, then
   * 	the velocity set-point is used as feed-forward. If feed-forward is
   * 	active, then the velocity component of the P-controller output has
   * 	priority over the feed-forward component.
   *
   * 	A setpoint that is NAN is considered as not set.
  */
  class PositionControl {
  public:
    PositionControl() = default;
    ~PositionControl() = default;

    /**
     * Update the current vehicle state.
     * @param PositionControlStates structure
     */
    void updateState(const PositionControlStates& states);

    /**
     * Update the desired setpoints.
     * @param setpoint a mavros_msgs::PositionTarget structure
     * @return true if setpoint has updated correctly
     */
    bool updateSetpoint(const mavros_msgs::PositionTarget& setpoint);

    /**
     * Set constraints that are stricter than the global limits.
     * @param constraints a PositionControl structure with supported constraints
     */
    void updateConstraints(const Constraints& constraints);

    /**
     * Apply P-position and PID-velocity controller that updates the member
     * thrust, yaw- and yawspeed-setpoints.
     * @see _thr_sp
     * @see _yaw_sp
     * @see _yawspeed_sp
     * @param dt the delta-time
     */
    void generateThrustYawSetpoint(const scalar_t dt);

    /**
     * 	Set the integral term in xy to 0.
     * 	@see _thr_int
     */
    void resetIntegralXY() { _thr_int(0) = _thr_int(1) = 0.0f; }

    /**
     * 	Set the integral term in z to 0.
     * 	@see _thr_int
     */
    void resetIntegralZ() { _thr_int(2) = 0.0f; }

    /**
     * 	Get the
     * 	@see _thr_sp
     * 	@return The thrust set-point member.
     */
    const vec3& getThrustSetpoint() { return _thr_sp; }

    /**
     * 	Get the
     * 	@see _yaw_sp
     * 	@return The yaw set-point member.
     */
    const scalar_t& getYawSetpoint() { return _yaw_sp; }

    /**
     * 	Get the
     * 	@see _yawspeed_sp
     * 	@return The yawspeed set-point member.
     */
    const scalar_t& getYawspeedSetpoint() { return _yawspeed_sp; }

    /**
     * 	Get the
     * 	@see _vel_sp
     * 	@return The velocity set-point member.
     */
    const vec3& getVelSp() { return _vel_sp; }

    /**
     * 	Get the
     * 	@see _pos_sp
     * 	@return The position set-point member.
     */
    const vec3& getPosSp() { return _pos_sp; }

    void setDt(scalar_t dt) {
      _vel_x_deriv.setDt(dt);
      _vel_y_deriv.setDt(dt);
      _vel_z_deriv.setDt(dt);
    }

  private:
    /**
     * Maps setpoints to internal-setpoints.
     * @return true if mapping succeeded.
     */
    bool _interfaceMapping();

    void _positionController(); /** applies the P-position-controller */
    void _velocityController(const scalar_t &dt); /** applies the PID-velocity-controller */

    vec3 _pos{}; /**< MC position */
    vec3 _vel{}; /**< MC velocity */
    vec3 _vel_dot{}; /**< MC velocity derivative */
    scalar_t _yaw; /**< MC yaw */
    vec3 _pos_sp{}; /**< desired position */
    vec3 _vel_sp{}; /**< desired velocity */
    vec3 _thr_sp{}; /**< desired thrust */
    scalar_t _yaw_sp{}; /**< desired yaw */
    scalar_t _yawspeed_sp{}; /** desired yaw-speed */
    vec3 _thr_int{}; /**< thrust integral term */
    bool _skip_controller{false}; /**< skips position/velocity controller. true for stabilized mode */

    scalar_t MPC_THR_MAX = 0.9;
    scalar_t MPC_THR_HOVER = 0.5;
    scalar_t MPC_THR_MIN = 0.12;
    scalar_t MPC_MANTHR_MIN = 0.08;
    scalar_t MPC_XY_VEL_MAX = 12;
    scalar_t MPC_Z_VEL_MAX_DN = 1;
    scalar_t MPC_Z_VEL_MAX_UP = 2;
    scalar_t MPC_TILTMAX_AIR = 7;
    scalar_t MPC_MAN_TILT_MAX = 7;
    scalar_t MPC_TILTMAX_AIR_rad = MPC_TILTMAX_AIR * 0.0174533; // maximum tilt for any position controlled mode in radians
    scalar_t MPC_MAN_TILT_MAX_rad = MPC_MAN_TILT_MAX * 0.0174533; // maximum til for stabilized/altitude mode in radians
    scalar_t MPC_Z_P = 1.0;
    scalar_t MPC_Z_VEL_P = 0.17;
    scalar_t MPC_Z_VEL_I = 0.08;
    scalar_t MPC_Z_VEL_D = 0.00;
    scalar_t MPC_XY_P = 0.95;
    scalar_t MPC_XY_VEL_P = 0.09;
    scalar_t MPC_XY_VEL_I = 0.02;
    scalar_t MPC_XY_VEL_D = 0.01;

    Constraints _constraints;

    BlockDerivative _vel_x_deriv; /**< velocity derivative in x */
    BlockDerivative _vel_y_deriv; /**< velocity derivative in y */
    BlockDerivative _vel_z_deriv; /**< velocity derivative in z */
};

}
#endif // MC_POS_CONTROL_HPP_