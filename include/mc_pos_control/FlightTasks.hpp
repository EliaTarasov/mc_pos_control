/**
 * 
 * Library class to hold and manage all implemented flight task instances
 *
 */

#ifndef FLIGHT_TASKS_HPP
#define FLIGHT_TASKS_HPP

#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/ManualControl.h>
#include <common.h>

namespace mc_pos_control {

class FlightTasks {
public:
  FlightTasks();
  virtual ~FlightTasks() = default;

  /**
   * Call regularly in the control loop cycle to execute the task
   * @return true on success, false on error
   */
  virtual bool update();
  virtual bool activate();
  virtual bool updateInitialize();

  /**
   * Get the output data from the current task
   * @return output setpoint, to be executed by position control
   */
  virtual const mavros_msgs::PositionTarget getPositionSetpoint();

  /**
   * Get vehicle constraints.
   * The constraints can vary with task.
   * @return constraints
   */
  virtual const Constraints& getConstraints();

  virtual void updateSticks(const mavros_msgs::ManualControlConstPtr& manual);

  /**
   * Update the current vehicle state.
   * @param PositionControlStates structure
   */
  void updateState(const PositionControlStates& states);

protected:

  /**
   * Reset all setpoints to NAN
   */
  void _resetSetpoints();

  /* Current vehicle state */
  vec3 _position; /**< current vehicle position */
  vec3 _velocity; /**< current vehicle velocity */
  scalar_t _yaw = 0; /**< current vehicle yaw heading */

  /**
   * Setpoints which the position controller has to execute.
   * Setpoints that are set to NAN are not controlled. Not all setpoints can be set at the same time.
   * If more than one type of setpoint is set, then order of control is a as follow: position, velocity, acceleration, thrust. 
   * The exception is _position_setpoint together with _velocity_setpoint, where the _velocity_setpoint is used as feedforward.
   * _acceleration_setpoint and _jerk_setpoint are currently not supported.
   */

  vec3 _position_setpoint;
  vec3 _velocity_setpoint;
  vec3 _thrust_setpoint;
  scalar_t _yaw_setpoint;
  scalar_t _yawspeed_setpoint;

  Constraints _constraints;
};

}

#endif