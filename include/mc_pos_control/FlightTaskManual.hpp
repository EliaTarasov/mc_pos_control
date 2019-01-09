/**
 * 
 * Linear and exponential map from stick inputs to range -1 and 1.
 *
 */

#ifndef FLIGHT_TASK_MANUAL_HPP
#define FLIGHT_TASK_MANUAL_HPP

#include <FlightTasks.hpp>
#include <common.h>

namespace mc_pos_control {

class FlightTaskManual : public FlightTasks {
public:
  FlightTaskManual() = default;
  virtual ~FlightTaskManual() = default;

  bool update() override;
  bool activate() override;
  bool updateInitialize() override;
  void updateSticks(const mavros_msgs::ManualControlConstPtr& manual) final;

protected:

  bool _sticks_data_required = true; /**< let inherited task-class define if it depends on stick data */
  vec4 _sticks_expo; /**< modified manual sticks using expo function*/
  vec4 _sticks; /**< unmodified manual stick inputs */

  scalar_t stickDeadzone() const { return MPC_HOLD_DZ; }

private:

  bool _evaluateSticks(); /**< checks and sets stick inputs */

  scalar_t MPC_HOLD_DZ = 0.1; /**< 0-deadzone around the center for the sticks */
  scalar_t MPC_XY_MAN_EXPO = 0.0; /**< ratio of exponential curve for stick input in xy direction */
  scalar_t MPC_Z_MAN_EXPO = 0.35; /**< ratio of exponential curve for stick input in z direction */
  scalar_t MPC_YAW_EXPO = 0.0; /**< ratio of exponential curve for stick input in yaw for modes except acro */
};

}

#endif