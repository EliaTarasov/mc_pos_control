/**
 *
 * Flight task for manual position controlled mode.
 *
 */

#ifndef FLIGHT_TASK_MANUAL_POSITION_HPP
#define FLIGHT_TASK_MANUAL_POSITION_HPP

#include <FlightTaskManualAltitude.hpp>

namespace mc_pos_control {

class FlightTaskManualPosition : public FlightTaskManualAltitude {
public:
  FlightTaskManualPosition() = default;
  virtual ~FlightTaskManualPosition() = default;

  bool update() override;
  bool activate() override;
  bool updateInitialize() override;

protected:
  void _updateXYlock(); /**< applies position lock based on stick and velocity */
  void _updateSetpoints() override;
  void _scaleSticks() override;

  scalar_t MPC_VEL_MANUAL = 10.0;
  scalar_t MPC_ACC_HOR_MAX = 5.0;
  scalar_t MPC_HOLD_MAX_XY = 0.8;
  scalar_t MPC_ACC_HOR_ESTM = 0.5;

private:
  scalar_t _velocity_scale{0.0f}; //scales the stick input to velocity
  // timestamps
  ros::Time prevStamp_;
};

}

#endif