/**
 * 
 * Flight task for manual controlled altitude.
 * 
 */

#ifndef FLIGTH_TASK_MANUAL_ALTITUDE_HPP
#define FLIGHT_TASK_MANUAL_ALTITUDE_HPP

#include <FlightTaskManual.hpp>

namespace mc_pos_control {

class FlightTaskManualAltitude : public FlightTaskManual {
public:
  FlightTaskManualAltitude() = default;
  virtual ~FlightTaskManualAltitude() = default;

  bool activate() override;
  bool updateInitialize() override;
  bool update() override;

protected:
  void _updateHeadingSetpoints(); /**< sets yaw or yaw speed */
  virtual void _updateSetpoints(); /**< updates all setpoints */
  virtual void _scaleSticks(); /**< scales sticks to velocity in z */
  void _updateAltitudeLock();

  /**
   * rotates vector into local frame
   */
  void _rotateIntoHeadingFrame(vec2 &vec);

private:
  uint8_t _reset_counter = 0; /**< counter for estimator resets in z-direction */
  scalar_t _max_speed_up = 10.0f;
  scalar_t _min_speed_down = 1.0f;

  scalar_t MPC_HOLD_MAX_Z = 0.6;
  scalar_t MPC_HOLD_MAX_XY = 0.8;
  scalar_t MPC_Z_P = 1.0; /**< position controller altitude propotional gain */
  scalar_t MPC_MAN_Y_MAX = 200; /**< scaling factor from stick to yaw rate */
  scalar_t MPC_MAN_TILT_MAX = 7.0; /**< maximum tilt allowed for manual flight */
  int MPC_ALT_MODE = 0;
};

}

#endif