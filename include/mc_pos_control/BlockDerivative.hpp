#ifndef BLOCK_DERIVATIVE_HPP
#define BLOCK_DERIVATIVE_HPP

#include <BlockLowPass.hpp>
#include <math.h>

namespace mc_pos_control {

/**
 * A simple derivative approximation.
 * This uses the previous and current input.
 * This has a built in low pass filter.
 * @see LowPass
 */
class BlockDerivative {
public:
// methods
  BlockDerivative() : _u(0), _initialized(false), _lowPass() {}
  virtual ~BlockDerivative() {}

  /**
   * Update the state and get current derivative
   *
   * This call updates the state and gets the current
   * derivative. As the derivative is only valid
   * on the second call to update, it will return
   * no change (0) on the first. To get a closer
   * estimate of the derivative on the first call,
   * call setU() one time step before using the
   * return value of update().
   *
   * @param input the variable to calculate the derivative of
   * @return the current derivative
   */
  scalar_t update(scalar_t input);
// accessors
  void setU(scalar_t u) { _u = u; }
  scalar_t getU() { return _u; }
  scalar_t getLP() { return _lowPass.getFCut(); }
  scalar_t getO() { return _lowPass.getState(); }
  void setDt(scalar_t dt) { _lowPass.setDt(dt); }
protected:
// attributes
  scalar_t _u; /**< previous input */
  bool _initialized;
  BlockLowPass _lowPass; /**< low pass filter */
};

} // namespace mc_pos_control

#endif //BLOCK_DERIVATIVE_HPP