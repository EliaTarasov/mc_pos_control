#ifndef BLOCK_LOW_PASS_HPP
#define BLOCK_LOW_PASS_HPP

#include <assert.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>

#include <common.h>

namespace mc_pos_control {

/**
 * A low pass filter as described here:
 * http://en.wikipedia.org/wiki/Low-pass_filter.
 */
class BlockLowPass {
public:
  // methods
  BlockLowPass() : _state(0.0f / 0.0f /* initialize to invalid val, force into is_finite() check on first call */) {}
  virtual ~BlockLowPass() {}
  scalar_t update(scalar_t input);
// accessors
  scalar_t getState() { return _state; }
  scalar_t getFCut() { return _fCut; }
  scalar_t getDt() { return _dt; }
  void setDt(scalar_t dt) { _dt = dt; }
  void setState(scalar_t state) { _state = state; }
protected:
// attributes
  scalar_t _state;
  scalar_t _fCut = 5;
  scalar_t _dt = 0.02;
};

} // namespace mc_pos_control

#endif //BLOCK_LOW_PASS_HPP