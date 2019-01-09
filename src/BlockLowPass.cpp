#include <math.h>
#include <float.h>

#include <BlockLowPass.hpp>

namespace mc_pos_control {

scalar_t BlockLowPass::update(scalar_t input) {
  if (!PX4_ISFINITE(getState())) {
    setState(input);
  }

  scalar_t b = 2 * scalar_t(M_PI) * getFCut() * getDt();
  scalar_t a = b / (1 + b);
  setState(a * input + (1 - a) * getState());
  return getState();
}

} // namespace control
