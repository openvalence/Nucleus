#pragma once

// arbiter_rig -- the door from test_main.cpp to the real MotionArbiter in
// arbiter_rig.cpp
// Constraints:
// - Test thread only. rigArbiterReset() releases the latch and zeroes the
//   park count; the arbiter itself lives for the whole binary.

void rigArbiterReset();
// MotionArbiter::estop(): on parks the emitter on this thread before it returns.
void rigArbiterEstop(bool on);
bool rigArbiterLatched();
int rigArbiterParks();
