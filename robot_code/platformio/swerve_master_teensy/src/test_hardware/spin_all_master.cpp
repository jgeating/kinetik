// Hardware validation test - MASTER Teensy 4.1
//
// Spins all four Moteus N1 motors on the MASTER's CAN3 (CAN-FD) bus:
//   steer IDs 1, 2  |  drive IDs 11, 12
// See spin_all_common.h for behaviour, safety notes, and parameters.
//
// Build/upload: pio run -e test_spin_master -t upload

#include "spin_all_common.h"

static const int MASTER_MOTOR_IDS[4] = {1, 2, 11, 12};

void setup() {
  spinTestSetup(MASTER_MOTOR_IDS, 4, "MASTER (steer 1,2 + drive 11,12)");
}

void loop() {
  spinTestLoop();
}
