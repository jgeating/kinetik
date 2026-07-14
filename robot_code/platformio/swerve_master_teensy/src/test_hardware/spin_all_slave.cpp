// Hardware validation test - SLAVE Teensy 4.1
//
// Spins all four Moteus N1 motors on the SLAVE's CAN3 (CAN-FD) bus:
//   steer IDs 20, 3  |  drive IDs 10, 13
// See spin_all_common.h for behaviour, safety notes, and parameters.
//
// Build/upload: pio run -e test_spin_slave -t upload

#include "spin_all_common.h"

static const int SLAVE_MOTOR_IDS[4] = {20, 3, 10, 13};

void setup() {
  spinTestSetup(SLAVE_MOTOR_IDS, 4, "SLAVE (steer 20,3 + drive 10,13)");
}

void loop() {
  spinTestLoop();
}
