// ============================================================
// MOTOR ADAPTER: I2C motor controller (professor's MecanumCar_v2 library)
// ============================================================
// The sketch only calls motorsBegin(), motorsStop() and motorsMove().
// A future direct-PWM robot gets its own motor_pwm.h with the same three
// functions (docs/specs/workspace_drive.md, section 5). Not written yet.
#pragma once
#include "MecanumCar_v2.h"

enum Move { FORWARD, BACKWARD, LEFT, RIGHT, ROT_LEFT, ROT_RIGHT };

// I2C controller at 0x30, SDA GPIO19, SCL GPIO21 (as in the professor's sketch)
mecanumCar mecanumCar(19, 21);

void motorsStop() {
  mecanumCar.Stop();
}

void motorsBegin() {
  motorsStop();
}

// speed 0..255 (the library's default is 80)
void motorsMove(Move m, uint8_t speed) {
  speed_Upper_L = speed;
  speed_Lower_L = speed;
  speed_Upper_R = speed;
  speed_Lower_R = speed;
  switch (m) {
    case FORWARD:   mecanumCar.Advance();    break;
    case BACKWARD:  mecanumCar.Back();       break;
    case LEFT:      mecanumCar.L_Move();     break;
    case RIGHT:     mecanumCar.R_Move();     break;
    case ROT_LEFT:  mecanumCar.Turn_Left();  break;
    case ROT_RIGHT: mecanumCar.Turn_Right(); break;
  }
}
