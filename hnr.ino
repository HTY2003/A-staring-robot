#include <EVN.h>

#define MOTOR_PORT_LEFT 4
#define MOTOR_PORT_RIGHT 1

EVNAlpha board(BUTTON_TOGGLE, true, true);
EVNMotor left(MOTOR_PORT_LEFT, CUSTOM_MOTOR, DIRECT, REVERSE);
EVNMotor right(MOTOR_PORT_RIGHT, CUSTOM_MOTOR, REVERSE, REVERSE);
EVNDrivebase db(58, 168, &left, &right);

void setup1() {
  left.begin();
  right.begin();
  db.begin();
}

void setup() {
  delay(15000);
  board.begin();
  Serial1.begin(115200);
}

void loop() {
  uint32_t start_time_ms = millis();

  if (Serial1.available())
  {
    uint8_t info = Serial1.read();

    if (info == 181)
    {
      while (!Serial1.available());
      int angles = (int) Serial1.read();
      db.turn(15, angles, STOP_BRAKE, false);
    }
    else if (info == 182)
    {
      while (!Serial1.available());
      int angles = (int) Serial1.read();
      db.turn(15, -angles, STOP_BRAKE, false);
    }
  }
}