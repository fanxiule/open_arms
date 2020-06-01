#include <OpenArmsStepper.h>
Stepper motor(23, 22, 2, 1.8);

void setup() {
  Serial.begin(9600);
  motor.disableMotor();
  delay(2000);
  motor.enableMotor();
  delay(2000);
}

void loop() {
  //motor.moveRev(false, 8);
  delay(1000);
  motor.moveRev(true, 8);
  delay(1000);
}
