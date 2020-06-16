#define ENCODER_OPTIMIZE_INTERRUPTS
#include <open_arms_common.h>
#include <open_arms_stepper.h>
#include <Encoder.h>
//#include <open_arms_joint.h>

Stepper motor(JOINT5_STP, JOINT5_DIR, 1.8);
Encoder encoder(ENC5_A, ENC5_B);
float ini_angle = 0;
float old_angle = 0;
float cur_angle = 0;
int miss_step = 0;
int old_enc_pos = 0;
int cur_enc_pos = 0;
float target_angle = 36;
bool done = false;

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  motor.setRPM(100);
  while (!done)
  {
    if (motor.getIterCounter() == 0 && target_angle > ini_angle)
    {
      motor.moveStepCW();
      cur_enc_pos = encoder.read();
      cur_angle = cur_angle + (360.0/(26.0*320.0)) * (cur_enc_pos - old_enc_pos);
      if(cur_angle <= old_angle)
      {
        miss_step++;
      }
      else
      {
        miss_step = 0;
      }
      Serial.println(cur_angle);
      if (cur_angle >= target_angle)
      {
        done = true;
      }
    }
    else if (motor.getIterCounter() == 0 && target_angle < ini_angle)
    {
      motor.moveStepCCW();
      cur_enc_pos = encoder.read();
      cur_angle = cur_angle + (360.0/(26.0*320.0)) * (cur_enc_pos - old_enc_pos);
      if(cur_angle >= old_angle)
      {
        miss_step++;
      }
      else
      {
        miss_step = 0;
      }
      Serial.println(cur_angle);
      if (cur_angle <= target_angle)
      {
        done = true;
      }
    }
    else if (motor.getIterCounter() == 0 && target_angle == ini_angle)
    {
      done = true;
    }

    else
    {
      motor.restStep();
    }
    old_enc_pos = cur_enc_pos;
    old_angle = cur_angle;

    if (miss_step > 3)
    {
      done = true;
      Serial.println("Over torque");
    }
    if (motor.getIterCounter() >= motor.getIteration())
    {
      motor.resetIterCounter();
    }
    delay(1);
  }
}
