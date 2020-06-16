#include <open_arms_joint.h>

//all motors
Stepper motor1(JOINT1_STP, JOINT1_DIR, JOINT1_ENB, STEP_ANGLE);
Stepper motor2(JOINT2_STP, JOINT2_DIR, JOINT2_ENB, STEP_ANGLE);
Stepper motor3(JOINT3_STP, JOINT3_DIR, JOINT3_ENB, STEP_ANGLE);
Stepper motor4(JOINT4_STP, JOINT4_DIR, JOINT4_ENB, STEP_ANGLE);
Stepper motor5(JOINT5_STP, JOINT5_DIR, JOINT5_ENB, STEP_ANGLE);
Stepper motor6(JOINT6_STP, JOINT6_DIR, JOINT6_ENB, STEP_ANGLE);
Stepper motor7(JOINT7_STP, JOINT7_DIR, JOINT7_ENB, STEP_ANGLE);
//all encoders
Encoder encoder1(ENC1_A, ENC1_B);
Encoder encoder2(ENC2_A, ENC2_B);
Encoder encoder3(ENC3_A, ENC3_B);
Encoder encoder4(ENC4_A, ENC4_B);
Encoder encoder5(ENC5_A, ENC5_B);
Encoder encoder6(ENC6_A, ENC6_B);
//all joints except for the end effector joint have encoders
ClosedJoint joint1(motor1, encoder1, 0);
ClosedJoint joint2(motor2, encoder2, 0);
ClosedJoint joint3(motor3, encoder3, 0);
ClosedJoint joint4(motor4, encoder4, 0);
ClosedJoint joint5(motor5, encoder5, 0);
ClosedJoint joint6(motor6, encoder6, 0);
OpenJoint joint7(motor7, 0);
//collection of all 7 motors
Joint *joints[7] = {&joint1, &joint2, &joint3, &joint4, &joint5, &joint6, &joint7};

float targets[7] = {1, 1.5, -0.5, 0.5, -1, 0.75, 0.125}; //for testing purpose
bool is_overtorqued = false;                             //false if no joints encounter collision, true if at least one of them does

void setup()
{
  //deterine direction of rotation for each joint
  for (int i = 0; i < 7; i++)
  {
    joints[i]->setDirection(targets[i]);
  }
}

void loop()
{
  for (int i = 0; i < 7; i++)
  {
    if (!joints[i]->getCompletionFlag() && !is_overtorqued)
    { //only move the joint if the joint hasn't completed its motion
      //and if no collision has occured to the joint
      if (i == 6)
      { //the joint without encoder
        joints[i]->moveJoint();
      }
      else if (!joints[i]->isOvertorqued())
      { //move the joints with encoder if no collision occurs to them
        joints[i]->moveJoint();
      }
      else
      { //collision has happend to at least one of the joint
        //essentially disable motion for the next iteration
        is_overtorqued = true;
      }
    }
  }
}
