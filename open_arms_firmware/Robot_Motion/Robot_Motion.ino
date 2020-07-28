#include <ros.h>
#include <open_arms_joint.h>
#include <open_arms_driver/RobotAngleRelay.h>

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
ClosedJoint joint1(motor1, encoder1, 0, 4);
ClosedJoint joint2(motor2, encoder2, 0, 1.25);
ClosedJoint joint3(motor3, encoder3, 0);
ClosedJoint joint4(motor4, encoder4, 0);
ClosedJoint joint5(motor5, encoder5, 0);
ClosedJoint joint6(motor6, encoder6, 0);
OpenJoint joint7(motor7, 0);
//collection of all 7 motors
Joint *joints[7] = {&joint1, &joint2, &joint3, &joint4, &joint5, &joint6, &joint7};

bool moveMotor(float targets[], unsigned short int num_motor = 7)
{                                                                               //the main method to move all joints given their goal angles
  bool is_overtorqued = false;                                                  //a flag to indicate if any joints encounter collision
  unsigned short int completed_motor = 0;                                       //number of motor that completes its motion
  bool motor_completion[7] = {false, false, false, false, false, false, false}; //indicates which motor has completed its motion
  //deterine direction of rotation for each joint
  for (unsigned short int i = 0; i < joints[0]->getJointCount(); i++)
  {
    joints[i]->setDirection(targets[i]);
  }
  while (completed_motor < num_motor)
  { //move the motors
    for (unsigned short int i = 0; i < num_motor; i++)
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
        { //collision has happended to at least one of the joint
          //essentially disable motion for the next iteration
          is_overtorqued = true;
          return false;
        }
      }
      else if (joints[i]->getCompletionFlag() && !motor_completion[i])
      { //if one of the motor completes its rotation in the current iteration
        completed_motor++;
        motor_completion[i] = true;
      }
    }
  }
  return true; //when the program reaches here, all 7 motors have completed their motions
}

ros::NodeHandle nh; //set up node handler for ROS node
void motion(open_arms_driver::RobotAngleRelay::Request &req, open_arms_driver::RobotAngleRelay::Response &res)
{                                                                     //callback function of ROS service server
  bool completion = moveMotor(req.angle, joints[0]->getJointCount()); // move the motor
  if (completion)
  { //motion completed successfully
    res.motion_done = true;
    res.overtorque = false;
  }
  else
  { //collision occured
    res.motion_done = false;
    res.overtorque = true;
  }
}
//ROS service server
ros::ServiceServer<open_arms_driver::RobotAngleRelay::Request, open_arms_driver::RobotAngleRelay::Response> server("open_arms_robot", &motion);

void setup()
{
  Serial.begin(9600); //
  //disable and enable each motor individually to avoid current spike
  for (unsigned short int i = 0; i < joints[0]->getJointCount(); i++)
  {
    joints[i]->disableMotor();
    Serial.println("Disabled motor"); //
  }
  delay(2000);
  for (unsigned short int i = 0; i < joints[0]->getJointCount(); i++)
  {
    joints[i]->enableMotor();
    delay(2000);
    Serial.println("Enable motor"); //
    joints[i]->setRPM(100);         //
    Serial.println("RPM set");      //
  }
  nh.initNode();
  nh.advertiseService(server);
}

void loop()
{
  nh.spinOnce();
  delay(100);
}
