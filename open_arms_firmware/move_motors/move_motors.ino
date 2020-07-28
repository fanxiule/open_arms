#include <ros.h>
#include <open_arms_common.h>
#include <open_arms_stepper.h>
#include <open_arms_driver/MoveMotorArduino.h>

Stepper joint1(JOINT1_STP, JOINT1_DIR, JOINT1_ENB, 1.8);
Stepper joint2(JOINT2_STP, JOINT2_DIR, JOINT2_ENB, 1.8);
Stepper joint3(JOINT3_STP, JOINT3_DIR, JOINT3_ENB, 1.8);
Stepper joint4(JOINT4_STP, JOINT4_DIR, JOINT4_ENB, 1.8);
Stepper joint5(JOINT5_STP, JOINT5_DIR, JOINT5_ENB, 1.8);
Stepper joint6(JOINT6_STP, JOINT6_DIR, JOINT6_ENB, 1.8);
Stepper joint7(JOINT7_STP, JOINT7_DIR, JOINT7_ENB, 1.8);
Stepper *motors[7] = {&joint1, &joint2, &joint3, &joint4, &joint5, &joint6, &joint7};

ros::NodeHandle nh;

void moveMotor(open_arms_driver::MoveMotorArduino::Request &req,
               open_arms_driver::MoveMotorArduino::Response &res)
{
  unsigned short int motor_ind = 1000;
  if (req.joint_name == "joint1")
  {
    motor_ind = 0;
  }
  else if (req.joint_name == "joint2")
  {
    motor_ind = 1;
  }
  else if (req.joint_name == "joint3")
  {
    motor_ind = 2;
  }
  else if (req.joint_name == "joint4")
  {
    motor_ind = 3;
  }
  else if (req.joint_name == "joint5")
  {
    motor_ind = 4;
  }
  else if (req.joint_name == "joint6")
  {
    motor_ind = 5;
  }
  else if (req.joint_name == "joint7")
  {
    motor_ind = 6;
  }

  //if (motor_ind != 1000)
  //{
    motors[motor_ind]->moveRev(req.direction, req.revolution);
    char str[8];
    itoa(motor_ind, str, 10);
    res.response = str;
    //res.response = "Arduino received valid request";
  //}
  //else
  //{
  //  res.response = "Arduino received invalid request";
  //}
}

ros::ServiceServer<open_arms_driver::MoveMotorArduino::Request, open_arms_driver::MoveMotorArduino::Response> service("move_motor_arduino", &moveMotor);

void setup()
{
  Serial.begin(57600);
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.advertiseService(service);
  for (unsigned short int i = 0; i<joint7.getStepperCount(); i++)
  {
    motors[i]->disableMotor();
  }
  delay(2000);
  for (unsigned short int i = 0; i<joint7.getStepperCount(); i++)
  {
    motors[i]->enableMotor();
    delay(2000);
  }
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
