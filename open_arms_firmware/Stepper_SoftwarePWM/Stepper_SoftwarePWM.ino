#include <open_arms_common.h>
#include <open_arms_stepper.h>
#include <ros.h>
#include <std_msgs/Int32.h>

Stepper base_piv(JOINT1_STP, JOINT1_DIR, JOINT1_ENB, STEP_ANGLE); //0.35A
Stepper base_rot(JOINT2_STP, JOINT2_DIR, JOINT2_ENB, STEP_ANGLE);
Stepper elbow_piv(JOINT3_STP, JOINT3_DIR, JOINT3_ENB, STEP_ANGLE); //0.75A
Stepper elbow_rot(JOINT4_STP, JOINT4_DIR, JOINT4_ENB, STEP_ANGLE); //0.75A
Stepper hand_piv(JOINT5_STP, JOINT5_DIR, JOINT5_ENB, STEP_ANGLE);
Stepper hand_rot(JOINT6_STP, JOINT6_DIR, JOINT6_ENB, STEP_ANGLE);
Stepper gripper(JOINT7_STP, JOINT7_DIR, JOINT7_ENB, STEP_ANGLE); //0.25A
Stepper* Motor[7] = {&base_rot, &base_piv, &elbow_piv, &elbow_rot, &hand_piv, &hand_rot, &gripper};
bool move = false;

//ros::NodeHandle nh;
//std_msgs::Int32 int_msg;

/*void messageCb(const std_msgs::Int32& msg)
{
  if(msg.data == 10)
  {
    move = false;
  }

  if(msg.data == 20)
  {
    move = true;
  }
}*/

//ros::Subscriber<std_msgs::Int32> sub("command", &messageCb);

void setup()
{
  //Serial.begin(57600);
  //nh.initNode();
  //nh.getHardware()->setBaud(57600);
  //nh.subscribe(sub);
  for (unsigned short int i = 0; i<gripper.getStepperCount(); i++)
  {
    Motor[i]->disableMotor();
  }
  delay(2000);
  for (unsigned short int i = 0; i<gripper.getStepperCount(); i++)
  {
    Motor[i]->enableMotor();
    delay(2000);
  }
  Serial.begin(9600);
  delay(1000);
}

void loop()
{
  move = true;
  gripper.setRPM(10);
  hand_rot.setRPM(10);
  hand_piv.setRPM(10);
  elbow_piv.setRPM(10);
  elbow_rot.setRPM(10);
  base_rot.setRPM(10);
  base_piv.setRPM(10);
  
  while (move)
  {
    for (unsigned short int i = 0; i < gripper.getStepperCount(); i++)
    {
      if (Motor[i]->getIterCounter() == 0)
      {
        Motor[i]->moveStepCW();
      }
      else
      {
        Motor[i]->restStep();
      }

      if (Motor[i]->getIterCounter() >= Motor[i]->getIteration())
      {
        Motor[i]->resetIterCounter();
      }
    }
     //nh.spinOnce();
  }
     //nh.spinOnce();
  delay(1);
}
