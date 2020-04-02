#include <open_arms_common.h>
#include <OpenArmsStepper.h>
#include <ros.h>
#include <std_msgs/Int32.h>

Stepper gripper(22, 23, 3, 1.8); //stp, dir, enable, angle //0.25A
Stepper hand_rot(24, 25, 4, 1.8); //stp, dir, angle
Stepper hand_piv(26, 27, 5, 1.8);
Stepper elbow_rot(28, 29, 6, 1.8); //0.75A
Stepper elbow_piv(30, 31, 7, 1.8); //0.75A
Stepper base_rot(32, 33, 8, 1.8);
Stepper base_piv(34, 35, 9, 1.8); //0.35 A
//Stepper* Motor[7] = {&gripper, &hand_rot, &hand_piv, &elbow_rot, &elbow_piv, &base_rot, &base_piv}; //
Stepper* Motor[7] = {&base_rot, &base_piv, &elbow_piv, &elbow_rot, &hand_piv, &hand_rot, &gripper};
int rev_counter[7] = {0, 0, 0, 0, 0, 0, 0};
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
      if (Motor[i]->rev_counter == 0)
      {
        Motor[i]->moveStepCW();
      }
      else
      {
        Motor[i]->restStep();
      }

      if (Motor[i]->rev_counter >= Motor[i]->iteration)
      {
        Motor[i]->rev_counter = 0;
      }
    }
     //nh.spinOnce();
  }
     //nh.spinOnce();
  delay(1);
}
