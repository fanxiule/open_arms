#include <ros/ros.h>
#include <algorithm>
#include <queue>
#include <vector>
#include <sensor_msgs/JointState.h>
#include <open_arms_driver/RobotAngleRelay.h>

std::queue<sensor_msgs::JointState> joint_states; //store joint_states that haven't been executed

void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    joint_states.push(*msg);                                   //store all of the angles from moveit to the container
    ROS_INFO("Queue size is: %lu", joint_states.size());       //
    sensor_msgs::JointState joint_state = joint_states.back(); //
    std::vector<double> joint_angles = joint_state.position;   //
    std::reverse(joint_angles.begin(), joint_angles.end());    //
    while (!joint_angles.empty())                              //
    {                                                          //
        ROS_INFO("Angle: %f", joint_angles.back());            //
        joint_angles.pop_back();                               //
    }                                                          //
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_states_relay");
    ros::NodeHandle nh;
    ros::Subscriber joint_states_sub = nh.subscribe("/joint_states", 100, jointStateCallback);          //subscriber to collect joint angles from MoveIt
    ros::ServiceClient client = nh.serviceClient<open_arms_driver::RobotAngleRelay>("open_arms_robot"); //a service client to request Arduino to move the robot
    while (!joint_states.empty())
    { //keep calling the service if the joints given by MoveIt are not all met yet
        open_arms_driver::RobotAngleRelay srv;
        sensor_msgs::JointState state = joint_states.front(); //get the first joint state in the queue
        joint_states.pop();
        srv.request.name = state.name;
        for (int i = 0; i < state.position.size(); i++)
        { //convert angle values by MoveIt from double to float
            srv.request.angle[i] = float(state.position[i]);
        }
        client.call(srv); //send request to the server
        if (srv.response.overtorque)
        { //if collision occured
            std::queue<sensor_msgs::JointState> empty_queue;
            std::swap(joint_states, empty_queue); //clear the joint_states queue
            break;                                //stop future motion
        }
    }
    ros::spin();
    return 0;
}