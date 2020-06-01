#include <Arduino.h>
#include "open_arms_common.h"
#include "open_arms_joint.h"

unsigned short int Joint::counter = 0;

void Joint::setDirection(float targetAngle)
{ //set the direction of rotation of the joint
    motor.resetIterCounter();
    if (targetAngle > angle)
    { //motor needs to rotate CCW when target is larger than current angle
        is_CCW = true;
        motor.setCCW();
        task_completed = false;
    }
    else if (targetAngle < angle)
    { //motor needs to rotate CW when target is smaller than current angle
        is_CCW = false;
        motor.setCW();
        task_completed = false;
    }
    else
    {
        task_completed = true; //no need to move the motor if target and current are equal
    }
}

OpenJoint::OpenJoint(Stepper &joint_motor, float ini_angle)
{
    motor = joint_motor;
    angle = ini_angle;
    task_completed = true;
    counter++;
}

OpenJoint::~OpenJoint()
{
    counter--;
}

void OpenJoint::moveJoint(float target_angle)
{
    if (task_completed || angle == target_angle)
    {
        task_completed = true;
        return;
    }
    if (motor.getIterCounter() == 0)
    { //only move step when the current iteration number is at the beginning of a PWM cycle
        motor.moveStep();
        //change curret angle of the joint
        //since the joint is direct drive, no need to include gear reduction
        if (is_CCW)
        {
            angle = angle + STEP_ANGLE * DEG_TO_RAD;
        }
        else
        {
            angle = angle - STEP_ANGLE * DEG_TO_RAD;
        }
    }
    else //motor does not rotate for the rest of PWM cycle
    {
        motor.restStep();
    }
    if (motor.getIterCounter() >= motor.getIteration())
    { //reset iter_counter of motor when it reaches the end of a PWM cycle
        motor.resetIterCounter();
    }
    //break the for loop when target angle is reached
    if (is_CCW && angle >= target_angle)
    {
        task_completed = true;
    }
    else if (!is_CCW && angle <= target_angle)
    {
        task_completed = true;
    }
}

ClosedJoint::ClosedJoint(Stepper &joint_motor, Encoder &joint_encoder, float ini_angle)
{
    motor = joint_motor;
    encoder = joint_encoder;
    angle = ini_angle;
    task_completed = true;
    net_reduction = CYCLOIDAL_REDUCTION;
    encoder_pos = 0;
    missed_step = 0;
    counter++;
}

ClosedJoint::ClosedJoint(Stepper &joint_motor, Encoder &joint_encoder, float ini_angle, float extra_red)
{
    motor = joint_motor;
    encoder = joint_encoder;
    angle = ini_angle;
    task_completed = true;
    net_reduction = CYCLOIDAL_REDUCTION * extra_red;
    encoder_pos = 0;
    missed_step = 0;
    counter++;
}

ClosedJoint::~ClosedJoint()
{
    counter--;
}

bool ClosedJoint::isOvertorqued()
{
    if (missed_step > 3)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void ClosedJoint::moveJoint(float target_angle)
{
    if (task_completed || angle == target_angle)
    {
        task_completed = true;
        return;
    }
    if (motor.getIterCounter() == 0)
    {
        motor.moveStep();
        short int new_enc_pos = encoder.read();
        float angle_change = (360 / (net_reduction * ENC_REDUCTION * ENC_RESOLUTION)) * (new_enc_pos - encoder_pos);
        float new_angle = angle + angle_change * DEG_TO_RAD;
        encoder_pos = new_enc_pos;
        if (is_CCW)
        {
            if (new_angle <= angle)
            {
                missed_step++;
            }
            else
            {
                missed_step = 0;
            }
            if (new_angle >= target_angle)
            {
                task_completed = true;
            }
        }
        else
        {
            if (new_angle >= angle)
            {
                missed_step++;
            }
            else
            {
                missed_step = 0;
            }
            if (new_angle <= target_angle)
            {
                task_completed = true;
            }
        }
        angle = new_angle;
    }
    else
    {
        motor.restStep();
    }
    if (motor.getIterCounter() >= motor.getIteration())
    {
        motor.resetIterCounter();
    }
    if (isOvertorqued())
    {
        task_completed = true;
    }
}