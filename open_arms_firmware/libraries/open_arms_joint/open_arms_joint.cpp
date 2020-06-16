#include <Arduino.h>
#include "open_arms_common.h"
#include "open_arms_joint.h"

unsigned short int Joint::counter = 0;

void Joint::setDirection(float target)
{ //set the direction of rotation of the joint
    target_angle = target;
    pMotor->resetIterCounter();
    if (target_angle > angle)
    { //motor needs to rotate CCW when target is larger than current angle
        is_CCW = true;
        pMotor->setCCW();
        task_completed = false;
    }
    else if (target_angle < angle)
    { //motor needs to rotate CW when target is smaller than current angle
        is_CCW = false;
        pMotor->setCW();
        task_completed = false;
    }
    else
    {
        task_completed = true; //no need to move the motor if target and current are equal
    }
}

bool Joint::getCompletionFlag()
{
    return task_completed;
}

Joint::Joint()
{
    pMotor = new Stepper();
}

Joint::~Joint()
{
    delete pMotor;
}

OpenJoint::OpenJoint(Stepper &joint_motor, float ini_angle)
{ //constructor for joint without encoder
    pMotor = &joint_motor;
    angle = ini_angle;
    task_completed = true;
    counter++;
}

OpenJoint::~OpenJoint()
{
    counter--;
}

bool OpenJoint::isOvertorqued()
{ //no encoder on the joint so no way to detect collition for OpenJoint
    //always return false to keep the Arduino program running
    return false;
}

void OpenJoint::moveJoint()
{
    if (task_completed || angle == target_angle)
    {
        task_completed = true;
        return;
    }
    if (pMotor->getIterCounter() == 0)
    { //only move step when the current iteration number is at the beginning of a PWM cycle
        pMotor->moveStep();
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
        pMotor->restStep();
    }
    if (pMotor->getIterCounter() >= pMotor->getIteration())
    { //reset iter_counter of motor when it reaches the end of a PWM cycle
        pMotor->resetIterCounter();
    }
    //mark the completion status if the target is reached
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
{ //constructor for joint with encoder and the default cycloidal gearbox
    pMotor = &joint_motor;
    pEncoder = new Encoder(0, 0);
    pEncoder = &joint_encoder;
    angle = ini_angle;
    task_completed = true;
    net_reduction = CYCLOIDAL_REDUCTION; //total motor reduction is the reduction ratio of the gearbox
    encoder_pos = 0;
    missed_step = 0;
    counter++;
}

ClosedJoint::ClosedJoint(Stepper &joint_motor, Encoder &joint_encoder, float ini_angle, float extra_red)
{ //constructor for joint with encoder, default cycloidal gearbox and additional speed reduction
    pMotor = &joint_motor;
    pEncoder = &joint_encoder;
    angle = ini_angle;
    task_completed = true;
    net_reduction = CYCLOIDAL_REDUCTION * extra_red; //extra_red is the redection from additonal belts or gears
    encoder_pos = 0;
    missed_step = 0;
    counter++;
}

ClosedJoint::~ClosedJoint()
{
    delete pEncoder;
    counter--;
}

bool ClosedJoint::isOvertorqued()
{
    if (missed_step > ALLOWABLE_MISSED_STEPS)
    { //if number of missed steps of the joint exceeds the allowable value
        return true;
    }
    else
    {
        return false;
    }
}

void ClosedJoint::moveJoint()
{
    if (task_completed || angle == target_angle)
    {
        task_completed = true;
        return;
    }
    if (pMotor->getIterCounter() == 0)
    { //only move step when the current iteration number is at the beginning of a PWM cycle
        pMotor->moveStep();
        short int new_enc_pos = pEncoder->read();
        //calculate the new joint angle after rotation attempt
        float angle_change = (360 / (net_reduction * ENC_REDUCTION * ENC_RESOLUTION)) * (new_enc_pos - encoder_pos);
        float new_angle = angle + angle_change * DEG_TO_RAD;
        encoder_pos = new_enc_pos;
        //check if missing steps occur or mark the completion status if motion is done
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
        angle = new_angle; //store the new joint angle
    }
    else //motor does not rotate for the rest of PWM cycle
    {
        pMotor->restStep();
    }
    if (pMotor->getIterCounter() >= pMotor->getIteration())
    { //reset iter_counter of motor when it reaches the end of a PWM cycle
        pMotor->resetIterCounter();
    }
    if (isOvertorqued()) //override task completion status if collision occurs
    {
        task_completed = true;
    }
}