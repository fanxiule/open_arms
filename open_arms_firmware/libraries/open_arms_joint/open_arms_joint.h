#define ALLOWABLE_MISSED_STEPS 3

#include "open_arms_common.h"
#include "open_arms_stepper.h"
#include "Encoder.h"

#ifndef OPENARMSJOINT_H
#define OPENARMSJOINT_H

class Joint
{ //base class
protected:
    static unsigned short int counter; //count total number of joints
    Stepper *pMotor;                   //motor of the current joint
    float angle;                       //current angle of the joint
    float target_angle;                //target angle
    bool task_completed;               //flag to show if the motion task_completed
    bool is_CCW;                       //flag to show which diretion to rotate to

    Joint();              //default constructor
    virtual ~Joint() = 0; //destructor
public:
    void setDirection(float target_angle); //determin direction of rotation
    bool getCompletionFlag();              //return completion status of the motion
    virtual bool isOvertorqued() = 0;      //return if the joint is overtorqued, i.e. if collision has occured
    virtual void moveJoint() = 0;          //move the joint
};

class OpenJoint : public Joint
{ //Joint with no encoder (open loop)
public:
    OpenJoint(Stepper &joint_motor, float ini_angle);
    ~OpenJoint();
    bool isOvertorqued() override;
    void moveJoint() override;
};

class ClosedJoint : public Joint
{ //Joint with encoder (closed loop)
private:
    Encoder *pEncoder;              //encoder of the current joint
    float net_reduction;            //store net gear reduction ratio of the joint
    int encoder_pos;                //encoder position
    unsigned short int missed_step; //number of consecutively missed encoder steps
public:
    ClosedJoint(Stepper &joint_motor, Encoder &joint_encoder, float ini_angle);                  //constructor for joint with gearbox only
    ClosedJoint(Stepper &joint_motor, Encoder &joint_encoder, float ini_angle, float extra_red); //constructor for joint gearbox and extra gear or belt
    ~ClosedJoint();
    bool isOvertorqued() override;
    void moveJoint() override;
};
#endif /* OPENARMSJOINT_H */