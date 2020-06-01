#define ALLOWABLE_MISSED_STEPS 3

#include <open_arms_common.h>
#include <open_arms_stepper.h>
#include <Encoder.h>

#ifndef OPENARMSJOINT_H
#define OPENARMSJOINT_H

class Joint
{ //base class
protected:
    static unsigned short int counter; //count total number of joints
    Stepper motor;                     //motor of the current joint
    float angle;                       //current angle of the joint
    bool task_completed;               //flag to show if the motion task_completed
    bool is_CCW;                       //flag to show which diretion to rotate to

    void setDirection(float target_angle); //determin direction of rotation

public:
    virtual ~Joint() = 0; //destructor
};

class OpenJoint : Joint
{ //Joint with no encoder (open loop)
public:
    OpenJoint(Stepper &joint_motor, float ini_angle);
    ~OpenJoint();
    void moveJoint(float target_angle);
};

class ClosedJoint : Joint
{ //Joint with encoder (closed loop)
private:
    Encoder encoder;                //encoder of the current joint
    float net_reduction;            //store net gear reduction ratio of the joint
    int encoder_pos;                //encoder position
    unsigned short int missed_step; //number of consecutively missed encoder steps

    bool isOvertorqued(); //flag to show if the joint is overtorqued, i.e. collision has occured
public:
    ClosedJoint(Stepper &joint_motor, Encoder &joint_encoder, float ini_angle);
    ClosedJoint(Stepper &joint_motor, Encoder &joint_encoder, float ini_angle, float extra_red);
    ~ClosedJoint();
    void moveJoint(float target_angle);
};
#endif /* OPENARMSJOINT_H */