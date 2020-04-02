#include <Encoder.h>
#include <OpenArmsStepper.h>
#ifndef OPENARMSFB_H
#define OPENARMSFB_H

class Feedback
{
private:
    static short int counter;   //count how many encoders have been instantiated
    static bool collision_flag; //a flag to show if collision happened
                                //if true, then one or more encoders show over torque
    Encoder encoder;            //encoder from the Encoder.h library

public:
    Feedback(uint8_t pinA, uint8_t pinB); //constructor
    ~Feedback();                                                //destructor
    //Feedback(const Feedback &other);                            //copy constructor
    //Feedback &operator=(const Feedback &other);                 //assignment operators

    static short int getCounter();  //return number of encoders that have been instantiated
    static bool getCollisionFlag(); //return the collision_flag
};

#endif /* OPENARMSFB_H */