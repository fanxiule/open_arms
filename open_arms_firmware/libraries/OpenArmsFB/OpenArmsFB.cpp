#include <Arduino.h>
#include <open_arms_common.h>
#include "OpenArmsFB.h"

short int Feedback::counter = 0;
bool Feedback::collision_flag = false;

Feedback::Feedback(uint8_t pinA, uint8_t pinB) : encoder(pinA, pinB)
{ //constructor
    counter++;
}

Feedback::~Feedback()
{ //destructor
    counter--;
}

/*Feedback::Feedback(const Feedback &other) : encoder(other.encoder)
{ //copy constructor
    counter++;
}

Feedback &Feedback::operator=(const Feedback &other)
{ //assignment operator
    if (this == &other)
    {
        return *this;
    }

    encoder = other.encoder;
}*/

short int Feedback::getCounter()
{
    return counter;
}

bool Feedback::getCollisionFlag()
{
    return collision_flag;
}