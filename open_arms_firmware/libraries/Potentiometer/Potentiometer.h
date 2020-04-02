#ifndef OPENARMSPOT_H
#define OPENARMSPOT_H

#include <Arduino.h>

class Potent
{
private:
    unsigned short int pin; //pin number
    double slope;           //potentiometer scaling, slope k in y = kx + b
    double offset;          //offset b in y = kx + b
    double current_reading; //current reading of the potentiometer
public:
    bool move;                                                                           //if move is true then the joint is not at the desired position
    Potent(unsigned short int pin);                                                      //constructor to initiate a potentiometer object with pin#
    void setScaling(double l_range, double u_range, double l_reading, double u_reading); //set scaling factor for potentiometer reading
    double getCurrentValue();
};
#endif /* OPENARMSPOT_H */
