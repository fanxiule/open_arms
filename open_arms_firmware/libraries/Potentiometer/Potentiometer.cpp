#include <Potentiometer.h>

Potent::Potent(unsigned short int pin)
{
    this->pin = pin;
}

void Potent::setScaling(double l_range, double u_range, double l_reading, double u_reading)
{
    u_range = u_range * M_PI / 180; //convert from degrees to radians
    l_range = l_range * M_PI / 180; //convert from degrees to radians
    this->slope = (u_range - l_range) / (u_reading - l_reading);
    this->offset = l_range;
    this->move = true;
    /*Serial.println(u_range);
    Serial.println(l_range);
    Serial.println("^^^^^^^^^^^^^^^^^^^^^^^^^^");
    Serial.println(this->slope);
    Serial.println(this->offset);
    Serial.println("***************************");*/
}

double Potent::getCurrentValue()
{
    current_reading = slope * analogRead(this->pin) + offset;
    current_reading = current_reading * 180 / M_PI; //convert from radians to degrees
    return current_reading;
}
