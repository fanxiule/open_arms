#include <Arduino.h>
#include "open_arms_stepper.h"

unsigned short int Stepper::counter = 0;       //initialized number of motors is 0
float Stepper::iter_time = MIN_ACTUATION_TIME; //default actuation time is minimum time
//float Stepper::speed_scale = 1;                //initialized speed scale as 1, i.e. maximum speed allowecd

Stepper::Stepper(unsigned short int stp, unsigned short int dir, float angle)
{ //constructor when motor driver's enable pin is not connected
    stp_num = stp;
    dir_num = dir;
    enable_num = -1; // essentially disable the enable pin
    step_angle = angle;
    total_stp = 360 / step_angle;
    iter_counter = 0;
    RPM = MAX_RPM;
    setNewIteration();
    counter++;
    //Serial.println("Motor Initialized");
}

Stepper::Stepper(unsigned short int stp, unsigned short int dir, unsigned short int enable, float angle)
{ //constructor when motor driver's enable pin is connected
    stp_num = stp;
    dir_num = dir;
    enable_num = enable;
    step_angle = angle;
    total_stp = 360 / step_angle;
    iter_counter = 0;
    RPM = MAX_RPM;
    setNewIteration();
    counter++;
    //Serial.println("Motor Initialized");
}

Stepper::~Stepper()
{ //destructor
    counter--;
}

Stepper::Stepper(const Stepper &other) : stp_num(other.stp_num),
                                         dir_num(other.dir_num),
                                         step_angle(other.step_angle),
                                         total_stp(other.total_stp),
                                         time_per_step(other.time_per_step),
                                         iteration(other.iteration)
{ //copy constructor
    counter++;
}

Stepper &Stepper::operator=(const Stepper &other)
{ //assignment operator
    if (this == &other)
    {
        return *this;
    }

    stp_num = other.stp_num;
    dir_num = other.dir_num;
    step_angle = other.step_angle;
    total_stp = other.total_stp;
    time_per_step = other.time_per_step;
    iteration = other.iteration;
    return *this;
}

void Stepper::setNewIteration()
{ //calculate new iteration numbers needed for each PWM cycle
    //convert RPM to ms/step
    time_per_step = 1 / (RPM / 60 / 1000 * total_stp);
    //Serial.print("Time per step is: "); //
    //Serial.print(time_per_step); //
    //Serial.print(" ms \n");      //
    /***************************/
    //time_per_step /= speed_scale;
    if (time_per_step < iter_time)
    { //check if time needed for each step is lower than the actuation time
        Serial.println("Time needed per step is too low, the shortest time possible is used");
        time_per_step = 2 * iter_time; //use the current one
    }
    iteration = time_per_step / iter_time;
}

void Stepper::moveStep()
{ //move one step
    digitalWrite(stp_num, HIGH);
    iter_counter++;
    delay(iter_time);
}

short int Stepper::getStepperCount()
{ //get number of motors that have been instantiated
    return counter;
}

unsigned short int Stepper::getIteration()
{ //get how many iterations needed to complete a PWM cycle
    return iteration;
}

unsigned short int Stepper::getIterCounter()
{ //get current iteration number of the motor
    return iter_counter;
}

void Stepper::setIterTime(float time)
{ //set actuation time for each step, time in ms
    if (time <= MIN_ACTUATION_TIME)
    {
        Serial.println("Requested actuation time too low, minimum actuation time is used");
        time = MIN_ACTUATION_TIME; //use minimum allowable actuation time
    }
    else
    {
        iter_time = time;
    }
    //setNewIteration(); //calculate new iteration numbers
}

/*void Stepper::setScale(float scale)
{ //set scale for maximum speed, allowable range is 0.01-1
    if (scale < 0.01)
    {
        Serial.println("Speed scale too low"); //
        speed_scale = 0.01;
    }

    else if (scale > 1)
    {
        Serial.println("Speed scale too high"); //
        speed_scale = 1;
    }

    else
    {
        speed_scale = scale;
    }
}*/

void Stepper::setRPM(float velocity)
{ //set RPM of the motors
    if (velocity >= MAX_RPM)
    {
        Serial.println("Requested RPM too high, maximum RPM is used");
        RPM = MAX_RPM; //use maximm allowable RPM
    }
    else
    {
        RPM = velocity;
    }
    setNewIteration();
}

void Stepper::enableMotor()
{
    if (enable_num != -1)
    {
        digitalWrite(enable_num, LOW);
    }
}

void Stepper::disableMotor()
{
    if (enable_num != -1)
    {
        digitalWrite(enable_num, HIGH);
    }
}

void Stepper::setCCW()
{ //set motor's direction to counter clockwise
    digitalWrite(dir_num, LOW);
}

void Stepper::setCW()
{ //set motor's direction to clockwise
    digitalWrite(dir_num, HIGH);
}

void Stepper::restStep()
{ //rest
    digitalWrite(stp_num, LOW);
    iter_counter++;
    delay(iter_time);
}

void Stepper::moveStepCCW()
{ //move one step CCW
    setCCW();
    moveStep();
}

void Stepper::moveStepCW()
{ //move one step CW
    setCW();
    moveStep();
}

void Stepper::moveRev(float revolution)
{ //move for given revolutions, +ve input -> CCW, -ve input -> CW
    int steps_needed = total_stp * revolution;
    moveStep(steps_needed);
}

void Stepper::moveStep(int steps)
{ //move motors for given steps
    // if steps > 0, move in CCW
    // if steps < 0, move in CW
    if (steps > 0)
    {
        setCCW();
    }
    else if (steps < 0)
    {
        setCW();
    }
    else
    {
        return;
    }
    int steps_done = 0;
    int steps_needed = abs(steps);
    while (steps_done < steps_needed)
    {
        moveStep();
        restStep();
        steps_done++;
    }
}

void Stepper::resetIterCounter()
{ // set the iter_counter back to 0
    iter_counter = 0;
}

void Stepper::printSummary()
{
    Serial.print("Step: ");
    Serial.print(stp_num);
    Serial.print(" Direction: ");
    Serial.print(dir_num);
    Serial.print(" Step Angle: ");
    Serial.print(step_angle);
    Serial.print("\n");
    Serial.print("RPM: ");
    Serial.print(RPM);
    Serial.print(" Time per step in ms: ");
    Serial.print(time_per_step);
    Serial.print(" Iteration: ");
    Serial.print(iteration);
    Serial.print("\n");
    Serial.println("===============================================");
}