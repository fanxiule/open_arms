#include <Arduino.h>
#include "OpenArmsStepper.h"

short int Stepper::counter = 0;                //initialized number of motors is 0
float Stepper::step_time = MIN_ACTUATION_TIME; //default actuation time is minimum time
float Stepper::speed_scale = 1;                //initialized speed scale as 1, i.e. maximum speed allowecd

Stepper::Stepper(unsigned short int stp, unsigned short int dir, float angle)
{ //constructor
    stp_num = stp;
    dir_num = dir;
    enable_num = -1; // essentially disable the enable pin
    step_angle = angle;
    total_stp = 360 / step_angle;
    setNewIteration();
    counter++;
    //Serial.println("Motor Initialized");
}

Stepper::Stepper(unsigned short int stp, unsigned short int dir, unsigned short int enable, float angle)
{ //constructor
    stp_num = stp;
    dir_num = dir;
    enable_num = enable;
    step_angle = angle;
    total_stp = 360 / step_angle;
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

short int Stepper::getStepperCount()
{ //get number of motors that have been instantiated
    return counter;
}

void Stepper::setStepTime(float time)
{ //set actuation time for each step, time in ms
    if (time <= MIN_ACTUATION_TIME)
    {
        Serial.println("Actuation time too low"); //
        time = MIN_ACTUATION_TIME;
    }
    else
    {
        step_time = time;
    }
}

void Stepper::setRPM(float velocity)
{ //set RPM of the motors
    if (velocity >= MAX_RPM)
    {
        Serial.println("RPM too high"); //
        RPM = MAX_RPM;
    }
    else
    {
        RPM = velocity;
    }
    setNewIteration();
}

void Stepper::setScale(float scale)
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
}

void Stepper::setNewIteration()
{                                                      //calculate new iteration numbers needed for each PWM cycle
    time_per_step = 1 / (RPM / 60 / 1000 * total_stp); //convert RPM to ms/step
    //Serial.print("Time per step is: "); //
    Serial.print(time_per_step); //
    Serial.print(" ms \n");      //
    time_per_step /= speed_scale;
    if (time_per_step < step_time)
    {                                                   //check if time needed for each step is lower than the actuation time
        Serial.println("Time needed per step too low"); //
        time_per_step = 2 * step_time;
    }
    iteration = time_per_step / step_time;
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

void Stepper::moveStep()
{ //move one step
    digitalWrite(stp_num, HIGH);
    rev_counter++;
    delay(step_time);
}

void Stepper::restStep()
{ //rest
    digitalWrite(stp_num, LOW);
    rev_counter++;
    delay(step_time);
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

void Stepper::moveRev(bool direction, float revolution)
{ //move in given direction for given revolutions
    //diretion: true -> CCW, false -> CW
    if (direction == true)
    {
        setCCW();
    }
    else
    {
        setCW();
    }
    setNewIteration();
    int StepNeeded = total_stp * revolution;
    for (int StepCompleted = 0; StepCompleted < StepNeeded; StepCompleted++)
    {
        moveStep();
        restStep();
        delay(25);
    }
}

void Stepper::moveMotorStep(int steps)
{   //move motors for given steps
    // if steps > 0, move in CCW
    // if steps < 0, move in CW
    int steps_done = 0;
    int steps_needed = abs(steps);
    if (steps > 0 && steps_done < steps_needed)
    {
        moveStepCCW();
        restStep();
        steps_done++;
    }
    else if (steps < 0 && steps_done < steps_needed)
    {
        moveStepCW();
        restStep();
        steps_done++;
    }
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