#define MIN_ACTUATION_TIME 0.1
#define MAX_RPM 120.0
//note 300 RPM -> 1 ms per step

#ifndef OPENARMSSTEPPER_H
#define OPENARMSSTEPPER_H

class Stepper
{
private:
    static short int counter; //count how many stepper motors have been instantiated
    static float step_time;   //actuation time for each step in ms
    static float speed_scale; //scale in terms of the maximum speed, 0.01-1

    unsigned short int stp_num;         //store the step pin number on Arduino
    unsigned short int dir_num;         //store the direction pin number on Arduino
    unsigned short int enable_num; //store enable pin number on Arduino
    unsigned short int total_stp;       //total number of steps per rev
    float RPM = MAX_RPM;                //RPM of each motor
    float step_angle;                   //in degrees, angle per step of the motor
    float time_per_step;                //time required for each step interval

public:
    int iteration;       //number of iterations needed to complete one step of motion (1 PWM cycle)
    int rev_counter = 0; //counter + 1 if motor move or rest one step
    //constructors and destructor
    Stepper(unsigned short int stp, unsigned short int dir, float angle);                            //constructor with arguments to initialize step pin and dir pin numbers
    Stepper(unsigned short int stp, unsigned short int dir, unsigned short int enable, float angle); //constructor with additional enable pin
    ~Stepper();                                                                                      //destructor
    Stepper(const Stepper &other);                                                                   //copy constructor
    Stepper &operator=(const Stepper &other);                                                        //assignment operator

    //setters and getters
    static short int getStepperCount();  //returns number of motors that have been instantiated
    static void setStepTime(float time); //set the actuation time for each step
    static void setScale(float scale);   //set speed scale, 0.01-1 input
    void setNewIteration();              //set new iteration based on new actuation time, RPM or scale
    void setRPM(float velocity);         //set RPM of a motor

    //public method
    void enableMotor();                             //enable current to pass through motor
    void disableMotor();                            //disable current flow
    void setCCW();                                  //set direction to counter clockwise
    void setCW();                                   //set direction to clockwise
    void moveStep();                                //move one step
    void restStep();                                //rest
    void moveStepCCW();                             //move one step CCW
    void moveStepCW();                              //move one step CW
    void moveRev(bool direction, float revolution); //move motor in certain direction for given revolutions
    void moveMotorStep(int steps);                  //move the motor by given number of steps
    void printSummary();                            //print summary of the motor
};
#endif /* OPENARMSSTEPPER_H */