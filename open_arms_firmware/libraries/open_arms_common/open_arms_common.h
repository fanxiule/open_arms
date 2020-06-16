#ifndef OPENARMSCOMMON_H
#define OPENARMSCOMMON_H

//store arduino pin numbers for connection

//from motor driver to arduino
#define JOINT7_STP 22
#define JOINT7_DIR 23
#define JOINT7_ENB 41
#define JOINT6_STP 24
#define JOINT6_DIR 25
#define JOINT6_ENB 43
#define JOINT5_STP 26
#define JOINT5_DIR 27
#define JOINT5_ENB 45
#define JOINT4_STP 28
#define JOINT4_DIR 29
#define JOINT4_ENB 47
#define JOINT3_STP 30
#define JOINT3_DIR 31
#define JOINT3_ENB 49
#define JOINT2_STP 32
#define JOINT2_DIR 33
#define JOINT2_ENB 51
#define JOINT1_STP 34
#define JOINT1_DIR 35
#define JOINT1_ENB 53

//from encoder to arduino
//the pins for ENC#_A are interrupt pins
#define ENC6_A 21
#define ENC6_B 17
#define ENC5_A 20
#define ENC5_B 16
#define ENC4_A 19
#define ENC4_B 15
#define ENC3_A 18
#define ENC3_B 14
#define ENC2_A 3
#define ENC2_B 1
#define ENC1_A 2
#define ENC1_B 0

//some constants
#define CYCLOIDAL_REDUCTION 26.0
#define ENC_REDUCTION 4.0
#define ENC_RESOLUTION 20.0
#define STEP_ANGLE 1.8

#endif /* OPENARMSCOMMON_H */