#ifndef DATASTRUCTURE_H
#define DATASTRUCTURE_H

#include "mbed.h"

//Photointerrupter input pins
#define I1pin D3
#define I2pin D6
#define I3pin D5

//Incremental encoder input pins
#define CHApin   D12
#define CHBpin   D11

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D1           //0x01
#define L1Hpin A3           //0x02
#define L2Lpin D0           //0x04
#define L2Hpin A6           //0x08
#define L3Lpin D10          //0x10
#define L3Hpin D2           //0x20

#define PWMpin D9

//Motor current sense
#define MCSPpin   A1
#define MCSNpin   A0

//Test outputs
#define TP0pin D4
#define TP1pin D13
#define TP2pin A2

#define commandBufferSize 128

/* DEFAULT */
#define DEFAULT_T_SPEED 100
#define DEFAULT_T_ROTATION 0

/* FLAGS */
#define SIGNAL_NEWLINE 0x1

#define SIGNAL_MOTOR_T_SPEED_CHANGE 0x2
#define SIGNAL_MOTOR_T_ROTATION_CHANGE 0x4
#define SIGNAL_MOTOR_T_TUNE_CHANGE 0x8
#define SIGNAL_MOTOR_PID_RUN 0x20

#define SIGNAL_HASH_KEY_CHANGE 0x10
#define SIGNAL_HASH_TICK 0x40


typedef struct
{
    float TSpeed;
    float TRotation;
    uint8_t Tunes[16] = {0};
    // data for motor
} MotorCfg;

//ASSUMPTION: the buffer will never get full, no check done
extern CircularBuffer<char, commandBufferSize> commandBuffer;

extern volatile MotorCfg motorCfg;
extern volatile uint64_t hashKey;

extern Mutex motorCfgMutex;
extern Mutex hashKeyMutex;


extern RawSerial pc;

// Threads
extern Thread commandParser;
extern Thread motorController;
extern Thread hashCracker;

#endif