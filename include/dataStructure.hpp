#ifndef DATASTRUCTURE_H
#define DATASTRUCTURE_H

#include "mbed.h"


#define commandBufferSize 128

/* FLAGS */
#define SIGNAL_NEWLINE 0x1

#define SIGNAL_MOTOR_T_SPEED_CHANGE 0x2
#define SIGNAL_MOTOR_T_ROTATION_CHANGE 0x4
#define SIGNAL_MOTOR_T_TUNE_CHANGE 0x8
#define SIGNAL_MOTOR_PID_RUN 0x11

#define SIGNAL_HASH_KEY_CHANGE 0x10

typedef struct
{
    float TSpeed;
    float TRotation;
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