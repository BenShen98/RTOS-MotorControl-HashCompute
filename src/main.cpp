#include "dataStructure.hpp"

#include "commandParser.hpp"
#include "motorController.hpp"



/* Global Variable Definations */

// Devices
RawSerial pc(PA_2, PA_15);

// Threads
Thread commandParser(osPriorityNormal);
Thread motorController(osPriorityRealtime);
Thread hashCracker(osPriorityBelowNormal); //TODO: consider overtake idle thread?


// Variables
CircularBuffer<char, commandBufferSize> commandBuffer;

Mutex motorCfgMutex;
Mutex hashKeyMutex;

volatile MotorCfg motorCfg={.TSpeed=DEFAULT_T_SPEED, .TRotation=DEFAULT_T_ROTATION};
volatile uint64_t hashKey=0x0;

/* Main */
int main (void) {
    pc.attach(&ISR_command_getchar);

    commandParser.start(&TRD_command_parser);
    motorController.start(&TRD_motor_controller);
}
