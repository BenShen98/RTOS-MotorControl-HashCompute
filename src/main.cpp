#include "dataStructure.hpp"

#include "commandParser.hpp"
#include "motorController.hpp"
#include "hashCracker.hpp"



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

#ifdef STATISTIC
void print_cpu_stats()
{
    mbed_stats_cpu_t stats;
    mbed_stats_cpu_get(&stats);

    // Calculate the percentage of CPU usage
    printf("Time(us): Up: %lld", stats.uptime);
    printf("   Idle: %lld", stats.idle_time);
    printf("   Sleep: %lld", stats.sleep_time);
    printf("   DeepSleep: %lld\n", stats.deep_sleep_time);
}
#endif


/* Main */
int main (void) {
    pc.attach(&ISR_command_getchar);

    commandParser.start(&TRD_command_parser);
    motorController.start(&TRD_motor_controller);
    hashCracker.start(&TRD_hash_cracker);

#ifdef STATISTIC
    while(1){
        print_cpu_stats();
        wait(1.0);
    }

#endif

}
