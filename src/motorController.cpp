#include "dataStructure.hpp"

#include "motorController.hpp"

inline float get_tspeed(){
    float v;
    motorCfgMutex.lock();
    v = motorCfg.TSpeed;
    motorCfgMutex.unlock();
    return v;
}

inline float get_trotation(){
    float v;
    motorCfgMutex.lock();
    v = motorCfg.TRotation;
    motorCfgMutex.unlock();
    return v;
}


void TRD_motor_controller(){
    // local data
    float TSpeed, TRotation;

    // example while loop
    while (1)
    {
        uint32_t flag = ThisThread::flags_wait_any(SIGNAL_MOTOR_T_SPEED_CHANGE | SIGNAL_MOTOR_T_ROTATION_CHANGE | SIGNAL_MOTOR_T_TUNE_CHANGE);
        if(flag & SIGNAL_MOTOR_T_SPEED_CHANGE) pc.printf("\nChange speed to %f\n", get_tspeed());
        if(flag & SIGNAL_MOTOR_T_ROTATION_CHANGE) pc.printf("\nChange rotation to %f\n", get_trotation());
        if(flag & SIGNAL_MOTOR_T_TUNE_CHANGE) pc.printf("\nChange Tune\n");
    }
}
