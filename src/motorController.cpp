#include "dataStructure.hpp"
#include "motorController.hpp"

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


// motor characteristics
int8_t originalState = 0;       // (constant) set by motor home, depends on assembly

// motor monitor
volatile int32_t accPosition = 0;   //live Position, updated every encoder change
volatile int32_t motorPosition = 0; //like accPosition, but only undated in ISR_PID_trigger
volatile int32_t motorVelocity = 0; //unit as encoder position per 0.1 second

// PID config
#define V_KP 5.0/2000
#define V_KI 0.0
#define V_INTERGAL_CAP 100000.0
#define D_KP 0.0
#define D_KD 0.0


// PID trigger
Ticker controllerTicker;

// TARGET




// int8_t  s = 0;
// float kp =5;
// float   ys = 0;
// int test = 0;
// float rotation = 1;


// int8_t vel_er =0;
// int8_t integral_vel_er =0;
// int8_t old_vel_er = 0;
// int8_t max_int_vel_err = 0;
// int8_t diff_vel_error = 0;

//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
#define LEAD 2
int8_t lead = LEAD;  // should only be changed by setTorque


//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
DigitalOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
DigitalOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

DigitalOut TP1(TP1pin);
PwmOut MotorPWM(PWMpin);

// helper function for set torque
// cap at between -1.0 <--> 1.0
void setTorque(float t){
    if (t>0){
        lead = LEAD;
    }else{
        lead = -LEAD;
        t = -t;
    }
    MotorPWM.write(t); // if t>1.0, t will be set to 1.0
}

//Set a given drive state
void motorOut(int8_t driveState){

    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];

    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;

    //Then turn on
    if (driveOut & 0x01) L1L = 1;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = 1;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L = 1;
    if (driveOut & 0x20) L3H = 0;
    //MotorPWM.pulsewidth(ys);
    //is this the problem? the code online put it here
    }

    //Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
    }

//Basic synchronisation routine
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(2.0);

    //Get the rotor state
    return readRotorState();
}


void ISR_update_position () {
    //ASSUMPTION: when code start, motor is at state 0
    static int8_t oldRotorState = 0;
    int8_t newRotorState = readRotorState();

    // calculate circular increment
    int8_t forwardIncrement = (newRotorState+6-oldRotorState)%6;


    /*
    if turn forward:
        accPosition += 6-forwardIncrement)
    if turn backward:
        accPosition -= (6-forwardIncrement)
            eqv to  += forwardIncrement -6
    */
    accPosition += forwardIncrement; //correct for forward
    if (lead < 0) accPosition -= 6;

    // write for next ISR
    oldRotorState = newRotorState;

    motorOut((newRotorState-originalState+lead+6)%6);
}
void ISR_PID_trigger(){
    static int32_t oldPosition = 0;

    motorPosition = accPosition;

    motorVelocity = motorPosition - oldPosition;

    oldPosition = motorPosition;
    // static float oldDeltaPosition = 0;

    // // calculate P
    // deltaPosition = accPosition - oldPosition;

    // // calculate I with cap
    // deltaPositionIntegral += deltaPosition;
    // if (deltaPositionIntegral > V_KI_CAP_POSITIVE) deltaPositionIntegral = V_KI_CAP_POSITIVE;
    // if (deltaPositionIntegral < V_KI_CAP_NEGATIVE) deltaPositionIntegral = V_KI_CAP_NEGATIVE;

    // // calculate D
    // deltaPositionDiff = deltaPosition - oldDeltaPosition;


    // oldPosition = accPosition;
    // oldDeltaPosition = deltaPosition;

    // accPosition =
    motorController.flags_set(SIGNAL_MOTOR_PID_RUN);
}

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

// THIS FUNCTION SHOULD NOT USE accPosition
void TRD_motor_controller(){
    float tSpeed; // convert unit to encoderPostion per 0.1 seconds
    float tPosition; // convert unit to encoderPostion

    tSpeed=50.0*6/10; //here is speed, no direction
    tPosition=4000.0*6/10;

    const int32_t PWM_PRD = 2000;
    MotorPWM.period_us(PWM_PRD);
    MotorPWM.write(1.0);

    // call ISR_PID_trigger every 0.1 s
    controllerTicker.attach_us(&ISR_PID_trigger,100000);

    originalState = motorHome();

    motorOut(5);


    I1.rise(&ISR_update_position);
    I1.fall(&ISR_update_position);
    I2.rise(&ISR_update_position);
    I2.fall(&ISR_update_position);
    I3.rise(&ISR_update_position);
    I3.fall(&ISR_update_position);



    // more PID data
    // volatile float errorVelocityDiff = 0;           // acceleration (has sign)
    // volatile float errorVelocityIntegral = 0;   // distance (caped, has sign)
    // volatile int32_t oldErrorVelocity = errorVelocity;
    // volatile int32_t errorVelocity = 0;           // velocity (has sign)

    // Distance control
    int32_t errorPosition;
    static int32_t errorPositionOld = tPosition; // motorPosition=0 when start;

    //Speed control
    int32_t errorSpeed;
    static int32_t errorSpeedIntegral = 0;




    // example while loop
    while (1)
    {


        uint32_t flags = ThisThread::flags_wait_any(SIGNAL_MOTOR_PID_RUN); // auto clear

        if (flags & SIGNAL_MOTOR_PID_RUN){
            /*
            CODE BELOW ARE FOR PID CONTROL
            */

            float torque_d;
            float torque_s;
            float torque;

            // Distance controller
            errorPosition = tPosition - motorPosition;
            float errorPosition_Diff = errorPosition - errorPositionOld;
            torque_d = D_KP*errorPosition + D_KD*(errorPosition - errorPositionOld); //checked
            errorPositionOld = errorPosition;

            // Speed controller
            errorSpeed = tSpeed - abs(motorVelocity);
            errorSpeedIntegral += errorSpeed;
            if(errorSpeedIntegral>V_INTERGAL_CAP) errorSpeedIntegral = V_INTERGAL_CAP;

            torque_s = V_KP*errorSpeed + V_KI*errorSpeedIntegral;
            if(errorPosition_Diff<0) torque_s = -torque_s;

            // select torque
            //TODO
            setTorque(torque_d);
        }

    //     //set rotation for testig
    //     if(rotation>0){

    //     ys = kp*(TSpeed - deltaPosition) + integral_vel_er + diff_vel_error;
    //     // if (ys<0){
    //     //     lead = -2;
    //     // }
    //     ys = (ys>0) ? ys : 0;
    //     ys = (ys > 1000 )? 1000 : ys;//Pwm limit
    //     //if(ys!=0){
    //     MotorPWM.pulsewidth(ys);
    //    // }
    //     pc.printf("\n%i, %f, %f\n", int(accPosition), float(deltaPosition)*10/6, ys);
    //     }
    //     else{
    //     ys = kp*(TSpeed - abs(deltaPosition)) + integral_vel_er+diff_vel_error;
    //     ys = (ys>0) ? ys : 0;
    //     ys = (ys > 1000 )? 1000 : ys;
    //     MotorPWM.pulsewidth(ys);
    //     pc.printf("\n%i, %f, %f\n", int(accPosition), float(deltaPosition)*10/6, ys);
    //     }

        // velocity = (newPosition - oldPosition)*60*10;
        // pc.printf("the velocity is %i", (int)velocity);
        // uint32_t flag = ThisThread::flags_wait_any(SIGNAL_MOTOR_T_SPEED_CHANGE | SIGNAL_MOTOR_T_ROTATION_CHANGE | SIGNAL_MOTOR_T_TUNE_CHANGE);
        // if(flag & SIGNAL_MOTOR_T_SPEED_CHANGE) pc.printf("\nChange speed to %f\n", get_tspeed());
        // if(flag & SIGNAL_MOTOR_T_ROTATION_CHANGE) pc.printf("\nChange rotation to %f\n", get_trotation());
        // if(flag & SIGNAL_MOTOR_T_TUNE_CHANGE) pc.printf("\nChange Tune\n");
    }
}
