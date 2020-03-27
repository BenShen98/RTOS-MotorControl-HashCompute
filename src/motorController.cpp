#include "dataStructure.hpp"
#include "motorController.hpp"


//debug
DigitalOut led(LED1);


// motor characteristics
int8_t originalState = 0;       // (constant) set by motor home, depends on assembly

// motor monitor
volatile float accPosition = 0;   //live Position, updated every encoder change
volatile float motorPosition = 0; //like accPosition, but only undated in ISR_PID_trigger
volatile float motorVelocity = 0; //unit as encoder position per 0.1 second

// PID config
#define V_KP 0.1 //0.1
#define V_KI 0.05 //0.05
#define V_KD 0.0//0.05
 //0.01
//#define V_KI 200.0/2000
#define V_INTERGAL_CAP_UPPER 25
#define V_INTERGAL_CAP_LOWER -V_INTERGAL_CAP_UPPER
#define D_KP 0.1
#define D_KD 0.0

#define APPROACH_V 10 *6/10 // in encoder position per 0.1 second
#define APPROACH_D 100 *6


// PID trigger
Ticker controllerTicker;


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
volatile int lead = LEAD;  // should only be changed by setTorque


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

inline void motor_forward(){
    motorOut((readRotorState()-originalState+1+6)%6);
}

inline void motor_backward(){
    motorOut((readRotorState()-originalState-1+6)%6);
}

// helper function for set torque
// cap at between -1.0 <--> 1.0
void setTorque(float t){
    // safety check, check if motor is stack
    static float motorPositionOld;
    if(abs(t)>0.2 && motorPositionOld==motorPosition){
        // motor stack
        if (t>0)  motor_forward(); else motor_backward();
    }
    motorPositionOld = motorPosition;

    // set torque
    if (t>0){
        lead = LEAD;
    }else{
        lead = -LEAD;
        t = -t;
    }
    MotorPWM.write(t); // if t>1.0, t will be set to 1.0
}


// Tune
// (char-'A'+1) => if ~: -1; if #: +1
const int tuneTable[]={
1000000/416,   //0 |G#/A~
1000000/440,   //1 |A
1000000/467,   //2 |A#/B~
1000000/494,   //3 |B
1000000/2000,  //4 | UNDEFINED
1000000/262,   //5 |C
1000000/278,   //6 |C#/D~
1000000/294,   //7 |D
1000000/312,   //8 |D#/E~
1000000/330,   //9 |E
1000000/2000,  //A| UNDEFINED
1000000/349,   //B |F
1000000/370,   //C |F#/G~
1000000/392,   //D |G
1000000/416,   //E |G#/A~
};

Timeout tuner;
volatile uint8_t tunes[16] = {0}; // remove volitale
volatile uint8_t tune_idx; // remove volitale

void ISR_tuner(){
    uint8_t tune = tunes[tune_idx];

    // set current tune
    MotorPWM.period_us(tuneTable[tune&0xf]);

    // debug
    led = !led;

    // perpare next tune
    // see docs, legal range for t are 1-8
    uint8_t t = tune>>4;
    tuner.attach_us(&ISR_tuner, 125000*t);
    if (t==0 || tune_idx==15 ){
        tune_idx = 0;  // t=0 has special meaing as end of tune sequence
    }else{
        tune_idx++;
    }
}

void ISR_update_position () {
    //ASSUMPTION: when code start, motor is at state 0
    static int8_t oldRotorState;
    int8_t newRotorState = readRotorState();

    // calculate circular increment
    int8_t forwardIncrement = (newRotorState+6-oldRotorState)%6;


    /*
    if turn forward:
        accPosition += forwardIncrement
    if turn backward:
        accPosition -= (6-forwardIncrement)
            eqv to  += forwardIncrement -6
    */

    //ASSUMPTION: ISR is not skiped for more than three encoder postion
    accPosition += forwardIncrement; //correct for forward
    if (forwardIncrement>3) accPosition -= 6; // by assumption, this would only happen if motor is turn backward

    // write for next ISR
    oldRotorState = newRotorState;

    motorOut((newRotorState-originalState+lead+6)%6);
}
void ISR_PID_trigger(){
    static float oldPosition;

    motorPosition = accPosition;

    motorVelocity = motorPosition - oldPosition;

    oldPosition = motorPosition;
    // static float oldDeltaPosition = 0;

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
    float tSpeed = DEFAULT_T_SPEED*6/10; // convert unit to encoderPostion per 0.1 seconds
    float tPosition = DEFAULT_T_ROTATION*6; // convert unit to encoderPostion

    MotorPWM.period_us(2000);
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



    // Distance control
    float errorPosition;
    static float errorPositionOld; // motorPosition=0 when start;

    //Speed control
    float errorSpeed;
    static float errorSpeedOld;
    static float errorSpeedIntegral;
    static float errorSpeedDiff;

    // example while loop
    while (1)
    {



        uint32_t flags = ThisThread::flags_wait_any(SIGNAL_MOTOR_PID_RUN | SIGNAL_MOTOR_T_TUNE_CHANGE | SIGNAL_MOTOR_T_SPEED_CHANGE | SIGNAL_MOTOR_T_ROTATION_CHANGE); // auto clear



        if (flags & SIGNAL_MOTOR_T_SPEED_CHANGE){
            // read data
            motorCfgMutex.lock();
            tSpeed = float(motorCfg.TSpeed) * 6/10; // div 10 for 1s -> 0.1s; muti 6 for rotation -> encoder postion
            motorCfgMutex.unlock();
        }

        if (flags & SIGNAL_MOTOR_T_ROTATION_CHANGE){
            // read data
            motorCfgMutex.lock();
            tPosition = motorPosition + float(motorCfg.TRotation) * 6;
            motorCfgMutex.unlock();
        }


        if (flags & SIGNAL_MOTOR_PID_RUN){
            /*
            CODE BELOW ARE FOR PID CONTROL
            */

            float torque_d;
            float torque_s;
            float torque;

            // Distance controller
            errorPosition = tPosition - motorPosition;
            if(abs(errorPosition)>2){ // 3 encoder postion, NOT rotation
                float errorPosition_Diff = errorPosition - errorPositionOld;
                torque_d = D_KP*errorPosition + D_KD*(errorPosition_Diff); //checked
            }else{
                torque_d = 0;
            }

            errorPositionOld = errorPosition;


            // Speed controller
            if (abs(errorPosition) > APPROACH_D){
                errorSpeed = (errorPosition>=0) ? tSpeed - motorVelocity :  -tSpeed - motorVelocity;
                errorSpeedIntegral += errorSpeed;
                if(errorSpeedIntegral>V_INTERGAL_CAP_UPPER) errorSpeedIntegral = V_INTERGAL_CAP_UPPER;
                if(errorSpeedIntegral<V_INTERGAL_CAP_LOWER) errorSpeedIntegral = V_INTERGAL_CAP_LOWER;
            }else{
                errorSpeed = (errorPosition>=0) ? APPROACH_V - motorVelocity : -APPROACH_V - motorVelocity;
                errorSpeedIntegral = 0; //only use P controller for stability
            }
            errorSpeedDiff = errorSpeed - errorSpeedOld;
            errorSpeedOld = errorSpeed;


            torque_s = V_KP*errorSpeed + V_KI*errorSpeedIntegral + V_KD*errorSpeedDiff;


            torque = abs(torque_d) > abs(torque_s) ? torque_s : torque_d; // taken min
            setTorque(torque);



            // pc.printf("\nR:%f->%f, V:%f->%f, td:%f, ts:%f, %f\n", float(motorPosition)/6, float(tPosition)/6, float(motorVelocity)*10/6, float(tSpeed)*10/6, torque_d, torque_s, errorSpeedIntegral);
        }


        if (flags & SIGNAL_MOTOR_T_TUNE_CHANGE){
            led=0;
            // cancel current sequence
            tuner.detach();

            // copy data
            tune_idx = 0;
            motorCfgMutex.lock();
            for (int i=0; i<16; ++i){
                tunes[i] = motorCfg.Tunes[i];
            }
            motorCfgMutex.unlock();


            ISR_tuner();

        }

    }
}
