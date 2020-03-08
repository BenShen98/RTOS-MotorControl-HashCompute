#include "commandParser.hpp"

void ISR_command_getchar(){
    char c = (char) pc.getc();
    pc.putc(c); //DEBUG: provide feedback to show char is processed

    if(c!='\n') commandBuffer.push(c); // spec says end with cr, ignore nl

    if (c=='\r') commandParser.flags_set(SIGNAL_NEWLINE); //send signal to command parser

}

inline void cmd_buffer_skip_line(){
    char c;
    for (commandBuffer.pop(c); c!='\r'; commandBuffer.pop(c)){}
}

void debug_char(char* buffer, int size){
    pc.printf("\n***\n");
    for (int i=0; i<size; ++i){
        pc.printf("%02x ", (unsigned) buffer[i]);
    }
    pc.printf("\n***\n");
}

void cmd_buffer_to_tune(volatile uint8_t Tunes[16]){
    char note;
    char c;
    int8_t tune; // [7:4] duration, [3:0] tune table index
    for (int i=0; i<16; ++i){
        commandBuffer.pop(note);
        if (note<'A' || note>'G') {
            //early exit, end of line or unexpected format
            Tunes[i] = 0;
            break;
        }
        tune=(note-'A')*2+1; // see docs for conversion


        commandBuffer.pop(c); // can be number OR #^
        if (c>='0' and c<='9'){
            Tunes[i] = (c-'0')<<4 | tune;
        }else{
            if (c=='#') tune += 1;
            else tune -= 1;

            commandBuffer.pop(c);
            Tunes[i] = (c-'0')<<4 | tune;
        }
    }
}

inline float cmd_buffer_to_number(){
    // THIS FUNCTION WILL READ UNTIL END OF LINE
    // Accept input format -?\d{1,4}(\.\d)? OR \d{1,3}(\.\d)?
    // Otherwise is undefined, buffer overflow!

    char buffer[32]; //  size need >=8, use 32 for safety
    char* p=buffer;

    // copy int from shared circularBuffer to local buffer
    commandBuffer.pop(*p);
    while(*p!='\r'){
        p+=1;
        commandBuffer.pop( *p ); //per-increment
    }
    *p='\0'; // replace \r with null


    return atof(buffer);
}

void TRD_command_parser(){
    while(true){
        ThisThread::flags_wait_any(SIGNAL_NEWLINE); // blocking

        // parse command
        char cmd_type;
        commandBuffer.pop(cmd_type);


        switch (cmd_type){
        case 'R':
            motorCfgMutex.lock();
            motorCfg.TRotation = cmd_buffer_to_number();
            motorCfgMutex.unlock();

            motorController.flags_set(SIGNAL_MOTOR_T_ROTATION_CHANGE);
            break;

        case 'V':
            motorCfgMutex.lock();
            motorCfg.TSpeed = cmd_buffer_to_number();
            motorCfgMutex.unlock();

            motorController.flags_set(SIGNAL_MOTOR_T_SPEED_CHANGE);
            break;

        case 'K':
            pc.printf("\nK command\n");
            cmd_buffer_skip_line();
            break;

        case 'T':
            motorCfgMutex.lock();
            cmd_buffer_to_tune(motorCfg.Tunes);
            motorCfgMutex.unlock();

            motorController.flags_set(SIGNAL_MOTOR_T_TUNE_CHANGE);
            break;

        default:
            pc.printf("\nUnkown command\n");
            cmd_buffer_skip_line();
        }


    }
}

