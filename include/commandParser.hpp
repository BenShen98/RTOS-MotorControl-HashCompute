#ifndef COMMANDPARSER_H
#define COMMANDPARSER_H

#include "dataStructure.hpp"

/*
All function below make use of EXTERN GLOBAL
    * pc
    * commandBuffer
*/

void ISR_command_getchar();

void cmd_buffer_skip_line();

void debug_char(char* buffer, int size);


// THIS FUNCTION WILL READ UNTIL END OF LINE
// Accept input format -?\d{1,4}(\.\d)? OR \d{1,3}(\.\d)?
// Otherwise is undefined, buffer overflow!
float cmd_buffer_to_number();

void TRD_command_parser();

#endif