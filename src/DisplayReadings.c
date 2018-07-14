#include "board.h"
#include "chip.h"
#include "PWM_Functions.h"

/* Output setup */

//#define MATLAB_OUT
//#define MATLAB_OUT_2
#define DEBUG_XL
#define DEBUG_GYRO
#define DEBUG_FILTER
//#define DEBUG_DUTY_CYCLE

/*-------------------------*/

/* Displays sensor readings and elaborated data on the serial port */
void Display_Readings(double *gyro, double *acc, double *yaw, double *roll,double *pitch_f)
{
    static int clearscreen = 0;
    if (!clearscreen){
        DEBUGOUT("\e[1;1H\e[2J");   // Clear screen, set cursor on the first line
        DEBUGOUT("\033[s");         // Save cursor position
        clearscreen = 1;
    }
    #ifdef MATLAB_OUT
        #undef MATLAB_OUT_2
        #undef DEBUG_XL
        #undef DEBUG_GYRO
        #undef DEBUG_FILTER
        #undef DEBUG_DUTY_CYCLE
        DEBUGOUT("%f %f %f %f %f %f\n\r", gyro[0],gyro[1],gyro[2],acc[0],acc[1],acc[2]);
    #endif
    #ifdef MATLAB_OUT_2
        #undef DEBUG_XL
        #undef DEBUG_GYRO
        #undef DEBUG_FILTER
        #undef DEBUG_DUTY_CYCLE
        DEBUGOUT("%f %f %f\n\r",pitch_f,roll,yaw);
    #endif
    #ifdef DEBUG_XL
        DEBUGOUT(" AccX = %5.3f      \t AccY = %5.3f      \t AccZ = %5.3f", acc[0],acc[1],acc[2]);
        DEBUGOUT("          \n\r");
    #endif
    #ifdef DEBUG_GYRO
        DEBUGOUT(" GyrX = %5.3f      \t GyrY = %5.3f      \t GyrZ = %5.3f", gyro[0],gyro[1],gyro[2]);
        DEBUGOUT("    \n\r");
    #endif
    #ifdef DEBUG_FILTER
        DEBUGOUT(" Pitch = %5.3f", 90+*pitch_f);
        DEBUGOUT("    \n\r");
    #endif
    #ifdef DEBUG_DUTY_CYCLE
        DEBUGOUT(" Duty = %5.3f", Angle_To_DutyCycle(90-*pitch_f));
        DEBUGOUT("    \n\r");
    #endif
    #ifndef MATLAB_OUT
        #ifndef MATLAB_OUT_2
            DEBUGOUT("\033[u"); // Load cursor position
        #endif
    #endif
}
