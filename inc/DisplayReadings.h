#ifndef __DISPLAYREADINGS_H__
#define __DISPLAYREADINGS_H__

#include "board.h"
#include "chip.h"
#include "PWM_Functions.h"

//#define MATLAB_OUT
//#define MATLAB_OUT_2
#define DEBUG_XL
#define DEBUG_GYRO
#define DEBUG_FILTER
#define DEBUG_DUTY_CYCLE

/* Displays sensor readings and elaborated data on the serial port */
void Display_Readings(double *gyro, double *acc, double *yaw, double *roll,double *pitch_f);

#endif
