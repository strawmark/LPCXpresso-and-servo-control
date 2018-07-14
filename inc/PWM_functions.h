#ifndef __PWM_FUNCTIONS_H__
#define __PWM_FUNCTIONS_H__

#include "chip.h"

#define SCT_PWM         LPC_SCT0                          /* Use SCT0 for PWM */
#define SCT_PWM_PIN_OUT 1                                 /* COUT1 Generate square wave */
#define SCT_PWM_OUT     1                                 /* Index of OUT PWM */
#define SCT_PWM_RATE    50                                /* PWM frequency, 50 Hz for servo control */

#define MAX_DUTY_CYCLE  (11.25)                           /* Using Parallax Standard Servo */
#define MIN_DUTY_CYCLE  (3.75)

void App_Setup_Pin(void);                                                       // Setup board specific pin muxing
double Chip_SCTPWM_PercentageToTicks_Float(LPC_SCT_T *pSCT, double percent);	// Percentage to ticks conversion, with float values
double Angle_To_DutyCycle(double angle);										// Operations

#endif
