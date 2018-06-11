#ifndef __PWM_FUNCTIONS_H__
#define __PWM_FUNCTIONS_H__

#define SCT_PWM         LPC_SCT0                          /* Use SCT0 for PWM */
#define SCT_PWM_PIN_OUT 1                                 /* COUT1 Generate square wave */
#define SCT_PWM_OUT     1                                 /* Index of OUT PWM */
#define SCT_PWM_RATE    50                                /* PWM frequency, 50 Hz for servo control */

#define MAX_DUTY_CYCLE  (11.25)                           /* Using Parallax Standard Servo */
#define MIN_DUTY_CYCLE  (3.75)

/* Setup board specific pin muxing */

void App_Setup_Pin(void)
{
    /* Enable SWM clock before altering SWM */
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);

    /* Connect SCT_OUT1 to P0_12 (bottom side of the LCP Board) */
    Chip_SWM_MovablePinAssign(SWM_SCT0_OUT1_O, 12);
    Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
}

/* Percentage to ticks conversion, with float values */

double Chip_SCTPWM_PercentageToTicks_Float(LPC_SCT_T *pSCT, double percent)
{
    return (Chip_SCTPWM_GetTicksPerCycle(pSCT) * percent) / 100;
}

/* Operations */

double Angle_To_DutyCycle(double angle)
{
    static double x         = (double)(MAX_DUTY_CYCLE - MIN_DUTY_CYCLE);
    static double min_duty  = (double)(MIN_DUTY_CYCLE);
    return (angle/180) * x + min_duty;
}
#endif