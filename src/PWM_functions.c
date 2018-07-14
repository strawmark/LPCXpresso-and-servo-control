#include "PWM_functions.h"

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

/* Conversion from an angle between 0 and 180 degrees to a duty cycle between MIN_DUTY_CYCLE and MAX_DUTY_CYCLE */

double Angle_To_DutyCycle(double angle)
{
    static double x         = (double)(MAX_DUTY_CYCLE - MIN_DUTY_CYCLE);
    static double min_duty  = (double)(MIN_DUTY_CYCLE);
    return (angle/180) * x + min_duty;
}
