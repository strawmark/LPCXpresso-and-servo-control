#include <stdint.h>
#include "board.h"
#include "LSM6DSL.h"
#include "PWM_functions.h"
#include "Filters.h"
#include "DisplayReadings.h"

#define TICKRATE_HZ    (50) /* SysTick rate in Hz */

void Servo_Reset ();        /* Prototype for the servomotor initialization */

int ticks           = 0;    /* Time tracking variable */
int16_t ofs_gyro[3] = {};   /* Zeroing values for the gyro readings */
double gyro[3]      = {};   /* Arrays for elaborated sensors data */
double acc[3]       = {};

double gyro_scale   = 0;    /* Parameters for the transformation from raw to real data*/
double acc_scale    = 0;

double pitch_f      = 0;    /* Filtered pitch angle value */
double roll         = 0;
double yaw          = 0;

double dutycycle    = 7.5;

void SysTick_Handler(void)
{
    ++ticks; // Used for time tracking
    Chip_SCTPWM_SetDutyCycle(SCT_PWM, 1, Chip_SCTPWM_PercentageToTicks_Float(SCT_PWM,dutycycle));
}

int main(void)
{
    SystemCoreClockUpdate();                                        // Generic Initialization
    Board_Init();
    Init_I2C_PinMux();                                              // Setup I2C
    setupI2CMaster();
    NVIC_EnableIRQ(I2C0_IRQn);                                      // Enable the interrupt for the I2C

    Chip_SCTPWM_Init(SCT_PWM);                                      // Initialize the SCT as PWM and set frequency
    Chip_SCTPWM_SetRate(SCT_PWM, SCT_PWM_RATE);
    App_Setup_Pin();                                                // Setup Board specific output pin
    Chip_SCTPWM_SetOutPin(SCT_PWM, SCT_PWM_OUT, SCT_PWM_PIN_OUT);   // Use SCT0_OUT1
    Chip_SCTPWM_Start(SCT_PWM);
    SysTick_Config(SystemCoreClock/TICKRATE_HZ);                    // Enable SysTick Timer
    Servo_Reset();                                                  // Move servo to neutral position
    LSM6DSL_Setup_I2CM(&gyro_scale,&acc_scale);                     // Setup LSM6DSL control registers using I2C
    LSM6DSL_Calibration_Acc(ACC_OFS_X,ACC_OFS_Y,ACC_OFS_Z);         // Startup calibration routines
    LSM6DSL_Calibration_Gyro(ofs_gyro);

    while (1){
        while (ticks >= 1){                                                     // First measurement or pitch is 0, execute at least every 20 ms
            LSM6DSL_Read_G_I2CM(gyro,ofs_gyro,&gyro_scale);                     // Check sensors and compute pitch
            LSM6DSL_Read_XL_I2CM(acc,&acc_scale);
            pitch_f     = Pitch_KalmanFilter(ticks, TICKRATE_HZ, acc, gyro);    
            dutycycle   = Angle_To_DutyCycle(90+pitch_f);                       // First duty correction (nothing happens if pitch_f = 0)
            //Display_Readings(gyro,acc,&yaw,&roll,&pitch_f);                   // Warning - lowers performance
            ticks       = 0;

            while (pitch_f != 0) {                                              // Correction cycle
                if (pitch_f > 4.0)                                              // If it's inside the tolerance range, do nothing
                    dutycycle+=(0.005);
                if (pitch_f < -4.0)
                    dutycycle -= (0.005);

                LSM6DSL_Read_G_I2CM(gyro,ofs_gyro,&gyro_scale);                 // Check sensors and compute pitch
                LSM6DSL_Read_XL_I2CM(acc,&acc_scale);
                pitch_f = Pitch_KalmanFilter(ticks, TICKRATE_HZ, acc, gyro);
                //Display_Readings(gyro,acc,&yaw,&roll,&pitch_f);               // Warning - lowers performance
                ticks = 0;
            }
        }
    }
    return 0;
}

void Servo_Reset ()
{
    /* Reset the servomotor to 90 degrees */

    ticks = 0;
    Chip_SCTPWM_SetDutyCycle(SCT_PWM, 1, Chip_SCTPWM_PercentageToTicks_Float(SCT_PWM, 7.5));

    while (ticks <= 75){
    __WFI();    // Wait 1.5 seconds to reset
    }
    ticks = 0;
}
