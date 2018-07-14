#include "LSM6DSL.h"
#include "PWM_functions.h"
#include "Filters.h"

//#define MATLAB_OUT
//#define MATLAB_OUT_2
#define DEBUG_XL
#define DEBUG_GYRO
#define DEBUG_FILTER
#define DEBUG_DUTY_CYCLE

#define TICKRATE_HZ    (50)             /* SysTick rate in Hz */

#define ACC_ODR        (104)            /* [Hz] 0; 12.5; 26; 52; 104; 208; 416; 833; 1660; 3330; 6660 */
#define ACC_FS         (2)              /* [g] 2; 4; 8; 16; */

#define ACC_OFS_X      (5)              /* Board specific values for the user offset registers */
#define ACC_OFS_Y      (15)
#define ACC_OFS_Z      (18)

#define GYRO_ODR       (104)            /* [Hz] 0; 12.5; 26; 52; 104; 208; 416; 833; 1660; 3330; 6660 */
#define GYRO_FS        (250)            /* [dps] 125; 250; 500; 1000; 2000 */

int ticks           = 0;                /* Time tracking variable */
int ticks_pwm       = 0;
int16_t ofs_gyro[3] = {};               /* Zeroing values for the gyro readings */

double gyro[3]      = {};               /* Arrays for elaborated sensors data */
double acc[3]       = {};
double pitch_f      = 0;                /* Filtered pitch angle value */
double roll         = 0;
double yaw          = 0;
double dutycycle    = 7.5;
double state        = 0;

void LSM6DSL_Read_XL_I2CM()
{
    int s = 0;
    int16_t rawdata_xl[3];

    /* Read from the LSM6DSL accelerometer */

    Sensor_Get_Raw_Data(LSM6DSL_I2C_ADDR_7BIT, LSM6DSL_OUTX_L_XL, rawdata_xl);

    for (s = 0;s < 3;s++){
        acc[s] = rawdata_xl[s]*LSM6DSL_Get_Sensitivity_Acc(ACC_FS);    // Introduce sensitivity [g]
    }
}

void LSM6DSL_Read_G_I2CM()
{
    int s = 0;
    int16_t rawdata_g[3];

    /* Read from the LSM6DSL gyroscope */

    Sensor_Get_Raw_Data(LSM6DSL_I2C_ADDR_7BIT, LSM6DSL_OUTX_L_G, rawdata_g);

    for (s = 0;s < 3;s++){
        rawdata_g[s]-=ofs_gyro[s];                                               // Offset correction
        gyro[s] = rawdata_g[s]*LSM6DSL_Get_Sensitivity_Gyro(GYRO_FS);            // Introduce sensitivity [dps]
    }
}

void LSM6DSL_Gyro_Calibration(void)
{
    /* Assuming the gyro still when turned on, this function computes the zeroing values for the axes */

    int16_t calibration_vector[3][5] = {};
    int16_t rawdata_calibration[3];
    int i = 0;
    int s = 0;

    while (i < 5){
        __WFI();
        Sensor_Get_Raw_Data(LSM6DSL_I2C_ADDR_7BIT, LSM6DSL_OUTX_L_G, rawdata_calibration);
        for (s = 0;s <= 2;s++){
            calibration_vector[s][i] = rawdata_calibration[s];
        }
        i++;
    }
    for(s = 0;s < 5;s++){
        ofs_gyro[0] += calibration_vector[0][s];
        ofs_gyro[1] += calibration_vector[1][s];
        ofs_gyro[2] += calibration_vector[2][s];
    }
    ofs_gyro[0] /= 5;
    ofs_gyro[1] /= 5;
    ofs_gyro[2] /= 5;
}

void Display_Readings()
{
    /* Displays sensor readings and elaborated data on the serial port */

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
        DEBUGOUT(" Pitch = %5.3f", 90+pitch_f);
        DEBUGOUT("    \n\r");
    #endif
    #ifdef DEBUG_DUTY_CYCLE
        DEBUGOUT(" Duty = %5.3f", Angle_To_DutyCycle(90-pitch_f));
        DEBUGOUT("    \n\r");
    #endif
    #ifndef MATLAB_OUT
        #ifndef MATLAB_OUT_2
            DEBUGOUT("\033[u"); // Load cursor position
        #endif
    #endif
}

void Servo_Reset ()
{
    /* Reset the servomotor to 90 degrees */

    ticks = 0;
    Chip_SCTPWM_SetDutyCycle(SCT_PWM, 1, Chip_SCTPWM_PercentageToTicks_Float(SCT_PWM, dutycycle));
    
    while (ticks <= 75){
    __WFI();    // Wait 1.5 seconds to reset
    }
    ticks = 0;
}

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
    LSM6DSL_Setup_I2CM(ACC_ODR,ACC_FS,GYRO_ODR,GYRO_FS);            // Setup LSM6DSL control registers using I2C
    LSM6DSL_Calibration_Acc(ACC_OFS_X,ACC_OFS_Y,ACC_OFS_Z);         // Startup calibration routines
    LSM6DSL_Gyro_Calibration();

    while (1){
        while (ticks >= 1){                                                     // First measurement or pitch is 0, execute at least every 20 ms
            LSM6DSL_Read_G_I2CM();                                              // Check sensors and compute pitch
            LSM6DSL_Read_XL_I2CM();
            pitch_f     = Pitch_KalmanFilter(ticks, TICKRATE_HZ, acc, gyro);
            dutycycle   = Angle_To_DutyCycle(90+pitch_f);                       // First duty correction (nothing happens if pitch_f = 0)
            Display_Readings();                                                 // Warning - lowers performance
            ticks       = 0;

            while (pitch_f != 0) {                                              // Correction cycle
                if (pitch_f > 4.0)                                              // If it's inside the tolerance range, do nothing
                    dutycycle+=(0.005);
                if (pitch_f < -4.0)
                    dutycycle -= (0.005);

                LSM6DSL_Read_G_I2CM();                                          // Check sensors and compute pitch
                LSM6DSL_Read_XL_I2CM();
                pitch_f = Pitch_KalmanFilter(ticks, TICKRATE_HZ, acc, gyro);
                //Display_Readings();                                             // Warning - lowers performance
                ticks = 0;
            }
        }
    }
    return 0;
}
