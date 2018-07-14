#include "LSM6DSL.h"

/* Read from the LSM6DSL gyroscope */
void LSM6DSL_Read_G_I2CM(double *gyro_data, int16_t *offsets, double *gyro_scale)
{    
    int s = 0;
    int16_t rawdata_g[3];

    Sensor_Get_Raw_Data(LSM6DSL_I2C_ADDR_7BIT, LSM6DSL_OUTX_L_G, rawdata_g);

    for (s = 0;s < 3;s++){
        rawdata_g[s]-=offsets[s];                       // Offset correction
        gyro_data[s] = rawdata_g[s] * *gyro_scale;      // Introduce sensitivity [dps]
    }
}

/* Read from the LSM6DSL accelerometer */
void LSM6DSL_Read_XL_I2CM(double *acc_data, double *acc_scale)
{	
    int s = 0;
    int16_t rawdata_xl[3];

    Sensor_Get_Raw_Data(LSM6DSL_I2C_ADDR_7BIT, LSM6DSL_OUTX_L_XL, rawdata_xl);

    for (s = 0;s < 3;s++){
        acc_data[s] = rawdata_xl[s]* *acc_scale;    // Introduce sensitivity [g]
    }
}

/* Setup for register writing, the inputs should be global variables */
/* used to transform the raw data input to real data.                */
void LSM6DSL_Setup_I2CM (double *gyro_scale, double *acc_scale)
{
    LSM6DSL_Setup_Acc(ACC_ODR,ACC_FS);
    LSM6DSL_Setup_Gyro(GYRO_ODR,GYRO_FS);
    WriteRegister(LSM6DSL_I2C_ADDR_7BIT,LSM6DSL_CTRL3_C,
            LSM6DSL_CTRL3_C_BDU_ENABLE |                    // Enable Block data update
            LSM6DSL_CTRL3_C_IF_INC_ENABLE                   // Increment register address automatically during multiple byte access
            );
    *gyro_scale = LSM6DSL_Get_Sensitivity_Gyro(GYRO_FS);
    *acc_scale  = LSM6DSL_Get_Sensitivity_Acc(ACC_FS);
}

/* Returns the correct accelerometer scale factor correspondent to the selected fullscale */
double LSM6DSL_Get_Sensitivity_Acc(int fs)
{
    double sensitivity = 0;
    switch(fs){
    case 2:
        sensitivity = LSM6DSL_ACC_SENSITIVITY_FS_2G;
    break;
    case 4:
        sensitivity = LSM6DSL_ACC_SENSITIVITY_FS_4G;
    break;
    case 8:
        sensitivity = LSM6DSL_ACC_SENSITIVITY_FS_8G;
    break;
    case 16:
        sensitivity = LSM6DSL_ACC_SENSITIVITY_FS_16G;
    break;
    }
    return sensitivity/1000; // mg->g
}

/* Returns the correct gyroscope scale factor correspondent to the selected fullscale */
double LSM6DSL_Get_Sensitivity_Gyro(int fs)
{
    double sensitivity = 0;
    switch(fs){

    case 125:
        sensitivity = LSM6DSL_GYRO_SENSITIVITY_FS_125DPS;
    break;
    case 250:
        sensitivity = LSM6DSL_GYRO_SENSITIVITY_FS_250DPS;
    break;
    case 500:
        sensitivity = LSM6DSL_GYRO_SENSITIVITY_FS_500DPS;
    break;
    case 1000:
        sensitivity = LSM6DSL_GYRO_SENSITIVITY_FS_1000DPS;
    break;
    case 2000:
        sensitivity = LSM6DSL_GYRO_SENSITIVITY_FS_2000DPS;
    break;
    }
    return sensitivity/1000; // mdps->dps
}

/* Writes the offset registers for the accelerometer */
void LSM6DSL_Calibration_Acc(int ofs_x,int ofs_y,int ofs_z)
{
    WriteRegister(LSM6DSL_I2C_ADDR_7BIT,LSM6DSL_CTRL6_C,LSM6DSL_CTRL6_C_USR_OFF_W_2e_10);    // Set offset weight 2^-10

    /* Setting User offset registers */

    WriteRegister(LSM6DSL_I2C_ADDR_7BIT,LSM6DSL_X_OFS_USR,ofs_x);
    WriteRegister(LSM6DSL_I2C_ADDR_7BIT,LSM6DSL_Y_OFS_USR,ofs_y);
    WriteRegister(LSM6DSL_I2C_ADDR_7BIT,LSM6DSL_Z_OFS_USR,ofs_z);
}

/* Assuming the gyro still when turned on, this function computes the zeroing values for the axes */
void LSM6DSL_Calibration_Gyro(int16_t *offsets)
{
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
    	offsets[0] += calibration_vector[0][s];
    	offsets[1] += calibration_vector[1][s];
    	offsets[2] += calibration_vector[2][s];
    }
    offsets[0] /= 5;
    offsets[1] /= 5;
    offsets[2] /= 5;
}

/* Sets up the gyroscope to the chosen output and fullscale */
void LSM6DSL_Setup_Gyro (float new_odr, uint16_t new_fs)
{
    uint8_t status = 0 ;

    switch ((uint16_t)new_odr){

    case 0:
        status |= LSM6DSL_CTRL2_G_ODR_G_POWER_DOWN;
        break;

    case 12:
        status |= LSM6DSL_CTRL2_G_ODR_G_12_5Hz;
        break;

    case 26:
        status |= LSM6DSL_CTRL2_G_ODR_G_26Hz;
        break;

    case 52:
        status |= LSM6DSL_CTRL2_G_ODR_G_52Hz;
        break;

    case 104:
        status |= LSM6DSL_CTRL2_G_ODR_G_104Hz ;
        break;

    case 208:
        status |= LSM6DSL_CTRL2_G_ODR_G_208Hz;
        break;

    case 416:
        status |= LSM6DSL_CTRL2_G_ODR_G_416Hz;
        break;

    case 833:
        status |= LSM6DSL_CTRL2_G_ODR_G_833Hz;
        break;

    case 1660:
        status |= LSM6DSL_CTRL2_G_ODR_G_1660Hz;
        break;

    case 3330:
        status |= LSM6DSL_CTRL2_G_ODR_G_3330Hz;
        break;

    case 6660:
        status |= LSM6DSL_CTRL2_G_ODR_G_6660Hz;
        break;
    }

    switch (new_fs) {
    case 125:
        status |= LSM6DSL_CTRL2_G_FS_G_125dps;
        break;

    case 250:
        status |= LSM6DSL_CTRL2_G_FS_G_250dps;
        break;

    case 500:
        status |= LSM6DSL_CTRL2_G_FS_G_500dps;
        break;

    case 1000:
        status |= LSM6DSL_CTRL2_G_FS_G_1000dps;
        break;

    case 2000:
        status |= LSM6DSL_CTRL2_G_FS_G_2000dps;
        break;
    }
    WriteRegister(LSM6DSL_I2C_ADDR_7BIT,LSM6DSL_CTRL2_G,status);    // Write the new data to the register
}

/* Sets up the accelerometer to the chosen output and fullscale */
void LSM6DSL_Setup_Acc (float new_odr, uint16_t new_fs)
{
    uint8_t status = 0 ;

    switch ((uint16_t)new_odr){

    case 0:
    status |= LSM6DSL_CTRL1_XL_ODR_XL_POWER_DOWN;
    break;

    case 12:
    status |= LSM6DSL_CTRL1_XL_ODR_XL_12_5Hz;
        break;

    case 26:
    status |= LSM6DSL_CTRL1_XL_ODR_XL_26Hz;
    break;

    case 52:
    status |= LSM6DSL_CTRL1_XL_ODR_XL_52Hz;
    break;

    case 104:
    status |= LSM6DSL_CTRL1_XL_ODR_XL_104Hz ;
    break;

    case 208:
    status |= LSM6DSL_CTRL1_XL_ODR_XL_208Hz;
    break;

    case 416:
    status |= LSM6DSL_CTRL1_XL_ODR_XL_416Hz;
    break;

    case 833:
    status |= LSM6DSL_CTRL1_XL_ODR_XL_833Hz;
    break;

    case 1660:
    status |= LSM6DSL_CTRL1_XL_ODR_XL_1660Hz;
    break;

    case 3330:
    status |= LSM6DSL_CTRL1_XL_ODR_XL_3330Hz;
    break;

    case 6660:
    status |= LSM6DSL_CTRL1_XL_ODR_XL_6660Hz;
    break;
    }

    switch (new_fs) {

    case 2:
    status |= LSM6DSL_CTRL1_XL_FS_XL_2g;
    break;
    
    case 4:
    status |= LSM6DSL_CTRL1_XL_FS_XL_4g;
    break;
    
    case 8:
    status |= LSM6DSL_CTRL1_XL_FS_XL_8g;
    break;
    
    case 16:
    status |= LSM6DSL_CTRL1_XL_FS_XL_16g;
    break;
    }
    WriteRegister(LSM6DSL_I2C_ADDR_7BIT,LSM6DSL_CTRL1_XL,status);    // Write the new data to the register
}

/* Reset the LSM6DSL sensor */
void LSM6DSL_Reset (void)
{    
    WriteRegister(LSM6DSL_I2C_ADDR_7BIT,LSM6DSL_CTRL2_G,LSM6DSL_CTRL2_G_ODR_G_POWER_DOWN);  // CTRL2_G; Power-Down mode
    WriteRegister(LSM6DSL_I2C_ADDR_7BIT,LSM6DSL_CTRL6_C,LSM6DSL_CTRL6_C_XL_HM_MODE_ENABLE); // CTRL6_C; High Performance Mode
    WriteRegister(LSM6DSL_I2C_ADDR_7BIT,LSM6DSL_CTRL3_C,LSM6DSL_CTRL3_C_SW_RESET);          // CTRL_3_C, SW_RESET
    __WFI();                                                                                // Reset phase takes 50us, wait one Systick interrupt
}

/* Executes a boot sequence */
void LSM6DSL_Boot (void)
{
    WriteRegister(LSM6DSL_I2C_ADDR_7BIT,LSM6DSL_CTRL2_G,LSM6DSL_CTRL2_G_ODR_G_POWER_DOWN);  // Power-Down, 250 dps
    WriteRegister(LSM6DSL_I2C_ADDR_7BIT,LSM6DSL_CTRL6_C,LSM6DSL_CTRL6_C_XL_HM_MODE_ENABLE); // CTRL6_C; High Performance Mode
    WriteRegister(LSM6DSL_I2C_ADDR_7BIT,LSM6DSL_CTRL3_C,LSM6DSL_CTRL3_C_BOOT);              // Reboot memory content

    int i;
    for(i = 0;i <= 100;i++){                                                                // Wait before doing anything
        __WFI();
    }
}
