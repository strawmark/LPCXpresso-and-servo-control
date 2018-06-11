#ifndef __LSM6DSL_FUNCTIONS_H__
#define __LSM6DSL_FUNCTIONS_H__

#include "I2C_functions.h"
#include "LSM6DSL.h"

void   LSM6DSL_Boot (void);
void   LSM6DSL_Reset (void);
void   LSM6DSL_Calibration_Acc(int ofs_x,int ofs_y,int ofs_z);
void   LSM6DSL_Setup_Acc (float new_odr, uint16_t new_fs);
void   LSM6DSL_Setup_Gyro (float new_odr, uint16_t new_fs);
void   LSM6DSL_Setup_I2CM (float acc_odr, uint16_t acc_fullscale, float gyro_odr, uint16_t gyro_fullscale);
double LSM6DSL_Get_Sensitivity_Gyro(int fs);
double LSM6DSL_Get_Sensitivity_Acc(int fs);

#define LSM6DSL_ACC_SENSITIVITY_FS_2G       0.061   // Sensitivity values for the accelerometer [mg/LSB]
#define LSM6DSL_ACC_SENSITIVITY_FS_4G       0.122
#define LSM6DSL_ACC_SENSITIVITY_FS_8G       0.244
#define LSM6DSL_ACC_SENSITIVITY_FS_16G      0.488

#define LSM6DSL_GYRO_SENSITIVITY_FS_125DPS  04.375  // Sensitivity values for the gyroscope [mdps/LSB]
#define LSM6DSL_GYRO_SENSITIVITY_FS_250DPS  08.750
#define LSM6DSL_GYRO_SENSITIVITY_FS_500DPS  17.500
#define LSM6DSL_GYRO_SENSITIVITY_FS_1000DPS 35.000
#define LSM6DSL_GYRO_SENSITIVITY_FS_2000DPS 70.000

void LSM6DSL_Setup_I2CM (float acc_odr, uint16_t acc_fullscale, float gyro_odr, uint16_t gyro_fullscale)
{
    /* Setup for register writing */

    LSM6DSL_Setup_Acc(acc_odr,acc_fullscale);
    LSM6DSL_Setup_Gyro(gyro_odr,gyro_fullscale);
    WriteRegister(LSM6DSL_I2C_ADDR_7BIT,LSM6DSL_CTRL3_C,
            LSM6DSL_CTRL3_C_BDU_ENABLE |                    // Enable Block data update
            LSM6DSL_CTRL3_C_IF_INC_ENABLE                   // Increment register address automatically during multiple byte access
            );
}

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

void LSM6DSL_Calibration_Acc(int ofs_x,int ofs_y,int ofs_z)
{
    WriteRegister(LSM6DSL_I2C_ADDR_7BIT,LSM6DSL_CTRL6_C,LSM6DSL_CTRL6_C_USR_OFF_W_2e_10);    // Set offset weight 2^-10

    /* Setting User offset registers */

    WriteRegister(LSM6DSL_I2C_ADDR_7BIT,LSM6DSL_X_OFS_USR,ofs_x);
    WriteRegister(LSM6DSL_I2C_ADDR_7BIT,LSM6DSL_Y_OFS_USR,ofs_y);
    WriteRegister(LSM6DSL_I2C_ADDR_7BIT,LSM6DSL_Z_OFS_USR,ofs_z);
}

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

void LSM6DSL_Reset (void)
{
    /* Reset the LSM6DSL sensor */
    WriteRegister(LSM6DSL_I2C_ADDR_7BIT,LSM6DSL_CTRL2_G,LSM6DSL_CTRL2_G_ODR_G_POWER_DOWN);  // CTRL2_G; Power-Down mode
    WriteRegister(LSM6DSL_I2C_ADDR_7BIT,LSM6DSL_CTRL6_C,LSM6DSL_CTRL6_C_XL_HM_MODE_ENABLE); // CTRL6_C; High Performance Mode
    WriteRegister(LSM6DSL_I2C_ADDR_7BIT,LSM6DSL_CTRL3_C,LSM6DSL_CTRL3_C_SW_RESET);          // CTRL_3_C, SW_RESET
    __WFI();                                                                                // Reset phase takes 50us, wait one Systick interrupt
}

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
#endif
