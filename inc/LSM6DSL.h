#ifndef __LSM6DSL_H__
#define __LSM6DSL_H__

#include "I2C_functions.h"

/* Functions */

/* LSM6DSL_functions.c */
void   LSM6DSL_Boot (void);
void   LSM6DSL_Reset (void);
void   LSM6DSL_Calibration_Acc(int ofs_x,int ofs_y,int ofs_z);
void   LSM6DSL_Setup_Acc (float new_odr, uint16_t new_fs);
void   LSM6DSL_Setup_Gyro (float new_odr, uint16_t new_fs);
void   LSM6DSL_Setup_I2CM (float acc_odr, uint16_t acc_fullscale, float gyro_odr, uint16_t gyro_fullscale);
double LSM6DSL_Get_Sensitivity_Gyro(int fs);
double LSM6DSL_Get_Sensitivity_Acc(int fs);

/* Registers */

#define LSM6DSL_I2C_ADDR_7BIT                           (0x6B) // 7-bit I2C address of LSM6DSL

#define LSM6DSL_WHO_AM_I                                (0x0F)
#define LSM6DSL_CTRL1_XL                                (0x10) // Accelerometer control register
#define LSM6DSL_CTRL2_G                                 (0x11) // Gyro control register
#define LSM6DSL_CTRL3_C                                 (0x12)
#define LSM6DSL_CTRL5_C                                 (0x14)
#define LSM6DSL_CTRL6_C                                 (0x15)

#define LSM6DSL_OUTX_L_G                                (0x22) // Raw gyroscope data registers
#define LSM6DSL_OUTX_H_G                                (0x23)
#define LSM6DSL_OUTY_L_G                                (0x24)
#define LSM6DSL_OUTY_H_G                                (0x25)
#define LSM6DSL_OUTZ_L_G                                (0x26)
#define LSM6DSL_OUTZ_H_G                                (0x27)

#define LSM6DSL_OUTX_L_XL                               (0x28) // Raw accelerometer data registers
#define LSM6DSL_OUTX_H_XL                               (0x29)
#define LSM6DSL_OUTY_L_XL                               (0x2A)
#define LSM6DSL_OUTY_H_XL                               (0x2B)
#define LSM6DSL_OUTZ_L_XL                               (0x2C)
#define LSM6DSL_OUTZ_H_XL                               (0x2D)

#define LSM6DSL_X_OFS_USR                               (0x73) // Accelerometer user offset correction registers
#define LSM6DSL_Y_OFS_USR                               (0x74)
#define LSM6DSL_Z_OFS_USR                               (0x75)

/* Masks */

                                                        /* Accelerometer */
typedef enum {
    LSM6DSL_CTRL1_XL_ODR_XL_POWER_DOWN                  = 0x00, // ODR selection
    LSM6DSL_CTRL1_XL_ODR_XL_12_5Hz                      = 0x10,
    LSM6DSL_CTRL1_XL_ODR_XL_26Hz                        = 0x20,
    LSM6DSL_CTRL1_XL_ODR_XL_52Hz                        = 0x30,
    LSM6DSL_CTRL1_XL_ODR_XL_104Hz                       = 0x40,
    LSM6DSL_CTRL1_XL_ODR_XL_208Hz                       = 0x50,
    LSM6DSL_CTRL1_XL_ODR_XL_416Hz                       = 0x60,
    LSM6DSL_CTRL1_XL_ODR_XL_833Hz                       = 0x70,
    LSM6DSL_CTRL1_XL_ODR_XL_1660Hz                      = 0x80,
    LSM6DSL_CTRL1_XL_ODR_XL_3330Hz                      = 0x90,
    LSM6DSL_CTRL1_XL_ODR_XL_6660Hz                      = 0xA0,
} LSM6DSL_CTRL1_XL_ODR_T;

typedef enum {
    LSM6DSL_CTRL1_XL_FS_XL_2g                           = 0x00, // Full scale selection
    LSM6DSL_CTRL1_XL_FS_XL_4g                           = 0x08,
    LSM6DSL_CTRL1_XL_FS_XL_8g                           = 0x0C,
    LSM6DSL_CTRL1_XL_FS_XL_16g                          = 0x04,
} LSM6DSL_CTRL1_XL_FS_T;

                                                        /* Gyroscope */
typedef enum {
    LSM6DSL_CTRL2_G_ODR_G_POWER_DOWN                    = 0x00, // ODR selection
    LSM6DSL_CTRL2_G_ODR_G_12_5Hz                        = 0x10,
    LSM6DSL_CTRL2_G_ODR_G_26Hz                          = 0x20,
    LSM6DSL_CTRL2_G_ODR_G_52Hz                          = 0x30,
    LSM6DSL_CTRL2_G_ODR_G_104Hz                         = 0x40,
    LSM6DSL_CTRL2_G_ODR_G_208Hz                         = 0x50,
    LSM6DSL_CTRL2_G_ODR_G_416Hz                         = 0x60,
    LSM6DSL_CTRL2_G_ODR_G_833Hz                         = 0x70,
    LSM6DSL_CTRL2_G_ODR_G_1660Hz                        = 0x80,
    LSM6DSL_CTRL2_G_ODR_G_3330Hz                        = 0x90,
    LSM6DSL_CTRL2_G_ODR_G_6660Hz                        = 0xA0,
} LSM6DSL_CTRL2_G_ODR_T;

typedef enum {
    LSM6DSL_CTRL2_G_FS_G_125dps                         = 0x02, // Full scale selection
    LSM6DSL_CTRL2_G_FS_G_250dps                         = 0x00,
    LSM6DSL_CTRL2_G_FS_G_500dps                         = 0x04,
    LSM6DSL_CTRL2_G_FS_G_1000dps                        = 0x08,
    LSM6DSL_CTRL2_G_FS_G_2000dps                        = 0x0C,
} LSM6DSL_CTRL2_G_FS_T;

                                                        /* CTRL_3_C */
typedef enum {
    LSM6DSL_CTRL3_C_BOOT                                = 0x80,
    LSM6DSL_CTRL3_C_SW_RESET                            = 0x01,
} LSM6DSL_CTRL3_C_T;

typedef enum {
    LSM6DSL_CTRL3_C_BDU_ENABLE                          = 0x40,
    LSM6DSL_CTRL3_C_BDU_DISABLE                         = 0x00,
} LSM6DSL_CTRL3_C_BDU_T;

typedef enum {
    LSM6DSL_CTRL3_C_H_LACTIVE_ENABLE                    = 0x20,
    LSM6DSL_CTRL3_C_H_LACTIVE_DISABLE                   = 0x00,
} LSM6DSL_CTRL3_C_H_LACTIVE_T;

typedef enum {
    LSM6DSL_CTRL3_C_PP_OD_ENABLE                        = 0x10,
    LSM6DSL_CTRL3_C_PP_OD_DISABLE                       = 0x00,
} LSM6DSL_CTRL3_C_PP_OD_T;

typedef enum {
    LSM6DSL_CTRL3_C_SIM_ENABLE                          = 0x08,
    LSM6DSL_CTRL3_C_SIM_DISABLE                         = 0x00,
} LSM6DSL_CTRL3_C_SIM_T;

typedef enum {
    LSM6DSL_CTRL3_C_IF_INC_ENABLE                       = 0x04,
    LSM6DSL_CTRL3_C_IF_INC_DISABLE                      = 0x00,
} LSM6DSL_CTRL3_C_IF_INC_T;

typedef enum {
    LSM6DSL_CTRL3_C_BLE_ENABLE                          = 0x02,
    LSM6DSL_CTRL3_C_BLE_DISABLE                         = 0x00,
} LSM6DSL_CTRL3_C_BLE_T;

/* Low-pass filter */
typedef enum {
    LSM6DSL_CTRL6_C_TRIGMODE_Edge_sensitive             = 0x80, // Trigger mode selection
    LSM6DSL_CTRL6_C_TRIGMODE_Level_sensitive            = 0x40,
    LSM6DSL_CTRL6_C_TRIGMODE_Level_sensitive_latched    = 0x60,
    LSM6DSL_CTRL6_C_TRIGMODE_Level_sensitive_FIFO       = 0xC0,
} LSM6DSL_CTRL6_C_TRIGMODE_T;

typedef enum {
    LSM6DSL_CTRL6_C_XL_HM_MODE_ENABLE                   = 0x00, // Enable High performance mode enabled
    LSM6DSL_CTRL6_C_XL_HM_MODE_DISABLE                  = 0x10, // Disable High performance mode
} LSM6DSL_CTRL6_C_XL_HM_MODE;

typedef enum {
    LSM6DSL_CTRL6_C_USR_OFF_W_2e_10                     = 0x00, // Set user accelerometer offset weight to 2^-10 g/LSB
    LSM6DSL_CTRL6_C_USR_OFF_W_2e_06                     = 0x08, // Set user accelerometer offset weight to 2^-06 g/LSB
} LSM6DSL_CTRL6_C_USR_OFF_W_T;

typedef enum {
 LSM6DSL_CTRL6_C_LPF1_245Hz                             = 0x00, // ODR = 800Hz
 LSM6DSL_CTRL6_C_LPF1_195Hz                             = 0x01,
 LSM6DSL_CTRL6_C_LPF1_155Hz                             = 0x02,
 LSM6DSL_CTRL6_C_LPF1_293Hz                             = 0x03,

 LSM6DSL_CTRL6_C_LPF1_315Hz                             = 0x00, // ODR = 1.6kHz
 LSM6DSL_CTRL6_C_LPF1_224Hz                             = 0x01,
 LSM6DSL_CTRL6_C_LPF1_168Hz                             = 0x02,
 LSM6DSL_CTRL6_C_LPF1_505Hz                             = 0x03,

 LSM6DSL_CTRL6_C_LPF1_343Hz                             = 0x00, // ODR = 3.3kHz
 LSM6DSL_CTRL6_C_LPF1_234Hz                             = 0x01,
 LSM6DSL_CTRL6_C_LPF1_172Hz                             = 0x02,
 LSM6DSL_CTRL6_C_LPF1_925Hz                             = 0x03,

 LSM6DSL_CTRL6_C_LPF1_351Hz                             = 0x00, // ODR = 6.6kHz
 LSM6DSL_CTRL6_C_LPF1_237Hz                             = 0x01,
 LSM6DSL_CTRL6_C_LPF1_173Hz                             = 0x02,
 LSM6DSL_CTRL6_C_LPF1_937Hz                             = 0x03,
} LSM6DSL_CTRL6_C_LPF1_T;

#endif
