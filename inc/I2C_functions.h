#ifndef __I2C_FUNCTIONS_H__
#define __I2C_FUNCTIONS_H__

#include <stdint.h>
#include "board.h"

#define I2C_CLK_DIVIDER         (40)        /* I2C clock is set to 1.8MHz */
#define I2C_BITRATE             (100000)    /* 100KHz I2C bit-rate */
#define I2C_MODE                (0)         /* Standard I2C mode */

I2CM_XFER_T  i2cmXferRec;            		/* I2CM transfer record */

void Init_I2C_PinMux(void);
void setupI2CMaster();
void WaitForI2cXferComplete(I2CM_XFER_T *xferRecPtr);
void SetupXferRecAndExecute(uint8_t devAddr,uint8_t *txBuffPtr,uint16_t txSize,uint8_t *rxBuffPtr,uint16_t rxSize);
void WriteRegisterBit (uint8_t devi2c_address, uint8_t address,uint8_t bit_pos,uint8_t newvalue);
void WriteRegister (uint8_t devi2c_address, uint8_t address,uint8_t newvalue);
uint8_t ReadRegister(uint8_t devi2c_address,uint8_t address);
void Sensor_Get_Raw_Data(uint8_t devi2c_address,uint8_t address,int16_t* data);
void I2C0_IRQHandler(void);

#endif
