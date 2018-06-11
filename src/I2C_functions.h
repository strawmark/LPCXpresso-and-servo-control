#ifndef __I2C_FUNCTIONS_H__
#define __I2C_FUNCTIONS_H__

#define I2C_CLK_DIVIDER         (40)        /* I2C clock is set to 1.8MHz */
#define I2C_BITRATE             (100000)    /* 100KHz I2C bit-rate */
#define I2C_MODE                (0)         /* Standard I2C mode */

static I2CM_XFER_T  i2cmXferRec;            /* I2CM transfer record */


static void Init_I2C_PinMux(void)
{
    /* Initializes pin muxing for I2C interface - note that SystemInit() may already setup your pin muxing at system startup */

    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 22, IOCON_DIGMODE_EN | I2C_MODE);
    Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 23, IOCON_DIGMODE_EN | I2C_MODE);
    Chip_SWM_EnableFixedPin(SWM_FIXED_I2C0_SCL);
    Chip_SWM_EnableFixedPin(SWM_FIXED_I2C0_SDA);
}


static void setupI2CMaster()
{
    /* Setup I2C handle and parameters */

    /* Enable I2C clock and reset I2C peripheral - the boot ROM does not do this */
    Chip_I2C_Init(LPC_I2C0);

    /* Setup clock rate for I2C */
    Chip_I2C_SetClockDiv(LPC_I2C0, I2C_CLK_DIVIDER);

    /* Setup I2CM transfer rate */
    Chip_I2CM_SetBusSpeed(LPC_I2C0, I2C_BITRATE);

    /* Enable Master Mode */
    Chip_I2CM_Enable(LPC_I2C0);
}


static void WaitForI2cXferComplete(I2CM_XFER_T *xferRecPtr)
{
    /* Function to wait for I2CM transfer completion */
    /* Test for still transferring data */
    while (xferRecPtr->status == I2CM_STATUS_BUSY) {
        /* Sleep until next interrupt */
        __WFI();
    }
}


static void SetupXferRecAndExecute(uint8_t devAddr,uint8_t *txBuffPtr,uint16_t txSize,uint8_t *rxBuffPtr,uint16_t rxSize)
{
    /* Function to setup and execute I2C transfer request */

    /* Setup I2C transfer record */
    i2cmXferRec.slaveAddr   = devAddr;
    i2cmXferRec.status      = 0;
    i2cmXferRec.txSz        = txSize;
    i2cmXferRec.rxSz        = rxSize;
    i2cmXferRec.txBuff      = txBuffPtr;
    i2cmXferRec.rxBuff      = rxBuffPtr;

    /* Wait for master to go pending - needed in mixed master/slave mode on single I2C bus */
    while (Chip_I2CM_IsMasterPending(LPC_I2C0) == false) {}

    //Chip_I2CM_Xfer(LPC_I2C0, &i2cmXferRec);
    Chip_I2CM_XferBlocking(LPC_I2C0, &i2cmXferRec);
    /* Enable Master Interrupts */
    Chip_I2C_EnableInt(LPC_I2C0, I2C_INTENSET_MSTPENDING | I2C_INTENSET_MSTRARBLOSS | I2C_INTENSET_MSTSTSTPERR);
    /* Wait for transfer completion */
    WaitForI2cXferComplete(&i2cmXferRec);
    /* Clear all Interrupts */
    Chip_I2C_ClearInt(LPC_I2C0, I2C_INTENSET_MSTPENDING | I2C_INTENSET_MSTRARBLOSS | I2C_INTENSET_MSTSTSTPERR);
}

void WriteRegisterBit (uint8_t devi2c_address, uint8_t address,uint8_t bit_pos,uint8_t newvalue)
{
    /* Writes bit_pos bit of a register */

    uint8_t transferdata[2] = {address,0x00};
    uint8_t read_data;

    SetupXferRecAndExecute(devi2c_address,transferdata, 1,&read_data, 1);
    transferdata[1] = read_data;

    switch (newvalue){
    case 0:
        transferdata[1] &= ~(1U<<bit_pos);    // Clear bit in bit_pos
        break;
    case 1:
        transferdata[1] |= 1U<<bit_pos;       // Set bit in bit_pos
        break;
    }

    SetupXferRecAndExecute(devi2c_address, transferdata, 2, &read_data,0); // write the new data into the register
}

void WriteRegister (uint8_t devi2c_address, uint8_t address,uint8_t newvalue)
{
    /* Writes a new value into a register */

    uint8_t transferdata[2] = {address,newvalue};
    uint8_t read_data;

    SetupXferRecAndExecute(devi2c_address, transferdata, 2, &read_data,0); // put the new value into the register
}

uint8_t ReadRegister(uint8_t devi2c_address,uint8_t address)
{
    uint8_t data;
    SetupXferRecAndExecute(devi2c_address,&address, 1,&data, 1);
    return data;
}

void Sensor_Get_Raw_Data(uint8_t devi2c_address,uint8_t address,int16_t* data)
{
    uint8_t data_buffer[6];
    int s, i = 0;

    SetupXferRecAndExecute(devi2c_address,&address, 1,data_buffer, 6);
    if (i2cmXferRec.status == I2CM_STATUS_OK) {
        for (s = 0;s < 3;s++){
            data[s] = (int16_t)(data_buffer[i+1] << 8) | data_buffer [i];
            i+=2;
        }
    }
    else
        DEBUGOUT("Error %d.\r\n", i2cmXferRec.status );
}

void I2C0_IRQHandler(void)
{
    /* Call I2CM ISR function with the I2C device and transfer receive */
    Chip_I2CM_XferHandler(LPC_I2C0, &i2cmXferRec);
}
#endif
