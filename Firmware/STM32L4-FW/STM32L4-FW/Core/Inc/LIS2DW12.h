/*
 *
 * LIS2DW12 accelerometer I2C driver
 *
 * Author: Karzan M.
 * Created on: Aug 15, 2025
 * Datasheet download link: https://www.st.com/resource/en/datasheet/lis2dw12.pdf
 *
 */

#ifndef INC_LIS2DW12_H_
#define INC_LIS2DW12_H_

#include "stm32l4xx_hal.h" /* Needed for I2C HAL */

/*
 * DEFINES
 */
#define LIS2DW12_I2C_ADDR       (0b0011000 << 1)            /* 0b001100x where LSB x = 0 when SAO = 0 (p. 28) */

#define LIS2DW12_I2C_ADDR_R     (LIS2DW12_I2C_ADDR | 1)     /* ADDR + READ pattern for SAO = 0 */
#define LIS2DW12_I2C_ADDR_W     (LIS2DW12_I2C_ADDR)         /* ADDR + WRITE pattern for SAO = 0 */

#define LIS2DW12_ID             0b01000100                  /* WHO_AM_I default ID */

/*
 * REGISTERS (p. 33-34)
 */
#define LIS2DW12_REG_OUT_T_L                0x0D
#define LIS2DW12_REG_OUT_T_H                0x0E
#define LIS2DW12_REG_WHO_AM_I               0x0F
#define LIS2DW12_REG_CTRL1                  0x20
#define LIS2DW12_REG_CTRL2                  0x21
#define LIS2DW12_REG_CTRL3                  0x22
#define LIS2DW12_REG_CTRL4_INT1_PAD_CTRL    0x23
#define LIS2DW12_REG_CTRL5_INT2_PAD_CTRL    0x24
#define LIS2DW12_REG_CTRL6                  0x25
#define LIS2DW12_REG_OUT_T                  0x26
#define LIS2DW12_REG_STATUS                 0x27
#define LIS2DW12_REG_OUT_X_L                0x28
#define LIS2DW12_REG_OUT_X_H                0x29
#define LIS2DW12_REG_OUT_Y_L                0x2A
#define LIS2DW12_REG_OUT_Y_H                0x2B
#define LIS2DW12_REG_OUT_Z_L                0x2C
#define LIS2DW12_REG_OUT_Z_H                0x2D
#define LIS2DW12_REG_FIFO_CTRL              0x2E
#define LIS2DW12_REG_FIFO_SAMPLES           0x2F
#define LIS2DW12_REG_TAP_THS_X              0x30
#define LIS2DW12_REG_TAP_THS_Y              0x31
#define LIS2DW12_REG_TAP_THS_Z             0x32
#define LIS2DW12_REG_INT_DUR                0x33
#define LIS2DW12_REG_WAKE_UP_THS            0x34
#define LIS2DW12_REG_WAKE_UP_DUR            0x35
#define LIS2DW12_REG_FREE_FALL              0x36
#define LIS2DW12_REG_STATUS_DUP             0x37
#define LIS2DW12_REG_WAKE_UP_SRC            0x38
#define LIS2DW12_REG_TAP_SRC                0x39
#define LIS2DW12_REG_SIXD_SRC               0x3A
#define LIS2DW12_REG_ALL_INT_SRC            0x3B
#define LIS2DW12_REG_X_OFS_USR              0x3C
#define LIS2DW12_REG_Y_OFS_USR              0x3D
#define LIS2DW12_REG_Z_OFS_USR              0x3E
#define LIS2DW12_REG_CTRL7                  0x3F

/*
 * SENSOR STRUCT
 */

typedef struct {

    /* I2C handle for hal functions */
    I2C_HandleTypeDef *i2cHandle;

    /* Accelerometer (x, y, z) data in m/s^2 */
    float acc_mps2[3];

    /* temperature data in deg celsius */
    float temp_C;

} LIS2DW12;

/*
 * INITIALIZATION
 */
uint8_t LIS2DW12_Init( LIS2DW12 *dev, I2C_HandleTypeDef *i2cHandle );

/*
 * DATA AQUISITION
 */

HAL_StatusTypeDef LIS2DW12_ReadTemperature( LIS2DW12 *dev );
HAL_StatusTypeDef LIS2DW12_ReadAccelerations( LIS2DW12 *dev );

/*
 * LOW-LEVEL FUNCTIONS
 */

HAL_StatusTypeDef LIS2DW12_ReadRegister( LIS2DW12 *dev, uint8_t reg, uint8_t *data );
HAL_StatusTypeDef LIS2DW12_ReadRegisters( LIS2DW12 *dev, uint8_t reg, uint8_t *data, uint8_t length );
HAL_StatusTypeDef LIS2DW12_WriteRegister( LIS2DW12 *dev, uint8_t reg, uint8_t *data );


#endif /* INC_LIS2DW12_H_ */
