/*
 *
 * LIS2DW12 accelerometer I2C driver for data-ready interrupt
 *
 * Author: Karzan M.
 * Created on: Aug 15, 2025
 * Datasheet download link: https://www.st.com/resource/en/datasheet/lis2dw12.pdf
 *
 */

#include "LIS2DW12.h"
#include <stdint.h>

#define LIS2DW12_OFFSET_TEMP    25.0f                   /* Reference temp for offset */
#define LIS2DW12_OFFSET_LSB     0.0f                    /* 0 LSB offset @ 25 degC */
#define LIS2DW12_SLOPE          0.0625f                 /* inverse of LSB/degC = degC/LSB (p. 8, TSDr) */
#define LIS2DW12_SENSITIVITY    (0.000244f * 9.81f)     /* page 6 sensitivity 0.244 */

/*
 * DEINITIALIZATION
 */
uint8_t LIS2DW12_DeInit(LIS2DW12 *dev, I2C_HandleTypeDef *i2cHandle) {
	/*
	 * DATASHEET: page 37
	 * Soft reset control registers
	 */
	uint8_t reg_data = 0x40;

	HAL_StatusTypeDef status = LIS2DW12_WriteRegister(dev, LIS2DW12_REG_CTRL2, &reg_data);
	return status != HAL_OK;
}

/*
 * INITIALIZATION
 */
uint8_t LIS2DW12_Init(LIS2DW12 *dev, I2C_HandleTypeDef *i2cHandle) {

	/* reset all control registers */
	LIS2DW12_DeInit(dev, i2cHandle);

	/* Setup struct parameters */
	dev->i2cHandle = i2cHandle;

	dev->acc_mps2[0] = 0.0f;
	dev->acc_mps2[1] = 0.0f;
	dev->acc_mps2[2] = 0.0f;

	dev->temp_C = 0.0f;

	/* Keep track of error occurances (count), simple debug alternative */
	uint8_t err_count = 0;
	HAL_StatusTypeDef status;

	/* Ensure correct device ID */
	uint8_t reg_data;

	status = LIS2DW12_ReadRegister(dev, LIS2DW12_REG_WHO_AM_I, &reg_data);
	err_count += (status != HAL_OK);

	if (reg_data != LIS2DW12_ID) {

		return 255; /* Can't talk to the Accelerometer */

	}

	/*
	 * Page 15 and 36 DATASHEET
	 * Setup ODR and power mode ( ODR 200 Hz, Low power mode 4, 14 bit 77uA supply current )
	 */
	reg_data = 0x63;

	status = LIS2DW12_WriteRegister(dev, LIS2DW12_REG_CTRL1, &reg_data);
	err_count += (status != HAL_OK);

	/*
	 * Page 37 DATASHEET
	 * Turn on Block Data Update so data does not get updated while reading
	 */
	reg_data = 0x0;

	LIS2DW12_ReadRegister(dev, LIS2DW12_REG_CTRL2, &reg_data);
	reg_data |= (1 << 3);
	status = LIS2DW12_WriteRegister(dev, LIS2DW12_REG_CTRL2, &reg_data);
	err_count += (status != HAL_OK);

	/*
	 * Page 41 DATASHEET
	 * Setup filters and scale selectoin ( low noise config, LP route, bandwidth = ODR/4, sensitivity= +-2g )
	 */
	reg_data = 0x44;
	status = LIS2DW12_WriteRegister(dev, LIS2DW12_REG_CTRL6, &reg_data);
	err_count += (status != HAL_OK);

	/*
	 * Page 39 DATASHEET
	 * Setup data-ready interrupt for Pin INT1
	 */
	reg_data = 0x01;

	status = LIS2DW12_WriteRegister(dev, LIS2DW12_REG_CTRL4_INT1_PAD_CTRL, &reg_data);
	err_count += (status != HAL_OK);
	/*
	 * Page 55 DATASHEET
	 * Enable Interrupts for the acceleromter
	 */
	reg_data = 0xB0;

	status = LIS2DW12_WriteRegister(dev, LIS2DW12_REG_CTRL7, &reg_data);
	err_count += (status != HAL_OK);

	/* Return the total error count ( 0 if successful ) */
	return err_count;
}

/*
 * DATA AQUISITION
 */

HAL_StatusTypeDef LIS2DW12_ReadTemperature(LIS2DW12 *dev) {

	/* DATASHEET: page 35 */

	/* Raw 16 bit word in twos complement for temperature, 12 bit resolution */
	uint8_t reg_data[2];

	/* Read 2 bytes starting from the LSB output register [LSB, MSB] */
	HAL_StatusTypeDef status = LIS2DW12_ReadRegisters(dev, LIS2DW12_REG_OUT_T_L, reg_data, 2);

	/* combine to 12 bit data (SIGNED 2 complement) */
	int16_t temp_raw = (int16_t) ((int16_t) reg_data[1] << 8 | (reg_data[0] & 0xF0)) >> 4;

	/*
	 * DATASHEET: page 8
	 * convert to degC ( offset @ 25 degC = 0 LSB, slope @ 12 bits resolution = 16 LSB/degC
	 */
	dev->temp_C = LIS2DW12_SLOPE * ((float) temp_raw - LIS2DW12_OFFSET_LSB)+ LIS2DW12_OFFSET_TEMP;

	return status;

}

HAL_StatusTypeDef LIS2DW12_ReadAccelerations(LIS2DW12 *dev) {

	/* DATASHEET: page 43 */

	/* raw 16 bit word in twos complement for accleration, 14 bit resolution */
	uint8_t reg_data[6];

	/* read 2 bytes starting from the LSB output register [LSB, MSB] */
	HAL_StatusTypeDef status = LIS2DW12_ReadRegisters(dev, LIS2DW12_REG_OUT_X_L,
			reg_data, 6);

	/* Combine register readings to combine raw 14 bit accelerometer readings */
	int16_t acc_raw_signed[3];

	/* combine to 14 bit singed data for x axis acceleration */
	acc_raw_signed[0] = (int16_t) ((int16_t) reg_data[1] << 8 | (reg_data[0] & 0xFC)) >> 2;
	acc_raw_signed[1] = (int16_t) ((int16_t) reg_data[3] << 8 | (reg_data[2] & 0xFC)) >> 2;
	acc_raw_signed[2] = (int16_t) ((int16_t) reg_data[5] << 8 | (reg_data[4] & 0xFC)) >> 2;

	/* convert raw data to acceleration ( m/s^2 ) */
	dev->acc_mps2[0] = LIS2DW12_SENSITIVITY * acc_raw_signed[0];
	dev->acc_mps2[1] = LIS2DW12_SENSITIVITY * acc_raw_signed[1];
	dev->acc_mps2[2] = LIS2DW12_SENSITIVITY * acc_raw_signed[2];

	return status;

}

/*
 * LOW-LEVEL FUNCTIONS
 */

HAL_StatusTypeDef LIS2DW12_ReadRegister(LIS2DW12 *dev, uint8_t reg,
		uint8_t *data) {

	return (HAL_I2C_Mem_Read(dev->i2cHandle, LIS2DW12_I2C_ADDR_R, reg,
	I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY));

}

HAL_StatusTypeDef LIS2DW12_ReadRegisters(LIS2DW12 *dev, uint8_t reg,
		uint8_t *data, uint8_t length) {

	return (HAL_I2C_Mem_Read(dev->i2cHandle, LIS2DW12_I2C_ADDR_R, reg,
	I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY));

}

HAL_StatusTypeDef LIS2DW12_WriteRegister(LIS2DW12 *dev, uint8_t reg,
		uint8_t *data) {

	return (HAL_I2C_Mem_Write(dev->i2cHandle, LIS2DW12_I2C_ADDR_W, reg,
	I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY));

}
