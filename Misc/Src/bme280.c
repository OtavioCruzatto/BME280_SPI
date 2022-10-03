/*
 * bme280.c
 *
 *  Created on: Jul 12, 2022
 *      Author: Otavio
 */

#include "bme280.h"

static Bme280Reg bme280Reg;
static int32_t fineTemperature = 0;
static const uint32_t timeoutSpi = 100;

CommStatus bme280Init(SPI_HandleTypeDef *hspi, Bme280DeviceData *bme280Device)
{
	CommStatus communicationStatus = NOK;

	bme280Reg.id 					= 0xD0;
	bme280Reg.reset[0]				= 0xE0;
	bme280Reg.reset[1]				= 0xB6;
	bme280Reg.ctrlHum				= 0xF2 & 0x7F;
	bme280Reg.status				= 0xF3;
	bme280Reg.ctrlMeas				= 0xF4 & 0x7F;
	bme280Reg.config				= 0xF5 & 0x7F;
	bme280Reg.press					= 0xF7;
	bme280Reg.temp					= 0xFA;
	bme280Reg.hum					= 0xFD;
	bme280Reg.digT1T3				= 0x88;
	bme280Reg.digP1P9				= 0x8E;
	bme280Reg.digH1					= 0xA1;
	bme280Reg.digH2H6				= 0xE1;

	bme280Device->id				= 0x60;
	bme280Device->spiChipSelectPort	= SPI_CS_GPIO_Port;
	bme280Device->spiChipSelectPin	= SPI_CS_Pin;
	bme280Device->pressMsb			= 0x00;
	bme280Device->pressLsb			= 0x00;
	bme280Device->pressXlsb			= 0x00;
	bme280Device->tempMsb			= 0x00;
	bme280Device->tempLsb			= 0x00;
	bme280Device->tempXlsb			= 0x00;
	bme280Device->humMsb			= 0x00;
	bme280Device->humLsb			= 0x00;
	bme280Device->digT1				= 0x0000;
	bme280Device->digT2				= 0x0000;
	bme280Device->digT3				= 0x0000;
	bme280Device->digP1				= 0x0000;
	bme280Device->digP2				= 0x0000;
	bme280Device->digP3				= 0x0000;
	bme280Device->digP4				= 0x0000;
	bme280Device->digP5				= 0x0000;
	bme280Device->digP6				= 0x0000;
	bme280Device->digP7				= 0x0000;
	bme280Device->digP8				= 0x0000;
	bme280Device->digP9				= 0x0000;
	bme280Device->digH1				= 0x00;
	bme280Device->digH2				= 0x0000;
	bme280Device->digH3				= 0x00;
	bme280Device->digH4				= 0x0000;
	bme280Device->digH5				= 0x0000;
	bme280Device->digH6				= 0x00;
	bme280Device->temperature		= 0x00000000;
	bme280Device->pressure			= 0x00000000;
	bme280Device->humidity     		= 0x00000000;

	if (bme280CheckId(hspi, bme280Device) == bme280Device->id)
	{
		bme280GetCoefficients(hspi, bme280Device);
		bme280Config(hspi, bme280Device, T_10_MS, FILTER_COEF_OFF);
		bme280Control(hspi, bme280Device, OVER_EN_X_1, OVER_EN_X_1, OVER_EN_X_1, NORMAL);
		communicationStatus = OK;
	}

	return communicationStatus;
}

void bme280GetCoefficients(SPI_HandleTypeDef *hspi, Bme280DeviceData *bme280Device)
{
	uint8_t coefBytes[18] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	uint8_t temperatureCoefQtyBytes = 6;
	uint8_t pressureCoefQtyBytes = 18;
	uint8_t humidityCoefQtyBytes = 8 - 1;

	// Get temperature coefficients
	HAL_GPIO_WritePin(bme280Device->spiChipSelectPort, bme280Device->spiChipSelectPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &bme280Reg.digT1T3, sizeof(bme280Reg.digT1T3), timeoutSpi);
	HAL_SPI_Receive(hspi, coefBytes, temperatureCoefQtyBytes, timeoutSpi);
	HAL_GPIO_WritePin(bme280Device->spiChipSelectPort, bme280Device->spiChipSelectPin, GPIO_PIN_SET);
	bme280Device->digT1 = (coefBytes[1] << 8) | coefBytes[0];
	bme280Device->digT2 = (coefBytes[3] << 8) | coefBytes[2];
	bme280Device->digT3 = (coefBytes[5] << 8) | coefBytes[4];

	// Get pressure coefficients
	HAL_GPIO_WritePin(bme280Device->spiChipSelectPort, bme280Device->spiChipSelectPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &bme280Reg.digP1P9, sizeof(bme280Reg.digP1P9), timeoutSpi);
	HAL_SPI_Receive(hspi, coefBytes, pressureCoefQtyBytes, timeoutSpi);
	HAL_GPIO_WritePin(bme280Device->spiChipSelectPort, bme280Device->spiChipSelectPin, GPIO_PIN_SET);
	bme280Device->digP1 = (coefBytes[1] << 8) | coefBytes[0];
	bme280Device->digP2 = (coefBytes[3] << 8) | coefBytes[2];
	bme280Device->digP3 = (coefBytes[5] << 8) | coefBytes[4];
	bme280Device->digP4 = (coefBytes[7] << 8) | coefBytes[6];
	bme280Device->digP5 = (coefBytes[9] << 8) | coefBytes[8];
	bme280Device->digP6 = (coefBytes[11] << 8) | coefBytes[10];
	bme280Device->digP7 = (coefBytes[13] << 8) | coefBytes[12];
	bme280Device->digP8 = (coefBytes[15] << 8) | coefBytes[14];
	bme280Device->digP9 = (coefBytes[17] << 8) | coefBytes[16];

	// Get humidity coefficients
	HAL_GPIO_WritePin(bme280Device->spiChipSelectPort, bme280Device->spiChipSelectPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &bme280Reg.digH1, sizeof(bme280Reg.digH1), timeoutSpi);
	HAL_SPI_Receive(hspi, coefBytes, 1, timeoutSpi);
	HAL_GPIO_WritePin(bme280Device->spiChipSelectPort, bme280Device->spiChipSelectPin, GPIO_PIN_SET);
	bme280Device->digH1 = coefBytes[0];
	HAL_GPIO_WritePin(bme280Device->spiChipSelectPort, bme280Device->spiChipSelectPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &bme280Reg.digH2H6, sizeof(bme280Reg.digH2H6), timeoutSpi);
	HAL_SPI_Receive(hspi, coefBytes, humidityCoefQtyBytes, timeoutSpi);
	HAL_GPIO_WritePin(bme280Device->spiChipSelectPort, bme280Device->spiChipSelectPin, GPIO_PIN_SET);
	bme280Device->digH2 = (coefBytes[1] << 8) | coefBytes[0];
	bme280Device->digH3 = coefBytes[2];
	bme280Device->digH4 = (coefBytes[3] << 4) | (coefBytes[4] & 0x0F);
	bme280Device->digH5 = ((coefBytes[4] & 0xF0) >> 4) | (coefBytes[5] << 4);
	bme280Device->digH6 = coefBytes[6];
}

uint8_t bme280CheckId(SPI_HandleTypeDef *hspi, Bme280DeviceData *bme280Device)
{
	uint8_t id = 0;
	
	HAL_GPIO_WritePin(bme280Device->spiChipSelectPort, bme280Device->spiChipSelectPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &bme280Reg.id, sizeof(bme280Reg.id), timeoutSpi);
	HAL_SPI_Receive(hspi, &id, sizeof(id), timeoutSpi);
	HAL_GPIO_WritePin(bme280Device->spiChipSelectPort, bme280Device->spiChipSelectPin, GPIO_PIN_SET);
	
	return id;
}

void bme280Reset(SPI_HandleTypeDef *hspi, Bme280DeviceData *bme280Device)
{
	HAL_GPIO_WritePin(bme280Device->spiChipSelectPort, bme280Device->spiChipSelectPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, bme280Reg.reset, sizeof(bme280Reg.reset), timeoutSpi);
	HAL_GPIO_WritePin(bme280Device->spiChipSelectPort, bme280Device->spiChipSelectPin, GPIO_PIN_SET);
}

void bme280Control(SPI_HandleTypeDef *hspi, Bme280DeviceData *bme280Device, Oversampling hum, Oversampling press, Oversampling temp, Mode mode)
{
	uint8_t osrs_h = hum & 0x07;
	uint8_t osrs_t = temp & 0x07;
	uint8_t osrs_p = press & 0x07;
	uint8_t operation_mode = mode & 0x03;

	uint8_t ctrl_hum[2] =
	{
			bme280Reg.ctrlHum,
			osrs_h
	};

	uint8_t ctrl_meas[2] =
	{
			bme280Reg.ctrlMeas,
			(osrs_t << 5) | (osrs_p << 2) | operation_mode
	};

	HAL_GPIO_WritePin(bme280Device->spiChipSelectPort, bme280Device->spiChipSelectPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, ctrl_hum, sizeof(ctrl_hum), timeoutSpi);
	HAL_GPIO_WritePin(bme280Device->spiChipSelectPort, bme280Device->spiChipSelectPin, GPIO_PIN_SET);
	
	HAL_GPIO_WritePin(bme280Device->spiChipSelectPort, bme280Device->spiChipSelectPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, ctrl_meas, sizeof(ctrl_meas), timeoutSpi);
	HAL_GPIO_WritePin(bme280Device->spiChipSelectPort, bme280Device->spiChipSelectPin, GPIO_PIN_SET);
}

MeasuringStatus bme280GetStatus(SPI_HandleTypeDef *hspi, Bme280DeviceData *bme280Device)
{
	MeasuringStatus status = AVAILABLE;
	HAL_GPIO_WritePin(bme280Device->spiChipSelectPort, bme280Device->spiChipSelectPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &bme280Reg.status, sizeof(bme280Reg.status), timeoutSpi);
	HAL_SPI_Receive(hspi, &status, sizeof(status), timeoutSpi);
	HAL_GPIO_WritePin(bme280Device->spiChipSelectPort, bme280Device->spiChipSelectPin, GPIO_PIN_SET);
	return status;
}

void bme280Config(SPI_HandleTypeDef *hspi, Bme280DeviceData *bme280Device, TimeStandby timeStandby, FilterCoef filterCoef)
{
	uint8_t t_sb = timeStandby & 0x07;
	uint8_t filter = filterCoef & 0x07;
	uint8_t spi3w_en = 0 & 0x01;

	uint8_t config[2] =
	{
			bme280Reg.config,
			(t_sb << 5) | (filter << 2) | spi3w_en
	};

	bme280Control(hspi, bme280Device, OVER_SKIPPED, OVER_SKIPPED, OVER_SKIPPED, SLEEP);
	
	HAL_GPIO_WritePin(bme280Device->spiChipSelectPort, bme280Device->spiChipSelectPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, config, sizeof(config), timeoutSpi);
	HAL_GPIO_WritePin(bme280Device->spiChipSelectPort, bme280Device->spiChipSelectPin, GPIO_PIN_SET);
}

/**
 * Returns temperature in degC.
 * Output value of "5123" equals 51.23 degC
 */
int32_t bme280ReadTemperature(SPI_HandleTypeDef *hspi, Bme280DeviceData *bme280Device)
{
	int32_t temperature_degC_x_100 = 0;
	int8_t temperatureDataQtyBytes = 3;
	uint8_t dataBytes[3] = {0, 0, 0};

	HAL_GPIO_WritePin(bme280Device->spiChipSelectPort, bme280Device->spiChipSelectPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &bme280Reg.temp, sizeof(bme280Reg.temp), timeoutSpi);
	HAL_SPI_Receive(hspi, dataBytes, temperatureDataQtyBytes, timeoutSpi);
	HAL_GPIO_WritePin(bme280Device->spiChipSelectPort, bme280Device->spiChipSelectPin, GPIO_PIN_SET);
	bme280Device->tempMsb = dataBytes[0];
	bme280Device->tempLsb = dataBytes[1];
	bme280Device->tempXlsb = dataBytes[2];

	int32_t adc_T = (bme280Device->tempMsb << 12) | (bme280Device->tempLsb << 4) | (bme280Device->tempXlsb >> 4);
	int32_t var1 = ((((adc_T >> 3) - ((int32_t) bme280Device->digT1 << 1))) * ((int32_t) bme280Device->digT2)) >> 11;
	int32_t var2 = (((((adc_T >> 4) - ((int32_t) bme280Device->digT1)) * ((adc_T >> 4) - ((int32_t) bme280Device->digT1))) >> 12) * ((int32_t) bme280Device->digT3)) >> 14;
	fineTemperature = var1 + var2;
	bme280Device->temperature = (fineTemperature * 5 + 128) >> 8;
	temperature_degC_x_100 = bme280Device->temperature;

	return temperature_degC_x_100;
}

/**
 * Returns pressure in Pa
 */
uint32_t bme280ReadPressure(SPI_HandleTypeDef *hspi, Bme280DeviceData *bme280Device)
{
	uint32_t pressure_Pa = 0;
	int8_t pressureDataQtyBytes = 3;
	uint8_t dataBytes[3] = {0, 0, 0};

	HAL_GPIO_WritePin(bme280Device->spiChipSelectPort, bme280Device->spiChipSelectPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &bme280Reg.press, sizeof(bme280Reg.press), timeoutSpi);
	HAL_SPI_Receive(hspi, dataBytes, pressureDataQtyBytes, timeoutSpi);
	HAL_GPIO_WritePin(bme280Device->spiChipSelectPort, bme280Device->spiChipSelectPin, GPIO_PIN_SET);
	bme280Device->pressMsb = dataBytes[0];
	bme280Device->pressLsb = dataBytes[1];
	bme280Device->pressXlsb = dataBytes[2];

	int32_t adc_P = (bme280Device->pressMsb << 12) | (bme280Device->pressLsb << 4) | (bme280Device->pressXlsb >> 4);
	bme280ReadTemperature(hspi, bme280Device);
	int64_t var1 = ((int64_t) fineTemperature) - 128000;
	int64_t var2 = var1 * var1 * (int64_t) bme280Device->digP6;
	var2 = var2 + ((var1 * (int64_t) bme280Device->digP5) << 17);
	var2 = var2 + (((int64_t) bme280Device->digP4) << 35);
	var1 = ((var1 * var1 * (int64_t) bme280Device->digP3) >> 8) + ((var1 * (int64_t) bme280Device->digP2) << 12);
	var1 = (((((int64_t) 1) << 47) + var1)) * ((int64_t) bme280Device->digP1) >> 33;
	if (var1 == 0) { return 0; }
	int64_t p_aux = 1048576 - adc_P;
	p_aux = (((p_aux << 31) - var2) * 3125) / var1;
	var1 = (((int64_t) bme280Device->digP9) * (p_aux >> 13) * (p_aux >> 13)) >> 25;
	var2 = (((int64_t) bme280Device->digP8) * p_aux) >> 19;
	p_aux = ((p_aux + var1 + var2) >> 8) + (((int64_t) bme280Device->digP7) << 4);
	pressure_Pa = p_aux / 256;
	bme280Device->pressure = pressure_Pa;

	return bme280Device->pressure;
}

/**
 * Returns humidity in %RH
 */
uint32_t bme280ReadHumidity(SPI_HandleTypeDef *hspi, Bme280DeviceData *bme280Device)
{
	uint32_t humidity_RH_x_10 = 0;
	int8_t humidityDataQtyBytes = 2;
	uint8_t dataBytes[2] = {0, 0};

	HAL_GPIO_WritePin(bme280Device->spiChipSelectPort, bme280Device->spiChipSelectPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &bme280Reg.hum, sizeof(bme280Reg.hum), timeoutSpi);
	HAL_SPI_Receive(hspi, dataBytes, humidityDataQtyBytes, timeoutSpi);
	HAL_GPIO_WritePin(bme280Device->spiChipSelectPort, bme280Device->spiChipSelectPin, GPIO_PIN_SET);
	bme280Device->humMsb = dataBytes[0];
	bme280Device->humLsb = dataBytes[1];

	int32_t adc_H = (bme280Device->humMsb << 8) | bme280Device->humLsb;
	bme280ReadTemperature(hspi, bme280Device);
	int32_t v_x1_u32r = (fineTemperature - ((int32_t) 76800));
	v_x1_u32r = (((((adc_H << 14) - (((int32_t) bme280Device->digH4) << 20) - (((int32_t) bme280Device->digH5) * v_x1_u32r)) +
			((int32_t) 16384)) >> 15) * (((((((v_x1_u32r * ((int32_t) bme280Device->digH6)) >> 10) *
			(((v_x1_u32r * ((int32_t) bme280Device->digH3)) >> 11) + ((int32_t) 32768))) >> 10) + ((int32_t) 2097152)) *
			((int32_t) bme280Device->digH2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t) bme280Device->digH1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	humidity_RH_x_10 = (((uint32_t) (v_x1_u32r >> 12)) / 102);
	bme280Device->humidity = humidity_RH_x_10;

	return bme280Device->humidity;
}



