/*
 * bme280.c
 *
 *  Created on: Jul 12, 2022
 *      Author: Otavio
 */

#include "bme280.h"

static Bme280Reg sBme280Reg;
static int32_t i32FineTemperature = 0;
static const uint32_t ui32TimeoutSpi = 100;

CommStatus bme280Init(SPI_HandleTypeDef *hspi, Bme280DeviceData *sBme280Device)
{
	CommStatus communicationStatus = NOK;

	sBme280Reg.ui8Id 					= 0xD0;
	sBme280Reg.ui8Reset[0]				= 0xE0;
	sBme280Reg.ui8Reset[1]				= 0xB6;
	sBme280Reg.ui8CtrlHum				= 0xF2 & 0x7F;
	sBme280Reg.ui8Status				= 0xF3;
	sBme280Reg.ui8CtrlMeas				= 0xF4 & 0x7F;
	sBme280Reg.ui8Config				= 0xF5 & 0x7F;
	sBme280Reg.ui8Press					= 0xF7;
	sBme280Reg.ui8Temp					= 0xFA;
	sBme280Reg.ui8Hum					= 0xFD;
	sBme280Reg.ui8DigT1T3				= 0x88;
	sBme280Reg.ui8DigP1P9				= 0x8E;
	sBme280Reg.ui8DigH1					= 0xA1;
	sBme280Reg.ui8DigH2H6				= 0xE1;

	sBme280Device->ui8Id				= 0x60;
	sBme280Device->sSpiChipSelectPort	= SPI_CS_GPIO_Port;
	sBme280Device->ui16SpiChipSelectPin	= SPI_CS_Pin;
	sBme280Device->ui8PressMsb			= 0x00;
	sBme280Device->ui8PressLsb			= 0x00;
	sBme280Device->ui8PressXlsb			= 0x00;
	sBme280Device->ui8TempMsb			= 0x00;
	sBme280Device->ui8TempLsb			= 0x00;
	sBme280Device->ui8TempXlsb			= 0x00;
	sBme280Device->ui8HumMsb			= 0x00;
	sBme280Device->ui8HumLsb			= 0x00;
	sBme280Device->ui16DigT1			= 0x0000;
	sBme280Device->i16DigT2				= 0x0000;
	sBme280Device->i16DigT3				= 0x0000;
	sBme280Device->ui16DigP1			= 0x0000;
	sBme280Device->i16DigP2				= 0x0000;
	sBme280Device->i16DigP3				= 0x0000;
	sBme280Device->i16DigP4				= 0x0000;
	sBme280Device->i16DigP5				= 0x0000;
	sBme280Device->i16DigP6				= 0x0000;
	sBme280Device->i16DigP7				= 0x0000;
	sBme280Device->i16DigP8				= 0x0000;
	sBme280Device->i16DigP9				= 0x0000;
	sBme280Device->ui8DigH1				= 0x00;
	sBme280Device->i16DigH2				= 0x0000;
	sBme280Device->ui8DigH3				= 0x00;
	sBme280Device->i16DigH4				= 0x0000;
	sBme280Device->i16DigH5				= 0x0000;
	sBme280Device->i8DigH6				= 0x00;
	sBme280Device->i32Temperature		= 0x00000000;
	sBme280Device->ui32Pressure			= 0x00000000;
	sBme280Device->ui32Humidity     	= 0x00000000;

	if (bme280CheckId(hspi, sBme280Device) == sBme280Device->ui8Id)
	{
		bme280GetCoefficients(hspi, sBme280Device);
		bme280Config(hspi, sBme280Device, T_10_MS, FILTER_COEF_OFF);
		bme280Control(hspi, sBme280Device, OVER_EN_X_1, OVER_EN_X_1, OVER_EN_X_1, NORMAL);
		communicationStatus = OK;
	}

	return communicationStatus;
}

void bme280GetCoefficients(SPI_HandleTypeDef *hspi, Bme280DeviceData *sBme280Device)
{
	uint8_t coefBytes[18] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	uint8_t temperatureCoefQtyBytes = 6;
	uint8_t pressureCoefQtyBytes = 18;
	uint8_t humidityCoefQtyBytes = 8 - 1;

	// Get temperature coefficients
	HAL_GPIO_WritePin(sBme280Device->sSpiChipSelectPort, sBme280Device->ui16SpiChipSelectPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &sBme280Reg.ui8DigT1T3, sizeof(sBme280Reg.ui8DigT1T3), ui32TimeoutSpi);
	HAL_SPI_Receive(hspi, coefBytes, temperatureCoefQtyBytes, ui32TimeoutSpi);
	HAL_GPIO_WritePin(sBme280Device->sSpiChipSelectPort, sBme280Device->ui16SpiChipSelectPin, GPIO_PIN_SET);
	sBme280Device->ui16DigT1 = (coefBytes[1] << 8) | coefBytes[0];
	sBme280Device->i16DigT2  = (coefBytes[3] << 8) | coefBytes[2];
	sBme280Device->i16DigT3  = (coefBytes[5] << 8) | coefBytes[4];

	// Get pressure coefficients
	HAL_GPIO_WritePin(sBme280Device->sSpiChipSelectPort, sBme280Device->ui16SpiChipSelectPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &sBme280Reg.ui8DigP1P9, sizeof(sBme280Reg.ui8DigP1P9), ui32TimeoutSpi);
	HAL_SPI_Receive(hspi, coefBytes, pressureCoefQtyBytes, ui32TimeoutSpi);
	HAL_GPIO_WritePin(sBme280Device->sSpiChipSelectPort, sBme280Device->ui16SpiChipSelectPin, GPIO_PIN_SET);
	sBme280Device->ui16DigP1 = (coefBytes[1] << 8) | coefBytes[0];
	sBme280Device->i16DigP2  = (coefBytes[3] << 8) | coefBytes[2];
	sBme280Device->i16DigP3  = (coefBytes[5] << 8) | coefBytes[4];
	sBme280Device->i16DigP4  = (coefBytes[7] << 8) | coefBytes[6];
	sBme280Device->i16DigP5  = (coefBytes[9] << 8) | coefBytes[8];
	sBme280Device->i16DigP6  = (coefBytes[11] << 8) | coefBytes[10];
	sBme280Device->i16DigP7  = (coefBytes[13] << 8) | coefBytes[12];
	sBme280Device->i16DigP8  = (coefBytes[15] << 8) | coefBytes[14];
	sBme280Device->i16DigP9  = (coefBytes[17] << 8) | coefBytes[16];

	// Get humidity coefficients
	HAL_GPIO_WritePin(sBme280Device->sSpiChipSelectPort, sBme280Device->ui16SpiChipSelectPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &sBme280Reg.ui8DigH1, sizeof(sBme280Reg.ui8DigH1), ui32TimeoutSpi);
	HAL_SPI_Receive(hspi, coefBytes, 1, ui32TimeoutSpi);
	HAL_GPIO_WritePin(sBme280Device->sSpiChipSelectPort, sBme280Device->ui16SpiChipSelectPin, GPIO_PIN_SET);
	sBme280Device->ui8DigH1 = coefBytes[0];
	HAL_GPIO_WritePin(sBme280Device->sSpiChipSelectPort, sBme280Device->ui16SpiChipSelectPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &sBme280Reg.ui8DigH2H6, sizeof(sBme280Reg.ui8DigH2H6), ui32TimeoutSpi);
	HAL_SPI_Receive(hspi, coefBytes, humidityCoefQtyBytes, ui32TimeoutSpi);
	HAL_GPIO_WritePin(sBme280Device->sSpiChipSelectPort, sBme280Device->ui16SpiChipSelectPin, GPIO_PIN_SET);
	sBme280Device->i16DigH2 = (coefBytes[1] << 8) | coefBytes[0];
	sBme280Device->ui8DigH3 = coefBytes[2];
	sBme280Device->i16DigH4 = (coefBytes[3] << 4) | (coefBytes[4] & 0x0F);
	sBme280Device->i16DigH5 = ((coefBytes[4] & 0xF0) >> 4) | (coefBytes[5] << 4);
	sBme280Device->i8DigH6  = coefBytes[6];
}

uint8_t bme280CheckId(SPI_HandleTypeDef *hspi, Bme280DeviceData *sBme280Device)
{
	uint8_t id = 0;
	
	HAL_GPIO_WritePin(sBme280Device->sSpiChipSelectPort, sBme280Device->ui16SpiChipSelectPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &sBme280Reg.ui8Id, sizeof(sBme280Reg.ui8Id), ui32TimeoutSpi);
	HAL_SPI_Receive(hspi, &id, sizeof(id), ui32TimeoutSpi);
	HAL_GPIO_WritePin(sBme280Device->sSpiChipSelectPort, sBme280Device->ui16SpiChipSelectPin, GPIO_PIN_SET);
	
	return id;
}

void bme280Reset(SPI_HandleTypeDef *hspi, Bme280DeviceData *sBme280Device)
{
	HAL_GPIO_WritePin(sBme280Device->sSpiChipSelectPort, sBme280Device->ui16SpiChipSelectPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, sBme280Reg.ui8Reset, sizeof(sBme280Reg.ui8Reset), ui32TimeoutSpi);
	HAL_GPIO_WritePin(sBme280Device->sSpiChipSelectPort, sBme280Device->ui16SpiChipSelectPin, GPIO_PIN_SET);
}

void bme280Control(SPI_HandleTypeDef *hspi, Bme280DeviceData *sBme280Device, Oversampling hum, Oversampling press, Oversampling temp, Mode mode)
{
	uint8_t osrs_h = hum & 0x07;
	uint8_t osrs_t = temp & 0x07;
	uint8_t osrs_p = press & 0x07;
	uint8_t operation_mode = mode & 0x03;

	uint8_t ctrl_hum[2] =
	{
			sBme280Reg.ui8CtrlHum,
			osrs_h
	};

	uint8_t ctrl_meas[2] =
	{
			sBme280Reg.ui8CtrlMeas,
			(osrs_t << 5) | (osrs_p << 2) | operation_mode
	};

	HAL_GPIO_WritePin(sBme280Device->sSpiChipSelectPort, sBme280Device->ui16SpiChipSelectPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, ctrl_hum, sizeof(ctrl_hum), ui32TimeoutSpi);
	HAL_GPIO_WritePin(sBme280Device->sSpiChipSelectPort, sBme280Device->ui16SpiChipSelectPin, GPIO_PIN_SET);
	
	HAL_GPIO_WritePin(sBme280Device->sSpiChipSelectPort, sBme280Device->ui16SpiChipSelectPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, ctrl_meas, sizeof(ctrl_meas), ui32TimeoutSpi);
	HAL_GPIO_WritePin(sBme280Device->sSpiChipSelectPort, sBme280Device->ui16SpiChipSelectPin, GPIO_PIN_SET);
}

MeasuringStatus bme280GetStatus(SPI_HandleTypeDef *hspi, Bme280DeviceData *sBme280Device)
{
	MeasuringStatus status = AVAILABLE;
	HAL_GPIO_WritePin(sBme280Device->sSpiChipSelectPort, sBme280Device->ui16SpiChipSelectPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &sBme280Reg.ui8Status, sizeof(sBme280Reg.ui8Status), ui32TimeoutSpi);
	HAL_SPI_Receive(hspi, &status, sizeof(status), ui32TimeoutSpi);
	HAL_GPIO_WritePin(sBme280Device->sSpiChipSelectPort, sBme280Device->ui16SpiChipSelectPin, GPIO_PIN_SET);
	return status;
}

void bme280Config(SPI_HandleTypeDef *hspi, Bme280DeviceData *sBme280Device, TimeStandby timeStandby, FilterCoef filterCoef)
{
	uint8_t t_sb = timeStandby & 0x07;
	uint8_t filter = filterCoef & 0x07;
	uint8_t spi3w_en = 0 & 0x01;

	uint8_t config[2] =
	{
			sBme280Reg.ui8Config,
			(t_sb << 5) | (filter << 2) | spi3w_en
	};

	bme280Control(hspi, sBme280Device, OVER_SKIPPED, OVER_SKIPPED, OVER_SKIPPED, SLEEP);
	
	HAL_GPIO_WritePin(sBme280Device->sSpiChipSelectPort, sBme280Device->ui16SpiChipSelectPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, config, sizeof(config), ui32TimeoutSpi);
	HAL_GPIO_WritePin(sBme280Device->sSpiChipSelectPort, sBme280Device->ui16SpiChipSelectPin, GPIO_PIN_SET);
}

/**
 * Returns temperature in degC.
 * Output value of "5123" equals 51.23 degC
 */
int32_t bme280ReadTemperature(SPI_HandleTypeDef *hspi, Bme280DeviceData *sBme280Device)
{
	int32_t temperature_degC_x_100 = 0;
	int8_t temperatureDataQtyBytes = 3;
	uint8_t dataBytes[3] = {0, 0, 0};

	HAL_GPIO_WritePin(sBme280Device->sSpiChipSelectPort, sBme280Device->ui16SpiChipSelectPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &sBme280Reg.ui8Temp, sizeof(sBme280Reg.ui8Temp), ui32TimeoutSpi);
	HAL_SPI_Receive(hspi, dataBytes, temperatureDataQtyBytes, ui32TimeoutSpi);
	HAL_GPIO_WritePin(sBme280Device->sSpiChipSelectPort, sBme280Device->ui16SpiChipSelectPin, GPIO_PIN_SET);
	sBme280Device->ui8TempMsb = dataBytes[0];
	sBme280Device->ui8TempLsb = dataBytes[1];
	sBme280Device->ui8TempXlsb = dataBytes[2];

	int32_t adc_T = (sBme280Device->ui8TempMsb << 12) | (sBme280Device->ui8TempLsb << 4) | (sBme280Device->ui8TempXlsb >> 4);
	int32_t var1 = ((((adc_T >> 3) - ((int32_t) sBme280Device->ui16DigT1 << 1))) * ((int32_t) sBme280Device->i16DigT2)) >> 11;
	int32_t var2 = (((((adc_T >> 4) - ((int32_t) sBme280Device->ui16DigT1)) * ((adc_T >> 4) - ((int32_t) sBme280Device->ui16DigT1))) >> 12) * ((int32_t) sBme280Device->i16DigT3)) >> 14;
	i32FineTemperature = var1 + var2;
	sBme280Device->i32Temperature = (i32FineTemperature * 5 + 128) >> 8;
	temperature_degC_x_100 = sBme280Device->i32Temperature;

	return temperature_degC_x_100;
}

/**
 * Returns pressure in Pa
 */
uint32_t bme280ReadPressure(SPI_HandleTypeDef *hspi, Bme280DeviceData *sBme280Device)
{
	uint32_t pressure_Pa = 0;
	int8_t pressureDataQtyBytes = 3;
	uint8_t dataBytes[3] = {0, 0, 0};

	HAL_GPIO_WritePin(sBme280Device->sSpiChipSelectPort, sBme280Device->ui16SpiChipSelectPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &sBme280Reg.ui8Press, sizeof(sBme280Reg.ui8Press), ui32TimeoutSpi);
	HAL_SPI_Receive(hspi, dataBytes, pressureDataQtyBytes, ui32TimeoutSpi);
	HAL_GPIO_WritePin(sBme280Device->sSpiChipSelectPort, sBme280Device->ui16SpiChipSelectPin, GPIO_PIN_SET);
	sBme280Device->ui8PressMsb = dataBytes[0];
	sBme280Device->ui8PressLsb = dataBytes[1];
	sBme280Device->ui8PressXlsb = dataBytes[2];

	int32_t adc_P = (sBme280Device->ui8PressMsb << 12) | (sBme280Device->ui8PressLsb << 4) | (sBme280Device->ui8PressXlsb >> 4);
	bme280ReadTemperature(hspi, sBme280Device);
	int64_t var1 = ((int64_t) i32FineTemperature) - 128000;
	int64_t var2 = var1 * var1 * (int64_t) sBme280Device->i16DigP6;
	var2 = var2 + ((var1 * (int64_t) sBme280Device->i16DigP5) << 17);
	var2 = var2 + (((int64_t) sBme280Device->i16DigP4) << 35);
	var1 = ((var1 * var1 * (int64_t) sBme280Device->i16DigP3) >> 8) + ((var1 * (int64_t) sBme280Device->i16DigP2) << 12);
	var1 = (((((int64_t) 1) << 47) + var1)) * ((int64_t) sBme280Device->ui16DigP1) >> 33;
	if (var1 == 0) { return 0; }
	int64_t p_aux = 1048576 - adc_P;
	p_aux = (((p_aux << 31) - var2) * 3125) / var1;
	var1 = (((int64_t) sBme280Device->i16DigP9) * (p_aux >> 13) * (p_aux >> 13)) >> 25;
	var2 = (((int64_t) sBme280Device->i16DigP8) * p_aux) >> 19;
	p_aux = ((p_aux + var1 + var2) >> 8) + (((int64_t) sBme280Device->i16DigP7) << 4);
	pressure_Pa = p_aux / 256;
	sBme280Device->ui32Pressure = pressure_Pa;

	return sBme280Device->ui32Pressure;
}

/**
 * Returns humidity in %RH
 */
uint32_t bme280ReadHumidity(SPI_HandleTypeDef *hspi, Bme280DeviceData *sBme280Device)
{
	uint32_t humidity_RH_x_10 = 0;
	int8_t humidityDataQtyBytes = 2;
	uint8_t dataBytes[2] = {0, 0};

	HAL_GPIO_WritePin(sBme280Device->sSpiChipSelectPort, sBme280Device->ui16SpiChipSelectPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, &sBme280Reg.ui8Hum, sizeof(sBme280Reg.ui8Hum), ui32TimeoutSpi);
	HAL_SPI_Receive(hspi, dataBytes, humidityDataQtyBytes, ui32TimeoutSpi);
	HAL_GPIO_WritePin(sBme280Device->sSpiChipSelectPort, sBme280Device->ui16SpiChipSelectPin, GPIO_PIN_SET);
	sBme280Device->ui8HumMsb = dataBytes[0];
	sBme280Device->ui8HumLsb = dataBytes[1];

	int32_t adc_H = (sBme280Device->ui8HumMsb << 8) | sBme280Device->ui8HumLsb;
	bme280ReadTemperature(hspi, sBme280Device);
	int32_t v_x1_u32r = (i32FineTemperature - ((int32_t) 76800));
	v_x1_u32r = (((((adc_H << 14) - (((int32_t) sBme280Device->i16DigH4) << 20) - (((int32_t) sBme280Device->i16DigH5) * v_x1_u32r)) +
			((int32_t) 16384)) >> 15) * (((((((v_x1_u32r * ((int32_t) sBme280Device->i8DigH6)) >> 10) *
			(((v_x1_u32r * ((int32_t) sBme280Device->ui8DigH3)) >> 11) + ((int32_t) 32768))) >> 10) + ((int32_t) 2097152)) *
			((int32_t) sBme280Device->i16DigH2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t) sBme280Device->ui8DigH1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	humidity_RH_x_10 = (((uint32_t) (v_x1_u32r >> 12)) / 102);
	sBme280Device->ui32Humidity = humidity_RH_x_10;

	return sBme280Device->ui32Humidity;
}



