#include <stdlib.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"


extern volatile WarpI2CDeviceState	deviceINA219State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;

void
initINA219(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	deviceStatePointer->i2cAddress	= i2cAddress;
	deviceStatePointer->signalType	= (
						kWarpTypeMaskCurrent | // in the typedef in warp.h
						kWarpTypeMaskPower
					);

	return;
}

WarpStatus
writeSensorRegisterINA219(uint8_t deviceRegister, uint8_t* payload, uint16_t menuI2cPullupValue)
{
	uint8_t		commandByte[1]; // payload needs to be two bytes
	uint8_t		payloadSize;
	i2c_status_t	status;

	switch (deviceRegister)
	{
		case 0x00: case 0x05:
		{
			/* OK */
			payloadSize = 2;
			break;
		}
		
		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
	{
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	commandByte[0] = deviceRegister;

	status = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							1,
							payload,
							payloadSize,
							gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
configureSensorINA219(uint8_t* payload, uint16_t menuI2cPullupValue)
{
	// TODO
	WarpStatus	i2cWriteStatus;

	i2cWriteStatus = writeSensorRegisterINA219(kWarpSensorConfigurationRegisterINA219_CONFIG /* register address F_SETUP */,
							payload /* payload: Disable FIFO */,
							menuI2cPullupValue);

	return i2cWriteStatus;
}

WarpStatus
readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;


	USED(numberOfBytes);
	switch (deviceRegister)
	{
		case 0x00: case 0x01: case 0x02: case 0x03: 
		case 0x04: case 0x05:
		{
			/* OK */
			break;
		}
		
		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}


	i2c_device_t slave =
	{
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};


	cmdBuf[0] = deviceRegister;

	status = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceINA219State.i2cBuffer,
							numberOfBytes,
							gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
printSensorDataINA219(bool hexModeFlag, int pga_setting)
{
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;

	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219OUT_SHUNT_VOLTAGE, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = (((readSensorRegisterValueMSB & (0xFF) >> (4-pga_setting)) << 8) | (readSensorRegisterValueLSB));


	if (i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_WriteString(0, " ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			SEGGER_RTT_printf(0, "\r\tShunt Voltage 0x%02x 0x%02x => %d (x0.01 to get mV)\n", readSensorRegisterValueMSB, readSensorRegisterValueLSB, readSensorRegisterValueCombined);
		}
		else
		{
			SEGGER_RTT_printf(0, " %d,", readSensorRegisterValueCombined);
		}
	}

	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219OUT_BUS_VOLTAGE, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((((readSensorRegisterValueMSB & (0xFF)) << 8) | (readSensorRegisterValueLSB)) >> 3);

	if (i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_WriteString(0, " ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			SEGGER_RTT_printf(0, "\r\tBus Voltage 0x%02x 0x%02x => %d (x0.004 to get V)\n", readSensorRegisterValueMSB, readSensorRegisterValueLSB, readSensorRegisterValueCombined);
		}
		else
		{
			SEGGER_RTT_printf(0, " %d,", readSensorRegisterValueCombined);
		}
	}

	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219OUT_POWER, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & (0xFF)) << 8) | (readSensorRegisterValueLSB);

	if (i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_WriteString(0, " ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			SEGGER_RTT_printf(0, "\r\tPower 0x%02x 0x%02x => %d (W)\n", readSensorRegisterValueMSB, readSensorRegisterValueLSB, readSensorRegisterValueCombined);
		}
		else
		{
			SEGGER_RTT_printf(0, " %d,", readSensorRegisterValueCombined);
		}
	}

	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219OUT_CURRENT, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & (0xFF)) << 8) | (readSensorRegisterValueLSB);

	if (i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_WriteString(0, " ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			SEGGER_RTT_printf(0, "\r\tCurrent 0x%02x 0x%02x => %d (xLSB to get A (cal1: LSB = 10^-5))\n", readSensorRegisterValueMSB, readSensorRegisterValueLSB, readSensorRegisterValueCombined);
		}
		else
		{
			SEGGER_RTT_printf(0, " %d,", readSensorRegisterValueCombined);
		}
	}
}

void
repeatReadsINA219(int pga_setting, int num_iters)
{
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;

	SEGGER_RTT_printf(0, "\r\t Shunt Voltage, Bus Voltage, Power, Current\n");
	SEGGER_RTT_printf(0, "\r\t (x0.01 mV), (x0.004 V), (mW), (xLSB A)\n");

	for(int i=0; i<num_iters; i++)
	{
		i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219OUT_SHUNT_VOLTAGE, 2 /* numberOfBytes */);
		readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
		readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
		readSensorRegisterValueCombined = (((readSensorRegisterValueMSB & (0xFF) >> (4-pga_setting)) << 8) | (readSensorRegisterValueLSB));


		if (i2cReadStatus != kWarpStatusOK)
		{
			SEGGER_RTT_WriteString(0, " ----,");
		}
		else
		{
			SEGGER_RTT_printf(0, "\t%d, ", readSensorRegisterValueCombined);
		}

		i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219OUT_BUS_VOLTAGE, 2 /* numberOfBytes */);
		readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
		readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
		readSensorRegisterValueCombined = ((((readSensorRegisterValueMSB & (0xFF)) << 8) | (readSensorRegisterValueLSB)) >> 3);

		if (i2cReadStatus != kWarpStatusOK)
		{
			SEGGER_RTT_WriteString(0, " ----,");
		}
		else
		{
			SEGGER_RTT_printf(0, "\t%d, ", readSensorRegisterValueCombined);
		}

		i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219OUT_POWER, 2 /* numberOfBytes */);
		readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
		readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
		readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & (0xFF)) << 8) | (readSensorRegisterValueLSB);

		if (i2cReadStatus != kWarpStatusOK)
		{
			SEGGER_RTT_WriteString(0, " ----,");
		}
		else
		{
			SEGGER_RTT_printf(0, "\t%d, ", readSensorRegisterValueCombined);
		}

		i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219OUT_CURRENT, 2 /* numberOfBytes */);
		readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
		readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
		readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & (0xFF)) << 8) | (readSensorRegisterValueLSB);

		if (i2cReadStatus != kWarpStatusOK)
		{
			SEGGER_RTT_WriteString(0, " ----,");
		}
		else
		{
			SEGGER_RTT_printf(0, "\t%d\n", readSensorRegisterValueCombined);
		}

		OSA_TimeDelay(100);
	}
}

void
MaxTestFuncINA219(bool hexModeFlag)
{
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;

	for(int i=0; i<6; i++)
	{
		i2cReadStatus = readSensorRegisterINA219(0x00 + i, 2 /* numberOfBytes */);
		readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
		readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
		readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB);

		if (i2cReadStatus != kWarpStatusOK)
		{
			SEGGER_RTT_WriteString(0, " ----,");
		}
		else
		{
			if (hexModeFlag)
			{
				SEGGER_RTT_printf(0, "\r\n\t Register Number 0x%02x", i);
				SEGGER_RTT_printf(0, "\r\n\t 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
			}
			else
			{
				SEGGER_RTT_printf(0, " %d,", readSensorRegisterValueCombined);
			}
		}
	}
}

void 
SetCalibrationINA219(uint16_t menuI2cPullupValue)
{
	// case for the resistor, expect around 10mA of current
	// minLSB is 3x10^07, max is 24x10^-7
	// the lowest we can go is acually around 6*10^-6, so just go for 10^-5
	// with R_shunt = 0.1
	// => cal = 40960 = 0xA000
	WarpStatus	i2cWriteStatus;
	uint8_t payload[] = {0xA0, 0x00};

	i2cWriteStatus = writeSensorRegisterINA219(kWarpSensorConfigurationRegisterINA219_CALIBRATION, payload, menuI2cPullupValue);

	if(i2cWriteStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "\nError when writing to I2C device");
	}
}

void 
SetShuntResolutionINA219(uint16_t menuI2cPullupValue, uint8_t resolution)
{
	// from 0b0000 to 0b1111

	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueLSB_new;
	uint16_t	readSensorRegisterValueMSB;
	WarpStatus	i2cReadStatus;

	i2cReadStatus = readSensorRegisterINA219(kWarpSensorConfigurationRegisterINA219_CONFIG, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];

	if (i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_WriteString(0, " ----,");
	}
	else
	{
		SEGGER_RTT_printf(0, "\r\n\t Config Was 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
	}

	readSensorRegisterValueLSB_new = (readSensorRegisterValueLSB & 0x80) | (resolution << 3) | (readSensorRegisterValueLSB & 0x7);

	SEGGER_RTT_printf(0, "\r\n\t Config Becomes 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB_new);

	WarpStatus	i2cWriteStatus;
	uint8_t payload[] = {readSensorRegisterValueMSB, readSensorRegisterValueLSB_new};

	i2cWriteStatus = writeSensorRegisterINA219(kWarpSensorConfigurationRegisterINA219_CONFIG, payload, menuI2cPullupValue);

	if(i2cWriteStatus != kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "\nError when writing to I2C device");
	}
}


