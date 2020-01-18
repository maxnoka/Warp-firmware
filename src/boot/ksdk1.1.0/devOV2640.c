#include <stdint.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devOV2640.h"

// I2C Stuff
extern volatile WarpI2CDeviceState	deviceOV2640State_I2C;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;

extern volatile WarpSPIDeviceState	deviceOV2640State_SPI;
extern volatile uint32_t		gWarpSPIBaudRateKbps;
extern volatile uint32_t		gWarpSpiTimeoutMicroseconds;

static volatile ov2640_bank_t reg_bank = BANK_MAX;

/*
 *	Override Warp firmware's use of these pins and define new aliases.
 */
enum
{
	kOV2640PinMISO		= GPIO_MAKE_PIN(HW_GPIOA, 6),
	kOV2640PinMOSI		= GPIO_MAKE_PIN(HW_GPIOA, 8),
	kOV2640PinSCK		= GPIO_MAKE_PIN(HW_GPIOA, 9),
	kOV2640PinCSn		= GPIO_MAKE_PIN(HW_GPIOB, 13),
};

void
initOV2640(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer_I2C, WarpSPIDeviceState volatile *  deviceStatePointer_SPI)
{
	deviceStatePointer_I2C->i2cAddress	= i2cAddress;
	deviceStatePointer_I2C->signalType	= (
						kWarpTypeMaskCurrent | // in the typedef in warp.h
						kWarpTypeMaskPower
					);

	deviceStatePointer_SPI->signalType	= (	kWarpTypeMaskAccelerationX |
						kWarpTypeMaskAccelerationY |
						kWarpTypeMaskAccelerationZ |
						kWarpTypeMaskTemperature
					);

	return;
}

int
devOV2640init()
{
	SEGGER_RTT_printf(0, "devOV2640init \n");
	
	devOV2640SetBank(BANK_SENSOR);
	readSensorRegisterOV2640_I2C(OV2640_CHIPID_HIGH, 1);

	uint16_t	readSensorRegisterValueMSB;
	readSensorRegisterValueMSB = deviceOV2640State_I2C.i2cBuffer[0];

	readSensorRegisterOV2640_I2C(OV2640_CHIPID_HIGH, 1);
	uint16_t	readSensorRegisterValueLSB;
	readSensorRegisterValueLSB = deviceOV2640State_I2C.i2cBuffer[0];

    if ((readSensorRegisterValueMSB != 0x26 ) && (( readSensorRegisterValueLSB != 0x41 ) || ( readSensorRegisterValueLSB != 0x42 ))){
      SEGGER_RTT_printf(0, "Something Wrong With I2C \n");
    }
    else
    {
    	SEGGER_RTT_printf(0, "I2C Good \n");
    }

	writeSensorRegisterOV2640_I2C(BANK_SENSOR, 0x12, 0x80); // system reset
	OSA_TimeDelay(100);

	writeOV2640_SPI(0x07, 0x80);
	OSA_TimeDelay(100);
	writeOV2640_SPI(0x07, 0x00);

	writeSensorRegisterMultipleOV2640_I2C(OV2640_QVGA);
	writeSensorRegisterMultipleOV2640_I2C(OV2640_RGB565);

	/* BEHOLD!!!!
	The graveyard off different initializations and configurations I tried:
	*/

	// trying the micropython thing:
	// writeSensorRegisterOV2640_I2C(BANK_DSP, R_BYPASS, R_BYPASS_DSP_BYPAS);
	// writeSensorRegisterMultipleOV2640_I2C(OV2640_TO_CIF);
	// writeSensorRegisterMultipleOV2640_I2C(OV2640_TO_JPEG);

	// writeSensorRegisterMultipleOV2640_I2C(OV2640_JPEG_INIT);
	// writeSensorRegisterMultipleOV2640_I2C(OV2640_YUV422);
	// writeSensorRegisterMultipleOV2640_I2C(OV2640_JPEG);

	// writeSensorRegisterOV2640_I2C(BANK_SENSOR, 0x15, 0x00);

	// writeSensorRegisterOV2640_I2C(BANK_DSP, R_BYPASS, R_BYPASS_DSP_BYPAS);
	// writeSensorRegisterMultipleOV2640_I2C(OV2640_TO_CIF);
	// writeSensorRegisterMultipleOV2640_I2C(OV2640_TO_JPEG);


	// writeSensorRegisterMultipleOV2640_I2C(OV2640_176x144_JPEG);

	// devOV2640setFrameSizeQVGA();
	// OSA_TimeDelay(10);
	// writeSensorRegisterMultipleOV2640_I2C(OV2640_TO_JPEG);

	// writeSensorRegisterMultipleOV2640_I2C(OV2640_RGB565);

	// arducam
	// devOV2640SetBank(BANK_SENSOR);
	// writeSensorRegisterMultipleOV2640_I2C(OV2640_TO_CIF);

	// writeSensorRegisterMultipleOV2640_I2C(OV2640_JPEG_INIT);
	// writeSensorRegisterMultipleOV2640_I2C(OV2640_YUV422);
	// writeSensorRegisterMultipleOV2640_I2C(OV2640_JPEG);

	// writeSensorRegisterOV2640_I2C(BANK_SENSOR, 0x15, 0x00);


	OSA_TimeDelay(10);

	return 0;
}

/* I2C */

WarpStatus
writeSensorRegisterOV2640_I2C_master(uint8_t deviceRegister, uint8_t* payload)
{

	uint8_t		commandByte[1]; // payload needs to be two bytes
	uint8_t		payloadSize;
	i2c_status_t	status;

	switch (deviceRegister)
	{
		// (0xFF = 1)
		case 0x17: case 0x18: case 0x19: case 0x1A:
		{
			/* OK */
			payloadSize = 2;
			break;
		}
		default:
		{
			payloadSize = 1;
			//return kWarpStatusBadDeviceCommand;
			// doesn't make sense with two register sets (unless I keep track which I won't)...
		}
	}

	i2c_device_t slave =
	{
		.address = deviceOV2640State_I2C.i2cAddress,
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
readSensorRegisterOV2640_I2C(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;


	USED(numberOfBytes);

	i2c_device_t slave =
	{
		.address = deviceOV2640State_I2C.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};


	cmdBuf[0] = deviceRegister;

	status = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceOV2640State_I2C.i2cBuffer,
							numberOfBytes,
							gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		SEGGER_RTT_printf(0, "Failure \n");
		return kWarpStatusDeviceCommunicationFailed;
	}
	SEGGER_RTT_printf(0, "Success \n");
	return kWarpStatusOK;
}

WarpStatus
writeSensorRegisterOV2640_I2C(ov2640_bank_t bank, uint8_t deviceRegister, uint8_t value)
{
	uint8_t payload[] = {value};
	devOV2640SetBank(bank);
	return writeSensorRegisterOV2640_I2C_master(deviceRegister, payload);
}

WarpStatus
writeSensorRegisterMultipleOV2640_I2C(const struct sensor_reg reglist[])
{
	uint8_t reg_addr = 0;
	uint8_t reg_val = 0;
	const struct sensor_reg *next = reglist;

	while((reg_val != 0xff) || (reg_addr != 0xff)) // end signal
	{
		reg_addr = next->reg;
		reg_val = next->val;
		writeSensorRegisterOV2640_I2C_master(reg_addr, &reg_val);
		next++;
	}

	return kWarpStatusOK;
}

void 
readAndPrintOV2640_I2C(uint8_t deviceRegister, int numberOfBytes)
{
	SEGGER_RTT_printf(0, "Reading Register: 0x%02x \n", deviceRegister);
	readSensorRegisterOV2640_I2C(deviceRegister, numberOfBytes);

	uint16_t	readSensorRegisterValue;
	readSensorRegisterValue = deviceOV2640State_I2C.i2cBuffer[0];
	SEGGER_RTT_printf(0, "\r\n\t Value  is 0x%02x \n", readSensorRegisterValue);
}

void 
devOV2640SetBank(ov2640_bank_t bank)
{
    if (bank != reg_bank) {
        reg_bank = bank;
        uint8_t payload[] = {bank};
        writeSensorRegisterOV2640_I2C_master(BANK_SEL, payload);
    }
}

/* SPI */

WarpStatus
writeOV2640_SPI_master(uint8_t command, uint8_t writeValue, int numberOfBytes)
{
	deviceOV2640State_SPI.spiSourceBuffer[0] = command;
	deviceOV2640State_SPI.spiSourceBuffer[1] = writeValue;

	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */

	GPIO_DRV_SetPinOutput(kOV2640PinCSn);
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(kOV2640PinCSn);

	deviceOV2640State_SPI.ksdk_spi_status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)deviceOV2640State_SPI.spiSourceBuffer,
					(uint8_t * restrict)deviceOV2640State_SPI.spiSinkBuffer,
					numberOfBytes		/* transfer size */,
					gWarpSpiTimeoutMicroseconds		/* timeout in microseconds (unlike I2C which is ms) */);

	/*
	 *	Drive /CS high
	 */
	GPIO_DRV_SetPinOutput(kOV2640PinCSn);

	return kWarpStatusOK;
}

WarpStatus
devOV2640_start_FIFO_burst_read()
{
	deviceOV2640State_SPI.spiSourceBuffer[0] = kSSD1331CommandBURSTFIFOREAD;

	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */

	GPIO_DRV_SetPinOutput(kOV2640PinCSn);
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(kOV2640PinCSn);

	deviceOV2640State_SPI.ksdk_spi_status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)deviceOV2640State_SPI.spiSourceBuffer,
					(uint8_t * restrict)deviceOV2640State_SPI.spiSinkBuffer,
					1		/* transfer size */,
					gWarpSpiTimeoutMicroseconds		/* timeout in microseconds (unlike I2C which is ms) */);

	return kWarpStatusOK;
}

WarpStatus
devOV2640_end_FIFO_burst_read()
{
	/*
	 *	Drive /CS high
	 */
	GPIO_DRV_SetPinOutput(kOV2640PinCSn);

	return kWarpStatusOK;
}

WarpStatus
devOV2640_FIFO_burst_read(uint8_t* buffer, uint8_t buffer_size)
{
	deviceOV2640State_SPI.ksdk_spi_status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					NULL, /* Send Buffer */
					(uint8_t * restrict) buffer,
					buffer_size		/* transfer size */,
					gWarpSpiTimeoutMicroseconds		/* timeout in microseconds (unlike I2C which is ms) */);

	return kWarpStatusOK;
}

WarpStatus 	
writeOV2640_SPI(uint8_t deviceRegister, uint8_t writeValue)
{
	uint8_t command = deviceRegister | 0x80;
	enableSPIpins();
	writeOV2640_SPI_master(command, writeValue, 2);
	disableSPIpins();
	return kWarpStatusOK;
}

WarpStatus
readOV2640_SPI(uint8_t deviceRegister, int numberOfBytes)
{	
	uint8_t command = deviceRegister & 0x7F;
	// SEGGER_RTT_printf(0, "Command: 0x%02x \n", command);
	enableSPIpins();
	writeOV2640_SPI_master(command, 0x00, numberOfBytes);
	disableSPIpins();
	return kWarpStatusOK;
}

void 
readAndPrintOV2640_SPI(uint8_t deviceRegister, int numberOfBytes)
{
	SEGGER_RTT_printf(0, "Reading Register: 0x%02x \n", deviceRegister);
	readOV2640_SPI(deviceRegister, numberOfBytes);

	SEGGER_RTT_printf(0, "\r\t Value  is 0x%02x \n\n", deviceOV2640State_SPI.spiSinkBuffer[1]);
}

void 
TestFunctionOV2640_I2C()
{
	SEGGER_RTT_printf(0, "Starting I2C Test Function \n");
	
	readAndPrintOV2640_I2C(0xff, 1);
	readAndPrintOV2640_I2C(0x12, 1);
	readAndPrintOV2640_I2C(0x04, 1);

	SEGGER_RTT_printf(0, "Writing \n");
	writeSensorRegisterMultipleOV2640_I2C(OV2640_TEST);

	readAndPrintOV2640_I2C(0xff, 1);
	readAndPrintOV2640_I2C(0x12, 1);
	readAndPrintOV2640_I2C(0x04, 1);
}

void
TestFunctionOV2640_SPI_burst()
{
	uint8_t buffer[8];

	enableSPIpins();
	devOV2640_start_FIFO_burst_read();
	devOV2640_FIFO_burst_read(buffer, 8);
	devOV2640_end_FIFO_burst_read();
	disableSPIpins();
	uint8_t i = 0;
	SEGGER_RTT_printf(0, "%02x %02x %02x %02x, %02x %02x %02x %02x \n", buffer[i], buffer[i+1], buffer[i+2], buffer[i+3], buffer[i+4], buffer[i+5], buffer[i+6], buffer[i+7]);
	OSA_TimeDelay(200);

	enableSPIpins();
	devOV2640_start_FIFO_burst_read();
	devOV2640_FIFO_burst_read(buffer, 8);
	devOV2640_end_FIFO_burst_read();
	disableSPIpins();
	i = 0;
	SEGGER_RTT_printf(0, "%02x %02x %02x %02x, %02x %02x %02x %02x \n", buffer[i], buffer[i+1], buffer[i+2], buffer[i+3], buffer[i+4], buffer[i+5], buffer[i+6], buffer[i+7]);
	OSA_TimeDelay(200);

	enableSPIpins();
	devOV2640_start_FIFO_burst_read();
	devOV2640_FIFO_burst_read(buffer, 8);
	devOV2640_end_FIFO_burst_read();
	disableSPIpins();
	i = 0;
	SEGGER_RTT_printf(0, "%02x %02x %02x %02x, %02x %02x %02x %02x \n", buffer[i], buffer[i+1], buffer[i+2], buffer[i+3], buffer[i+4], buffer[i+5], buffer[i+6], buffer[i+7]);
	OSA_TimeDelay(200);

	enableSPIpins();
	devOV2640_start_FIFO_burst_read();
	devOV2640_FIFO_burst_read(buffer, 8);
	devOV2640_end_FIFO_burst_read();
	disableSPIpins();
	i = 0;
	SEGGER_RTT_printf(0, "%02x %02x %02x %02x, %02x %02x %02x %02x \n", buffer[i], buffer[i+1], buffer[i+2], buffer[i+3], buffer[i+4], buffer[i+5], buffer[i+6], buffer[i+7]);
	OSA_TimeDelay(200);
}

void 
OV2640_whole_bmp_single_FIFO()
{
	SEGGER_RTT_printf(0, "Here We Go This Will Take A While \n");

	uint8_t buffer[8];
	enableSPIpins();
	for(int j = 0; j<20000; j++)
	{
		for(int i = 0; i < 8; i++)
		{
			writeOV2640_SPI_master(kSSD1331CommandSINGLEFIFOREAD, 0x00, 2);
			buffer[i] = deviceOV2640State_SPI.spiSinkBuffer[1];
			OSA_TimeDelay(20);
		}

		SEGGER_RTT_printf(0, "%02x%02x%02x%02x%02x%02x%02x%02x", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7]);
		OSA_TimeDelay(10);
	}
	disableSPIpins();
	SEGGER_RTT_printf(0, "\n");
}

void 
TestFunctionOV2640_SPI_single()
{
	SEGGER_RTT_printf(0, "Starting I2C Test Function \n");

	uint8_t buffer[8];

	for(int j = 0; j < 5; j++)
	{
		for(int i = 0; i < 8; i++)
		{
			readOV2640_SPI(kSSD1331CommandSINGLEFIFOREAD, 2);
			buffer[i] = deviceOV2640State_SPI.spiSinkBuffer[1];
			OSA_TimeDelay(10);
		}

		SEGGER_RTT_printf(0, "%02x %02x %02x %02x, %02x %02x %02x %02x \n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7]);
		OSA_TimeDelay(10);
	}
}

void 
TestFunctionOV2640_SPI()
{
	SEGGER_RTT_printf(0, "Starting SPI Test Function \n");

	devOV2640FlushFifo();
	devOV2640ClearCaptureDoneFlag();
	devOV2640StartCapture();
	readAndPrintOV2640_SPI(kSSD1331CommandTRIGGER, 2);
	OSA_TimeDelay(200);
	readAndPrintOV2640_SPI(kSSD1331CommandTRIGGER, 2);
	OSA_TimeDelay(200);	
	readAndPrintOV2640_SPI(kSSD1331CommandTRIGGER, 2);
	readAndPrintOV2640_SPI(kSSD1331CommandTRIGGER, 2);
	readAndPrintOV2640_SPI(kSSD1331CommandTRIGGER, 2);
	readAndPrintOV2640_SPI(kSSD1331CommandSINGLEFIFOREAD, 2);
	readAndPrintOV2640_SPI(kSSD1331CommandSINGLEFIFOREAD, 2);
	readAndPrintOV2640_SPI(kSSD1331CommandSINGLEFIFOREAD, 2);
	readAndPrintOV2640_SPI(kSSD1331CommandSINGLEFIFOREAD, 2);
	readAndPrintOV2640_SPI(kSSD1331CommandSINGLEFIFOREAD, 2);
}

void 
OV2640_whole_bmp_burst_FIFO()
{
	#define NUMBER_OF_BUFFERS_AT_ONCE 3
	#define BUFFER_SIZE 40
	#define NUMBER_PRINTS 10 

	uint8_t buffer[BUFFER_SIZE];

	SEGGER_RTT_printf(0, "\r\t Here We Go \n\n");

	enableSPIpins();
	for(int j=0; j<1280; j++)
	{
		devOV2640_start_FIFO_burst_read();
		for(int k=0; k<NUMBER_OF_BUFFERS_AT_ONCE; k++)
		{
			devOV2640_FIFO_burst_read(buffer, BUFFER_SIZE);
			for(int i=0; i<BUFFER_SIZE; i = (i + NUMBER_PRINTS))
			{
				SEGGER_RTT_printf(0, "%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x", buffer[i], buffer[i+1], buffer[i+2], buffer[i+3], buffer[i+4], buffer[i+5], buffer[i+6], buffer[i+7], buffer[i+8], buffer[i+9]);
				OSA_TimeDelay(10);
			}
		}
		devOV2640_end_FIFO_burst_read();
		OSA_TimeDelay(20);
	}
	disableSPIpins();
	SEGGER_RTT_printf(0, "\n");
}

void
devOV2640setFrameSizeCIF()
{
	writeSensorRegisterOV2640_I2C(BANK_DSP, R_BYPASS, R_BYPASS_DSP_BYPAS);
	writeSensorRegisterMultipleOV2640_I2C(OV2640_TO_CIF);
}

void
devOV2640setFrameSizeQVGA()
{
	devOV2640setFrameSizeCIF();

	uint16_t w = 160; //320;
	uint16_t h = 120; //240;

	writeSensorRegisterOV2640_I2C(BANK_DSP, ZMOW, (w>>2)&0xFF); // OUTW[7:0] (real/4)
	writeSensorRegisterOV2640_I2C(BANK_DSP, ZMOH, (h>>2)&0xFF); // OUTH[7:0] (real/4)
	writeSensorRegisterOV2640_I2C(BANK_DSP, ZMHH, ((h>>8)&0x04)|((w>>10)&0x03)); // OUTH[8]/OUTW[9:8]
	writeSensorRegisterOV2640_I2C(BANK_DSP, RESET, 0x00);
	writeSensorRegisterOV2640_I2C(BANK_DSP, R_BYPASS, R_BYPASS_DSP_EN);
	OSA_TimeDelay(10);
}

void 
devOV2640setPixFormatGrayScale()
{
	writeSensorRegisterMultipleOV2640_I2C(OV2640_SETTINGS_YUV422);
	OSA_TimeDelay(10);
}

WarpStatus
devOV2640FlushFifo()
{
	return writeOV2640_SPI(kSSD1331CommandARDUCHIP_FIFO, kOV2640FIFO_CLEAR_MASK);
}

WarpStatus
devOV2640ClearCaptureDoneFlag()  // I feel like this is wrong but it's what's in the arduino driver...
{
	return writeOV2640_SPI(kSSD1331CommandARDUCHIP_FIFO, kOV2640FIFO_CLEAR_MASK);
}

WarpStatus
devOV2640StartCapture()
{

	return writeOV2640_SPI(kSSD1331CommandARDUCHIP_FIFO, kOV2640FIFO_START_MASK);
}

void
devOV2640HoldUntilCaptureFinished()
{
	uint8_t done_bit;
	do
	{
		readOV2640_SPI(kSSD1331CommandTRIGGER, 2);
		SEGGER_RTT_printf(0, "\r\t kSSD1331CommandTRIGGER  is 0x%02x \n\n", deviceOV2640State_SPI.spiSinkBuffer[1]);
		done_bit = deviceOV2640State_SPI.spiSinkBuffer[1] & kOV2640CAP_DONE_MASK;
		SEGGER_RTT_printf(0, "\r\t done_bit  is 0x%02x \n\n", done_bit);
	} while(!done_bit);
}

void
devOV2640MakePhoto()
{
	uint32_t counter = 0;
	SEGGER_RTT_printf(0, "Flush FIFO \n");
	devOV2640FlushFifo();
	SEGGER_RTT_printf(0, "Clear Capture Done Flag \n");
	devOV2640ClearCaptureDoneFlag();
	SEGGER_RTT_printf(0, "Start Capture \n");
	devOV2640StartCapture();
	devOV2640HoldUntilCaptureFinished();
	SEGGER_RTT_printf(0, "Capture Done \n");
	// single FIFO read
	do
	{
		// single FIFO read
		readOV2640_SPI(kSSD1331CommandSINGLEFIFOREAD, 2);
		SEGGER_RTT_printf(0, "\r\t Value  is 0x%02x \n\n", deviceOV2640State_SPI.spiSinkBuffer[1]);
		counter++;
	} while(counter < 10);
}
