/*
LUFA Library
Copyright (C) Dean Camera, 2014.

dean [at] fourwalledcubicle [dot] com
www.lufa-lib.org
*/

/*
Copyright 2014  Dean Camera (dean [at] fourwalledcubicle [dot] com)

Permission to use, copy, modify, distribute, and sell this
software and its documentation for any purpose is hereby granted
without fee, provided that the above copyright notice appear in
all copies and that both that the copyright notice and this
permission notice and warranty disclaimer appear in supporting
documentation, and that the name of the author not be used in
advertising or publicity pertaining to distribution of the
software without specific, written prior permission.

The author disclaims all warranties with regard to this
software, including all implied warranties of merchantability
and fitness.  In no event shall the author be liable for any
special, indirect or consequential damages or any damages
whatsoever resulting from loss of use, data or profits, whether
in an action of contract, negligence or other tortious action,
arising out of or in connection with the use or performance of
this software.
*/

/** \file
*
*  Main source file for the Joystick demo. This file contains the main tasks of
*  the demo and is responsible for the initial application hardware configuration.
*/
#define COUGAR_OLD
//#define USE_TM_VID

#include "Joystick.h"
#include "LUFA/Drivers/Peripheral/ADC.h"
#include "LUFA/Drivers/Peripheral/SPI.h"

/** Buffer to hold the previously generated HID report, for comparison purposes inside the HID class driver. */
static uint8_t PrevJoystickHIDReportBuffer[sizeof(USB_JoystickReport_Data_t)];

/** LUFA HID Class driver interface configuration and state information. This structure is
*  passed to all HID Class driver functions, so that multiple instances of the same class
*  within a device can be differentiated from one another.
*/
USB_ClassInfo_HID_Device_t Joystick_HID_Interface =
{
	.Config =
	{
		.InterfaceNumber              = INTERFACE_ID_Joystick,
		.ReportINEndpoint             =
		{
			.Address              = JOYSTICK_EPADDR,
			.Size                 = JOYSTICK_EPSIZE,
			.Banks                = 1,
		},
		.PrevReportINBuffer           = PrevJoystickHIDReportBuffer,
		.PrevReportINBufferSize       = sizeof(PrevJoystickHIDReportBuffer),
	},
};


void SetupADC(void)
{
	ADC_SetupChannel(7); // A0 - ANT
	ADC_SetupChannel(6); // A1 - RNG
	ADC_SetupChannel(5); // A2 - Y
	ADC_SetupChannel(4); // A3 - X
	ADMUX |= ADC_RIGHT_ADJUSTED | ADC_REFERENCE_AVCC;
	ADC_Init(ADC_SINGLE_CONVERSION|ADC_PRESCALE_128);
}

void SetupSPI(void)
{
	DDRB |= (1<<DDB6); // set output
	PORTB |= (1<<PB6); // pull high
	SPI_Init(SPI_SPEED_FCPU_DIV_4|SPI_MODE_MASTER);
}

void SetupPins(void)
{
	// Pins
	// Active pins 2(PD1),3(PD0),4(PD4)
	// Sensing Pins 6(PD7), 7(PE6), 8(PB4) ,9(PB5)
	// Set Active pins to Output
	DDRD |= (1<<DDD0) | (1<<DDD1) | (1<<DDD4);// set output
	PORTD |= (1<<PD0) | (1<<PD1) | (1<<PD4);// pull high
	
	// Make sure passive pins are input
	DDRD &= ~(1<<PD7);
	DDRE &= ~(1<<PE6);
	DDRB &= ~(1<<PB4);
	DDRB &= ~(1<<PB5);
}


long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


int32_t mapLargeNumbers(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
	#define FACTOR 10000
	int ratio = ((((float)out_max - (float)out_min) / ((float)in_max - (float)in_min))*FACTOR);
	return (((x - in_min) * (ratio))/FACTOR) + out_min;
}

uint16_t readSPIADC(){
		PORTB &= ~(1<<PB6); // Pull shift register CS low
		SPI_SendByte(0x01); //send start
		uint16_t result = mapLargeNumbers((((uint16_t)(SPI_TransferByte(0xa0) & ~(0xF0)) << 8) | ((uint16_t)SPI_ReceiveByte())),350,3600,0,4095);
		PORTB |= (1<<PB6); // Pull Shift register CS high
		return result;
}
uint8_t ReadMinistick(uint8_t adcChannel,bool invert_axis) {
	if (invert_axis) {
		return  ~(map(ADC_GetChannelReading(adcChannel|ADC_RIGHT_ADJUSTED | ADC_REFERENCE_AVCC),250,850,0,255));
	} else {
		return  map(ADC_GetChannelReading(adcChannel|ADC_RIGHT_ADJUSTED | ADC_REFERENCE_AVCC),250,850,0,255);
	};
}
uint16_t ReadAxis(uint8_t adcChannel,bool invert_axis) {
	if (invert_axis) {
		return  ~(ADC_GetChannelReading(adcChannel|ADC_RIGHT_ADJUSTED | ADC_REFERENCE_AVCC));
		} else {
		return  ADC_GetChannelReading(adcChannel|ADC_RIGHT_ADJUSTED | ADC_REFERENCE_AVCC);
	};
}


#ifdef COUGAR_OLD
	#define X_PIN 4
	#define Y_PIN 5
	#define ANT_PIN 6
	#define RNG_PIN 7
#else
	#define X_PIN 7
	#define Y_PIN 6
	#define ANT_PIN 4
	#define RNG_PIN 5
#endif
#define DEADZONE 15
#define ReadX ReadMinistick(X_PIN,1)
#define ReadY ReadMinistick(Y_PIN,0)
#define ReadANT ReadAxis(ANT_PIN,0)
#define ReadRNG ReadAxis(RNG_PIN,0)
Microstick_Report_Data_t MicrostickZero = {0};
Axis_Report_Data_t AxisLastRun = {0};
Microstick_Report_Data_t MicrostickHistory[3] = {0};

void ReadMicrostickZero() {
	MicrostickZero.X = (ReadX+ReadX)/2;
	MicrostickZero.Y = (ReadY+ReadY)/2;
}	
	
/** Main program entry point. This routine contains the overall program flow, including initial
*  setup of all components and the main program loop.
*/
int main(void)
{
	SetupHardware();

	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
	GlobalInterruptEnable();
	SetupPins();
	SetupSPI();
	SetupADC();
	ReadMicrostickZero();

	for (;;)
	{
		HID_Device_USBTask(&Joystick_HID_Interface);
		USB_USBTask();
	}
}


/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
	#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
	#elif (ARCH == ARCH_XMEGA)
	/* Start the PLL to multiply the 2MHz RC oscillator to 32MHz and switch the CPU core to run from it */
	XMEGACLK_StartPLL(CLOCK_SRC_INT_RC2MHZ, 2000000, F_CPU);
	XMEGACLK_SetCPUClockSource(CLOCK_SRC_PLL);

	/* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
	XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
	XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, F_USB);

	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	#endif

	/* Hardware Initialization */
	Joystick_Init();
	LEDs_Init();
	Buttons_Init();
	USB_Init();
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= HID_Device_ConfigureEndpoints(&Joystick_HID_Interface);

	USB_Device_EnableSOFEvents();

	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	HID_Device_ProcessControlRequest(&Joystick_HID_Interface);
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
	HID_Device_MillisecondElapsed(&Joystick_HID_Interface);
}

/** HID class driver callback function for the creation of HID reports to the host.
*
*  \param[in]     HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
*  \param[in,out] ReportID    Report ID requested by the host if non-zero, otherwise callback should set to the generated report ID
*  \param[in]     ReportType  Type of the report to create, either HID_REPORT_ITEM_In or HID_REPORT_ITEM_Feature
*  \param[out]    ReportData  Pointer to a buffer where the created report should be stored
*  \param[out]    ReportSize  Number of bytes written in the report (or zero if no report is to be sent)
*
*  \return Boolean \c true to force the sending of the report, \c false to let the library determine if it needs to be sent
*/
bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
uint8_t* const ReportID,
const uint8_t ReportType,
void* ReportData,
uint16_t* const ReportSize)
{
	USB_JoystickReport_Data_t* JoystickReport = (USB_JoystickReport_Data_t*)ReportData;
	
	////// Get data from TQS //////

	// Get Throttle via SPI
	JoystickReport->Z = (((readSPIADC()*2)+AxisLastRun.Z)/3);
	if (abs(JoystickReport->Z - AxisLastRun.Z) < 5) {
		JoystickReport->Z = AxisLastRun.Z;
	}
	
	// Sense Button Matrix - this is a mess!
	// Active pins 2(PD1),3(PD0),4(PD4)
	// Sensing Pins 6(PD7), 7(PE6), 8(PB4) ,9(PB5)
	
	// I needed to put a delay after each active pin changes state. to avoid delay, I've put the ADC poll for all 4 axis as the delay
	
    uint16_t buttonbuffer = 0;

	PORTD &= ~(1 << PD4); // pull "pin 4" down
	JoystickReport->X = (((ReadX*4)+(MicrostickHistory[0].X*3)+(MicrostickHistory[1].X*2)+MicrostickHistory[2].X)/10); // run ADC conversion as delay, do average in 3:1 ratio to reduce jitter
	//if (abs(JoystickReport->X - ((AxisLastRun[0].X+AxisLastRun[1].X)/2)) < 4) {
		//JoystickReport->X = AxisLastRun[0].X;
	//}
	if (abs((int32_t)JoystickReport->X-(int32_t)MicrostickZero.X) < DEADZONE)
	{
		JoystickReport->X = MicrostickZero.X;
	}
	// T6
	if ((PIND & _BV(7))) {
		buttonbuffer &= ~(1 << 6);
		} else {
		buttonbuffer |= (1 << 6);
	}
	// T1
	if (PINE & _BV(6)) {
		buttonbuffer &= ~(1 << 1);
		} else {
		buttonbuffer |= (1 << 1);
	}
	PORTD |= (1 << PD4); // bring "pin 4" back up

	// poll toggles	T2-5
	PORTD &= ~(1 << PD1); // pull "pin 3" down
	JoystickReport->Y =  (((ReadY*4)+(MicrostickHistory[0].Y*3)+(MicrostickHistory[1].Y*2)+MicrostickHistory[2].Y)/10); // run ADC conversion as delay, do average in 3:1 ratio to reduce jitter
	if (abs((int32_t)JoystickReport->Y-(int32_t)MicrostickZero.Y) < DEADZONE)
	{
		JoystickReport->Y = MicrostickZero.Y;
	}
	//T2
	if ((PIND & _BV(7))) {
		buttonbuffer &= ~(1 << 2);
		} else {
		buttonbuffer |= (1 << 2);
	}
	// T3
	if (PINE & _BV(6)) {
		buttonbuffer &= ~(1 << 3);
		} else {
		buttonbuffer |= (1 << 3);
	}
	//T4
	if ((PINB & _BV(4))) {
		buttonbuffer &= ~(1 << 4);
		} else {
		buttonbuffer |= (1 << 4);
	}
	// T5
	if (PINB&_BV(5)) {
		buttonbuffer &= ~(1 << 5);
		} else {
		buttonbuffer |= (1 << 5);
	}
	PORTD |= (1 << PD1); // bring "pin 3" back up
	//Serial.println(buttons, BIN);
	// poll toggles	T7-10
	PORTD &= ~(1 << PD0); // pull "pin 3" down
	JoystickReport->RNG = (((ReadRNG*2)+AxisLastRun.RNG)/3); // run ADC conversion as delay
	//T7
	if ((PIND & _BV(7))) {
		buttonbuffer &= ~(1 << 7);
		} else {
		buttonbuffer |= (1 << 7);
	}
	// T8
	if (PINE & _BV(6)) {
		buttonbuffer &= ~(1 << 8);
		} else {
		buttonbuffer |= (1 << 8);
	}
	//T9
	if ((PINB & _BV(4))) {
		buttonbuffer &= ~(1 << 9);
		} else {
		buttonbuffer |= (1 << 9);
	}
	// T10
	if (PINB&_BV(5)) {
		buttonbuffer &= ~(1 << 10);
		} else {
		buttonbuffer |= (1 << 10);
	}
	PORTD |= (1 << PD0); // bring "pin 3" back up

	JoystickReport->ANT = (((ReadANT*2)+AxisLastRun.ANT)/3); // run ADC conversion as delay
	JoystickReport->Buttons = (buttonbuffer >> 1);
	// Get Throttle via SPI and average to help reduce jitter
	//JoystickReport->Z = ((JoystickReport->Z + readSPIADC())/2);
	// Set past vars
	MicrostickHistory[2]=MicrostickHistory[1];
	MicrostickHistory[1]=MicrostickHistory[0];
	MicrostickHistory[0].X=JoystickReport->X;
	MicrostickHistory[0].Y=JoystickReport->Y;
	
	AxisLastRun.Z = JoystickReport->Z;
	AxisLastRun.RNG = JoystickReport->RNG;
	AxisLastRun.ANT = JoystickReport->ANT;
	*ReportSize = sizeof(USB_JoystickReport_Data_t);
	return true;
}

/** HID class driver callback function for the processing of HID reports from the host.
*
*  \param[in] HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
*  \param[in] ReportID    Report ID of the received report from the host
*  \param[in] ReportType  The type of report that the host has sent, either HID_REPORT_ITEM_Out or HID_REPORT_ITEM_Feature
*  \param[in] ReportData  Pointer to a buffer where the received report has been stored
*  \param[in] ReportSize  Size in bytes of the received HID report
*/
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
const uint8_t ReportID,
const uint8_t ReportType,
const void* ReportData,
const uint16_t ReportSize)
{
	// Unused (but mandatory for the HID class driver) in this demo, since there are no Host->Device reports
}

