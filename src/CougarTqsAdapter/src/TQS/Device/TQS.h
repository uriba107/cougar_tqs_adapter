#ifndef tqs_h__
#define tqs_h__

#define EARLY_PROTOTYPE

#pragma once

#include <avr/io.h>
#include <avr/eeprom.h>

#include <LUFA/Drivers/Peripheral/ADC.h>
#include <LUFA/Drivers/Peripheral/SPI.h>
#include "../Common/Helpers.h"

#define CONFIG_TIMEOUT 20000
#define DEADZONE 5
#define SENSETIVITY 3.0
#define ANT_SENSETIVITY 3.0



#define OUTPUT_MIN_8BIT -128
#define OUTPUT_MAX_8BIT 127
#define OUTPUT_MIN_10BIT 0
#define OUTPUT_MAX_10BIT 1023
#define OUTPUT_MIN_12BIT 0
#define OUTPUT_MAX_12BIT 4095

#define MICROSTICK_MIN 300
#define MICROSTICK_MAX 800
#define THROTTLE_MIN 350
#define THROTTLE_MAX 3600
#define RNG_DETENT 511
#define ANT_DETENT 511

#ifdef EARLY_PROTOTYPE
#define X_CH 4
#define Y_CH 5
#define ANT_CH 7
#define RNG_CH 6
#else
#define X_CH 7
#define Y_CH 6
#define ANT_CH 4
#define RNG_CH 5
#endif

#define ReadX ReadAxis(X_CH,0)
#define ReadY ReadAxis(Y_CH,0)
#define ReadANT ReadAxis(ANT_CH,0)
#define ReadRNG ReadAxis(RNG_CH,0)
#define ReadThrottle readSPIADC()


enum TqsButtons {
   CursorEnable = 0x01,
   VhfTrans = 0x02,
   UhfTrans = 0x04,
   IffIn = 0x08,
   IffOut = 0x10,
   Uncage = 0x20,
   DgftOveride = 0x40,
   MrmOveride = 0x80,
   SbOpen = 0x100,
   SbClose = 0x200,
   ConfigMode = 0x120,
   BootLoader = 0x121,
   AllButtons = 0x3FF,
};
// Structs
typedef struct  {
  uint16_t Center:12;
  uint16_t Min:12;
  uint16_t Max:12;
} AxisParams_t;

typedef struct  {
  AxisParams_t X;
  AxisParams_t Y;
  AxisParams_t Z;
} TqsLimits_t;

typedef struct
{
	int8_t  X; /**< Current absolute joystick X position, as a signed 8-bit integer */
	int8_t  Y; /**< Current absolute joystick Y position, as a signed 8-bit integer */
} Microstick_Report_Data_t;

typedef struct
{
	uint16_t  X:10; /**< Current absolute joystick X position, as a signed 8-bit integer */
	uint16_t  Y:10; /**< Current absolute joystick Y position, as a signed 8-bit integer */
} Microstick_Report_Data_Raw_t;

typedef struct
{
	int8_t  X; /** Microstick X */
	int8_t  Y; /** Microstick Y */
	uint16_t  Z:12; /** Throttle - Z */
	int16_t  ANT:10; /**  Ant - Ry */
	int16_t  RNG:10; /**  Range - Rx */
	uint16_t  Buttons:10;
} TQS_t;

extern TqsLimits_t TqsLimits;
extern Microstick_Report_Data_Raw_t gDetents;
//extern uint8_t gOptions;

extern bool gIsConfig;
extern bool gIsMinMaxing;

extern uint32_t gConfigTimer;
extern uint32_t gRebootTimer;
extern uint8_t gPowerLed;


//EEPROM
void WriteMem(void);
void ReadMem(void);

// Functions
void TQS_Init(void);

void SetupADC(void);
void SetupSPI(void);
void SetupPins(void);
void SetupLeds(void);
void setStatusLed(uint8_t power);

uint16_t readSPIADC(void);
Microstick_Report_Data_t mapMicrostick(Microstick_Report_Data_Raw_t* RawAxis);
int16_t MapRoteries(uint16_t RawData,uint16_t Detent);
int16_t MapCurveRoteries(uint16_t RawData,uint16_t Detent);



void ReadRoteriesDetent(void);

uint16_t ReadAxis(uint8_t adcChannel,bool invert_axis);
void ReadMicrostickZero(void);
void ReadTqs(TQS_t* JoystickReport);
void ConfigDetection(uint16_t Buttons);

void UpdateMinMax(void);
void exitConfig(void);
void processOutEndpoint(uint8_t inOptions);

#endif // tqs_h__
