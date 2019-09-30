/*
* tqs.c
*
* Created: 17/10/2016 03:43:35
*  Author: me
*/
#include "TQS.h"


// configure Global EEPROM pointers
uint8_t EEMEM NonVolatileOptions; // Unused stuff. left here not to mess up the address allocation for data retrivals.

uint16_t EEMEM NonVolatileThrottleMin =  THROTTLE_MIN;
uint16_t EEMEM NonVolatileThrottleMax = THROTTLE_MAX;
uint16_t EEMEM NonVolatileMicrostickXMin = MICROSTICK_MIN;
uint16_t EEMEM NonVolatileMicrostickXMax = MICROSTICK_MAX;
uint16_t EEMEM NonVolatileMicrostickYMin = MICROSTICK_MIN;
uint16_t EEMEM NonVolatileMicrostickYmax = MICROSTICK_MAX;
uint16_t EEMEM NonVolatileRngDetent = RNG_DETENT;
uint16_t EEMEM NonVolatileAntDetent = RNG_DETENT;


TqsLimits_t gTqsLimits;
Microstick_Report_Data_Raw_t gDetents;


bool gIsConfig = false;
bool gIsMinMaxing = false;
uint32_t gConfigTimer = 0;
uint32_t gRebootTimer = 0;
uint8_t gPowerLed = 255;


void TQS_Init(void) {
	SetupPins();
	SetupSPI();
	SetupADC();
	TimerInit();
	SetupLeds();

	ReadMicrostickZero();
	ReadMem();
}

void SetupADC(void)
{
	ADC_SetupChannel(X_CH); // A0 - ANT
	ADC_SetupChannel(Y_CH); // A1 - RNG
	ADC_SetupChannel(ANT_CH); // A2 - Y
	ADC_SetupChannel(RNG_CH); // A3 - X
	ADMUX |= ADC_RIGHT_ADJUSTED | ADC_REFERENCE_AVCC;
	ADC_Init(ADC_SINGLE_CONVERSION|ADC_PRESCALE_128);
}

void SetupSPI(void)
{
	DDRB |= (1<<DDB6); // set output
	PORTB |= (1<<PINB6); // pull high
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
	DDRD &= ~(1<<PIND7);
	DDRE &= ~(1<<PINE6);
	DDRB &= ~(1<<PINB4);
	DDRB &= ~(1<<PINB5);
}

void SetupLeds(void)
{
	// Set PC7 (Arduino Pin 13) For PWM
	// Set pin to output
	DDRC |= (1<<DDC7);

	// initiallize PWM4A
	// TCCR4A |= (1 << PWM4A)
	cbi(TCCR4A, COM4A0);

	// Now turn LED on for full cycle
	setStatusLed(gPowerLed);
}


void setStatusLed(uint8_t power){
	if (power == 0 ){
		cbi(TCCR4A, COM4A1);
		PORTC &= ~(1<<PORTC7);
		} else if (power == 255) {
		cbi(TCCR4A, COM4A1);
		PORTC |= (1<<PORTC7);
		} else {
		// set timers. pin is OC4A in PMW
		//		TCCR4A |= (1 << PWM4A) |(1 << COM4A1);
		//		TCCR4A &= ~(1 << COM4A0);
		sbi(TCCR4A, COM4A1);
		OCR4A = power;
	}

}

///////// EEPROM /////////////////////
void WriteMem(void){
	// This function will write to EEPROM any values that had been changed in runtime (to save EEPROM lifetime)

	//uint8_t volatile VolatileOptions;
	uint16_t volatile VolatileThrottleMin;
	uint16_t volatile VolatileThrottleMax;
	uint16_t volatile VolatileMicrostickXMin;
	uint16_t volatile VolatileMicrostickXMax;
	uint16_t volatile VolatileMicrostickYMin;
	uint16_t volatile VolatileMicrostickYmax;
	uint16_t volatile VolatileRngDetent;
	uint16_t volatile VolatileAntDetent;

	// Read EEPROM
	VolatileThrottleMin = eeprom_read_word(&NonVolatileThrottleMin);
	VolatileThrottleMax = eeprom_read_word(&NonVolatileThrottleMax);
	VolatileMicrostickXMin = eeprom_read_word(&NonVolatileMicrostickXMin);
	VolatileMicrostickXMax = eeprom_read_word(&NonVolatileMicrostickXMax);
	VolatileMicrostickYMin = eeprom_read_word(&NonVolatileMicrostickYMin);
	VolatileMicrostickYmax = eeprom_read_word(&NonVolatileMicrostickYmax);
	VolatileRngDetent = eeprom_read_word(&NonVolatileRngDetent);
	VolatileAntDetent = eeprom_read_word(&NonVolatileAntDetent);


	if (gTqsLimits.Z.Min != VolatileThrottleMin) {
		eeprom_update_word(&NonVolatileThrottleMin,gTqsLimits.Z.Min);
	}
	if (gTqsLimits.Z.Max != VolatileThrottleMax) {
		eeprom_update_word(&NonVolatileThrottleMax,gTqsLimits.Z.Max);
	}
	if (gTqsLimits.X.Min != VolatileMicrostickXMin) {
		eeprom_update_word(&NonVolatileMicrostickXMin,gTqsLimits.X.Min);
	}
	if (gTqsLimits.X.Max != VolatileMicrostickXMax) {
		eeprom_update_word(&NonVolatileMicrostickXMax,gTqsLimits.X.Max);
	}
	if (gTqsLimits.Y.Min != VolatileMicrostickYMin) {
		eeprom_update_word(&NonVolatileMicrostickYMin,gTqsLimits.Y.Min);
	}
	if (gTqsLimits.Y.Max != VolatileMicrostickYmax) {
		eeprom_update_word(&NonVolatileMicrostickYmax,gTqsLimits.Y.Max);
	}
	if (gDetents.Y != VolatileAntDetent) {
		eeprom_update_word(&NonVolatileAntDetent,gDetents.Y);
	}
	if (gDetents.X != VolatileRngDetent) {
		eeprom_update_word(&NonVolatileRngDetent,gDetents.X);
	}
}

void ReadMem(void){

	//uint8_t volatile VolatileOptions;
	uint16_t volatile VolatileThrottleMin;
	uint16_t volatile VolatileThrottleMax;
	uint16_t volatile VolatileMicrostickXMin;
	uint16_t volatile VolatileMicrostickXMax;
	uint16_t volatile VolatileMicrostickYMin;
	uint16_t volatile VolatileMicrostickYmax;
	uint16_t volatile VolatileRngDetent;
	uint16_t volatile VolatileAntDetent;


	// Read EEPROM
	//VolatileOptions = eeprom_read_byte(&NonVolatileOptions);
	VolatileThrottleMin = eeprom_read_word(&NonVolatileThrottleMin);
	VolatileThrottleMax = eeprom_read_word(&NonVolatileThrottleMax);
	VolatileMicrostickXMin = eeprom_read_word(&NonVolatileMicrostickXMin);
	VolatileMicrostickXMax = eeprom_read_word(&NonVolatileMicrostickXMax);
	VolatileMicrostickYMin = eeprom_read_word(&NonVolatileMicrostickYMin);
	VolatileMicrostickYmax = eeprom_read_word(&NonVolatileMicrostickYmax);
	VolatileRngDetent = eeprom_read_word(&NonVolatileRngDetent);
	VolatileAntDetent = eeprom_read_word(&NonVolatileAntDetent);

	// Check and place values
	//#define EEPROM_EMPTY_BYTE(b) (b == 0xFF)
	//#define EEPROM_EMPTY_WORD(w) (w == 0xFFFF)
#define EEPROM_EMPTY_BYTE(b) ((b) == (0) ? 1 : (b) == (0xFF) ? 1 : 0)
#define EEPROM_EMPTY_WORD(w) ((w) == (0) ? 1 : (w) == (0xFFFF) ? 1 : 0)
	// result = a > b ? x : y;

	//gOptions = EEPROM_EMPTY_BYTE(VolatileOptions) ? 0 : VolatileOptions;
	gTqsLimits.Z.Min = EEPROM_EMPTY_WORD(VolatileThrottleMin) ? THROTTLE_MIN : VolatileThrottleMin;
	gTqsLimits.Z.Max = EEPROM_EMPTY_WORD(VolatileThrottleMax) ? THROTTLE_MAX : VolatileThrottleMax;
	gTqsLimits.X.Min = EEPROM_EMPTY_WORD(VolatileMicrostickXMin) ? MICROSTICK_MIN : VolatileMicrostickXMin;
	gTqsLimits.X.Max = EEPROM_EMPTY_WORD(VolatileMicrostickXMax) ? MICROSTICK_MAX : VolatileMicrostickXMax;
	gTqsLimits.Y.Min = EEPROM_EMPTY_WORD(VolatileMicrostickYMin) ? MICROSTICK_MIN : VolatileMicrostickYMin;
	gTqsLimits.Y.Max = EEPROM_EMPTY_WORD(VolatileMicrostickYmax) ? MICROSTICK_MAX : VolatileMicrostickYmax;
	gDetents.Y = EEPROM_EMPTY_WORD(VolatileAntDetent) ? ANT_DETENT : VolatileAntDetent;
	gDetents.X = EEPROM_EMPTY_WORD(VolatileRngDetent) ? RNG_DETENT : VolatileRngDetent;
}

uint16_t readSPIADC(){
	PORTB &= ~(1<<PINB6); // Pull shift register CS low
	SPI_SendByte(0x01); //send start
	uint16_t result = ((uint16_t)(SPI_TransferByte(0xa0) & ~(0xF0)) << 8) | ((uint16_t)SPI_ReceiveByte());
	PORTB |= (1<<PINB6); // Pull Shift register CS high
	return result;
}

Microstick_Report_Data_t mapMicrostick(Microstick_Report_Data_Raw_t* RawAxis) {
	Microstick_Report_Data_t MappedAxis;
	if (abs(RawAxis->X-gTqsLimits.X.Center) < 15) {
		MappedAxis.X = 0;
		} else {
		MappedAxis.X = mapCurve(RawAxis->X-gTqsLimits.X.Center,gTqsLimits.X.Min-gTqsLimits.X.Center,gTqsLimits.X.Max-gTqsLimits.X.Center,OUTPUT_MAX_8BIT,OUTPUT_MIN_8BIT,SENSETIVITY);

	}
	if (abs(RawAxis->Y-gTqsLimits.Y.Center) < 15) {
		MappedAxis.Y = 0;
		} else {
		MappedAxis.Y = mapCurve(RawAxis->Y-gTqsLimits.Y.Center,gTqsLimits.Y.Min-gTqsLimits.Y.Center,gTqsLimits.Y.Max-gTqsLimits.Y.Center,OUTPUT_MIN_8BIT,OUTPUT_MAX_8BIT,SENSETIVITY);
	}
	return MappedAxis;
}

uint16_t ReadAxis(uint8_t adcChannel,bool invert_axis) {
	uint16_t retVal = ADC_GetChannelReading(adcChannel|ADC_RIGHT_ADJUSTED | ADC_REFERENCE_AVCC);
	if (invert_axis) {
		return  ~(retVal);
		} else {
		return  retVal;
	};
}

void ReadMicrostickZero(void) {
	gTqsLimits.X.Center = (uint32_t)(ReadX+ReadX+ReadX)/3;
	gTqsLimits.Y.Center = (uint32_t)(ReadY+ReadY+ReadY)/3;
}
void ReadRoteriesDetent(void) {
	gDetents.Y = (uint32_t)(ReadANT+ReadANT+ReadANT)/3;
	gDetents.X = (uint32_t)(ReadRNG+ReadRNG+ReadRNG)/3;
}

Microstick_Report_Data_t ReadMicrostick(void){
	Microstick_Report_Data_Raw_t NewData;
	NewData.X = ReadX;
	NewData.Y = ReadY;

	return mapMicrostick(&NewData);
}

int16_t MapRoteries(uint16_t RawData,uint16_t Detent) {
	int16_t mid = (OUTPUT_MIN_10BIT+OUTPUT_MAX_10BIT)/2;

	if (abs(RawData - Detent) < 2) {
		return mid;
	}

	if (RawData > Detent) {
		return map(RawData,Detent,1023,mid,OUTPUT_MAX_10BIT);
		} else {
		return map(RawData,0,Detent,OUTPUT_MIN_10BIT,mid);
	}
}

int16_t MapCurveRoteries(uint16_t RawData,uint16_t Detent) {
	//int16_t mid = (OUTPUT_MIN_10BIT+OUTPUT_MAX_10BIT)/2;
//
	//if (abs(RawData - Detent) < 4) {
		//return mid;
	//}

	return mapCurve(RawData-Detent,OUTPUT_MIN_10BIT-Detent,OUTPUT_MAX_10BIT-Detent,OUTPUT_MIN_10BIT,OUTPUT_MAX_10BIT,ANT_SENSETIVITY);
	//if (RawData > Detent) {
		//return mapCurve(RawData,Detent,1023,mid,OUTPUT_MAX_10BIT,ANT_SENSETIVITY);
	//} else {
		//return mapCurve(RawData,0,Detent,OUTPUT_MIN_10BIT,mid,ANT_SENSETIVITY);
	//}
}

void ReadTqs(TQS_t* TqsReport)
{
	static Microstick_Report_Data_t MicrostickHistory[3];
	static Microstick_Report_Data_t NewMicrostick;
	static Microstick_Report_Data_Raw_t Roteries;
	static uint16_t buttonbuffer;

	buttonbuffer = 0;

	// I needed to put a delay after each active pin changes state. to avoid delay, I've put the ADC poll for all 4 axis as the delay

	DDRD |= (1<<DDD4); //Set to output
	PORTD &= ~(1 << PIND4); // pull "pin 4" down
	
	// run ADC conversion as delay
	Roteries.X = ReadRNG;
		
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
	DDRD &= ~(1<<DDD4); // set port to Hi-Z
	PORTD &= ~(1 << PIND4);  // make sure you are Hi-Z and not pullup

	// do some math as delay
 	TqsReport->RNG = MapRoteries(Roteries.X,gDetents.X);

	// poll toggles	T2-5
	DDRD |= (1<<DDD1); //Set "pin 2" to output
	PORTD &= ~(1 << PD1); // pull "pin 3" down
	
	// Poll ADC as delay
	Roteries.Y = ReadANT;
		
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
	DDRD &= ~(1<<DDD1); // set "pin 2" to Hi-Z
	PORTD &= ~(1 << PIND1);  // make sure you are Hi-Z and not pullup

	// Math as delay
	TqsReport->ANT = MapCurveRoteries(Roteries.Y,gDetents.Y);

	// poll toggles	T7-10
	DDRD |= (1<<DDD0); //Set "pin 3" to output
	PORTD &= ~(1 << PIND0); // pull "pin 3" down

	NewMicrostick = ReadMicrostick();

	MicrostickHistory[2] = MicrostickHistory[1];
	MicrostickHistory[1] = MicrostickHistory[0];
	MicrostickHistory[0] = NewMicrostick;

	TqsReport->X = (((int32_t)MicrostickHistory[0].X*3)+((int32_t)MicrostickHistory[1].X*2)+((int32_t)MicrostickHistory[2].X))/6;
	TqsReport->Y = (((int32_t)MicrostickHistory[0].Y*3)+((int32_t)MicrostickHistory[1].Y*2)+((int32_t)MicrostickHistory[2].Y))/6;
	
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

	DDRD &= ~(1<<DDD0); // set "pin 3" to Hi-Z
	PORTD &= ~(1 << PIND0);  // make sure you are Hi-Z and not pullup

	TqsReport->Z  = mapLargeNumbers(ReadThrottle,gTqsLimits.Z.Min,gTqsLimits.Z.Max,OUTPUT_MIN_12BIT,OUTPUT_MAX_12BIT);


	buttonbuffer = (buttonbuffer >> 1);
	if ((buttonbuffer & AllButtons) == AllButtons) {
		TqsReport->Buttons = 0;
		} else {
		TqsReport->Buttons = buttonbuffer;
	}
}

void ConfigDetection(uint16_t Buttons){
	if (gIsMinMaxing) {
		UpdateMinMax();
	}

	if (gIsConfig) { // if we are in config mode
		gPowerLed -= 2; // will dim PC7 every round, to have a pulsing light to indicate config mode
		setStatusLed(gPowerLed);

		// Check for config timeout
		if (millis() - gConfigTimer >= CONFIG_TIMEOUT) {
			exitConfig();
		}

		if (Buttons & VhfTrans) {
			ReadMicrostickZero();
			exitConfig();
		}
		if (Buttons & IffOut) {
			UpdateMinMax();
		}
		if(Buttons & IffIn) {
			ReadRoteriesDetent();
			exitConfig();
		}
		if (Buttons & UhfTrans) {
			exitConfig();
		}
		} else { // if we are not in config mode
		if (gConfigTimer == 0) {
			if ((Buttons & ConfigMode) == ConfigMode) {
				//			if ((Buttons & SbOpen) && (Buttons & Uncage)) {
				gConfigTimer = millis();
			}
			} else if (millis() - gConfigTimer >= 1500) {
			gIsConfig = true;
			gConfigTimer = millis();
			} else {
			//		if (!(Buttons & SbOpen) && !(Buttons & Uncage)) {
			if ((Buttons & ConfigMode) != ConfigMode) {

				gConfigTimer = 0;
			}
		}
	}
}

void CheckBootTimer(uint16_t Buttons) {
	if (gRebootTimer == 0) {
		//		if ((Buttons & SbOpen) && (Buttons & Uncage) && (Buttons & CursorEnable)) {
		if ((Buttons & BootLoader) == BootLoader) {
			gRebootTimer = millis();
		}
		} else if (millis() - gRebootTimer >= 5000) {
		RebootToBootloader();
		} else {
		//		if (!(Buttons & SbOpen) && !(Buttons & Uncage) && !(Buttons & CursorEnable)) {
		if ((Buttons & BootLoader) != BootLoader) {
			gRebootTimer = 0;
		}
	}
}

void exitConfig(void) {
	WriteMem(); // Save all the changes to EEPROM
	gIsConfig = false;
	gIsMinMaxing = false;
	gConfigTimer = 0;
	gPowerLed = 255;
	setStatusLed(gPowerLed);
}

void UpdateMinMax(void) {
	if (!gIsMinMaxing){

		ReadMicrostickZero();
		int16_t Zmid = (OUTPUT_MAX_12BIT+OUTPUT_MIN_12BIT)/2;
		gTqsLimits.Z.Min = Zmid-5;
		gTqsLimits.Z.Max = Zmid+5;

		gTqsLimits.X.Min = gTqsLimits.X.Center - 5;
		gTqsLimits.X.Max = gTqsLimits.X.Center + 5;

		gTqsLimits.Y.Min = gTqsLimits.Y.Center - 5;
		gTqsLimits.Y.Max = gTqsLimits.Y.Center + 5;

		gIsMinMaxing = true;

	}
	uint16_t TempVar;
	// start with the throttle.
	TempVar = ReadThrottle;
	gTqsLimits.Z.Min = min(gTqsLimits.Z.Min,TempVar);
	gTqsLimits.Z.Max = max(gTqsLimits.Z.Max,TempVar);

	// Now microstick
	// X
	TempVar = ReadX;
	gTqsLimits.X.Min = min(gTqsLimits.X.Min,TempVar);
	gTqsLimits.X.Max = max(gTqsLimits.X.Max,TempVar);
	//Y
	TempVar = ReadY;
	gTqsLimits.Y.Min =  min(gTqsLimits.Y.Min,TempVar);
	gTqsLimits.Y.Max = max(gTqsLimits.Y.Max,TempVar);

}
