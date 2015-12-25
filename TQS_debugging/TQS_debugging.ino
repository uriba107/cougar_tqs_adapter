#include <SPI.h>

#define AXIS_X 3
#define AXIS_Y 2
#define AXIS_ANT 0
#define AXIS_RNG 1

#define READ_X analogRead(AXIS_X)
#define READ_Y analogRead(AXIS_Y)
#define READ_ANT analogRead(AXIS_ANT)
#define READ_RNG analogRead(AXIS_RNG)
struct tqs {
  uint8_t  X; /**< Current absolute joystick X position, as a signed 8-bit integer */
  uint8_t  Y; /**< Current absolute joystick Y position, as a signed 8-bit integer */
  uint16_t  Z: 12; /**< Current absolute joystick Z position, as a signed 8-bit integer */
  uint16_t  ANT: 10; /**< Current absolute joystick Y position, as a signed 8-bit integer */
  uint16_t  RNG: 10; /**< Current absolute joystick Y position, as a signed 8-bit integer */
  uint16_t Buttons: 10;
};

struct Microstick {
    int16_t X;
    int16_t Y;
};

tqs throttle = {0};
Microstick microstick_zero = {0};

int32_t map_uri(int32_t InVal, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
  // check limits to avoid out of bound results
  if (InVal <= in_min) {
    return out_min;
    } else if (InVal >= in_max) {
    return out_max;
    } else {
    // if input checks out, do the math
    return (InVal - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
}
int32_t mapLargeNumbers(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{     
  #define FACTOR 10000
    if (x < in_min) {
    return out_min;
  } else if (x > in_max) {
    return out_max;
  } else {
  int ratio = ((((float)out_max - (float)out_min) / ((float)in_max - (float)in_min))*FACTOR);
  return (((x - in_min) * (ratio))/FACTOR) + out_min;
  }
}
  
int32_t mapCurve(int32_t inVal, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max, int16_t axisZero, uint8_t sensetivity)
{
  /* Curve no 4 based on this source
  * https://github.com/achilleas-k/fs2open.github.com/blob/joystick_curves/joy_curve_notes/new_curves.md
  * 0 is mosed curved, 9 is linear
  */
  #define DEADZONE 15
  int16_t midRange = (in_min+in_max)/2;
  if (abs(inVal-axisZero) < DEADZONE) {
    inVal = axisZero;
  }
  double relativePos = ((double)(inVal - axisZero)/midRange)*2;
  float curveFactor = pow(abs(relativePos),(3-(sensetivity/4.5)));
//    float curveFactor = pow(abs(relativePos),sensetivity/9) * pow(((1-cos(abs(relativePos)*PI))/2),((9-sensetivity)/4.5));
    
  if (relativePos < 0) {
    curveFactor *= -1;
  }
  int32_t computedVal = (midRange*curveFactor)+midRange;
  int32_t retVal = map_uri(computedVal,in_min,in_max,out_min,out_max);

        Serial.print("mapCurve: ");
    Serial.print(inVal);
    Serial.print("|");
    Serial.print(axisZero); 
    Serial.print("|");
    Serial.print(relativePos);
    Serial.print("|");
    Serial.print(computedVal);
    Serial.print("|");
    Serial.print(retVal);
    Serial.println("");
  return retVal;
}




void GetMicrostickZero() {
//    uint32_t x = (analogRead(3)+analogRead(3)+analogRead(3))/3;
//    uint32_t y = (analogRead(2)+analogRead(2)+analogRead(2))/3;
//
////
//  microstick_zero.X = map(x,250,850,-127,128);
//  microstick_zero.Y = map(y,250,850,-127,128);

//  microstick_zero.X = map((READ_X+READ_X+READ_X)/3,250,850,-127,128);
//  microstick_zero.Y = map((READ_Y+READ_Y+READ_Y)/3,250,850,-127,128);
  microstick_zero.X = (uint32_t)(READ_X+READ_X+READ_X)/3;
  microstick_zero.Y = (uint32_t)(READ_Y+READ_Y+READ_Y)/3;  

}

uint16_t pullMatrix(void) {
  uint16_t buttons = 0;
  //  byte debounce = 0xff;

  PORTD &= ~(1 << PD4); // pull "pin 4" down
  delay(1);
  // T6
  if ((PIND & _BV(7))) {
    buttons &= ~(1 << 6);
  } else {
    buttons |= (1 << 6);
  }
  // T1
  if (PINE & _BV(6)) {
    buttons &= ~(1 << 1);
  } else {
    buttons |= (1 << 1);
  }
  PORTD |= (1 << PD4); // bring "pin 4" back up

  // poll toggles	T2-5
  PORTD &= ~(1 << PD1); // pull "pin 3" down
  delay(1);
//T2
  if ((PIND & _BV(7))) {
    buttons &= ~(1 << 2);
  } else {
    buttons |= (1 << 2);
  }
  // T3
  if (PINE & _BV(6)) {
    buttons &= ~(1 << 3);
  } else {
    buttons |= (1 << 3);
  }
//T4
  if ((PINB & _BV(4))) {
    buttons &= ~(1 << 4);
  } else {
    buttons |= (1 << 4);
  }
  // T5
  if (PINB&_BV(5)) {
    buttons &= ~(1 << 5);
  } else {
    buttons |= (1 << 5);
  }
  PORTD |= (1 << PD1); // bring "pin 3" back up
  //Serial.println(buttons, BIN);
  // poll toggles	T7-10
  PORTD &= ~(1 << PD0); // pull "pin 3" down
  delay(1);

//T7
  if ((PIND & _BV(7))) {
    buttons &= ~(1 << 7);
  } else {
    buttons |= (1 << 7);
  }
  // T8
  if (PINE & _BV(6)) {
    buttons &= ~(1 << 8);
  } else {
    buttons |= (1 << 8);
  }
//T9
  if ((PINB & _BV(4))) {
    buttons &= ~(1 << 9);
  } else {
    buttons |= (1 << 9);
  }
  // T10
  if (PINB&_BV(5)) {
    buttons &= ~(1 << 10);
  } else {
    buttons |= (1 << 10);
  }
  PORTD |= (1 << PD0); // bring "pin 3" back up

  return (buttons >> 1);
}

uint16_t ReadADC(void) {
  uint16_t buff = 0;
  digitalWrite(10, LOW);
  SPI.transfer(0x01);
  buff = ((uint16_t)((SPI.transfer(0xa0) & ~(0xF0)) << 8)) | ((uint16_t)SPI.transfer(0));
  digitalWrite(10, HIGH);
  return buff;
}

uint16_t Readthrottle(void) {
  uint16_t buff = 0;
  digitalWrite(10, LOW);
  SPI.transfer(0x01);
  buff = ((uint16_t)((SPI.transfer(0xa0) & ~(0xF0)) << 8)) | ((uint16_t)SPI.transfer(0));
  digitalWrite(10, HIGH);
  return mapLargeNumbers(buff,400,3650,0,4095);
}

void setup() {
  // put your setup code here, to run once:
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  pinMode(9, INPUT);

  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH);
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  Serial.begin(9600);
  Serial.println("Start");
  
  GetMicrostickZero();
}

short col = 2;
uint16_t throttlemax = 0;
uint16_t throttlemin = 4096;
int xmin = 1023;
int xmax = 0;
int ymin =1023;
int ymax =0;
void loop() {
  // Get throttle - 0 for now
 
  throttle.Z = Readthrottle();
  if (throttle.Z > throttlemax) {
    throttlemax = throttle.Z;
  }
  if (throttle.Z < throttlemin) {
    throttlemin = throttle.Z;
  }

  // Get Axis data
  throttle.RNG = READ_RNG;
  throttle.ANT = READ_ANT;
  int x = READ_X;
  int y = READ_Y;
//
  throttle.X = mapCurve(READ_X,250,850,0,255,microstick_zero.X,9);
  throttle.Y = mapCurve(READ_Y,250,850,0,255,microstick_zero.Y,3);
//
  if (x > xmax) {
    xmax = x;
  }
  if (x < xmin) {
    xmin = x;
  }

  if (y > ymax) {
    ymax = y;
  }
  if (y < ymin) {
    ymin = y;
  }
  // Get buttons

  throttle.Buttons = pullMatrix();

  /// Print stuff
//    Serial.print(" | Throttle: ");
//    Serial.print(throttle.Z);
////    Serial.print(",Throttle max: ");
////    Serial.print(throttlemax);
////    Serial.print(",Throttle min: ");
////    Serial.print(throttlemin);
//
    Serial.print(" | ministick: ");
    Serial.print(throttle.X);
////        Serial.print(x);
    Serial.print(',');
    Serial.print(throttle.Y);
//    Serial.print(y);

////    Serial.print(",X max: ");
////    Serial.print(xmax);
////    Serial.print(",X min: ");
////    Serial.print(xmin);
//  Serial.print(" | Microstick_zero: ");
//  Serial.print(microstick_zero.X);
//  Serial.print(",");
//  Serial.print(microstick_zero.Y);
////////  //
//    Serial.print(" | axis: ");
//    Serial.print(throttle.RNG);
//    Serial.print(',');
//    Serial.print(throttle.ANT);
//  //
//    Serial.print(" | buttons: ");
//    Serial.print(throttle.Buttons, BIN);


Serial.println("");

delay(250);
    

}

