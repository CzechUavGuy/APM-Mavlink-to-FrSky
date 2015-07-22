#include "Arduino.h"

#define NUMBER_OF_CHANNELS 8

volatile byte _state = HIGH;      // The sum signal state toggle
volatile unsigned long _lastChange;
volatile unsigned int receiverChannel[NUMBER_OF_CHANNELS];
volatile byte _nr=0;

void stateChange()
{
  unsigned long timeNow = micros();    //in interrupt routine it's not necessary to store it in variable, but I do it for code clarity
  unsigned int difference = timeNow - _lastChange;
  if (_nr & 1 == 1) receiverChannel[_nr >> 1] = difference; 
  if (difference > 2500) {
    _nr = 0;
  } else {
    if (_nr < NUMBER_OF_CHANNELS * 2) _nr++;
  }
  _lastChange = timeNow;
}

void receiverStart(byte interruptNumber)	//interrupt can be 0 or 1 which are pins 2 or 3
{
    attachInterrupt(interruptNumber, stateChange, CHANGE);
    _nr = 0;
    _lastChange = micros();
}
