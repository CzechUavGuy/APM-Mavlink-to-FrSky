/*
        @author 	CzechUavGuy
        @contact 	czechuavguy@gmail.com
    
        code was shamelessly stolen from MultiWii project, which is GPS license.
	
	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/	

/*
basic example:

#include <receiver2.h>

void setup() {
  configureReceiver();
  Serial.begin(115200);
}

void loop() {
  delay(10);
  for (int i=0;i<6; i++) {Serial.print(readRawRC(i)); Serial.print(" ");}
  Serial.println();
}

*/

#ifndef Receiver2_h
#define Receiver2_h

#include "Arduino.h"

  #define PCIR_PORT_BIT              (1<<2)
  #define PCINT_RX_MASK              PCMSK2
  #define PCINT_RX_PORT              PORTD
  #define PCINT_PIN_COUNT            5   // if you want to add more, redefine PCINT_RX_BITS
  #define RX_PCINT_PIN_PORT          PIND
  #define PCINT_RX_BITS              (1<<2),(1<<4),(1<<5),(1<<6),(1<<7)
  #define THROTTLEPIN                2  // if you want to redefine pin #, change it also in function ISR()
  #define ROLLPIN                    4
  #define PITCHPIN                   5
  #define YAWPIN                     6
  #define AUX1PIN                    7
  #define AUX2PIN                    0 // unused, optional PIN 8 or PIN 12
  #define AUX3PIN                    1 // unused 
  #define AUX4PIN                    3 // unused 
  #define RX_PC_INTERRUPT            PCINT2_vect

volatile uint16_t rcValue[8] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]
static uint8_t rcChannel[8]  = {ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, AUX1PIN,AUX2PIN,AUX3PIN,AUX4PIN};
static uint8_t PCInt_RX_Pins[PCINT_PIN_COUNT] = {PCINT_RX_BITS};  

void configureReceiver() {
  // PCINT activation
  for(uint8_t i = 0; i < PCINT_PIN_COUNT; i++){ // i think a for loop is ok for the init.
    PCINT_RX_PORT |= PCInt_RX_Pins[i];
    PCINT_RX_MASK |= PCInt_RX_Pins[i];
  }
  PCICR = PCIR_PORT_BIT;
}

#define RX_PIN_CHECK(pin_pos, rc_value_pos)                        \
  if (mask & PCInt_RX_Pins[pin_pos]) {                             \
    if (!(pin & PCInt_RX_Pins[pin_pos])) {                         \
      dTime = cTime-edgeTime[pin_pos];                             \
      if (900<dTime && dTime<2200) {                               \
        rcValue[rc_value_pos] = dTime;                             \
      }                                                            \
    } else edgeTime[pin_pos] = cTime;                              \
  }

// port change Interrupt
ISR(RX_PC_INTERRUPT) { //this ISR is common to every receiver channel, it is call everytime a change state occurs on a RX input pin
  uint8_t mask;
  uint8_t pin;
  uint16_t cTime,dTime;
  static uint16_t edgeTime[8];
  static uint8_t PCintLast;

  pin = RX_PCINT_PIN_PORT; // RX_PCINT_PIN_PORT indicates the state of each PIN for the arduino port dealing with Ports digital pins
 
  mask = pin ^ PCintLast;   // doing a ^ (xor) between the current interruption and the last one indicates wich pin changed
  cTime = micros();         // micros() return a uint32_t, but it is not usefull to keep the whole bits => we keep only 16 bits
  sei();                    // re enable other interrupts at this point, the rest of this interrupt is not so time critical and can be interrupted safely
  PCintLast = pin;          // we memorize the current state of all PINs [D0-D7]

  #if (PCINT_PIN_COUNT > 0)
    RX_PIN_CHECK(0,2);
  #endif
  #if (PCINT_PIN_COUNT > 1)
    RX_PIN_CHECK(1,4);
  #endif
  #if (PCINT_PIN_COUNT > 2)
    RX_PIN_CHECK(2,5);
  #endif
  #if (PCINT_PIN_COUNT > 3)
    RX_PIN_CHECK(3,6);
  #endif
  #if (PCINT_PIN_COUNT > 4)
    RX_PIN_CHECK(4,7);
  #endif
  #if (PCINT_PIN_COUNT > 5)
    RX_PIN_CHECK(5,0);
  #endif
  #if (PCINT_PIN_COUNT > 6)
    RX_PIN_CHECK(6,1);
  #endif
  #if (PCINT_PIN_COUNT > 7)
    RX_PIN_CHECK(7,3);
  #endif
}
  
uint16_t readRawRC(uint8_t chan) {
  uint16_t data;
  uint8_t oldSREG;
  oldSREG = SREG; cli(); // Let's disable interrupts
  data = rcValue[rcChannel[chan]]; // Let's copy the data Atomically
  SREG = oldSREG;        // Let's restore interrupt state
  return data; // We return the value correctly copied when the IRQ's where disabled
}
#endif