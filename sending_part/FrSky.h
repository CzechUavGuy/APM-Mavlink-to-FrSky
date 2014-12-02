/*
    @author     Nils H�gberg
    @contact     nils.hogberg@gmail.com

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

#ifndef frsky_h
#define frsky_h

//#define lowByte(w) ((uint8_t) ((w) & 0xff))
//#define highByte(w) ((uint8_t) ((w) >> 8))

#include <SoftwareSerial.h>
#include "defines.h"
#include "Mavlink.h"
#include <Arduino.h>

class FrSky
{
    
public:
        FrSky();
        ~FrSky(void);
        void sendFrSky10Hz(SoftwareSerial* serialPort, Mavlink* dataProvider, byte counter);
//        void printValues(SoftwareSerial* serialPort, Mavlink* dataProvider);
private:
        unsigned char        frskyBuffer[64];
        int                    bufferLength;
        unsigned char        addBufferData(const char id, Mavlink* dataProvider);
                unsigned char           addToBuffer(uint16_t data, uint8_t type, byte offset);
        unsigned char        writeBuffer(const int length, SoftwareSerial* frSkySerial);
        byte                lsByte(int value);
        byte                msByte(int value);
};

#endif
