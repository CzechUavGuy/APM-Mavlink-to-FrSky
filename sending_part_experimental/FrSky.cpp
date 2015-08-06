/*
    @author     CzechUavGuy
    @contact    czechuavguy@gmail.com
    
    based on work of
        Nils Hogberg
        nils.hogberg@gmail.com

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

#include "FrSky.h"

FrSky::FrSky() {
    bufferLength = 0;
}

FrSky::~FrSky(void) {
}

void FrSky::sendFrSky10Hz(SoftwareSerial* serialPort, Mavlink* dataProvider, byte counter) {
  //theoretical bandwidth is 150 B/s
  //practical bandwidth is only 100B/s :((
  // -> you can send only 10 B (= 5 messages) per 100ms
    bufferLength += addBufferData(ACCY, dataProvider);
    if ((counter % 2) == 0) {
        bufferLength += addBufferData(ACCX, dataProvider);
        bufferLength += addBufferData(ACCZ, dataProvider);
    }
    if ((counter % 6) == 1) {
        bufferLength += addBufferData(TEMP2, dataProvider);
        bufferLength += addBufferData(GPSSPEED,dataProvider); 
        bufferLength += addBufferData(ALTITUDE, dataProvider);
    }
    if ((counter % 6) == 3) {bufferLength += addBufferData(LATITUDE,dataProvider);}
    if ((counter % 6) == 5) {bufferLength += addBufferData(LONGITUDE,dataProvider);}
    bufferLength = writeBuffer(bufferLength, serialPort);
}

byte FrSky::lsByte(int value) {
    return ((byte) ((value) & 0xff));
}

byte FrSky::msByte(int value) {
    return ((byte) ((value) >> 8));
}

unsigned char FrSky::addToBuffer(uint16_t data, uint8_t type, byte offset) {
    byte firstByte;
    byte secondByte;
    firstByte  = 0b10000000 | (type << 3) | lowByte(data >> 7);    //first bit is one -> first byte
    frskyBuffer[bufferLength + offset] = firstByte;
    secondByte = /*0b00000000 |*/ (lowByte(data) & 0b01111111);    //first bit is zero -> second byte
    frskyBuffer[bufferLength + offset + 1] = secondByte;
    return 2;
} 

unsigned char FrSky::addBufferData(const char id, Mavlink* dataProvider) {
    switch(id) {
        case GPSSPEED : {
                        //snad je to v m/s, asi to ve floatech
                        //I hope it's in m/s, perhaps in float
            float gpsSpeed  = dataProvider->getGpsGroundSpeed();
            uint16_t data = (uint16_t)(gpsSpeed);    //mohl bych vynasobit treba 4x abych mel vyssi presnost, ale myslim ze to neni treba. Rozsah rychlosti meho letadla je imho 0-30m/s, takze mi zde zbyva 5 bitu
            return addToBuffer(data, karelVGND, 0);  //I could multiply it by 4 and get better precision, but I don't think it's needed. Speed range of my plane is imho 0-30m/s, so 5 bits are left
            break;
        }
        case LATITUDE : {
            float gpsLatitude = dataProvider->karelGetLatitude();    
                //karellatitude je float cislo v desetinach mikrostupne, tzn. -900 000 000.0 - +900 000 000.0  TODO: mavlink to ma v celych cislech, chyba je v mavlink.cpp ze to konvertuje do floatu
                //karellatitude is float number in tenths of microdegree, ie -900 000 000.0 - + 900 000 000.0  TODO: mavlink transmits integers, unnecessary conversion to floats is in mavlink.cpp
            uint32_t latInt = (gpsLatitude / 10.0) + 90000000;  //range 0 - 180 000 000 
            
            uint16_t data = (uint16_t)((latInt ) / 1000000);
            addToBuffer(data, karelLatDEG, 0);

            data = (uint16_t)((latInt - (data * 1000000)) / 1000);
            addToBuffer(data, karelLatMIL, 2);

            data = (uint16_t)(latInt % 1000);
            addToBuffer(data, karelLatMIC, 4);
            return 6;
            break;
        }

        case LONGITUDE : {
            float gpsLongitude = dataProvider->karelGetLongitude();
            uint32_t lonInt = (gpsLongitude / 10.0) + 180000000;  //range 0 - 360 000 000
            
            uint16_t data = (uint16_t)((lonInt ) / 1000000);
            addToBuffer(data, karelLonDEG, 0);

            data = (uint16_t)((lonInt - (data * 1000000)) / 1000);
            addToBuffer(data, karelLonMIL, 2);

            data = (uint16_t)(lonInt % 1000);
            addToBuffer(data, karelLonMIC, 4);
            return 6;
            break;
        }

        case ALTITUDE : {
            //I think it is in meters
            //chci mit rozsah 0 ~ 2048 m.n.m., s presnosti na 2 metry
            //I want range 0 - 2048 meters ASL with 2m precision
            
            float altitude = dataProvider->getAltitude();
            uint16_t data = (uint16_t)(altitude / 2);
            return addToBuffer(data, karelALT, 0);
            break;
        }
/*        case ACCXY : {
            float accX = dataProvider->getAccX();  //accX is pitch in range -90.0 to +90.0
            float accY = dataProvider->getAccY();  //accY is roll in range -180.0 to +180.0
            uint16_t data = (uint16_t)(accY + 180.0);  //0 to 360
            data = data / 6;  //0 to 60 (that means 6 bits)
            data = data << 4;
            const int8_t pitchLimits[16] = {-90, -70, -50, -30, -20, -10, -5, 0, 5, 10, 15, 20, 30, 50, 70, 90};
            byte resi = 0;    //0 to 15 (that means 4 bits)
            for (byte i = 1; i< 16; i++) {
                if (accX > pitchLimits[i]) {resi = i;}
            }
            data = data + resi;
            return addToBuffer(data, karelACCXY, 0);
            break;
        }*/
        case ACCX : {
            float accX = dataProvider->getAccX();  //accX is in range -180.0 az +180.0
            uint16_t data = (uint16_t)(accX + 180.0);
            return addToBuffer(data, karelACCX, 0);
            break;
        }
        case ACCY : {
            float accY = dataProvider->getAccY(); 
            uint16_t data = (uint16_t)(accY + 180.0);
            return addToBuffer(data, karelACCY, 0);
            break;
        }
        case ACCZ : {
            float accZ = dataProvider->getAccZ(); 
            uint16_t data = (uint16_t)(accZ + 180.0);
            return addToBuffer(data, karelACCZ, 0);
            break;
        }
        case TEMP2 : {
            //warning, HACK here:
            //using one variable called TEMP2 I transmit three things: gps status, number of satellites and apmMode
            uint16_t gpsStatus = dataProvider->getGpsStatus();
            uint16_t numberOfSatelites = dataProvider->getNumberOfSatelites();
            uint16_t apmMode = dataProvider->getTemp1();  // gets apmMode
            uint16_t data = (gpsStatus << 8) + (numberOfSatelites << 4) + (apmMode);
            return addToBuffer(data, karelSTAT, 0);
            break;
        }
        default :
            return 0;
  }
  return 0;
}

unsigned char FrSky::writeBuffer(const int length, SoftwareSerial* frSkySerial) {
    for (int i = 0; i < length; i++) {
//        frSkySerial->write((byte)frskyBuffer[i]);
    }
    return 0;
}
