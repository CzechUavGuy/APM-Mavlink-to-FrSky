/*
	@author 	CzechUavGuy
	@contact 	czechuavguy@gmail.com
	
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

FrSky::FrSky()
{
	bufferLength = 0;
}
FrSky::~FrSky(void)
{
}

void FrSky::sendFrSky05Hz(SoftwareSerial* serialPort, Mavlink* dataProvider){}

void FrSky::sendFrSky2Hz(SoftwareSerial* serialPort, Mavlink* dataProvider)
{
	bufferLength += addBufferData(LATITUDE,dataProvider);
	bufferLength += addBufferData(LONGITUDE, dataProvider);
	bufferLength += addBufferData(GPSSPEED, dataProvider);
	bufferLength += addBufferData(ALTITUDE, dataProvider);
	bufferLength += addBufferData(TEMP2, dataProvider);
	bufferLength = writeBuffer(bufferLength, serialPort);
}

void FrSky::sendFrSky10Hz(SoftwareSerial* serialPort, Mavlink* dataProvider)
{
	bufferLength += addBufferData(ACCX,dataProvider);
	bufferLength += addBufferData(ACCY, dataProvider);
	bufferLength = writeBuffer(bufferLength, serialPort);
}
void FrSky::sendFrSky5Hz(SoftwareSerial* serialPort, Mavlink* dataProvider)
{
	bufferLength += addBufferData(ACCZ,dataProvider);
	bufferLength = writeBuffer(bufferLength, serialPort);
}

byte FrSky::lsByte(int value)
{
  return ((byte) ((value) & 0xff));
}

byte FrSky::msByte(int value)
{
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

unsigned char FrSky::addBufferData(const char id, Mavlink* dataProvider)
{
	switch(id) {
		case GPSSPEED :
		{
                        //snad je to v m/s, asi to ve floatech
                        //I hope it's in m/s, perhaps in float
			float gpsSpeed  = dataProvider->getGpsGroundSpeed();
                        uint16_t data = (uint16_t)(gpsSpeed);    //mohl bych vynasobit treba 4x abych mel vyssi presnost, ale myslim ze to neni treba. Rozsah rychlosti meho letadla je imho 0-30m/s, takze mi zde zbyva 5 bitu
                        return addToBuffer(data, karelVGND, 0);  //I could multiply it by 4 and get better precision, but I don't think it's needed. Speed range of my plane is imho 0-30m/s, so 5 bits are left
			break;
		}
		case LATITUDE :
		{
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

		case LONGITUDE :
		{
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


		case ALTITUDE :
		{
                        //I think it is in meters
                        //chci mit rozsah 0 ~ 2048 m.n.m., s presnosti na 2 metry
                        //I want range 0 - 2048 meters ASL with 2m precision
                        
			float altitude = dataProvider->getAltitude();
                        uint16_t data = (uint16_t)(altitude / 2);
                        return addToBuffer(data, karelALT, 0);
			break;
		}
		case ACCX :
		{
			float accX = dataProvider->getAccX();  //accX is in range -180.0 az +180.0
                        uint16_t data = (uint16_t)(accX + 180.0);
                        return addToBuffer(data, karelACCX, 0);
			break;
		}
		case ACCY :
		{
			float accY = dataProvider->getAccY(); 
                        uint16_t data = (uint16_t)(accY + 180.0);
                        return addToBuffer(data, karelACCY, 0);
			break;
		}
		case ACCZ :
		{
			float accZ = dataProvider->getAccZ(); 
                        uint16_t data = (uint16_t)(accZ + 180.0);
                        return addToBuffer(data, karelACCZ, 0);
			break;
		}
		case TEMP2 :
		{
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

unsigned char FrSky::writeBuffer(const int length, SoftwareSerial* frSkySerial)
{

  int i = 0;
  while(i < length)
  {
    frSkySerial->write((byte)frskyBuffer[i]);
    i++;
  }
  
  return 0;
}

/*void FrSky::printValues(SoftwareSerial* serialPort, Mavlink* dataProvider)
{
	serialPort->print("Voltage: ");
	serialPort->print(dataProvider->getMainBatteryVoltage(), 2);
	serialPort->print(" Current: ");
	serialPort->print(dataProvider->getBatteryCurrent(), 2);
	serialPort->print(" Fuel: ");
	serialPort->print(dataProvider->getFuelLevel());
	serialPort->print(" Latitude: ");
	serialPort->print(dataProvider->getLatitude(), 6);
	serialPort->print(" Longitude: ");
	serialPort->print(dataProvider->getLongitud(), 6);
	serialPort->print(" GPS Alt: ");
	serialPort->print(dataProvider->getGpsAltitude(), 2);
	//serialPort->print(" GPS hdop: ");
	//serialPort->print(dataProvider->getGpsHdop(), 2);
	serialPort->print(" GPS status + sats: ");
	serialPort->print(dataProvider->getTemp2());
	serialPort->print(" GPS speed: ");
	serialPort->print(dataProvider->getGpsGroundSpeed(), 2);
	serialPort->print(" Home alt: ");
	serialPort->print(dataProvider->getAltitude(), 2);
	serialPort->print(" Mode: ");
	serialPort->print(dataProvider->getTemp1());
	serialPort->print(" Course: ");
	serialPort->print(dataProvider->getCourse(), 2);
	serialPort->print(" RPM: ");
	serialPort->print(dataProvider->getEngineSpeed());
	serialPort->print(" AccX: ");
	serialPort->print(dataProvider->getAccX(), 2);
	serialPort->print(" AccY: ");
	serialPort->print(dataProvider->getAccY(), 2);
	serialPort->print(" AccZ: ");
	serialPort->print(dataProvider->getAccZ(), 2);
	serialPort->println("");
}*/
