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



/*
How to set-up:
Connect RX wire of FrSky module in transmitter to D11 (see frSkySerial initialization, it may change). 
Ideally connect it through 2K resistor. 
Also connect GND from module to Arduino.
Depending on whether you get 5V from ground station hardware, connect it. If you have 5V from ground station like a computer or powered OTG, do not connect it!

How to debug:
Instead of connecting to Mavlink based ground station, connect it to computer via hardware Serial, open serial console and see output that would be sent to ground station in human readable format.
*/
#include <SoftwareSerial.h>
#include <FastSerial.h>    //mavlink requires this
#include "Mavlink.h"
#include <GCS_MAVLink.h>
#include "defines.h"

FastSerialPort0(Serial);    //I don't know why this is here. Serial port does not work without it
mavlink_message_t msg;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
uint16_t len;

float accx=0.0;
float accy=0.0;
float accz=0.0;
int32_t altitude=0;    //mm above sea level
uint16_t latDEG=0;
uint16_t latMIL=0;
uint16_t latMIC=0;
uint16_t lonDEG=0;
uint16_t lonMIL=0;
uint16_t lonMIC=0;
uint16_t numberOfSatelites=0;
uint16_t gpsStatus=0;
uint16_t apmMode=0;
uint16_t rssi=0;
uint16_t vcc=0;    //battery voltage
int idx=0;
float vgnd=0.0;    //cm/s

SoftwareSerial frSkySerial(11, 9, true); // RX, TX, inverted. Only RX used now.
bool wasLast7E=false;
unsigned char ch;
bool hasFirstByte=false;
byte firstByte;
byte secondByte;

//#define DEBUG
#ifdef DEBUG
  #define DEBUG_PRINTLN(x)  Serial.println (x)
  #define DEBUG_PRINT(x)  Serial.print (x)
  #define DEBUG_PRINT2(x, y)  Serial.print (x, y)
  #define DEBUG_PRINT2LN(x, y)  Serial.println (x, y)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINT2(x, y)
  #define DEBUG_PRINT2LN(x, y)
#endif

unsigned char primReadFromFrSkySerial() {
  while(!frSkySerial.available()) ;
  return frSkySerial.read();
}

unsigned char readFromFrSkySerial() {
  ch = primReadFromFrSkySerial();
  if (ch == 0x7D) {
    ch = primReadFromFrSkySerial();
    ch = ch ^ 0x20;
  }
  return ch;
}

bool isFirstByte(byte data) {return data >= 0x80;}
bool isSecondByte(byte data) {return data < 0x80;}

void readUserFrame() {
  DEBUG_PRINT("UD: ");
  byte len = readFromFrSkySerial();
  if (len > 10) {DEBUG_PRINTLN("LEN>10");return;} else {DEBUG_PRINT("LEN="); DEBUG_PRINT2(len, DEC); DEBUG_PRINT(" ");}
  readFromFrSkySerial(); //skip one byte
  for (int i = 0; i < 6; i++) {  //there are always 6 bytes, but only <len> of them contain useful data
    ch = readFromFrSkySerial();
    if (i < len) {
      DEBUG_PRINT2(ch, HEX); DEBUG_PRINT(" ");
      if (hasFirstByte) {
        if (isSecondByte(ch)) {
          secondByte = ch; 
          decodeBytes();
          hasFirstByte=false;
        } else {if (isFirstByte(ch)) {firstByte = ch; hasFirstByte = true;}}
      } else    if (isFirstByte(ch)) {firstByte = ch; hasFirstByte = true;}
    }
  }
  ch = readFromFrSkySerial();  //read packet tail, should be 0x7E
  if (ch != 0x7E) {DEBUG_PRINT(" KO: "); DEBUG_PRINT2LN(ch, HEX);} else {DEBUG_PRINTLN(" ");}
}

void readInternalFrame() {
  DEBUG_PRINT("ID: ");
  byte nr = readFromFrSkySerial();
  DEBUG_PRINT("AD1: "); DEBUG_PRINT2(nr, HEX);
  nr = readFromFrSkySerial();    //so far I do not have any use for AD1 value. Can be easily fixed.
  DEBUG_PRINT(" AD2: "); DEBUG_PRINT2(nr, HEX);
  vcc = (uint16_t)(25.87 * nr);
  nr = readFromFrSkySerial();
  DEBUG_PRINT(" RSSI: "); DEBUG_PRINT2(nr, HEX);
  rssi = nr;
  sendRssi();
  sendBatteryVoltage();
  for (int i = 0; i < 5; i++) {ch=readFromFrSkySerial();}  //ignored bytes
  ch = readFromFrSkySerial();
  if (ch != 0x7E) {DEBUG_PRINT(" KO: "); DEBUG_PRINT2LN(ch, HEX);} else {DEBUG_PRINTLN(" ");}
}

void decodeBytes() {
  uint8_t type = (firstByte & 0b01111000) >> 3;
  uint16_t data = (firstByte & 0b00000111);
  data = data << 7;
  data = data + secondByte;
  switch(type) {
    case karelACCX : {accx = ((int)data - 180) * 3.14159265 / 180.0; DEBUG_PRINT(" ACCX: "); DEBUG_PRINT2LN(accx, 3); sendAccelerometer(); break; }
    case karelACCY : {accy = ((int)data - 180) * 3.14159265 / 180.0; DEBUG_PRINT(" ACCY: "); DEBUG_PRINT2LN(accy, 3); sendAccelerometer(); break; }
    case karelACCZ : {accz = ((int)data - 180) * 3.14159265 / 180.0; DEBUG_PRINT(" ACCZ: "); DEBUG_PRINT2LN(accz, 3); sendAccelerometer(); break; }
/*    case karelACCXY : {
      accy = (((int) (data >> 4)) * 6 - 180) * 3.14159265 / 180.0; DEBUG_PRINT(" ACCY: "); DEBUG_PRINT2LN(accy, 3);
      byte index = (byte) (data & 0b00001111);
      const int8_t pitchLimits[16] = {-90, -70, -50, -30, -20, -10, -5, 0, 5, 10, 15, 20, 30, 50, 70, 90};
      accx = pitchLimits[index];
      accx = accx * 3.14159265 / 180.0; DEBUG_PRINT(" ACCX: "); DEBUG_PRINT2LN(accx, 3);
      sendAccelerometer(); break; 
    }*/
    case karelALT    : {altitude = ((int32_t)data) * 2000; DEBUG_PRINT(" ALT: "); DEBUG_PRINT2LN(altitude, DEC); sendAltitude(); break; }
    case karelLatDEG : {latDEG = data; DEBUG_PRINT(" latDEG: "); DEBUG_PRINT2LN(data, DEC); sendGpsCoord(); break;}
    case karelLatMIL : {latMIL = data; DEBUG_PRINT(" latMIL: "); DEBUG_PRINT2LN(data, DEC); sendGpsCoord2(); break;}
    case karelLatMIC : {latMIC = data; DEBUG_PRINT(" latMIC: "); DEBUG_PRINT2LN(data, DEC); sendGpsCoord(); break;}
    case karelLonDEG : {lonDEG = data; DEBUG_PRINT(" lonDEG: "); DEBUG_PRINT2LN(data, DEC); sendGpsCoord2(); break;}
    case karelLonMIL : {lonMIL = data; DEBUG_PRINT(" lonMIL: "); DEBUG_PRINT2LN(data, DEC); sendGpsCoord(); break;}
    case karelLonMIC : {lonMIC = data; DEBUG_PRINT(" lonMIC: "); DEBUG_PRINT2LN(data, DEC); sendGpsCoord2(); break;}
    case karelVGND   : {vgnd = (float)data; DEBUG_PRINT(" vGND: "); DEBUG_PRINT2LN(vgnd, DEC); sendAltitude(); break;}
    case karelSTAT   : {
      gpsStatus = data >> 8;
      numberOfSatelites = (data >> 4) & 0x0F;
      apmMode = data & 0x0F;
      DEBUG_PRINT(" gpsStatus: "); DEBUG_PRINT2LN(gpsStatus, DEC); 
      DEBUG_PRINT(" numberOfSatelites: "); DEBUG_PRINT2LN(numberOfSatelites, DEC); 
      DEBUG_PRINT(" apmMode: "); DEBUG_PRINT2LN(apmMode, DEC); 
      break;
    }
    default:
    {
      DEBUG_PRINT("Unknown FrSky data: "); DEBUG_PRINT2(firstByte, HEX); DEBUG_PRINT2(secondByte, HEX); DEBUG_PRINT(" "); DEBUG_PRINTLN(type);
    } 
  }
}

void setup() {
  Serial.begin(57600);
//  Serial.begin(115200);
  Serial.flush();  
  frSkySerial.begin(9600); 
}

void sendAccelerometer() {
//uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed
  mavlink_msg_attitude_pack (100, 200, &msg, 0, accy, accx, accz, 0.0, 0.0, 0.0);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  #ifndef DEBUG
    Serial.write(buf, len); 
  #endif
}

void sendRssi() {
//uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,uint8_t rssi, uint8_t remrssi, uint8_t txbuf, uint8_t noise, uint8_t remnoise, uint16_t rxerrors, uint16_t fixed
  mavlink_msg_radio_pack (100, 200, &msg, rssi, rssi, 0, 0, 0, 0, 0);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  #ifndef DEBUG
    Serial.write(buf, len); 
  #endif
}

void sendBatteryVoltage() {
  mavlink_msg_sys_status_pack(100, 200, &msg, 0, 0, 0, 0, vcc, 0, rssi, 0, 0, 0, 0, 0, 0);  //I am sending rssi here because of DroidPlanners location of remaining battery % is exactly the place where I want RSSI to show
  len = mavlink_msg_to_send_buffer(buf, &msg);
  #ifndef DEBUG
    Serial.write(buf, len); 
  #endif
}

void sendGpsCoord() {
  int32_t lon = (lonDEG * 10000000) + ((uint32_t) lonMIL * 10000) + (lonMIC * 10);
  lon = lon - 1800000000;
  int32_t lat = (latDEG * 10000000) + ((uint32_t)   latMIL * 10000) + (latMIC * 10);
  lat = lat - 900000000;
//uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible
  mavlink_msg_gps_raw_int_pack (100, 200, &msg, 0, gpsStatus, lat, lon, altitude, 0, 0, vgnd, 0, numberOfSatelites);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  #ifndef DEBUG
    Serial.write(buf, len);  
  #endif
}

void sendGpsCoord2() {
  int32_t lon = (lonDEG * 10000000) + ((uint32_t) lonMIL * 10000) + (lonMIC * 10);
  lon = lon - 1800000000;
  int32_t lat = (latDEG * 10000000) + ((uint32_t)   latMIL * 10000) + (latMIC * 10);
  lat = lat - 900000000;
//uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg
  mavlink_msg_global_position_int_pack (100, 200, &msg, 0, lat, lon, altitude, altitude, 0, 0, 0, 0);    //sending both altitudes the same - So far I can't compute relative alt. What's more, each GCS shows different alt, so its better for them to be the same.
  len = mavlink_msg_to_send_buffer(buf, &msg);
  #ifndef DEBUG
    Serial.write(buf, len);  
  #endif
}

void sendAltitude() {
  //uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, float airspeed, float groundspeed, int16_t heading, uint16_t throttle, float alt, float climb)
  mavlink_msg_vfr_hud_pack (100, 200, &msg, vgnd/100, vgnd/100, 0, 0, altitude/1000, 0);    //sending both speeds the same - So far I can't compute airspeed. What's more, each GCS shows different speed, so its better for them to be the same.
  len = mavlink_msg_to_send_buffer(buf, &msg);
  #ifndef DEBUG
    Serial.write(buf, len);
  #endif
}

void sendHeartBeat() {
//uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status
  mavlink_msg_heartbeat_pack (100, 200, &msg, 1, 1, 1, apmMode, 1);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  #ifndef DEBUG
    Serial.write(buf, len); 
  #endif
  #ifdef DEBUG
  Serial.print("HBEAT: ");
    for(int i=0; i<len; i++) {
      Serial.print(buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  #endif
}
void sendParam() {
//uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const char *param_id, float param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index
  mavlink_msg_param_value_pack (100, 200, &msg, "TKOFF_FLAP_PCNT", 0.0, 2, 1, 1);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  #ifndef DEBUG
    Serial.write(buf, len); 
  #endif
}

void loop() {
  if (frSkySerial.available()) {
    bool ok=false;
    ch = frSkySerial.read();
    if ((ch == 0xFE) && (wasLast7E)) {readInternalFrame(); ok = true;}
    if ((ch == 0xFD) && (wasLast7E)) {readUserFrame(); ok = true;}
    if (ch == 0x7E) {wasLast7E = true; ok = true;} else {wasLast7E = false;}
    if (!ok) {DEBUG_PRINT("??? "); DEBUG_PRINT2LN(ch, HEX);}
    idx++;
    if ((idx % 2) == 0) {sendHeartBeat();}  //I don't know where and when to send heartbeat. Maybe here is a good place? 
    if (((idx % 10) == 0) && (idx <= 100)) {sendParam();}  //HACK: Mission planner needs some parameters during initialization. This is a way to convince him he is talking to real Mavlink. There is a workaround, you can press a key combination to connect in readonly mode. Droid planner does not need this.
  }
}
