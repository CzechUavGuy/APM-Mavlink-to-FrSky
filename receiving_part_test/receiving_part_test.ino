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
This part can serve as a demonstration of what the entire project can do.
Simply upload this to an Arduino and then connect Arduino to ground control station that supports MavLink (MissionPlanner, DroidPlanner, DroidPlanner2, ArduPilot). 
To connect arduino to Android device, make sure you use arduino with FTDI (Uno, Nano), connect through OTG cable. No other HW required.
You should then see an airplane flying above Prague (CZ), banking from left to right, at an altitude 123m, RSSI of 69%, battery 4.2V, speed 10m/s
*/


#include <SoftwareSerial.h>   
#include <FastSerial.h>
#include <Mavlink.h>
#include <GCS_MAVLink.h>

FastSerialPort0(Serial); 
mavlink_message_t msg;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
uint16_t len;
float accx=0.0;
float accy=0.0;
float accz=0.0;
float addx=0.01;
int32_t altitude=123456;    //[mm above sea level]
uint16_t latDEG=0;
uint16_t latMIL=0;
uint16_t latMIC=0;
uint16_t lonDEG=0;
uint16_t lonMIL=0;
uint16_t lonMIC=0;
uint16_t numberOfSatelites=7;
uint16_t gpsStatus=0;
uint16_t apmMode=0;
uint16_t rssi=69;
uint16_t vcc=4200;
int idx=0;
float vgnd=1000.0;    //[cm/s ]

// SoftwareSerial frSkySerial(11, 12, true); // RX, TX, inverted
//gool wasLast7E=false;
//unsigned char ch;
//bool hasFirstByte=false;
//byte firstByte;
//byte secondByte;

void setup() {
  Serial.begin(57600);
//  Serial.begin(115200);
  Serial.flush();  
  lonDEG = 14+180;
  latDEG = 50+90;
  latMIL = 75;
  lonMIL = 470;  
}

void sendAccelerometer() {
//uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed
  mavlink_msg_attitude_pack (100, 200, &msg, 0, accy, accx, accz, 0.0, 0.0, 0.0);
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
  mavlink_msg_global_position_int_pack (100, 200, &msg, 0, lat, lon, altitude, altitude, 0, 0, 0, 0);
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

void sendBatteryVoltage() {
  //uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
//uint32_t onboard_control_sensors_present, uint32_t onboard_control_sensors_enabled, uint32_t onboard_control_sensors_health, uint16_t load, 
//uint16_t voltage_battery, int16_t current_battery, int8_t battery_remaining, uint16_t drop_rate_comm, uint16_t errors_comm, uint16_t errors_count1, 
//uint16_t errors_count2, uint16_t errors_count3, uint16_t errors_count4)
  mavlink_msg_sys_status_pack(100, 200, &msg, 0, 0, 0, 0, vcc, 0, rssi, 0, 0, 0, 0, 0, 0);  //I am sending rssi here because of DroidPlanners location of remaining battery % is exactly the place where I want RSSI to show
  
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

void sendAltitude() {
  //uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, float airspeed, float groundspeed, int16_t heading, uint16_t throttle, float alt, float climb)
  mavlink_msg_vfr_hud_pack (100, 200, &msg, vgnd/100, vgnd/100, 0, 0, altitude/1000, 0);      //sending both speeds the same - So far I can't compute airspeed. What's more, each GCS shows different speed, so its better for them to be the same.
  len = mavlink_msg_to_send_buffer(buf, &msg);
  #ifndef DEBUG
    Serial.write(buf, len);
  #endif
}

void loop() {
  delay(10);
  idx++;
  if ((idx % 5) == 0) {
    sendHeartBeat();
    accy = accy + addx;
    latMIC = latMIC + 7;
    if (accy > 1.0) {addx = 0 - addx;}
    if (accy < -1.0) {addx = 0 - addx;}
    sendAccelerometer();
  }
  if (((idx % 10) == 0) && (idx <= 100)) {sendParam();}  
  if ((idx % 15) == 0) {
    sendGpsCoord();
    sendGpsCoord2();
    sendBatteryVoltage();
    sendRssi();
    sendAltitude();
  }
}

