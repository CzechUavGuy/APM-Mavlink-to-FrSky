This project is based on https://github.com/vizual54/APM-Mavlink-to-FrSky.
It's modified so that ground station is not supposed to be FrSky FLD-02 or other display, but PC or Tablet/cell phone running Mavlink ground station software (eg. MissionPlanner/DroidPlanner/Andropilot). Two Arduinos are needed to make this project work.

It consists of two parts: Sending part that is on the plane and Receiving part that is connected to ground station.

Currently, only one-way transmission is supported, I am not sure why I can't manage to uplink data back to airplane using FrSky telemetry. Maybe it is not possible at all using FrSky.

Sending part receives telemetry stream from APM device using telemetry UART with speed 57600bps. Arduino caches state of APM (orientation, GPS coordinates, speed, altitude, ...) and sends those data down over FrSky downlink which has speed only 9600bps, but effectively manages to transmit only approximately 2400bps. Therefore, telemetry has to be filtered and compressed a lot. However I have managed to prioritize data, so that e.g. orientation is send *4* times per second, whereas GPS coordinates only once per second.

Mediating protocol is my own proprietary using two byte telemetry. 1 bit of each byte marks first byte or second byte. 4 bits mark message type. 10 bits left to actual data. See defines.h

Receiving part receives data and after each two byte packet it generates MavLink message that is immediately sent to Ground Station software over Arduino hardware UART.