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