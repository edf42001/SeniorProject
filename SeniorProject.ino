#include <Encoder.h>

const int trackEncA = 2; //pins two and 3 have interupts. Use one per encoder
const int trackEncB = 4;

const int armEncA = 3;
const int armEncB = 5;

Encoder trackEnc(trackEncA, trackEncB);
Encoder armEnc(armEncA, armEncB);

void setup() {
  Serial.begin(9600);
  Serial.println("Hello");
  
}

long trackEncValue = 0;
long armEncValue = 0;
long startTime = millis();

void loop() {
  long newTrackEncValue = trackEnc.read();
  if(newTrackEncValue != trackEncValue){
    trackEncValue = newTrackEncValue;
    Serial.println(trackEncValue);
  }
  long newArmEncValue = armEnc.read();
  if(newArmEncValue != armEncValue){
    armEncValue = newArmEncValue;
    Serial.println(armEncValue);
  }
}
