#include <Encoder.h>
#include <math.h>

//Track encoder pins
const int trackEncA = 2; //pins two and 3 have interupts. Use one per encoder
const int trackEncB = 4;

//Arm encoder pins
const int armEncA = 3;
const int armEncB = 5;

//Encoder directions
const int trackEncDir = 1; //1 for positive, -1 for negative
const int armEncDir = 1;

//Encoder calibrations
const float armEncCali = 360.0/2400;
const float trackEncCali = 1.0;

const int pwm1 = 10;
const int pwm2 = 11;
const int motorDir = 1;

Encoder trackEnc(trackEncA, trackEncB);
Encoder armEnc(armEncA, armEncB);

enum PendulumStates {
  RESTING,
  BALANCING,
  HIT_EDGE,
  E_STOP
};

enum PendulumStates state = RESTING;

void setup() {
  Serial.begin(9600);
  Serial.println("Starting");

  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  
}

float trackEncValue = 0;
float armEncValue = -180;
long startTime = millis();
long startTimeMicros = micros();

int testDir = 1;
void loop() {
  readTrackEncoder();
  readArmEncoder();

  if(millis()-startTime>20){ //50 Hz loop
    int motorSpeed = 0;
    float setpoint = 0;
    int pGain = 160;
    int iGain = 0;
    int dGain = 0;

    float dt = (micros() - startTimeMicros) * 0.000001;
    
    startTime = millis();
    startTimeMicros = micros();
    
    switch(state){
      case RESTING:
        motorSpeed = 0; //don't move
        if(armEncValue < 2 && armEncValue > -2){ //enter balancing mode once arm is up
          state = BALANCING;
        }else {
          state = RESTING;
        }
      break;
      case BALANCING:
        setpoint = -trackEncValue * 0.0003; // try to tilt pendulum to move towards center of track
        
        float error = setpoint - armEncValue; //find error
        motorSpeed = error * pGain; //PID
        motorSpeed = -motorSpeed; //invert
        
        //plot values to serial plotter
        Serial.print(setpoint);
        Serial.print(" ");
        Serial.print(armEncValue);
        Serial.print(" ");
        Serial.println(motorSpeed);
  
        if(armEncValue < 10 && armEncValue > -10){ //stay in balancing state unless arm falls outside safe range
          state = BALANCING;
        }else {
          state = RESTING;
        }
      break;
      case HIT_EDGE:
      break;
      case E_STOP:
      break;
      default:
      break;
    }

    motorSpeed = min(255, max(motorSpeed, -255));
    setMotorSpeed(motorSpeed);
  }
}


//sets motor speed. One PWM for forward, other for reverse
void setMotorSpeed(int power){
  power = power * motorDir;
  if (power >= 0) {
    analogWrite(pwm1, power);
    analogWrite(pwm2, 0);
  }else{
    analogWrite(pwm2, -power);
    analogWrite(pwm1, 0);
  }
}


//reads the track encoder
void readTrackEncoder(){
  float newTrackEncValue = trackEncCali * trackEncDir * trackEnc.read();
  if(newTrackEncValue != trackEncValue){
    trackEncValue = newTrackEncValue;
//    Serial.println(trackEncValue);
  }
}


//reads the arm encoder
long oldArmEncValue = 0; //this one needs a seperate variable to store the old value
//because we use the fmod function to wrap the encoder value
float armEncOffset = -180; //Used to calibrate arm encoder
void readArmEncoder(){
  long newArmEncValue = armEnc.read();
  if(newArmEncValue != oldArmEncValue){
    oldArmEncValue = newArmEncValue;
    
    armEncValue = armEncCali * armEncDir * newArmEncValue + armEncOffset;
    armEncValue = (armEncValue + 180) - floor((armEncValue + 180)/360) * 360 - 180; //stupid mod functions returning - numbers (covert to [-180, 180])
//    Serial.println(armEncValue);
  }
}

void resetArmEncoder(){
  armEncOffset = -180 - armEncValue; //set to -180 (down) because that's where arm naturally lies. 
}

void serialEvent(){
  while(Serial.available()){
    Serial.read(); //clear buffer
  }
  resetArmEncoder(); //reset the encoder
}
