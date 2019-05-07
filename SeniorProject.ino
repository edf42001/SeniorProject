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
const float armEncCali = 360.0/2400; //degrees
const float trackEncCali = 0.001865; //cm

//Initialize encoders
Encoder trackEnc(trackEncA, trackEncB);
Encoder armEnc(armEncA, armEncB);

//motor constants
const int pwm1 = 10;
const int pwm2 = 11;
const int motorDir = 1;

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
float lastError = 0;
float integral = 0;
int trackSide = 0; //which side of the track the cart hit (1 for right, -1 for left)

void loop() {
  readTrackEncoder();
  readArmEncoder();

  if(millis()-startTime>20){ //50 Hz loop
    int motorSpeed = 0;
    float setpoint = 0;

    float dt = (micros() - startTimeMicros) * 0.000001;
    
    startTime = millis();
    startTimeMicros = micros();

    switch(state){
      case RESTING: {
        motorSpeed = 0; //don't move
        if(armEncValue < 2 && armEncValue > -2){ //enter balancing mode once arm is up
          state = BALANCING;
        }else {
          state = RESTING;
        }
      }
      break;
      case BALANCING: {
        int pGain = 50;
        int iGain = 0;
        int dGain = 0;
        
        setpoint = -trackEncValue * 0.0003; // try to tilt pendulum to move towards center of track
        
        float error = setpoint - armEncValue; //find error
        float derivative = (error-lastError) / dt; //calculate derivative of error
        integral += error * dt; //find integral of error
        
        motorSpeed = error * pGain + integral * iGain + derivative * dGain; //PID
        motorSpeed = -motorSpeed; //invert
        lastError = error;
        
        //plot values to serial plotter
        Serial.print(setpoint);
        Serial.print(" ");
        Serial.print(armEncValue);
        Serial.print(" ");
        Serial.println(motorSpeed/20.0); //divide just to make it fit on the graph a bit better
  
        if(trackEncValue > 14 || trackEncValue < -14){
          state = HIT_EDGE;
          trackSide = trackEncValue > 0 ? 1:-1; //set which side the cart hit
        }else if(armEncValue < 20 && armEncValue > -20){ //stay in balancing state unless arm falls outside safe range
          state = BALANCING;
        }else{
          state = RESTING;
        }
      }
      break;
      case HIT_EDGE: {
        int sp = 100;
        if (trackSide > 0) {
          motorSpeed = -sp;
        }else{
          motorSpeed = sp;
        }

        if(trackSide>0 && trackEncValue < 0.02 || trackSide<0 && trackEncValue > -0.02){
          state = RESTING; 
        }else{
          state = HIT_EDGE;
        }
      }
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
  resetArmEncoder(); //reset the arm encoder
}
