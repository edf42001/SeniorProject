#include <Encoder.h>
#include <math.h>
#include <StateSpaceControl.h>
#include <BasicLinearAlgebra.h>

//Track encoder pins
const int trackEncA = 2; //pins two and 3 have interupts. Use one per encoder
const int trackEncB = 4;

//Arm encoder pins
const int armEncA = 3;
const int armEncB = 5;

//Encoder directions
const int trackEncDir = -1; //1 for positive, -1 for negative
const int armEncDir = 1;

//Encoder calibrations
const float armEncCali = 2*PI/2400; //radians
const float trackEncCali = 0.00005045; //meters

//limit switch pin
const int switchPin = 7;

//Initialize encoders
Encoder trackEnc(trackEncA, trackEncB);
Encoder armEnc(armEncA, armEncB);

// old motor constants
//const int pwm1 = 10;
//const int pwm2 = 11;
//const int motorDir = -1;

// new motor constants
const int pwm1 = 11;
const int inA = 10;
const int inB = 12;
const int motorDir = 1;

enum PendulumStates {
  RESTING,
  BALANCING,
  HIT_EDGE,
  E_STOP,
  CALIBRATE,
  SWING_UP
};

enum PendulumStates state = RESTING;
enum PendulumStates lastState = state;

//Model<4,1,4> model;
//StateSpaceController<4,1,4, false, true> controller(model);

Model<4,1,2> model;
StateSpaceController<4,1,2, true, true> controller(model);

void setup() {
  TCCR2B = TCCR2B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz
  Serial.begin(115200);
  Serial.println("Starting");

//  pinMode(pwm1, OUTPUT);
//  pinMode(pwm2, OUTPUT);

    pinMode(pwm1, OUTPUT);
    pinMode(inA, OUTPUT);
    pinMode(inB, OUTPUT);

    pinMode(switchPin, INPUT_PULLUP);

    model.A <<0,1,0,0,
              37.692,-.1,0,0,
              0,0,0,1,
              0,0,0,0;
    model.B <<0,
              -3.846,
              0,
              1;
//    model.C <<1,0,0,0,
//              0,1,0,0,
//              0,0,1,0,
//              0,0,0,1;
//    model.D <<0,
//              0,
//              0,
//              0;
    model.C <<1,0,0,0,
              0,0,1,0,
    model.D <<0,
              0;
      
    controller.K << -86.08,-13.03,-44.7,-26.4;
    controller.L <<  112.63,  0,
                     4551, 0,
                     0,  72.5,
                     0,  2628;
    controller.I << 0, 0;
    controller.initialise();
    controller.r << 0,
                    0;
}

float trackEncValue = 0;
float lastTrackEncValue = trackEncValue;
float armEncValue = -PI;
float lastArmEncValue = armEncValue;

long startTime = millis();
long startTimeMicros = micros();
int trackSide = 0; //which side of the track the cart hit (1 for right, -1 for left)
bool stateChanged = false;

bool switchPressed = false;
bool switchWasPressed = false;

float motorAccel = 0;
float motorVel = 0;
int i = 0;

int swingPulseStart = 0;
int pulseDirection = 0;
float lastPendulumSpeed = 0;
float amplitude = 0;

float targetCartPos = 0;
void loop() {
  readTrackEncoder();
  readArmEncoder();

  if(millis()-startTime>10){ //100 Hz loop
    int motorSpeed = 0;
    switchPressed = !digitalRead(switchPin);
    float dt = (micros() - startTimeMicros) * 0.000001;
    
    startTime = millis();
    startTimeMicros = micros();

    float cartSpeed = (trackEncValue - lastTrackEncValue) / dt;
    float pendulumSpeed = angleDiff(armEncValue, lastArmEncValue) / dt;

    i++;
    
//    Serial.print(trackEncValue, 4);
//    Serial.print(" ");
//    Serial.print(armEncValue, 4);
//    Serial.print(" ");
//    Serial.print(cartSpeed,4);
//    Serial.print(" ");
//    Serial.println(pendulumSpeed,4);
  
    switch(state){
      case RESTING: {
        motorSpeed = 0; //don't move
        if(armEncValue < 0.033 && armEncValue > -0.033){ //enter balancing mode once arm is up
          state = BALANCING;
        }else {
          state = RESTING;
        }
//        if(switchPressed){
//          state = SWING_UP;
//        }
      }
      break;
      case BALANCING: {
        Matrix<2,1> y;
        y << armEncValue,
             trackEncValue;
//     Matrix<4,1> y;
//     y << armEncValue,
//               pendulumSpeed,
//               trackEncValue,
//               cartSpeed;
        controller.update(y, dt);
        motorAccel = controller.u(0,0);

        if(stateChanged){ //zero vel upon initialization into this state
          motorVel = 0;
        }
        motorVel += motorAccel * dt;

        motorSpeed = motorSpeedClosedLoop(motorVel, motorAccel, cartSpeed, 650, 8, 500);    
        motorSpeed = min(255, max(motorSpeed, -255)); //constrain
        
//        if(i%500 == 0){
//          if(i%1000 == 0){
//            controller.r(1, 0) = 0.06;
//          }else{
//            controller.r(1, 0) = -0.06;
//          }
//        }

        //plot values to serial plotter
         Serial.print(controller.x_hat(3,0)*100);
         Serial.print(" ");
         Serial.print(cartSpeed*100);
         Serial.print(" ");
         Serial.print(controller.x_hat(1,0)*100);
         Serial.print(" ");
         Serial.println(pendulumSpeed*100);
//        Serial.print(motorVel*100);
//        Serial.print(" ");
//        Serial.println(cartSpeed*100);
//        Serial.print(" ");
//        Serial.println(motorSpeed/50.0); //divide just to make it fit on the graph a bit better
  
        if(trackEncValue > 0.14 || trackEncValue < -0.15){ //they're not symmetric because things aren't symmetric
          state = HIT_EDGE;
          trackSide = trackEncValue > 0 ? 1:-1; //set which side the cart hit
        }else if(armEncValue < 1 && armEncValue > -1){ //stay in balancing state unless arm falls outside safe range
          state = BALANCING;
        }else{
          state = RESTING; //stop
        }
      }
      break;
      case HIT_EDGE: {
        int sp = 90;
        if (trackSide > 0) {
          motorSpeed = -sp;
        }else{
          motorSpeed = sp;
        }

        if(trackSide>0 && trackEncValue < 0.0002 || trackSide<0 && trackEncValue > -0.0002){
          state = RESTING; 
        }else{
          state = HIT_EDGE;
        }
      }
      break;
      case CALIBRATE: {
        if(!switchWasPressed){
          motorSpeed = 40;
          if(switchPressed){
            resetTrackEnc();
            motorSpeed = 0;
            switchWasPressed = true;
          }
        }else{
          if(trackEncValue < 0.0005){
            motorSpeed = 0;
            state = RESTING;
            switchWasPressed = false;
          }else{
            motorSpeed = -90;
          }
        }
      }
      break;
      case SWING_UP: {
        float m = 0.1;  //mass of pendulum
        float l = .39;   //length of pendulum
        float g = 9.8;  //gravity
        float I = 1.0/3.0 * m * l*l;  //moment of inertia of pendulum

        float max_energy = 0.5 * l * m * g * 1.03;
        float energy = 0.5 * l * m * g * cos(armEncValue) + 0.5 * I * pendulumSpeed*pendulumSpeed;

        if(stateChanged){
          motorVel = 0;
          pulseDirection = 0;
          amplitude = 0;
        }
        
//          float motorAccel = -2.5*sign(max_energy-energy)*sign(cos(armEncValue)) * sign(pendulumSpeed) - 5*cartSpeed-4*trackEncValue;
//        float r = 2.1; //cart position weighting factor
//        float uA = 1.5; //max cart accel wanted
//        float epsilon = 0.001; //numbers too small value
//        float thingy = (sign(energy-max_energy) * pendulumSpeed * cos(armEncValue) - 2 * r * cartSpeed);
//        float numerator = uA * abs(thingy) + 2 * r * trackEncValue * cartSpeed;
//        Serial.println(numerator,4);
//        float denominator;
//        if(abs(thingy)>epsilon){
//          denominator = thingy;
//        }else if(thingy!=0){
//          denominator = epsilon*sign(thingy);
//        }else{
//          denominator = 1;
//        }
//        float motorAccel = numerator/denominator;

//        motorVel += motorAccel * dt;
        float best_angle = abs(amplitude) * 0.5;
        if((sign(angleDiff(armEncValue, amplitude))!=sign(angleDiff(lastArmEncValue, amplitude)) || 
            sign(angleDiff(armEncValue, -amplitude))!=sign(angleDiff(lastArmEncValue, -amplitude))) && pulseDirection == 0){
          swingPulseStart = i;
          if(sign(pendulumSpeed) < 0){
            pulseDirection = -1;
          }else{
            pulseDirection = 1;
          }
        }

        if(sign(pendulumSpeed)!=sign(lastPendulumSpeed)){
          amplitude = angleDiff(PI, armEncValue);
        }

        Serial.println(amplitude);
//
        if(i - swingPulseStart < 50){
          motorAccel = pulseDirection * 0.4;
          motorVel += motorAccel * dt;
        }else{
          motorVel = 0;
          pulseDirection = 0;
          motorAccel = 0;
        }

        
        motorSpeed = motorSpeedClosedLoop(motorVel, motorAccel, cartSpeed, 650, 8, 500);    
        motorSpeed = min(255, max(motorSpeed, -255)); //constrain
        // Serial.print(motorVel*100);
        // Serial.print(" ");
        // Serial.println(cartSpeed*100);
//          Serial.print(angleDiff(armEncValue,-PI));
//          Serial.print(" ");
//          Serial.println(trackEncValue);
          
        
        if(trackEncValue > 0.14 || trackEncValue < -0.15){ //they're not symmetric because things aren't symmetric
           state = HIT_EDGE;
           trackSide = trackEncValue > 0 ? 1:-1; //set which side the cart hit
           motorSpeed = 0;
         }else if(abs(armEncValue) < 0.04){ //enter balancing state once arm in safe range
           state = BALANCING;
         }else{
           state = SWING_UP; //stop
         }

      }
      break;
    }
    
    setMotorSpeed(motorSpeed);
    if(state!=lastState){
      stateChanged = true;
    }else{
      stateChanged = false;
    }
    lastState = state;

    lastTrackEncValue = trackEncValue;
    lastArmEncValue = armEncValue;
    lastPendulumSpeed = pendulumSpeed;
  }
}


//sets motor speed. One PWM for forward, other for reverse
//void setMotorSpeed(int power){
//  power = power * motorDir;
//  if (power >= 0) {
//    analogWrite(pwm1, power);
//    analogWrite(pwm2, 0);
//  }else{
//    analogWrite(pwm2, -power);
//    analogWrite(pwm1, 0);
//  }
//}

float motorSpeedClosedLoop(float targetVel, float targetAccel, float actualVel, float FFV, float FFA, float kP){
  float sp = targetVel * FFV + targetAccel * FFA + (targetVel-actualVel)*kP;
  return sp;
}

// for new controller
void setMotorSpeed(int power){
  power = power * motorDir;
  if (power > 0) {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
  }else if(power<0){
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
  }else{
    digitalWrite(inA, HIGH);
    digitalWrite(inB, HIGH);
  }
  analogWrite(pwm1, abs(power));
}


//reads the track encoder
float trackEncOffset = 0;
void readTrackEncoder(){
  float newTrackEncValue = trackEncCali * trackEncDir * trackEnc.read() + trackEncOffset;
  if(newTrackEncValue != trackEncValue){
    trackEncValue = newTrackEncValue;
//    Serial.println(trackEncValue,4);
  }
}

//this function will recalbirate the track enc when the cart hits the limit switch
float switchPosition = 0.1650;
void resetTrackEnc(){
  trackEncOffset = switchPosition - trackEncValue;
}

//reads the arm encoder
long oldArmEncValue = 0; //this one needs a seperate variable to store the old value
//because we use the fmod function to wrap the encoder value
float armEncOffset = -PI; //Used to calibrate arm encoder
void readArmEncoder(){
  long newArmEncValue = armEnc.read();
  if(newArmEncValue != oldArmEncValue){
    oldArmEncValue = newArmEncValue;
    
    armEncValue = armEncCali * armEncDir * newArmEncValue + armEncOffset;
    armEncValue = angleDiff(armEncValue, 0);
    // Serial.println(armEncValue,4);
  }
}

void resetArmEncoder(){
  armEncOffset = -PI - armEncValue; //set to -180 (down) because that's where arm naturally lies. 
}

void serialEvent(){
  while(Serial.available()){
    Serial.read(); //clear buffer
  }
  resetArmEncoder(); //reset the arm encoder
}

int sign(float val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}

float angleDiff(float a, float b){
  return (a-b + PI) - floor((a-b + PI)/(2*PI)) * (2*PI) - PI; //stupid mod functions returning - numbers (covert to [-180, 180])
}
