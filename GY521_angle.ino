#include "GY521.h"
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>
#include <PID_v1.h>

GY521 sensor(0x68);

// Define motor pins
#define PWMA 12    //Motor A PWM
#define DIRA1 34
#define DIRA2 35  //Motor A Direction
#define Encoder1_A 18
#define Encoder2_A 31 //Fuck you for purchasing Chinese hardwares with Pinouts nowhere to be found



#define PWMB 8    //Motor B PWM
#define DIRB1 37
#define DIRB2 36  //Motor B Direction
#define Encoder1_B 19
#define Encoder2_B 38



#define PWMC 9   //Motor C PWM --> from 6 to 9
#define DIRC1 43
#define DIRC2 42  //Motor C Direction
#define Encoder1_C 3
#define Encoder2_C 49


#define PWMD 5    //Motor D PWM
#define DIRD1 A4  //26  
#define DIRD2 A5  //27  //Motor D Direction
#define Encoder1_D 2
#define Encoder2_D A1

#define MOTORA_FORWARD(pwm)    do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,HIGH);analogWrite(PWMA,pwm);}while(0)
#define MOTORA_STOP(x)         do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,LOW); analogWrite(PWMA,0);}while(0)
#define MOTORA_BACKOFF(pwm)    do{digitalWrite(DIRA1,HIGH);digitalWrite(DIRA2,LOW); analogWrite(PWMA,pwm);}while(0)

#define MOTORB_FORWARD(pwm)    do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,HIGH);analogWrite(PWMB,pwm);}while(0)
#define MOTORB_STOP(x)         do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,LOW); analogWrite(PWMB,0);}while(0)
#define MOTORB_BACKOFF(pwm)    do{digitalWrite(DIRB1,HIGH);digitalWrite(DIRB2,LOW); analogWrite(PWMB,pwm);}while(0)

#define MOTORC_FORWARD(pwm)    do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,HIGH);analogWrite(PWMC,pwm);}while(0)
#define MOTORC_STOP(x)         do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,LOW); analogWrite(PWMC,0);}while(0)
#define MOTORC_BACKOFF(pwm)    do{digitalWrite(DIRC1,HIGH);digitalWrite(DIRC2,LOW); analogWrite(PWMC,pwm);}while(0)

#define MOTORD_FORWARD(pwm)    do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,HIGH);analogWrite(PWMD,pwm);}while(0)
#define MOTORD_STOP(x)         do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,LOW); analogWrite(PWMD,0);}while(0)
#define MOTORD_BACKOFF(pwm)    do{digitalWrite(DIRD1,HIGH);digitalWrite(DIRD2,LOW); analogWrite(PWMD,pwm);}while(0)

#define SERIAL  Serial //Why tf would anyone write this?

//PWM Definition
#define MAX_PWM   96 //Don't touch
#define MIN_PWM   64

volatile int encoderCounts[4] = {0}; // Stores counts for M1, M2, M3, M4
double NatOff[4] = {1.05, 1, 1, 1.05};
const int reset[4] = {0,0,0,0};

double SetpointL = 0; //No drift for all
double SetpointH = 0; //No drift for all
double SetpointS = 0; //No drift for all

double yaw, driftx, drifty;
double adjYaw, adjX, adjY;

// Declare PID Controllers

// Positional, 1-st Order Error, P is sufficient (If I remember correctly)
#define KP 0.1
#define TOLERANCE 5  // Allowable diff encoder counts



/*
// Lateral: No front-back drift when L_2/R_2.
double KpL=100, KiL=0, KdL=5;
PID LateralPID(&drifty, &adjY, &SetpointL, KpL, KiL, KdL, DIRECT); //In, Out, Set

// Heading: No side drift when ADV/BACK
double KpH=700, KiH=0, KdH=10; 
PID HeadingPID(&driftx, &adjX, &SetpointH, KpH, KiH, KdH, DIRECT); //In, Out, Set

// Spin: No Spin Whenever
double KpS=5, KiS=0, KdS=1;
PID SpinPID(&yaw, &adjYaw, &SetpointS, KpS, KiS, KdS, DIRECT); //In, Out, Set
*/
int BASE_PWM = 0;


void init_motors(){
  pinMode(Encoder1_A, INPUT_PULLUP);
  pinMode(Encoder2_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Encoder1_A), updateEncoderA, CHANGE);
  pinMode(DIRA1, OUTPUT);
  pinMode(DIRA2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(Encoder1_B, INPUT_PULLUP);
  pinMode(Encoder2_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Encoder1_B), updateEncoderB, CHANGE);
  pinMode(DIRB1, OUTPUT);
  pinMode(DIRB2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(Encoder1_C, INPUT_PULLUP);
  pinMode(Encoder2_C, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Encoder1_C), updateEncoderC, CHANGE);
  pinMode(DIRC1, OUTPUT);
  pinMode(DIRC2, OUTPUT);
  pinMode(PWMC, OUTPUT);

  pinMode(Encoder1_D, INPUT_PULLUP);
  pinMode(Encoder2_D, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Encoder1_D), updateEncoderD, CHANGE);
  pinMode(DIRC1, OUTPUT);
  pinMode(DIRC2, OUTPUT);
  pinMode(PWMC, OUTPUT);
}

// Encoder ISR for Motor 1
void updateEncoderA() {
  if (digitalRead(Encoder1_A) == digitalRead(Encoder2_A)) {
    encoderCounts[0]++; // CW
  } else {
    encoderCounts[0]--; // CCW
  }
}

void updateEncoderB() {
  if (digitalRead(Encoder1_B) == digitalRead(Encoder2_B)) {
    encoderCounts[1]++; // CW
  } else {
    encoderCounts[1]--; // CCW
  }
}

void updateEncoderC() {
  if (digitalRead(Encoder1_C) == digitalRead(Encoder2_C)) {
    encoderCounts[2]++; // CW
  } else {
    encoderCounts[2]--; // CCW
  }
}

void updateEncoderD() {
  if (digitalRead(Encoder1_D) == digitalRead(Encoder2_D)) {
    encoderCounts[3]++; // CW
  } else {
    encoderCounts[3]--; // CCW
  }
}


void updateMotor(int target[4]) {
  int pwm[4] = {0};
  int error[4] = {0};
  bool motorInPosition[4] = {false};
  bool allMotorsInPosition = false;
  for (int i = 0; i < 4; i++) {
    encoderCounts[i] = 0;
      // Calculate error for this motor
      error[i] = (target[i] * NatOff[i] - encoderCounts[i]) ;
      Serial.print(error[i]);
      Serial.print('\t');
      }
  Serial.println("new update>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");

  while (!allMotorsInPosition) {
    allMotorsInPosition = true; // Assume all are in position until proven otherwise
    
    for (int i = 0; i < 4; i++) {
      // Calculate error for this motor
      error[i] = (target[i] * NatOff[i] - encoderCounts[i]) ;
      
      
      // Check if motor is within tolerance
      if (abs(error[i]) <= TOLERANCE) {
        if (!motorInPosition[i]) {
          // Stop motor just reached position
          switch(i) {
            case 0: digitalWrite(DIRA1, LOW); digitalWrite(DIRA2, LOW); break;
            case 1: digitalWrite(DIRB1, LOW); digitalWrite(DIRB2, LOW); break;
            case 2: digitalWrite(DIRC1, LOW); digitalWrite(DIRC2, LOW); break;
            case 3: digitalWrite(DIRD1, LOW); digitalWrite(DIRD2, LOW); break;
          }
          motorInPosition[i] = true;

        }
      } else { //error > Tolerance
        // Motor still needs adjustment
        motorInPosition[i] = false;
        allMotorsInPosition = false;
        
        // Set direction
        if (error[i] > TOLERANCE) {
          switch(i) {
            case 0: digitalWrite(DIRA1, HIGH); digitalWrite(DIRA2, LOW); break;
            case 1: digitalWrite(DIRB1, HIGH); digitalWrite(DIRB2, LOW); break;
            case 2: digitalWrite(DIRC1, HIGH); digitalWrite(DIRC2, LOW); break;
            case 3: digitalWrite(DIRD1, HIGH); digitalWrite(DIRD2, LOW); break;
          }
        } 
        else{
          switch(i) {
          case 0: digitalWrite(DIRA1, LOW); digitalWrite(DIRA2, HIGH); break;
          case 1: digitalWrite(DIRB1, LOW); digitalWrite(DIRB2, HIGH); break;
          case 2: digitalWrite(DIRC1, LOW); digitalWrite(DIRC2, HIGH); break;
          case 3: digitalWrite(DIRD1, LOW); digitalWrite(DIRD2, HIGH); break;
          }
        }
        
        
        // Calculate PWM
        pwm[i] = (int)(KP * abs(error[i]));
        pwm[i] = constrain(pwm[i], MIN_PWM, MAX_PWM);
        
        // Apply PWM
        switch(i) {
          case 0: analogWrite(PWMA, pwm[i]); break;
          case 1: analogWrite(PWMB, pwm[i]); break;
          case 2: analogWrite(PWMC, pwm[i]); break;
          case 3: analogWrite(PWMD, pwm[i]); break;
        }
        
      }
    }

    for (int i = 0; i < 4; i++) {
      Serial.print(error[i]);
      Serial.print('\t');
    }
    Serial.println();
    delayMicroseconds(500000); // Small delay to avoid CPU overload
  }
  
  // Ensure all motors are stopped after loop exits
  digitalWrite(DIRA1, LOW); digitalWrite(DIRA2, LOW);
  analogWrite(PWMA, 0);
  digitalWrite(DIRB1, LOW); digitalWrite(DIRB2, LOW);
  analogWrite(PWMB, 0);
  digitalWrite(DIRC1, LOW); digitalWrite(DIRC2, LOW);
  analogWrite(PWMC, 0);
  digitalWrite(DIRD1, LOW); digitalWrite(DIRD2, LOW);
  analogWrite(PWMD, 0);
  
}



// i'm too tired let's talk about optimization later
void MOTOR_FORWARD(int motorNum, int pwm){
  switch (motorNum){
    case 0:
      MOTORA_FORWARD(pwm);
      break;
    case 1:
      MOTORB_FORWARD(pwm);
      break;
    case 2:
      MOTORC_FORWARD(pwm);
      break;
    case 3:
      MOTORD_FORWARD(pwm);
      break;

  }
  return;
}

void MOTOR_BACKOFF(int motorNum, int pwm){
  switch (motorNum){
    case 0:
      MOTORA_BACKOFF(pwm);
      break;
    case 1:
      MOTORB_BACKOFF(pwm);
      break;
    case 2:
      MOTORC_BACKOFF(pwm);
      break;
    case 3:
      MOTORD_BACKOFF(pwm);
      break;
      
  }
}

void MOTOR_STOP(int motorNum){
  switch (motorNum){
    case 0:
      MOTORA_STOP(0);
    case 1:
      MOTORB_STOP(0);
    case 2:
      MOTORC_STOP(0);
    case 3:
      MOTORD_STOP(0);
  }
}


//    ↑A-----B↑
//     |  ↑  |
//     |  |  |
//    ↑C-----D↑
void BACK(int a,int b,int c,int d)
{
  MOTORA_BACKOFF(BASE_PWM + a); 
  MOTORB_FORWARD(BASE_PWM + b);
  MOTORC_BACKOFF(BASE_PWM + c); 
  MOTORD_FORWARD(BASE_PWM + d);
}

//    ↓A-----B↓
//     |  |  |
//     |  ↓  |
//    ↓C-----D↓
void ADVANCE(int a,int b,int c,int d)
{
  MOTORA_FORWARD(BASE_PWM + a); 
  MOTORB_BACKOFF(BASE_PWM + b);
  MOTORC_FORWARD(BASE_PWM + c); 
  MOTORD_BACKOFF(BASE_PWM + d);
}
//    =A-----B↑
//     |   ↖ |
//     | ↖   |
//    ↑C-----D=
void LEFT_1(int a,int b,int c,int d)
{
  MOTORA_STOP(BASE_PWM + a); 
  MOTORB_FORWARD(BASE_PWM + b);
  MOTORC_BACKOFF(BASE_PWM + c); 
  MOTORD_STOP(BASE_PWM + d);
}

//    ↓A-----B↑
//     |  ←  |
//     |  ←  |
//    ↑C-----D↓
void LEFT_2(int a,int b,int c,int d)
//Fuck their notations.
// See above graph for signs of abcd. BackOff and Foward doesn't really mean anything
{
  MOTORA_BACKOFF(BASE_PWM - a); 
  MOTORB_BACKOFF(BASE_PWM + b);
  MOTORC_FORWARD(BASE_PWM + c); 
  MOTORD_FORWARD(BASE_PWM - d);
}
//    ↓A-----B=
//     | ↙   |
//     |   ↙ |
//    =C-----D↓
void LEFT_3(int a,int b,int c,int d)
{
  MOTORA_FORWARD(BASE_PWM + a); 
  MOTORB_STOP(BASE_PWM + b);
  MOTORC_STOP(BASE_PWM + c); 
  MOTORD_BACKOFF(BASE_PWM + d);
}
//    ↑A-----B=
//     | ↗   |
//     |   ↗ |
//    =C-----D↑
void RIGHT_1(int a,int b,int c,int d)
{
  MOTORA_BACKOFF(BASE_PWM + a); 
  MOTORB_STOP(BASE_PWM + b);
  MOTORC_STOP(BASE_PWM + c); 
  MOTORD_FORWARD(BASE_PWM + d);
}
//    ↑A-----B↓
//     |  →  |
//     |  →  |
//    ↓C-----D↑
void RIGHT_2(int a,int b,int c,int d)
{
  MOTORA_BACKOFF(BASE_PWM + a); 
  MOTORB_BACKOFF(BASE_PWM + b);
  MOTORC_FORWARD(BASE_PWM + c); 
  MOTORD_FORWARD(BASE_PWM + d);
}
//    =A-----B↓
//     |   ↘ |
//     | ↘   |
//    ↓C-----D=
void RIGHT_3(int a,int b,int c,int d)
{
  MOTORA_STOP(BASE_PWM + a); 
  MOTORB_BACKOFF(BASE_PWM + b);
  MOTORC_FORWARD(BASE_PWM + c); 
  MOTORD_STOP(BASE_PWM + d);
}

//    ↑A-----B↓
//     | ↗ ↘ |
//     | ↖ ↙ |
//    ↑C-----D↓
void rotate_1(int a,int b,int c,int d)  //tate_1(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
{
  MOTORA_BACKOFF(BASE_PWM + a); 
  MOTORB_BACKOFF(BASE_PWM + b);
  MOTORC_BACKOFF(BASE_PWM + c); 
  MOTORD_BACKOFF(BASE_PWM + d);
}

//    ↓A-----B↑
//     | ↙ ↖ |
//     | ↘ ↗ |
//    ↓C-----D↑
void rotate_2(int a,int b,int c,int d)  // rotate_2(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
{
  MOTORA_FORWARD(BASE_PWM + a);
  MOTORB_FORWARD(BASE_PWM + b);
  MOTORC_FORWARD(BASE_PWM + c);
  MOTORD_FORWARD(BASE_PWM + d);
}
//    =A-----B=
//     |  =  |
//     |  =  |
//    =C-----D=
void STOP()
{
  MOTORA_STOP(BASE_PWM + Motor_PWM);
  MOTORB_STOP(BASE_PWM + Motor_PWM);
  MOTORC_STOP(BASE_PWM + Motor_PWM);
  MOTORD_STOP(BASE_PWM + Motor_PWM);
}

void LEFT_2()
{
  MOTORA_BACKOFF(BASE_PWM); 
  MOTORB_BACKOFF(BASE_PWM);
  MOTORC_FORWARD(BASE_PWM); 
  MOTORD_FORWARD(BASE_PWM);
}

void left(int n){
  int target[4] = {n, n, -n, -n};
  updateMotor(target);
}

void right(int n){
  int target[4] = {-n, -n, +n, +n};
  updateMotor(target);
}

void forward(int n){
  int target[4] = {-n, +n, -n, +n};
  updateMotor(target);
}

void backward(int n){
  int target[4] = {+n, -n, +n, -n};
  updateMotor(target);
}


void setup()
{
  Serial.begin(115200);
  Serial.println("Init ");
  Serial.println("Init ");
  Serial.println("Init ");
  Serial.println("Init ");
  Serial.println("Init ");
  init_motors();
  delay(2000);
  //Wire.begin();
  /*while (sensor.wakeup() == false)
  {
    Serial.print(millis());
    Serial.println("\tCould not connect to GY521: please check the GY521 address (0x68/0x69)");
    delay(500);
  }
  sensor.calibrate(500);
  sensor.setAccelSensitivity(2);  //  8g
  sensor.setGyroSensitivity(1);   //  500 degrees/s

  sensor.setThrottle();
  sensor.setThrottleTime(25);
  Serial.println("start...");*/


  /*LateralPID.SetMode(AUTOMATIC);
  LateralPID.SetOutputLimits(-255, 255);

  SpinPID.SetMode(AUTOMATIC);
  SpinPID.SetOutputLimits(-255, 255);
  */

}


void loop()
{
  //sensor.read();

  //yaw = sensor.getGyroZ();
  //driftx = sensor.getAccelX();
  //drifty = sensor.getAccelY();
  //float deadbandacc = 0.03;  // Adjust based on sensor noise
  //float deadbandgyro = 2;
  //if (abs(drifty) < deadbandacc) {
    //drifty = 0;  // Treat small errors as zero
  //}
  //if (abs(yaw) < deadbandgyro) {
    //yaw = 0;  // Treat small errors as zero
  //}

  //LateralPID.Compute();
  //SpinPID.Compute();

  //Serial.print(yaw, 2);
  //Serial.print('\t');
  //Serial.print(driftx*10, 1);
  //Serial.print('\t');
  //Serial.print(drifty*10, 2);
  //Serial.print('\t');
  //Serial.print(adjY, 1);
  //Serial.println();
  //Serial.print(yaw, 2);
  //Serial.print('\t');
  //Serial.print(adjYaw, 1);
  //Serial.println();
  //AB
  //CD
  //moveMotorToPosition(1, 0);
  //moveMotorToPosition(2, 0);
  //moveMotorToPosition(3, 0);



  while(1){
    left(1000);
    right(1000);
    forward(1000);
    backward(1000);
    delay(500);
    
  }



  //LEFT_2(adjY+adjYaw,adjY-adjYaw,adjY+adjYaw,adjY-adjYaw);
  delay(100);
}


//  -- END OF FILE --

