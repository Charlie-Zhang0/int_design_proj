#include "GY521.h"
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>
#include <PID_v1.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

int L_cal;
int R_cal;
int L_brightness;
int R_brightness;

bool continueRight = 1;
bool boxOnRight = 0;
GY521 sensor(0x68);

int clkw=0;
int anticlkw=0;

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

#define echoPinR 32
#define trigPinR 30
#define echoPinL 29
#define trigPinL 33
#define trigPinT 25
#define echoPinT 28

bool stage1 = 1; 
bool stage2 = 0; 
bool stage3 = 0; 
bool stage4 = 0; 
bool flag4a =0;  
bool flag4b =0;  
bool stage5 = 0; 
bool stage6 =0;  
bool stage7 =0;  
bool stage8=0;   
bool stage9=0;
bool stage10=0;

float distL=0; //measurement at the begining of each loop
float distR=0;
float distT;

float prev_distL=0; //measurement of previous loop, updated at the end of each loop
float prev_distR=0;
float prev_distT=0;

float stg2_base;  //to store the dist for comparison
float stg4_base;
float stg3_base;
float stg6_timer=0; //for timeout if box not found



// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     28 //4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
float oldV=1, newV=0;
#include <SoftwareSerial.h>
//UNO: (2, 3)
//SoftwareSerial mySerial(4, 6); // RX, TX
int pan = 90;
int tilt = 120;
int window_size = 0;
int BT_alive_cnt = 0;
int voltCount = 0;
#include <Servo.h>
Servo servo_pan;
Servo servo_tilt;
int servo_min = 20;
int servo_max = 160;

unsigned long time;

//FaBoPWM faboPWM;
int pos = 0;
int MAX_VALUE = 2000;
int MIN_VALUE = 300;


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
#define BTSERIAL Serial3

#define LOG_DEBUG

#ifdef LOG_DEBUG
  #define M_LOG SERIAL.print
#else
  #define M_LOG BTSERIAL.println
#endif

void UART_Control()
{
  String myString;
  char BT_Data = 0;
  // USB data
  /****
   * Check if USB Serial data contain brackets
   */

  if (SERIAL.available())
  {
    char inputChar = SERIAL.read();
    if (inputChar == '(') { // Start loop when left bracket detected
      myString = "";
      inputChar = SERIAL.read();
      while (inputChar != ')')
      {
        myString = myString + inputChar;
        inputChar = SERIAL.read();
        if (!SERIAL.available()) {
          break;
        }// Break when bracket closed
      }
    }
    int commaIndex = myString.indexOf(','); //Split data in bracket (a, b, c)
    //Search for the next comma just after the first
    int secondCommaIndex = myString.indexOf(',', commaIndex + 1);
    String firstValue = myString.substring(0, commaIndex);
    String secondValue = myString.substring(commaIndex + 1, secondCommaIndex);
    String thirdValue = myString.substring(secondCommaIndex + 1); // To the end of the string
    if ((firstValue.toInt() > servo_min and firstValue.toInt() < servo_max) and  //Convert them to numbers
        (secondValue.toInt() > servo_min and secondValue.toInt() < servo_max)) {
      pan = firstValue.toInt();
      tilt = secondValue.toInt();
      window_size = thirdValue.toInt();
    }
    SERIAL.flush();
    Serial3.println(myString);
    Serial3.println("Done");
    if (myString != "") {
      display.clearDisplay();
      display.setCursor(0, 0);     // Start at top-left corner
      display.println("Serial_Data = ");
      display.println(myString);
      display.display();
    }
  }

  //BT Control
  /*
    Receive data from app and translate it to motor movements
  */
  // BT Module on Serial 3 (D14 & D15)
  if (Serial3.available())
  {
    BT_Data = Serial3.read();
    SERIAL.print(BT_Data);
    Serial3.flush();
    BT_alive_cnt = 100;
    display.clearDisplay();
    display.setCursor(0, 0);     // Start at top-left corner
    display.println("BT_Data = ");
    display.println(BT_Data);
    display.display();
  }

  BT_alive_cnt = BT_alive_cnt - 1;
  if (BT_alive_cnt <= 0) {
    STOP();
  }
  switch (BT_Data)
  {
    //case 'A':  forwardE();  M_LOG("Run!\r\n"); break;
    //case 'B':  RIGHT_2();  M_LOG("Right up!\r\n");     break;
    //case 'C':  rotate_1();                            break;
    //case 'D':  RIGHT_3();  M_LOG("Right down!\r\n");   break;
    //case 'E':  BACK(500, 500, 500, 500);     M_LOG("Run!\r\n");          break;
    //case 'F':  LEFT_3();   M_LOG("Left down!\r\n");    break;
    //case 'G':  rotate_2();                              break;
    //case 'H':  LEFT_2();   M_LOG("Left up!\r\n");     break;
    //case 'Z':  STOP();     M_LOG("Stop!\r\n");        break;
    //case 'z':  STOP();     M_LOG("Stop!\r\n");        break;
    //case 'd':  LEFT_2();   M_LOG("Left!\r\n");        break;
    //case 'b':  RIGHT_2();  M_LOG("Right!\r\n");        break;
    //case 'L':  Motor_PWM = 1500;                      break;
    //case 'M':  Motor_PWM = 500;                       break;
  }
}

void sendVolt(){
    newV = analogRead(A0)*5.0 / 1023.0; // receivingCoil +VE
    if(newV!=oldV) {
      if (!Serial3.available()) {
        Serial3.println(newV);
        Serial.println("Volt: " + String(newV));
      }
    }
    oldV=newV;
}

float measure_distanceL() {
  long duration;
  float distance_in_cm;
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
 
  digitalWrite(trigPinL, LOW); 
  delayMicroseconds(2); 
  digitalWrite(trigPinL, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPinL, LOW);
 
  duration = pulseIn(echoPinL, HIGH);

  distance_in_cm = (duration/2.0) / 29.1;

  return distance_in_cm + 0.5;

}

float measure_distanceR() {
  long duration;
  float distance_in_cm;
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
 
  digitalWrite(trigPinR, LOW); 
  delayMicroseconds(2); 
  digitalWrite(trigPinR, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPinR, LOW);
 
  duration = pulseIn(echoPinR, HIGH);
  distance_in_cm = (duration/2.0) / 29.1;

  return distance_in_cm;
}

float measure_distanceT() {
  long duration;
  float distance_in_cm;
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
 
  digitalWrite(trigPinT, LOW); 
  delayMicroseconds(2); 
  digitalWrite(trigPinT, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPinT, LOW);
 
  duration = pulseIn(echoPinT, HIGH);
  distance_in_cm = (duration/2.0) / 29.1;

  return distance_in_cm;
}


//PWM Definition
#define MAX_PWM   96 //Don't touch
#define MIN_PWM   64

volatile int encoderCounts[4] = {0}; // Stores counts for M1, M2, M3, M4
double NatOff[4] = {1, 0.85, 0.85, 1};
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
      //Serial.print(error[i]);
      //Serial.print('\t');
      }
  //Serial.println("new update>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");

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
      //Serial.print(error[i]);
      //Serial.print('\t');
    }
    //Serial.println();
    delayMicroseconds(5000); // Small delay to avoid CPU overload
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

void rotate(int n){
  int target[4] = {n, n, n, n};
  updateMotor(target);
}


void setup()
{

  init_motors();
  delay(200);
  SERIAL.begin(115200); // USB serial setup
  SERIAL.println("Start");
  STOP(); // Stop the robot
  Serial3.begin(9600); // BT serial setup
  //Pan=PL4=>48, Tilt=PL5=>47
  servo_pan.attach(48);
  servo_tilt.attach(47);
  
  
  ////////////////////////////////////////////
  //OLED Setup//////////////////////////////////
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
  }

  display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.setCursor(0, 0);     // Start at top-left corner
  display.println("Start");
  display.display();
  //delay(5000);

  //int_adc0_c=analogRead(A14);   // Left sensor at zero light intensity
  //int_adc1_c=analogRead(A15);   // Right sensor at zero light intensity

  //int_adc0_m=(int_adc0-int_adc0_c)/100;
  //int_adc1_m=(int_adc1-int_adc1_c)/100;

  display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.setCursor(0, 0);     // Start at top-left corner
  display.println("Start UP");
  display.display();
  //delay(5000);
  //Setup Voltage detector
  pinMode(A0, INPUT);
  
  //Ultrasonic
  pinMode(echoPinL, INPUT);
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinR, INPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinT, INPUT);
  pinMode(trigPinT, OUTPUT);
  
  L_cal=analogRead(A14);
  R_cal=analogRead(A15);

  distT = measure_distanceT();
  delay(5);
}


void loop()

{
  // run the code in every 20ms
  if (millis() > (time + 15)) {
    voltCount++;
    time = millis();
    UART_Control(); //get USB and BT serial data

    //constrain the servo movement
    pan = constrain(pan, servo_min, servo_max);
    tilt = constrain(tilt, servo_min, servo_max);
    
    //send signal to servo
    servo_pan.write(pan);
    servo_tilt.write(tilt);
  }if (voltCount>=5){
    voltCount=0;
    sendVolt();
  }
  
  distL = measure_distanceL();
  delay(20);
  distR = measure_distanceR();
  delay(20);
  distT = measure_distanceT();
  delay(20);

 
  L_brightness =analogRead(A14);
  R_brightness = analogRead(A15);
  //brightness set zero
  R_brightness= R_brightness -  R_cal;
  L_brightness=L_brightness - L_cal;


  display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.setCursor(0, 0);     // Start at top-left corner
  display.println("L:" + String(distL) + " R: "+ String(distR)+" T: "+String(distT)+"\nV: "+String(newV)+"\n"+"Lbri: "+String(L_brightness)+" Rbri: "+ String(R_brightness));

  display.display();

  if (stage1==1){//set the base distance of top sensor for later comparison
    Serial.println("stage1");
    stg2_base = distT;
    stage1=0;
    stage2=1;
  }

  if (stage2==1){  //turn clk wise, until top sensor senses the distance is becoming longer
    if ((abs(distL - distR) < 1) & ((distT -distR) <= 8) & ((distT -distL) <= 8)){
      stage2=0;
      stage3=1;
      continueRight=1;
      stg3_base = distT;
      float T=measure_distanceT();
        while(T>65){
          T=measure_distanceT();
          forward(100);
          delay(20);
        }
    }

    if (continueRight) {
      rotate_2(300,300,300,300); //clockwise
    } else {
      rotate_1(300,300,300,300);
    }
    delay(80); //adjustable
    STOP();

    float T = measure_distanceT();
    delay(5);

    if (continueRight & (T<stg2_base)){ //update base if its getting shorter
      stg2_base=T;
    }
    else if (continueRight & (T >(stg2_base+5))){ // if start becoming larger, stop
      continueRight=0;

    }
  }
  if (stage3==1){ //set the base distance of top sensor for later comparison
    if (continueRight){
      rotate_2(300,300,300,300); //clockwise
    } else {
      rotate_1(300,300,300,300);
    }
    delay(80); //adjustable
    STOP();

    float T = measure_distanceT();
    delay(5);

    if (T > (14 + distR)& T>(14 + distL)) {
      stage3=0;
      stage4=1;
      boxOnRight = continueRight;// ******* problem: finding the box while rotating left does not mean box must be on the left
      if (boxOnRight){
        rotate_1(300,300,300,300);
        delay(500); //adjustable
        STOP();}
      else{
        rotate_2(300,300,300,300);
        delay(400); //adjustable
        STOP();
    }

      stg4_base = T;

      display.clearDisplay();
      display.setTextSize(1);      // Normal 1:1 pixel scale
      display.setTextColor(SSD1306_WHITE); // Draw white text
      display.cp437(true);         // Use full 256 char 'Code Page 437' font
      display.setCursor(0, 0);     // Start at top-left corner
      display.println("Stage3: BOX FOUND");
      display.display();
      delay(2000);

    } else {
      if (T < stg3_base){ //update base if its getting shorter
        stg3_base=T;
      } else if (T > (stg3_base+12)){ // if start becoming larger, stop
        //forward(30);
        //delay(30);
        stg3_base=T;
        continueRight = !continueRight;
      }
    }
  }
  if (stage4==1){  
    if ((abs(distL - distR) < 1) & ((distT -distR) <= 8) & ((distT -distL) <= 8)){
      stage4=0;
      stage5=1;
    }
    if (boxOnRight){
      rotate_1(300,300,300,300);
      delay(50); //adjustable
      STOP();}
      else{
      rotate_2(300,300,300,300);
      delay(50); //adjustable
      STOP();
    }
  }
  if (stage5 == 1){
    stage5=0;
    stage6=1;
    if (boxOnRight) {
      right(500);
    } else {
      left(1200);
    }
  }
  
  if (stage6 == 1){ //move till see edge of box
    float fix = distT;
    if (distL > (distR) + 3){ //if facing too left, rotate clkwise
      rotate(-15);
      delay(20);
    } else if (distR > (distL) + 3){//if facing too right, rotate anitclkwise
      rotate(15);
      delay(20);
    } else {
      if (boxOnRight){ // should go left
        right(200);
        delay(20);}
      else{
        left(200);
        delay(20);
      }
    }
    delay(400);
    while (fix > measure_distanceT()+3) {
      backward(10);
      delay(100);
    }

    if (abs(distL - distR)>7){ //see edge of box
      stage6=0;
      stage7=1;
    }
  }
  if (stage7 == 1){ //move more precisely till inside the mid of box

    float fix = distT;
    if (boxOnRight){ 
      right(150);
      delay(30);
      }
    else{
      left(400);
      delay(30);
    }
    
    while (fix > measure_distanceT()+4) {
      backward(20);
      delay(100);
    }

    if (abs(distL - distR) < 4){ //at mid of box
      stage7=0;
      stage8=1;
    }

  }
  if (stage8 == 1){ //approaching the box with adjustment if tilted
    
    if (distL> distR+5){ //if facing too left, rotate clkwise
      rotate_2(400,400,400,400);
      delay(30);
      STOP();
    }
    else if (distR > (distL+5)){//if facing too right, rotate anitclkwise
      rotate_1(400,400,400,400);
      delay(30);
      STOP();
    }
    else{ // if two eyes see the box, so front
      forward(15);
      delay(30);
    }

    if (distL <= 10 | distR <=10){
      stage8=0;
      stage9=1;

    }
  }

  if (stage9==1){ // move sideways according to LED brightness


    if ((L_brightness-R_brightness)>1){
      left(20);
      delay(40);
    }
    else if ((R_brightness-L_brightness)>1){
      right(20);
      delay(40);
    }
    else{
      stage9=0;
      stage10=1;

    }
  }
   if (stage10==1){ // move final step forward
     if (distL>=9 | distR>=9){
       forward(20);
       //delay(20);
     }
     else{
        delay(2000);
        if (newV >= 3.5){
          stage10=0;
        }
        else{
          backward(500);
          stage10=0;
          stage8=1;
        }
     }
   }
  //update prev dists
  prev_distL = distL;
  prev_distR = distR;
  prev_distT = distT;
  //Serial3.println("BT test");
}




//  -- END OF FILE --