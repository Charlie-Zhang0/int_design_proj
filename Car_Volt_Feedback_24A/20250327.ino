#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define echoPinR 32
#define trigPinR 30
#define echoPinL 29
#define trigPinL 33
#define trigPinT 25
#define echoPinT 28

bool stage1 = 1; //set the base distance of top sensor for later comparison
bool stage2 = 0; //turn clk wise, until top sensor senses the distance is becoming longer
bool stage3 = 0; //set the base distance of top sensor for later comparison
bool stage4 = 0; //turn anti clk wise, until top sensor senses the distance is becoming longer
bool flag4a =0;  //flag to check alignment using Top sensor
bool flag4b =0;  //flag to check alignment using L & R sensor
bool stage5 = 0; //set timer for timeout if box not found on RHS
bool stage6 =0;  //go right until box found or timeout
bool stage7 =0;  //if timeout at stage6, move left until box found
bool stage8=0;   //box found, go straigh until any of the L or R sensor measure 9 cm.

long distL=0; //measurement at the begining of each loop
long distR=0;
long distT=0;

long prev_distL=0; //measurement of previous loop, updated at the end of each loop
long prev_distR=0;
long prev_distT=0;

long stg2_base;  //to store the dist for comparison
long stg4_base;
long stg6_timer=0; //for timeout if box not found

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     28 //4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
int oldV=1, newV=0;
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

// Define motor pins
#define PWMA 12    //Motor A PWM
#define DIRA1 34
#define DIRA2 35  //Motor A Direction
#define PWMB 8    //Motor B PWM
#define DIRB1 37
#define DIRB2 36  //Motor B Direction
#define PWMC 9   //Motor C PWM --> from 6 to 9
#define DIRC1 43
#define DIRC2 42  //Motor C Direction
#define PWMD 5    //Motor D PWM
#define DIRD1 A4  //26  
#define DIRD2 A5  //27  //Motor D Direction

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

#define SERIAL  Serial
#define BTSERIAL Serial3

#define LOG_DEBUG

#ifdef LOG_DEBUG
  #define M_LOG SERIAL.print
#else
  #define M_LOG BTSERIAL.println
#endif

//PWM Definition
#define MAX_PWM   2000
#define MIN_PWM   300

int Motor_PWM = 1900;


//    ↑A-----B↑
//     |  ↑  |
//     |  |  |
//    ↑C-----D↑
void BACK(uint8_t pwm_A, uint8_t pwm_B, uint8_t pwm_C, uint8_t pwm_D)
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_FORWARD(Motor_PWM);
}

//    ↓A-----B↓
//     |  |  |
//     |  ↓  |
//    ↓C-----D↓
void ADVANCE()
{
  MOTORA_FORWARD(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}
//    =A-----B↑
//     |   ↖ |
//     | ↖   |
//    ↑C-----D=
void RIGHT_1() // RB
{
  MOTORA_STOP(Motor_PWM); 
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_STOP(Motor_PWM);
}

//    ↓A-----B↑
//     |  ←  |
//     |  ←  |
//    ↑C-----D↓
void RIGHT_2() //This is to move right horizontally, calibration needed
{
  MOTORA_FORWARD(Motor_PWM); 
  MOTORB_FORWARD(Motor_PWM+10);
  MOTORC_BACKOFF(Motor_PWM+10); 
  MOTORD_BACKOFF(Motor_PWM+10);
}
//    ↓A-----B=
//     | ↙   |
//     |   ↙ |
//    =C-----D↓
void RIGHT_3() //RF
{
  MOTORA_FORWARD(Motor_PWM); 
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}
//    ↑A-----B=
//     | ↗   |
//     |   ↗ |
//    =C-----D↑
void LEFT_1() //LF
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM); 
  MOTORD_FORWARD(Motor_PWM);
}
//    ↑A-----B↓
//     |  →  |
//     |  →  |
//    ↓C-----D↑
void LEFT_2() //This is to move left horizontally, calibration needed
{
  MOTORA_BACKOFF(Motor_PWM+0); 
  MOTORB_BACKOFF(Motor_PWM-25);
  MOTORC_FORWARD(Motor_PWM+30); 
  MOTORD_FORWARD(Motor_PWM+10);
}
//    =A-----B↓
//     |   ↘ |
//     | ↘   |
//    ↓C-----D=
void LEFT_3() //L
{
  MOTORA_STOP(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); 
  MOTORD_STOP(Motor_PWM);
}

//    ↑A-----B↓
//     | ↗ ↘ |
//     | ↖ ↙ |
//    ↑C-----D↓
void rotate_1()  //tate_1(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}

//    ↓A-----B↑
//     | ↙ ↖ |
//     | ↘ ↗ |
//    ↓C-----D↑
void rotate_2()  // rotate_2(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
{
  MOTORA_FORWARD(Motor_PWM);
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);
  MOTORD_FORWARD(Motor_PWM);
}
//    =A-----B=
//     |  =  |
//     |  =  |
//    =C-----D=
void STOP()
{
  MOTORA_STOP(Motor_PWM);
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);
  MOTORD_STOP(Motor_PWM);
}

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
    case 'A':  ADVANCE();  M_LOG("Run!\r\n"); break;
    case 'B':  RIGHT_2();  M_LOG("Right up!\r\n");     break;
    case 'C':  rotate_1();                            break;
    case 'D':  RIGHT_3();  M_LOG("Right down!\r\n");   break;
    case 'E':  BACK(500, 500, 500, 500);     M_LOG("Run!\r\n");          break;
    case 'F':  LEFT_3();   M_LOG("Left down!\r\n");    break;
    case 'G':  rotate_2();                              break;
    case 'H':  LEFT_2();   M_LOG("Left up!\r\n");     break;
    case 'Z':  STOP();     M_LOG("Stop!\r\n");        break;
    case 'z':  STOP();     M_LOG("Stop!\r\n");        break;
    case 'd':  LEFT_2();   M_LOG("Left!\r\n");        break;
    case 'b':  RIGHT_2();  M_LOG("Right!\r\n");        break;
    case 'L':  Motor_PWM = 1500;                      break;
    case 'M':  Motor_PWM = 500;                       break;
  }
}



/*Voltage Readings transmitter
Sends them via Serial3*/
void sendVolt(){
    newV = analogRead(A0); // receivingCoil +VE
    if(newV!=oldV) {
      if (!Serial3.available()) {
        Serial3.println(newV);
        Serial.println(newV);
      }
    }
    oldV=newV;
}

long measure_distanceL() {
  long duration;
  long distance_in_cm;
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
 
  digitalWrite(trigPinL, LOW); 
  delayMicroseconds(2); 
  digitalWrite(trigPinL, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPinL, LOW);
 
  duration = pulseIn(echoPinL, HIGH);

  distance_in_cm = (duration/2.0) / 29.1;

  return distance_in_cm;

}

long measure_distanceR() {
  long duration;
  long distance_in_cm;
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

long measure_distanceT() {
  long duration;
  long distance_in_cm;
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


// Helper function to print a value with consistent spacing
void printPadded(int value) {
  if (value < 10) {
    Serial.print("   "); // 3 spaces for 1-digit numbers
  } else if (value < 100) {
    Serial.print("  ");  // 2 spaces for 2-digit numbers
  } else if (value < 1000) {
    Serial.print(" ");   // 1 space for 3-digit numbers
  }
  // For 4-digit numbers, no spaces are needed
  Serial.print(value); // Print the actual value
}

//Where the program starts
void setup()
{
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
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.setCursor(0, 0);     // Start at top-left corner
  display.println("Start UP");
  display.display();
  delay(1000);


  //Setup Voltage detector
  pinMode(A0, INPUT);

  //Ultrasonic
  pinMode(echoPinL, INPUT);
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinR, INPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinT, INPUT);
  pinMode(trigPinT, OUTPUT);

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
  delay(50);
  distR = measure_distanceR();
  delay(50);
  distT = measure_distanceT();
  delay(50);
  Serial.println("L:" + String(distL) + " R: "+ String(distR)+" T: "+String(distT));
  delay(50);

  display.clearDisplay();
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.setCursor(0, 0);     // Start at top-left corner
  display.println("L:" + String(distL) + " R: "+ String(distR)+" T: "+String(distT));
  display.display();

  if (stage1==1){//set the base distance of top sensor for later comparison
    stg2_base = distT;
    stage1=0;
    stage2=1;
  }

  if (stage2==1){  //turn clk wise, until top sensor senses the distance is becoming longer

    display.clearDisplay();
    display.setTextSize(2);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.cp437(true);         // Use full 256 char 'Code Page 437' font
    display.setCursor(0, 0);     // Start at top-left corner
    display.println("STAGE2");
    display.display();

    rotate_2(); //clockwise
    delay(30); //adjustable
    STOP();

    long T = measure_distanceT();

    display.clearDisplay();
    display.setTextSize(2);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.cp437(true);         // Use full 256 char 'Code Page 437' font
    display.setCursor(0, 0);     // Start at top-left corner
    display.println(String(T)+"VS"+String(stg2_base));
    display.display();

    if (T<stg2_base){ //update base if its getting shorter
      stg2_base=T;
    }

    if (T >(stg2_base+2) ){ // if start becoming larger, stop
      stage2=0;
      stage3=1;
    }
  }
  if (stage3==1){ //set the base distance of top sensor for later comparison
    stg4_base = distT;
    stage3=0;
    stage4=1;
  }
  if (stage4==1){  //turn anti clk wise, until top sensor senses the distance is becoming longer

    display.clearDisplay();
    display.setTextSize(2);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.cp437(true);         // Use full 256 char 'Code Page 437' font
    display.setCursor(0, 0);     // Start at top-left corner
    display.println("STAGE2");
    display.display();

    rotate_1(); //anti clockwise
    delay(30);//adjustable
    STOP();

    long T = measure_distanceT();

    display.clearDisplay();
    display.setTextSize(2);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.cp437(true);         // Use full 256 char 'Code Page 437' font
    display.setCursor(0, 0);     // Start at top-left corner
    display.println(String(T)+"VS"+String(stg4_base));
    display.display();

    if (T<stg4_base){ //update base if its getting shorter
      stg4_base=T;
    }
    if (abs(T -stg4_base)<2) { //flag to check if it is returned to the shortest distance (with an error)
      flag4a=1;
    }
    if (abs(distL - distR) <2) { // flag to verify by L & R dist to improve alignment
      flag4b=1;
    }

    if (flag4a && flag4b){ //if both satisfied should be good
      stage4=0;
      stage5=1;
    }
  }
  if (stage5 ==1){
    ADVANCE(); //just to move forward for a bit, not neccessary 
    delay(150);
    STOP();

    stg6_timer=millis(); //start timer
    stage5=0;
    stage6=1;
  }
  
  if (stage6 == 1){ //go right until box found or timeout
    RIGHT_2();  //<---- Calibration needed for the pwm in the function
    delay(150); //adjustable
    STOP();

    if ((millis()-stg6_timer>4000)){ // not found on RHS, timout in 4s (adjustable)
      stage6=0;
      stage7=1; //stage7 to go left
    }

    
    if (distL - distR>8){ // if L >R by 8 cm, found box and end, skip to stage8
      stage6=0;
      stage8=1;

    }
  }
  if (stage7 == 1){ //go Left till box found
    LEFT_2(); //<---- Calibration needed for the pwm in the function
    delay(120); // adjustable
    STOP();

    if (distR - distL>8){ //if R > L by 8 cm, box found
      stage7=0;
      stage8=1;
    }
  }
  if (stage8 == 1){ // go forward till any sensor measures 9cm.
    ADVANCE();
    delay(50);
    STOP();

    if (distL <= 9 | distR <=9){
      stage8=0;
    }
  }

  //update prev dists
  prev_distL = distL;
  prev_distR = distR;
  prev_distT = distT;
}
