#include <Servo.h>
#include <Wire.h>
#include <Adafruit_BME280.h>  // Install via Library Manager

// Pins
const int RAIN_SENSOR_PIN = A5;
const int FAN_PIN = 10;
const int SERVO_PIN = 9;

// Thresholds
const int DRY_VALUE = 900;
const int WET_VALUE = 300;
const int RAIN_THRESHOLD = 700;
const float FAN_ON_TEMP = 30.0;  // Temperature threshold (°C)

// Servo settings
const int SERVO_ANGLE_MIN = 0;
const int SERVO_ANGLE_MAX = 180;

// BME280 I2C Address (0x76 or 0x77)
#define BME280_ADDRESS 0x76  // Try 0x77 if this fails

Servo wiperServo;
Adafruit_BME280 bme;  // BME280 object

void setup() {
  Serial.begin(9600);
  wiperServo.attach(SERVO_PIN);
  wiperServo.write(SERVO_ANGLE_MIN);
  
  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW);

  // Initialize BME280
  if (!bme.begin(BME280_ADDRESS)) {
    Serial.println("Could not find BME280 sensor!");
    while (1);
  }
}

void loop() {
  // --- Read Temperature ---
  float temperature = bme.readTemperature();  // Get temp in °C
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println("°C");

  // Control Fan
  if (temperature > FAN_ON_TEMP) {
    digitalWrite(FAN_PIN, HIGH);
    Serial.println("Fan ON (Too hot!)");
  } else {
    digitalWrite(FAN_PIN, LOW);
  }

  // --- Rain Sensor & Wiper ---
  int rainValue = analogRead(RAIN_SENSOR_PIN);
  Serial.print("Rain value: ");
  Serial.println(rainValue);

  if (rainValue < RAIN_THRESHOLD) {
    int rainIntensity = map(rainValue, WET_VALUE, DRY_VALUE, 1, 8);
    rainIntensity = constrain(rainIntensity, 1, 8);

    for (int angle = SERVO_ANGLE_MIN; angle <= SERVO_ANGLE_MAX; angle++) {
      wiperServo.write(angle);
      delay(rainIntensity);
    }
    for (int angle = SERVO_ANGLE_MAX; angle >= SERVO_ANGLE_MIN; angle--) {
      wiperServo.write(angle);
      delay(rainIntensity);
    }
  } else {
    wiperServo.write(SERVO_ANGLE_MIN);
  }

  delay(100);
}
