#include "I2Cdev.h"
#include "MPU6050.h"

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

float ax, ay, az;
float gx, gy, gz;

const int pResistor = A0; // Photoresistor at Arduino analog pin A0

//Variables
int photoValue;				  // Store value from photoresistor (0-1023)

const int trigPin = 7;
const int echoPin = 8;
// defines variables
long duration;
int distance;

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO

#include <dht.h>

dht DHT;

#define DHT11_PIN 6

int const BUZZER = 4;
//  Motor A
int const ENA = 10;  
int const INA = 12;
//  Motor B
int const ENB = 11;  
int const INB = 13;

int const MIN_SPEED = 27;   // Set to minimum PWM value that will make motors turn
int const ACCEL_DELAY = 50; // delay between steps when ramping motor speed up or down.

Adafruit_ICM20948 icm;

void setup() {
  // put your setup code here, to run once:
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  pinMode(pResistor, INPUT);

  pinMode(ENA, OUTPUT);   // set all the motor control pins to outputs
  pinMode(ENB, OUTPUT);
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(BUZZER, OUTPUT);


  icm.begin_I2C();
 
  Serial.begin(9600);

}

float data[10];

int state = 0;

long prevTime = 0;

void loop() {
  // put your main code here, to run repeatedly:

    // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;

  photoValue = analogRead(pResistor);
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);

  ax = accel.acceleration.x;
  ay = accel.acceleration.y;
  az = accel.acceleration.z;
  gx = gyro.gyro.x;
  gy = gyro.gyro.y;
  gz = gyro.gyro.z;
  float temperature = temp.temperature;


  data[0] = ax;
  data[1] = ay;
  data[2] = az;
  data[3] = gx;
  data[4] = gy;
  data[5] = gz;
  data[6] = temperature;
  data[7] = 0;
  data[8] = distance;
  data[9] = photoValue;

  for (int i = 0; i < 10; i++) {
    Serial.print(data[i]);
    Serial.print(',');
  }
  Serial.print('\n');

 switch (state) {
  case 0:
    Motor('A', 'F', 100);
    Motor('B', 'R', 100);
    if (millis() - prevTime > 1000) {
      state++;
      prevTime = millis();
    }
    break;
  case 1:
    Motor('A', 'F', 0);
    Motor('B', 'R', 0);
    if (millis() - prevTime > 1000) {
      state++;
      prevTime = millis();
    }
    break;
  case 2:
    Motor('A', 'R', 100);
    Motor('B', 'F', 100);
    if (millis() - prevTime > 1000) {
      state++;
      prevTime = millis();
    }
    break;
  }
  state %= 3;
  //delay(1000);
}

void Motor(char mot, char dir, int speed)
{
  // remap the speed from range 0-100 to 0-255
  int newspeed;
  if (speed == 0)
    newspeed = 0;   // Don't remap zero, but remap everything else.
  else
    newspeed = map(speed, 1, 100, MIN_SPEED, 255);

  switch (mot) {
    case 'A':   // Controlling Motor A
      if (dir == 'F') {
        digitalWrite(INA, HIGH);
      }
      else if (dir == 'R') {
        digitalWrite(INA, LOW);
      }
      analogWrite(ENA, newspeed);
      break;

    case 'B':   // Controlling Motor B
      if (dir == 'F') {
        digitalWrite(INB, HIGH);
      }
      else if (dir == 'R') {
        digitalWrite(INB, LOW);
      }
      analogWrite(ENB, newspeed);
      break;

    case 'C':  // Controlling Both Motors
      if (dir == 'F') {
        digitalWrite(INA, HIGH);
        digitalWrite(INB, HIGH);
      }
      else if (dir == 'R') {
        digitalWrite(INA, LOW);
        digitalWrite(INB, LOW);
      }
      analogWrite(ENA, newspeed);
      analogWrite(ENB, newspeed);
      break;
  }
}
