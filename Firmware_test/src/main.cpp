//THE DICTATOR TEST FIRMWARE
//rn it only tests: servos, IMU, state indication

//Dependencies
#include <Arduino.h>
#include <Servo.h>
#include <MPU6050.h>
#include <I2Cdev.h>
#include <Adafruit_MPL3115A2.h>
#include <SD.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>

#define TVCX 29
#define TVCY 33

#define pyro1 32
#define pyro2 34
#define pyro3 22



Adafruit_NeoPixel pixel(1, 13, NEO_GRB + NEO_KHZ800);

int buzzer = 28;

Servo tvcx;
Servo tvcy;

MPU6050 imu;
int16_t ax, ay, az;
int16_t gx, gy, gz;



void setup(){

pinMode(pyro1, OUTPUT);
pinMode(pyro2, OUTPUT);
pinMode(pyro3, OUTPUT);

digitalWrite(pyro1, LOW);
digitalWrite(pyro2, LOW);
digitalWrite(pyro3, LOW);

  /*--Start I2C interface--*/
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
Wire.begin(); 
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
Fastwire::setup(400, true);
#endif

Serial.begin(115200);
Serial.println("Starting avionics testing");
delay(500);

Serial.print("Testing State Indication");
pixel.begin();
pixel.show(); // Initialize to 'off'

pixel.setPixelColor(0, pixel.Color(255, 0, 0)); 
pixel.show();
delay(1000);
  
  // Set pixel 0 to Blue
pixel.setPixelColor(0, pixel.Color(0, 0, 255));
pixel.show();
delay(1000);

tone(buzzer, 3000, 500); 

delay(500);
Serial.println("State Indication complete");
Serial.println("Testing TVC Servos");

tvcx.attach(TVCX);
tvcy.attach(TVCY);

for(int i = 0; i <= 180; i++){
    tvcx.write(i);
    tvcy.write(i);
    delay(10);
}

for(int i = 180; i >= 0; i--){
    tvcx.write(i);
    tvcy.write(i);
    delay(10);
}

Serial.println("TVC Servos Complete! Starting IMU testing");
delay(500);

imu.initialize();
 Serial.println("Testing MPU6050 connection...");
  if(imu.testConnection() ==  false){
    Serial.println("MPU6050 connection failed");
    while(true);
  }
  else{
    Serial.println("MPU6050 connection successful");
  }

  /* Use the code below to change accel/gyro offset values. Use MPU6050_Zero to obtain the recommended offsets */ 
  Serial.println("Updating internal sensor offsets...\n");
  imu.setXAccelOffset(0); //Set your accelerometer offset for axis X
  imu.setYAccelOffset(0); //Set your accelerometer offset for axis Y
  imu.setZAccelOffset(0); //Set your accelerometer offset for axis Z
  imu.setXGyroOffset(0);  //Set your gyro offset for axis X
  imu.setYGyroOffset(0);  //Set your gyro offset for axis Y
  imu.setZGyroOffset(0);  //Set your gyro offset for axis Z
  /*Print the defined offsets*/
  Serial.print("\t");
  Serial.print(imu.getXAccelOffset());
  Serial.print("\t");
  Serial.print(imu.getYAccelOffset()); 
  Serial.print("\t");
  Serial.print(imu.getZAccelOffset());
  Serial.print("\t");
  Serial.print(imu.getXGyroOffset()); 
  Serial.print("\t");
  Serial.print(imu.getYGyroOffset());
  Serial.print("\t");
  Serial.print(imu.getZGyroOffset());
  Serial.print("\n");


  Serial.println("Printing accelerometer and Gyroscoping values (25 sets)");
  for(int i = 0; i <= 25; i++ ){

    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    Serial.print("a/g:\t");

Serial.print("AX:");
Serial.print(ax);
Serial.print("\tAY:");
Serial.print(ay);
Serial.print("\tAZ:");
Serial.print(az);
Serial.print("\tGX:");
Serial.print(gx);
Serial.print("\tGY:");
Serial.print(gy);
Serial.print("\tGZ:");
Serial.println(gz);
  }

  delay(1000);

  Serial.println("Finally, testing the pyrochannels");

  delay(1000);
  Serial.println("PYRO 1 ON");
  digitalWrite(pyro1, HIGH);
  delay(2000);
  digitalWrite(pyro1, LOW);
  Serial.println("PYRO 1 OFF");

    delay(1000);
  Serial.println("PYRO 2 ON");
  digitalWrite(pyro2, HIGH);
  delay(2000);
  digitalWrite(pyro2, LOW);
  Serial.println("PYRO 2 OFF");

    delay(1000);
  Serial.println("PYRO 3 ON");
  digitalWrite(pyro3, HIGH);
  delay(2000);
  digitalWrite(pyro3, LOW);
  Serial.println("PYRO 3 OFF");

  
  Serial.println("TESTING COMPLETE");

    }



void loop(){

}