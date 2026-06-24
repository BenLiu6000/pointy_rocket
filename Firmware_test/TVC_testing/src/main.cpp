#include <Arduino.h>
#include <MPU6050.h>
#include <Servo.h>


#include <Wire.h>
#include <I2Cdev.h>
#include <__clang_cuda_cmath.h>

#define TVCx 29
#define TVCy  33



MPU6050 imu;

int16_t ax, ay, az;
int16_t gx, gy, gz;



Servo tvcx;
Servo tvcy;


void setup(){
  Serial.begin(38400);

    imu.initialize();

  Serial.println("Testing imu6050 connection...");
  if(imu.testConnection() ==  false){
    Serial.println("imu6050 connection failed");
    while(true);
  }
  else{
    Serial.println("imu6050 connection successful");
  }

  /* Use the code below to change accel/gyro offset values. Use imu6050_Zero to obtain the recommended offsets */ 
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

  tvcx.attach(TVCx);
  tvcy.attach(TVCy);

  tvcx.write(90);
  tvcy.write(90);


}

double whateberitscalledagain(double deg){
  if (deg > 8){
    deg = 8;
  } else if (deg < -8){
    deg = -8;
  }
  return deg;
}

void loop(){
  double pitch_deg = atan2(ay, ax) * (180.0 / M_PI);
  double roll_deg  = atan2(ax, az) * (180.0 / M_PI);

  tvcx.write(pitch_deg);
  tvcy.write(roll_deg);

  if (pitch_deg > 8){
    pitch_deg = 8;
  } else if (pitch_deg < -8){
    pitch_deg = -8;
  }

  if (roll_deg > 8){
    roll_deg = 8;
  } else if (roll_deg < -8){
    roll_deg = -8;
  }

}