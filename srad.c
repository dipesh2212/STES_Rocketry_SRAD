#include<SPI.h> 

#include <SoftwareSerial.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <TinyGPS++.h>
#include "SPI.h"
#include "SD.h"

SoftwareSerial gps_ss(33,32);

TinyGPSPlus gps;
Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;

float pressure,altitude,temperature,latitude,longitude,gps_altitude,acceleration_x,acceleration_y,acceleration_z,rotation_x,rotation_y,rotation_z;
int buzLed=D0;
const int chipSelect = D8; 

void setup() {

  pinMode(buzLed,OUTPUT);
  
  Serial.begin(115200);
  gps_ss.begin(9600);
  
  Serial.println("Initializing avionics board!");
  Serial.println(); 
  
  digitalWrite(buzLed,HIGH);
  delay(2000);
  digitalWrite(buzLed,LOW);
  delay(2000); 
  
  if(bmp.begin(0x76)){
    Serial.println("BMP initialized successfully!");
    Serial.println();
    digitalWrite(buzLed,HIGH);
    delay(2000);
    digitalWrite(buzLed,LOW);
    delay(2000);       
  }
  else{
    Serial.println("BMP initialization failed!");
  }

  if (mpu.begin(0x68)) {
    Serial.println("MPU6050 initialized successfully!");
    Serial.println(); 
    digitalWrite(buzLed,HIGH);
    delay(2000);
    digitalWrite(buzLed,LOW);
    delay(2000);   
  }

  else{
    Serial.println("MPU initialization failed!");
  }

  if (SD.begin(chipSelect)) {
    Serial.println("SD card initialized successfully!");
    Serial.println();
    digitalWrite(buzLed,HIGH);
    delay(2000);
    digitalWrite(buzLed,LOW);
    delay(2000);
  }

  else{
    Serial.println("SD card initialization failed!");    
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }  

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

}

void loop() {


  
  while(gps_ss.available()>0){
    if(gps.encode(gps_ss.read())){
      if(gps.location.isValid()){
        latitude=gps.location.lat();
        longitude=gps.location.lng();
      }
    }
delay(0);    
  }

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  pressure=bmp.readPressure();
  temperature=bmp.readTemperature();
  altitude=bmp.readAltitude()*3.28084;
  acceleration_x=a.acceleration.x;
  acceleration_y=a.acceleration.y;
  acceleration_z=a.acceleration.z;
  rotation_x=g.gyro.x;
  rotation_y=g.gyro.y;
  rotation_z=g.gyro.z;


Serial.print("Pressure : ");
  Serial.print(pressure);
  Serial.println(" Pa");

  Serial.print("Altitude : ");
  Serial.print(altitude);
  Serial.println(" ft");
    
  Serial.print("Temperature : ");
  Serial.print(temperature);
  Serial.println(" C");


  Serial.print("Latitude : ");
  Serial.println(latitude);
    
  Serial.print("Longitude : ");
  Serial.println(longitude);


  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");

  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");


  Serial.println(); 

  File dataFile = SD.open("datalog.txt", FILE_WRITE);  

  if (dataFile) {
    digitalWrite(buzLed, HIGH);
    
    dataFile.print("Pressure : ");
    dataFile.print(pressure);
    dataFile.println(" Pa");

    dataFile.print("Altitude : ");
    dataFile.print(altitude);
    dataFile.println(" ft");
    
    dataFile.print("Temperature : ");
    dataFile.print(temperature);
    dataFile.println(" C");


    dataFile.print("Latitude : ");
    dataFile.println(latitude);
    
    dataFile.print("Longitude : ");
    dataFile.println(longitude);


    dataFile.print("Acceleration X: ");
    dataFile.print(a.acceleration.x);
    dataFile.print(", Y: ");

    dataFile.print(a.acceleration.y);
    dataFile.print(", Z: ");
    dataFile.print(a.acceleration.z);
    dataFile.println(" m/s^2");

    dataFile.print("Rotation X: ");
    dataFile.print(g.gyro.x);
    dataFile.print(", Y: ");
    dataFile.print(g.gyro.y);
    dataFile.print(", Z: ");
    dataFile.print(g.gyro.z);
    dataFile.println(" rad/s");

    dataFile.println(); 
  
    dataFile.close();
    delay(1000);
    digitalWrite(buzLed, LOW);
  }

  else {
    Serial.println("error opening datalog.txt");
  }
  
  delay(2000);
}