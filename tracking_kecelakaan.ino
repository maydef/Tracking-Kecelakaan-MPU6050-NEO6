#include<Wire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
//#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>

#define MPU6050_AXOFFSET 158
#define MPU6050_AYOFFSET 9
#define MPU6050_AZOFFSET -91
#define MPU6050_GXOFFSET 19
#define MPU6050_GYOFFSET -42
#define MPU6050_GZOFFSET -26

double latitude,longitude;
int benturan;
int data_bentur=A0;
String link;
long sampling_timer;
const int MPU_addr=0x68;  // I2C address of the MPU-6050

char ssid[] = "actorpkm"; 
char password[] = "1234567890";
#define BOTtoken "807223134:AAHNgINQ76vCbEnpqNO15X6**********"
String chatid = "840454***";


int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; // Raw data of MPU6050
float GAcX, GAcY, GAcZ; // Convert accelerometer to gravity value
float Cal_GyX,Cal_GyY,Cal_GyZ; // Pitch, Roll & Yaw of Gyroscope applied time factor
float acc_pitch, acc_roll, acc_yaw; // Pitch, Roll & Yaw from Accelerometer
float angle_pitch, angle_roll, angle_yaw; // Angle of Pitch, Roll, & Yaw
float alpha = 0.96; // Complementary constant

TinyGPSPlus gps;
//SoftwareSerial ss(RXPin, TXPin);

WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);

void init_MPU6050(){
  //MPU6050 Initializing & Reset
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  //MPU6050 Clock Type
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x03);     // Selection Clock 'PLL with Z axis gyroscope reference'
  Wire.endTransmission(true);


  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);  // Gyroscope Configuration register
  
  Wire.write(0x18);     // FS_SEL=3, Full Scale Range = +/- 2000 [degree/sec]
  Wire.endTransmission(true);


  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);  // Accelerometer Configuration register

  Wire.write(0x10);     // AFS_SEL=2, Full Scale Range = +/- 8 [g]
  //Wire.write(0x18);     fahmi andriansyah
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1A);  // DLPF_CFG register
  Wire.write(0x00);     // Accel BW 260Hz, Delay 0ms / Gyro BW 256Hz, Delay 0.98ms, Fs 8KHz 

  Wire.endTransmission(true);
}

void WifiStatus() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  Serial.print("Connecting Wifi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) 
  {
    Serial.print(".");
    delay(100);
  }  
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup(){
  Wire.begin();
  Serial.begin(9600);
  WifiStatus();
  bot.sendMessage(chatid, "Let's start.");
  pinMode(data_bentur, INPUT);
 
  init_MPU6050();
}

void loop(){
   benturan=analogRead(data_bentur);

  //cek data Ublox
  while (Serial.available() > 0){
    if (gps.encode(Serial.read())){
      latitude=(gps.location.lat());
      longitude=(gps.location.lng());
      link="Terjadi indikasi kecelakaan di http://www.google.com/maps/place/"+String(latitude,8)+","+String(longitude,8);
    }
  }

  // Read raw data of MPU6050
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)


  GAcX = (float) AcX / 4096.0;
  GAcY = (float) AcY / 4096.0;
  GAcZ = (float) AcZ / 4096.0;


  acc_pitch = atan ((GAcY - (float)MPU6050_AYOFFSET/4096.0) / sqrt(GAcX * GAcX + GAcZ * GAcZ)) * 57.29577951; // 180 / PI = 57.29577951
  acc_roll = - atan ((GAcX - (float)MPU6050_AXOFFSET/4096.0) / sqrt(GAcY * GAcY + GAcZ * GAcZ)) * 57.29577951; 
  //acc_yaw = atan ((GAcZ - (float)MPU6050_AZOFFSET/4096.0) / sqrt(GAcX * GAcX + GAcZ * GAcZ)) * 57.29577951;
  acc_yaw = atan (sqrt(GAcX * GAcX + GAcZ * GAcZ) / (GAcZ - (float)MPU6050_AZOFFSET/4096.0)) * 57.29577951; 


  Cal_GyX += (float)(GyX - MPU6050_GXOFFSET) * 0.000244140625; // 2^15 / 2000 = 16.384, 250Hz, 1 /(250Hz * 16.384LSB)
  Cal_GyY += (float)(GyY - MPU6050_GYOFFSET) * 0.000244140625; // 2^15 / 2000 = 16.384, 250Hz, 1 /(250Hz * 16.384LSB)
  Cal_GyZ += (float)(GyZ - MPU6050_GZOFFSET) * 0.000244140625; // 2^15 / 2000 = 16.384, 250Hz, 1 /(250Hz * 16.384LSB)


  angle_pitch = alpha * (((float)(GyX - MPU6050_GXOFFSET) * 0.000244140625) + angle_pitch) + (1 - alpha) * acc_pitch;
  angle_roll = alpha * (((float)(GyY - MPU6050_GYOFFSET) * 0.000244140625) + angle_roll) + (1 - alpha) * acc_roll;
  angle_yaw += (float)(GyZ - MPU6050_GZOFFSET) * 0.000244140625; // Accelerometer doesn't have yaw value
  


  Serial.print(" | Cal_GyX = "); Serial.print(Cal_GyX);
  Serial.print(" | acc_pitch = "); Serial.print(acc_pitch);
  Serial.print(" | angle_pitch = "); Serial.println(angle_pitch);

  // Sampling Timer
  while(micros() - sampling_timer < 4000); //
  sampling_timer = micros(); //Reset the sampling timer 
  if((benturan==1024&&angle_pitch<=-55)||(benturan==1024&&angle_pitch>=-30))
  //if(benturan==1024)
  {
    Serial.println(link);
    bot.sendMessage(chatid, link); 
   } 
 }  
