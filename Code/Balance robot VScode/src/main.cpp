/*
B1: Đọc cảm biến IMU đơn giản
B2: Thêm Kalman Filter để lọc
B3: Điều khiển động cơ bằng TB6612
B4: Kết hợp IMU + TB6612 điều khiển động cơ theo góc Roll và Pitch
B5: Làm bộ PID điều khiển góc Pitch
B6: Làm bộ PID điều khiển góc Roll
B7: Compile tất cả
*/

//-------------------------Thư viện-------------------------//
#include <Arduino.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <SimpleKalmanFilter.h>

//-------------------------Cảm biến & Kalman-------------------------//
MPU6050 mpu6050(Wire);
SimpleKalmanFilter XKalman(5, 5, 0.1);  //0.01
SimpleKalmanFilter YKalman(5, 5, 0.1);

//-------------------------Biến góc-------------------------//
float XAngle, XAngle_Kalman;
float YAngle, YAngle_Kalman;

//------------------------PID Roll--------------------------//
const float kpX = 40.0; //40  
const float kdX = 10.0; //10.
const float kiX = 00000.0;
float eprevX = 0;
float eintegralX = 0;
const float targetAngleX = -1.15;

//------------------------PID Pitch--------------------------//
const float kpY = 1000.0;  //111 120 111 500      600  700  1000 1000             1150       1000
const float kdY = 24;   //5.3 5.3 2.5   5.3      10   15         30    24 20.5  100 20.7    24
const float kiY = 0.0001; //             0.0001   0
float eprevY = 0;
float eintegralY = 0;

const float targetAngleY = -3.0;

//-------------------------Timing----------------------------//
const unsigned long sampleTime = 10;
unsigned long lastTime = 0;

//--------------------TB6612 ESP32 Pin-----------------------//
const int ENA = 16;
const int IN2 = 17;
const int IN1 = 5;
const int ENB = 32;
const int IN4 = 33;
const int IN3 = 25;
const int STBY = 26;

// LEDC Channels & Resolution
const int pwmFreq = 90000;      // 80 kHz
const int pwmResolution = 8;    // 8-bit resolution (0 - 255)
const int channelA = 0;         // For ENA
const int channelB = 1;         // For ENB


void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu6050.begin();
 // mpu6050.calcGyroOffsets(false);
  // TB6612 pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  //digitalWrite(STBY,LOW);
  // Cấu hình LEDC cho PWM tốc độ cao
  ledcSetup(channelA, pwmFreq, pwmResolution);
  ledcAttachPin(ENA, channelA);

  ledcSetup(channelB, pwmFreq, pwmResolution);
  ledcAttachPin(ENB, channelB);
}

void loop() {
  if (millis() - lastTime >= sampleTime) {
    lastTime = millis();
    mpu6050.update();

    //---------------Đọc và lọc góc---------------//
    XAngle = mpu6050.getAngleX();
    YAngle = mpu6050.getAngleY();
    XAngle_Kalman = XKalman.updateEstimate(XAngle);
    YAngle_Kalman = YKalman.updateEstimate(YAngle);

    float deltaT = sampleTime / 1000.0;

    //=============== PID điều khiển trục Y (Pitch) =================//
    float errorY = targetAngleY - YAngle_Kalman;
    float dedtY = (errorY - eprevY) / deltaT;
    eintegralY += errorY * deltaT;
    float uY = kpY * errorY + kdY * dedtY + kiY * eintegralY;
int pwmY = constrain((int)fabs(uY), 0, 255);
    int dirY = (uY > 0) ? 1 : -1;

    if (dirY == 1) {
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    } else {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    }
    ledcWrite(channelB, pwmY);
    eprevY = errorY;

    //=============== PID điều khiển trục X (Roll) =================//
    float errorX = targetAngleX - XAngle_Kalman;
    float dedtX = (errorX - eprevX) / deltaT;
    eintegralX += errorX * deltaT;
    float uX = kpX * errorX + kdX * dedtX + kiX * eintegralX;

    int pwmX = constrain((int)fabs(uX), 0, 255);
    int dirX = (uX > 0) ? -1 : 1;

    if (dirX == 1) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    } else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
    }
    ledcWrite(channelA, pwmX);
    eprevX = errorX;

    //---------------Serial debug---------------//
    Serial.print(">Y: ");
    Serial.print(YAngle_Kalman);
    Serial.print("|");
    Serial.print(">X: ");
    Serial.println(XAngle_Kalman);
    //Serial.print(">pwmY: ");
    //Serial.println(pwmY);

    /*Serial.print(" | uY: ");
    Serial.print(uY);
    Serial.print(" || X: ");
    Serial.print(XAngle_Kalman);
    Serial.print(" | uX: ");
    Serial.println(uX);*/
    
  }
}