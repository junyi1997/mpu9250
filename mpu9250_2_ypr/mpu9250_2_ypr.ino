#include "MPU9250.h"
#include "Wire.h"    // I2C library
#include "SPI.h"     // SPI library

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
MPU9250 IMU1(Wire,0x69);
int status;
int status1;

// cosas de Timers
float dt;
long tiempo_prev;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;

float pitch_prev = 0;
float roll_prev = 0;

double pitch_accel = 0;
double roll_accel = 0;

float pitch1 = 0;
float roll1 = 0;
float yaw1 = 0;

float pitch_prev1 = 0;
float roll_prev1 = 0;

double pitch_accel1 = 0;
double roll_accel1 = 0;

void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  status1 = IMU1.begin();
  if (status < 0 && status1<0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.print(status);Serial.println(status1);
    while(1) {}
  }
}

void loop() {
  //diferencal de tiempo para su uso en el calculo de rotacion (pitch/ang Y, y roll/angX)
  /*calculo del dt en segundos*/ 
  dt = (millis()-tiempo_prev)/1000.0;
  /*tomo nota de tiempo_prev para siguiente calculo del dt*/
  tiempo_prev = millis();
  
  // read the sensor
  IMU.readSensor();
  IMU1.readSensor();
  //0X68
  //Roll & Pitch Equations con acelerometro ojo: double  atan2 (double __y, double __x)  ---> arc tangente of y/x
  //puede ser atan normal pero atan2 es por que esta en progreso que mida de -180 a 180 no solo de -90 a 90
  pitch_accel = atan2((IMU.getAccelY_mss()), sqrt(pow((IMU.getAccelX_mss()), 2) + pow((IMU.getAccelZ_mss()), 2)))*(180.0/3.14);
  roll_accel = atan2(-(IMU.getAccelX_mss()), sqrt(((IMU.getAccelY_mss()) * (IMU.getAccelY_mss())) + ((IMU.getAccelZ_mss()) * (IMU.getAccelZ_mss()))))*(180.0/3.14);
  
  float y_acc_nvo = atan2 ((IMU.getAccelY_mss()),(IMU.getAccelZ_mss()))*(180.0/3.14);

  //Calcular angulo de rotación (pitch/angulo en Y, y roll/angulo en X) con giroscopio y filtro complemento  
  pitch = 0.98*(pitch_prev+((IMU.getGyroX_rads())/131)*dt) + 0.02*pitch_accel;
  roll  = 0.98*(roll_prev+((IMU.getGyroY_rads())/131)*dt) + 0.02*roll_accel;
  
  float y_nvo = 0.98*(pitch_prev+((IMU.getGyroX_rads())/131)*dt) + 0.02*y_acc_nvo;
  float y_nvo_prev = y_nvo;
  
  pitch_prev = pitch;
  roll_prev = roll;
  
  //Yaw from mag
  float Yh = ((IMU.getMagY_uT()) * cos(roll)) - ((IMU.getMagZ_uT()) * sin(roll));
  float Xh = ((IMU.getMagX_uT()) * cos(pitch))+((IMU.getMagY_uT()) * sin(roll)*sin(pitch)) + ((IMU.getMagZ_uT()) * cos(roll) * sin(pitch));
  yaw = atan2(Yh, Xh)*(180.0/3.14);

  //0X69
  //Roll & Pitch Equations con acelerometro ojo: double  atan2 (double __y, double __x)  ---> arc tangente of y/x
  //puede ser atan normal pero atan2 es por que esta en progreso que mida de -180 a 180 no solo de -90 a 90
  pitch_accel1 = atan2((IMU1.getAccelY_mss()), sqrt(pow((IMU1.getAccelX_mss()), 2) + pow((IMU1.getAccelZ_mss()), 2)))*(180.0/3.14);
  roll_accel1 = atan2(-(IMU1.getAccelX_mss()), sqrt(((IMU1.getAccelY_mss()) * (IMU1.getAccelY_mss())) + ((IMU1.getAccelZ_mss()) * (IMU1.getAccelZ_mss()))))*(180.0/3.14);
  
  float y_acc_nvo1 = atan2 ((IMU1.getAccelY_mss()),(IMU1.getAccelZ_mss()))*(180.0/3.14);

  //Calcular angulo de rotación (pitch/angulo en Y, y roll/angulo en X) con giroscopio y filtro complemento  
  pitch1 = 0.98*(pitch_prev1+((IMU1.getGyroX_rads())/131)*dt) + 0.02*pitch_accel1;
  roll1  = 0.98*(roll_prev1+((IMU1.getGyroY_rads())/131)*dt) + 0.02*roll_accel1;
  
  float y_nvo1 = 0.98*(pitch_prev1+((IMU1.getGyroX_rads())/131)*dt) + 0.02*y_acc_nvo1;
  float y_nvo_prev1 = y_nvo1;
  
  pitch_prev1 = pitch1;
  roll_prev1 = roll1;
  
  //Yaw from mag
  float Yh1 = ((IMU1.getMagY_uT()) * cos(roll1)) - ((IMU1.getMagZ_uT()) * sin(roll1));
  float Xh1 = ((IMU1.getMagX_uT()) * cos(pitch1))+((IMU1.getMagY_uT()) * sin(roll1)*sin(pitch1)) + ((IMU1.getMagZ_uT()) * cos(roll1) * sin(pitch1));
  yaw1 = atan2(Yh1, Xh1)*(180.0/3.14);

  Serial.print("yaw：");Serial.print(yaw);Serial.print("\t");Serial.println(yaw1);
  Serial.print("pitch：");Serial.print(pitch);Serial.print("\t");Serial.print(pitch1);Serial.println();
  Serial.print("roll：");Serial.print(roll);Serial.print("\t");Serial.println(roll1);Serial.println();
  delay(500);


}
