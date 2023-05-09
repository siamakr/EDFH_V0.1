///// sensors.h /////

#ifndef _EDFH_SENSORS_H
#define _EDFH_SENSORS_H

#include "BNO080.h"
//#include <Arduino>
// #include <Wire.h>
#include <SPI.h>
#include <Math.h>
//#include <BasicLinearAlgebra.h>
#include <stdint.h>


//using namespace BLA;

//..... Defines .....//
#define DT_MSEC 10.00f
#define d2r (PI/180.00f)
#define r2d (180.00f/PI)


//..... SPI Pin Definitons .....//
const byte imuCSPin = 10;
const byte imuWAKPin = 7;
const byte imuINTPin = 8;
const byte imuRSTPin = 9;

//.......... Structure Definitions ..........//
typedef struct
{
    float roll, pitch, yaw;
    float gx, gy, gz;
    float ax, ay, az;
    float qi, qj, qk, qw;
    byte linAccuracy = 0;
    byte gyroAccuracy = 0;
    //byte magAccuracy = 0;
    float quatRadianAccuracy = 0;
    byte quatAccuracy = 0; 
} fsm_data_t;


//...... Class Definition .....//
class Sensors
{    
public:

//..... Object Definition .....//

    fsm_data_t data;

Sensors::Sensors(void){
//fsm = new BNO080();

}

void Sensors::init(void)
{
      // Disable all SPI devices  before setup
    pinMode( imuCSPin, OUTPUT );
    digitalWrite( imuCSPin, HIGH );

    delay(100);

  //..... FSM305/BNO080 Init .....//
  Serial.print("FSM Init start..."); 
  delay(300);

  if (fsm->beginSPI(imuCSPin, imuWAKPin, imuINTPin, imuRSTPin, 3000000) == false)
  {
    Serial.println("BNO080 over SPI not detected. Are you sure you have all 6 connections? Freezing...");
    while(1);
  }

    fsm->calibrateAll();
    fsm->enableLinearAccelerometer(DT_MSEC);  // m/s^2 no gravity
    fsm->enableRotationVector(DT_MSEC);  // quat
    //fsm.enableGameRotationVector(DT_MSEC);
    fsm->enableGyro(DT_MSEC);  // rad/s
    //fsm.enableGyroIntegratedRotationVector(DT_MSEC);
    //fsm.enableMagnetometer(DT_MSEC);  // cannot be enabled at the same time as RotationVector (will not produce data)
  
  Serial.println("FSM Init Finished..."); 
  delay(300);

  //..... PMW3901 Init .....//


}

///.......... FSM305/BNO080 FUNCTIONS ..........////

void save_calibrate_fsm(void)
{
  if(Serial.available())
  {
    byte incoming = Serial.read();

    if(incoming == 's')
    {
      fsm->saveCalibration(); //Saves the current dynamic calibration data (DCD) to memory
      fsm->requestCalibrationStatus(); //Sends command to get the latest calibration status

      //Wait for calibration response, timeout if no response
      int counter = 100;
      
      while(1)
      {
        if(--counter == 0) break;
        if(fsm->dataAvailable() == true)
        {
          //The IMU can report many different things. We must wait
          //for the ME Calibration Response Status byte to go to zero
          if(fsm->calibrationComplete() == true)
          {
            Serial.println("Calibration data successfully stored");
            delay(1000);
            break;
          }
        }

        delay(1);
      }

      if(counter == 0) Serial.println("Calibration data failed to store. Please try again.");

      //fsm.endCalibration(); //Turns off all calibration
      //In general, calibration should be left on at all times. The BNO080
      //auto-calibrates and auto-records cal data roughly every 5 minutes
    }
  } 
} 

void sample_fsm(void)
{
  if(fsm->dataAvailable() == true)
  {
    //..... Sample IMU .....//
    //... Linear Accel ...//
    fsm->getLinAccel(data.ax, data.ay, data.az, data.linAccuracy);

    // data.ax = fsm.getAccelX();
    // data.ay = fsm.getAccelY();
    // data.az = fsm.getAccelZ();

    //... Gyro ...//
    fsm->getGyro(data.gx, data.gy, data.gz, data.gyroAccuracy);

    // data.gx = fsm.getFastGyroX();
    // data.gy = fsm.getFastGyroY();
    // data.gz = fsm.getFastGyroZ();

    // data.gx = fsm.getGyroX();
    // data.gy = fsm.getGyroY();
    // data.gz = fsm.getGyroZ();
    
    //... Rotation Vector ...//
    fsm->getQuat(data.qi, data.qj, data.qk, data.qw, data.quatRadianAccuracy, data.quatAccuracy);

    // data.qi = fsm.getQuatI();
    // data.qj = fsm.getQuatJ();
    // data.qk = fsm.getQuatK();
    // data.qw = fsm.getQuatReal();


    //... Euler Angle Representation ...//
    data.roll = fsm->getRoll();
    data.pitch = fsm->getPitch();
    data.yaw = fsm->getYaw();    

  }

}


void print_fsm(void)
{
  char text[200];
  sprintf(text, "%0.5f, %0.5f, %0.5f,\t  %0.5f, %0.5f, %0.5f, \t  %0.5f, %0.5f, %0.5f    ",
  r2d*data.roll,
  r2d*data.pitch,
  r2d*data.yaw,
  r2d*data.gx,
  r2d*data.gy,
  r2d*data.gz,
  data.ax,
  data.ay,
  data.az);

  Serial.println(text);
}

void print_fsm_calibration(void)
{

  char text[250];
  sprintf(text, "%0.5f, %0.5f, %0.5f,\t   %0.5f, %0.5f, %0.5f, %0.5f,\t  %0.5f, %0.5f, %0.5f, \t  %0.5f, %0.5f, %0.5f    ",
  r2d*data.roll,
  r2d*data.pitch,
  r2d*data.yaw,
  data.qi,
  data.qj,
  data.qk,
  data.qw,
  r2d*data.gx,
  r2d*data.gy,
  r2d*data.gz,
  data.ax,
  data.ay,
  data.az);

  Serial.print(text);
  Serial.print(data.linAccuracy);
  Serial.print(", ");
  Serial.print(data.gyroAccuracy);
  Serial.print(", ");
  Serial.print(data.quatAccuracy);
  Serial.println(", ");
}

    //... FSM305 Functions End ...//
    
    //..... Object Definition .....//
    // fsm_data_t data; 
    // BNO080 fsm;

private:
    BNO080 * fsm;
    // adafruit_bno055_offsets_t bnoOffset;

    // float IIR( float newSample, float prevOutput, float alpha);

    // void rotate_to_world( float * vector );

    


};


#endif
