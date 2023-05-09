///// sensors.h /////

#ifndef _EDFH_SENSORS_H
#define _EDFH_SENSORS_H

#include "BNO080.h"
#include "LIDARLite_v3HP.h"
#include <Wire.h>
#include <SPI.h>
#include <Math.h>
#include <BasicLinearAlgebra.h>
#include <stdint.h>

using namespace BLA;

//..... Defines .....//
#define DT_MSEC 10.00f
#define d2r (PI/180.00f)
#define r2d (180.00f/PI)


//..... SPI Pin Definitons .....//
const byte imuCSPin = 10;
const byte imuWAKPin = 7;
const byte imuINTPin = 8;
const byte imuRSTPin = 9;

//..... Lidar Definitions .....//
uint8_t garminAddress{0x62};

//.......... Structure Definitions ..........//
typedef struct
{
    float roll, pitch, yaw;
    float gx, gy, gz;
    float ax, ay, az;
    float qi, qj, qk, qw;
    float vz, z, zb; //this will be removed when imu_data_t is envoked
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

    Sensors(void){

    }

    void init(void)
    {
        fsm_init();
        lidar_init();
        //flow_init();
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

            //fsm->endCalibration(); //Turns off all calibration
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

            // data.ax = fsm->getAccelX();
            // data.ay = fsm->getAccelY();
            // data.az = fsm->getAccelZ();

            //... Gyro ...//
            fsm->getGyro(data.gx, data.gy, data.gz, data.gyroAccuracy);

            // data.gx = fsm->getFastGyroX();
            // data.gy = fsm->getFastGyroY();
            // data.gz = fsm->getFastGyroZ();

            // data.gx = fsm->getGyroX();
            // data.gy = fsm->getGyroY();
            // data.gz = fsm->getGyroZ();
            
            //... Rotation Vector ...//
            fsm->getQuat(data.qi, data.qj, data.qk, data.qw, data.quatRadianAccuracy, data.quatAccuracy);

            // data.qi = fsm->getQuatI();
            // data.qj = fsm->getQuatJ();
            // data.qk = fsm->getQuatK();
            // data.qw = fsm->getQuatReal();


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
    
    //... GARMIN LIDAR functions Begin ...//
    void sample_lidar(void)
    {
        uint8_t newDistance = 0;
        uint16_t distance;
    
        if (garmin->getBusyFlag() == 0)
        {
            // Trigger the next range measurement
            garmin->takeRange();

            // Read new distance data from device registers
            distance = garmin->readDistance();

            // Report to calling function that we have new data
            newDistance = 1;
        }
        //convert from cm to m
        data.zb = (float)distance/100.00f;
        data.z = data.zb; //remove this when rotate_to_world is being used
    }
    //... GARMIN LIDAR functions end ...//
    void fsm_init(void)
    {
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
        //fsm->enableGameRotationVector(DT_MSEC);
        fsm->enableGyro(DT_MSEC);  // rad/s
        //fsm->enableGyroIntegratedRotationVector(DT_MSEC);
        //fsm->enableMagnetometer(DT_MSEC);  // cannot be enabled at the same time as RotationVector (will not produce data)
        
        Serial.println("FSM Init Finished..."); 
        delay(300);
    }

    void lidar_init(void)
    {   
        Wire.begin();
        // "2" = short-range, fast speed
        // "0" = normal operation 
        garmin->configure(0, garminAddress);
    }

    void flow_init(void)
    {

    }
    //..... Object Definition .....//
    fsm_data_t data;

private:
    BNO080 * fsm;
    LIDARLite_v3HP * garmin;



    float IIR(float newSample, float prevOutput, float alpha)
    {
        return ( (1.0f-alpha)*newSample + alpha * prevOutput);
    }


    void rotate_to_world( float * vector )
    {

        float p = data.roll;
        float q = data.pitch;
        float u = data.yaw;

        Matrix<3,1> in = { vector[0], vector[1], vector[2] };
        Matrix<3,1> out = {0,0,0};

        Matrix<3,3> R = {   cos(q)*cos(u), sin(p)*sin(q)*cos(u)-cos(p)*sin(u), cos(p)*sin(q)*cos(u)+sin(p)*sin(u) ,
                            cos(q)*sin(u), sin(p)*sin(q)*sin(u)+cos(p)*cos(u), cos(p)*sin(q)*sin(u)-sin(p)*cos(u) ,
                            -sin(q),       sin(p)*cos(q),                      cos(p)*cos(q)                      };

        out = R * in;

        vector[0] = out(0);
        vector[1] = out(1);
        vector[2] = out(2);

    }

};


#endif
