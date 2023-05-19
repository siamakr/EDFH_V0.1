///// sensors.h /////

#ifndef _EDFH_SENSORS_H
#define _EDFH_SENSORS_H

#include <Wire.h>
#include <SPI.h>
#include "BNO080.h"
#include "LIDARLite_v3HP.h"
#include <Math.h>
#include <BasicLinearAlgebra.h>
#include <stdint.h>

using namespace BLA;

//..... Defines .....//
#define DT_MSEC 10.00f
#define DT_SEC (DT_MSEC/1000)
#define d2r (PI/180.00f)
#define r2d (180.00f/PI)


//..... SPI Pin Definitons .....//
const byte imuCSPin = 10;
const byte imuWAKPin = 7;
const byte imuINTPin = 8;
const byte imuRSTPin = 9;

//..... Lidar Definitions .....//
//uint8_t garminAddress{0x62};

//.......... Structure Definitions ..........//
typedef struct
{
    float roll, pitch, yaw;
    float gx, gy, gz;
    float ax, ay, az;
    float qi, qj, qk, qw;
    float z; //this will be removed when imu_data_t is envoked
    byte linAccuracy{0};
    byte gyroAccuracy{0};
    //byte magAccuracy{0};
    float quatRadianAccuracy{0};
    byte quatAccuracy{0}; 
} fsm_data_t;

typedef struct
{
    float x, y, z;
    float vx, vy, vz; 

} estimater_data_t;


//...... Class Definition .....//
class Sensors
{    
    //..... Object Definition .....//



public:
            fsm_data_t data;
    estimater_data_t estimate;
        BNO080 fsm;
    LIDARLite_v3HP garmin;
    uint16_t distance;
    
    Sensors(void);


    void init(void);

        //... GARMIN LIDAR functions end ...//
    void fsm_init(void);


    void lidar_init(void);


    void flow_init(void);


    ///.......... FSM305/BNO080 FUNCTIONS ..........////

    void save_calibrate_fsm(void);
 

    void sample_fsm(void);



    void print_fsm(void);


    void print_fsm_calibration(void);

    void print_estimator(void);


    //... FSM305 Functions End ...//
    
    //... GARMIN LIDAR functions Begin ...//
    void sample_lidar(void);


    void run_estimator(void);

    void rotate_to_world( float * vector );
    
    float IIR(float newSample, float prevOutput, float alpha);


//     Sensors(void){

//     }

//     void init(void)
//     {
//         //fsm_init();
//         lidar_init();
//         fsm_init();

//         //flow_init();
//     }
//         //... GARMIN LIDAR functions end ...//
//     void fsm_init(void)
//     {
//                 //..... FSM305/BNO080 Init .....//
//         Serial.print("FSM Init start..."); 
//         //delay(300);

//         if (fsm.beginSPI(imuCSPin, imuWAKPin, imuINTPin, imuRSTPin, 3000000) == false)
//         {
//             Serial.println("BNO080 over SPI not detected. Are you sure you have all 6 connections? Freezing...");
//             while(1);
//         }

//         fsm.calibrateAll();
//         fsm.enableLinearAccelerometer(DT_MSEC);  // m/s^2 no gravity
//         fsm.enableRotationVector(DT_MSEC);  // quat
//         //fsm.enableGameRotationVector(DT_MSEC);
//         fsm.enableGyro(DT_MSEC);  // rad/s
//         //fsm.enableGyroIntegratedRotationVector(DT_MSEC);
//         //fsm.enableMagnetometer(DT_MSEC);  // cannot be enabled at the same time as RotationVector (will not produce data)
        
//         Serial.println("FSM Init Finished..."); 
//         delay(300);
//     }

//     void lidar_init(void)
//     {   
//         Wire.begin();
//         delay(100);
//         // "2" = short-range, fast speed
//         // "0" = normal operation 
//         garmin.configure(0, garminAddress);
//         delay(200);
//     }

//     void flow_init(void)
//     {

//     }

//     ///.......... FSM305/BNO080 FUNCTIONS ..........////

//     void save_calibrate_fsm(void)
//     {
//         if(Serial.available())
//         {
//             byte incoming = Serial.read();

//             if(incoming == 's')
//             {
//                 fsm.saveCalibration(); //Saves the current dynamic calibration data (DCD) to memory
//                 fsm.requestCalibrationStatus(); //Sends command to get the latest calibration status

//                 //Wait for calibration response, timeout if no response
//                 int counter = 100;
            
//                 while(1)
//                 {
//                     if(--counter == 0) break;
//                     if(fsm.dataAvailable() == true)
//                 {
//                     //The IMU can report many different things. We must wait
//                     //for the ME Calibration Response Status byte to go to zero
//                     if(fsm.calibrationComplete() == true)
//                     {
//                         Serial.println("Calibration data successfully stored");
//                         delay(1000);
//                         break;
//                     }
//                 }

//                 delay(1);
//             }

//             if(counter == 0) Serial.println("Calibration data failed to store. Please try again.");

//             //fsm.endCalibration(); //Turns off all calibration
//             //In general, calibration should be left on at all times. The BNO080
//             //auto-calibrates and auto-records cal data roughly every 5 minutes
//             }
//         } 
//     } 

//     void sample_fsm(void)
//     {
//         if(fsm.dataAvailable() == true)
//         {
//             //..... Sample IMU .....//
//             //... Linear Accel ...//
//             fsm.getLinAccel(data.ax, data.ay, data.az, data.linAccuracy);

//             // data.linAccuracy = fsm.getLinAccelAccuracy();
//             // data.ax = IIR(fsm.getLinAccelX(), data.ax, .30);
//             // data.ay = IIR(fsm.getLinAccelY(), data.ay, .30);
//             // data.az = IIR(fsm.getLinAccelZ(), data.az, .30);

//             //... Gyro ...//
//            // fsm.getGyro(data.gx, data.gy, data.gz, data.gyroAccuracy);

//             // data.gx = fsm.getFastGyroX();
//             // data.gy = fsm.getFastGyroY();
//             // data.gz = fsm.getFastGyroZ();
//             data.gyroAccuracy = fsm.getGyroAccuracy();
//             data.gx = IIR(fsm.getGyroX(), data.gx, .10);
//             data.gy = IIR(fsm.getGyroY(), data.gy, .10);
//             data.gz = IIR(fsm.getGyroZ(), data.gz, .10);
            
//             //... Rotation Vector ...//
//             fsm.getQuat(data.qi, data.qj, data.qk, data.qw, data.quatRadianAccuracy, data.quatAccuracy);

//             // data.qi = fsm.getQuatI();
//             // data.qj = fsm.getQuatJ();
//             // data.qk = fsm.getQuatK();
//             // data.qw = fsm.getQuatReal();


//             //... Euler Angle Representation ...//
//             this->data.roll = fsm.getRoll();
//             this->data.pitch = fsm.getPitch();
//             this->data.yaw = fsm.getYaw();    

//         }
//     }


//     void print_fsm(void)
//     {
//         char text[200];
//         sprintf(text, "%0.5f, %0.5f, %0.5f,\t  %0.5f, %0.5f, %0.5f, \t  %0.5f, %0.5f, %0.5f    ",
//         r2d*data.roll,
//         r2d*data.pitch,
//         r2d*data.yaw,
//         r2d*data.gx,
//         r2d*data.gy,
//         r2d*data.gz,
//         data.ax,
//         data.ay,
//         data.az);

//         Serial.println(text);
//     }

//     void print_fsm_calibration(void)
//     {
//         char text[250];

//         sprintf(text, "%0.5f, %0.5f, %0.5f,\t   %0.5f, %0.5f, %0.5f, %0.5f,\t  %0.5f, %0.5f, %0.5f, \t  %0.5f, %0.5f, %0.5f    ",
//         r2d*data.roll,
//         r2d*data.pitch,
//         r2d*data.yaw,
//         data.qi,
//         data.qj,
//         data.qk,
//         data.qw,
//         r2d*data.gx,
//         r2d*data.gy,
//         r2d*data.gz,
//         data.ax,
//         data.ay,
//         data.az);

//         Serial.print(text);
//         Serial.print(data.linAccuracy);
//         Serial.print(", ");
//         Serial.print(data.gyroAccuracy);
//         Serial.print(", ");
//         Serial.print(data.quatAccuracy);
//         Serial.println(", ");
//     }
//         void print_estimator(void)
//     {
//         char text[250];

//         sprintf(text, "%0.5f, %0.5f, %0.5f,\t   %0.5f, %0.5f, %0.5f,\t  %0.5f, %0.5f, %0.5f, \t  %0.5f, %0.5f, %0.5f ",
//         r2d*data.roll,
//         r2d*data.pitch,
//         r2d*data.yaw,
//         data.z,
//         float(distance),
//         estimate.z,
//         estimate.vx,
//         estimate.vy,
//         estimate.vz,
//         data.ax,
//         data.ay,
//         data.az
//         );

//         Serial.print(text);
//         Serial.print(data.linAccuracy);
//         Serial.print(", ");
//         Serial.print(data.gyroAccuracy);
//         Serial.print(", ");
//         Serial.print(data.quatAccuracy);
//         Serial.println(", ");
//     }

//     //... FSM305 Functions End ...//
    
//     //... GARMIN LIDAR functions Begin ...//
//     void sample_lidar(void)
//     {
//         //uint8_t newDistance = 0;
//         //uint16_t distance{0};
    
//         if (garmin.getBusyFlag() == 0)
//         {
//             // Trigger the next range measurement
//             garmin.takeRange();

//             // Read new distance data from device registers
//             data.z = (float)(garmin.readDistance()/100.00f);

//             // Report to calling function that we have new data
//          //   newDistance = 1;
//         }
//         //convert from cm to m
//        // data.z = (float)distance/100.00f;
//     }

//     void run_estimator(void){

//     /* ---- Sensor processing ---- */
//     float p[3] = {0}; // Position vector (z body to world)
//     //float v[3] = {0}; // Velocity vector (vx, vy to world)
//     float a[3] = {0}; // Acceleration vector (ax, ay, az to world)

//     // Rotate lidar measurement to world frame
//     p[2] = data.z;
//     rotate_to_world( p );

//     // Perform gyrocompensation on flow and rotate to world frame.
//     // v[0] = data.vx * p[2] + data.gy * p[2];
//     // v[1] = data.vy * p[2] - data.gx * p[2]; 
//     // rotate_to_world( v );

//     // Rotate acceleration to world frame
//     a[0] = data.ax; a[1] = data.ay; a[2] = data.az;
//     rotate_to_world( a );


//     /* ---- Estimation ---- */
//     // Fill input vector with acceleration
//     H.Fill(0);
//     Z.Fill(0);
//     U = { a[0], a[1], a[2] };

//     // Fill measurement vector with data
//    // if( data.status.pos == 1 ){
//         // H(0,0) = 1; H(1,1) = 1;
//         // Z(0) = data.x;
//         // Z(1) = data.y;
//     //}

//    // if( data.status.lidar == 1){
//         H(2,2) = 1;
//         Z(2) = p[2]; // p[2]: z
//     //}

//     // if( data.status.flow == 1){
//     //     H(3,3) = 1; H(4,4) = 1;
//     //     Z(3) = v[0]; // vx
//     //     Z(4) = v[1]; // vy
//     // }

//     // Prediction, based on previous state and current input
//     Xpre = A*X + B*U; 

//     // Update prediction with update using measurements 
//     X = Xpre + Kf*(Z - H*Xpre); 
    
//     // Fill estimate struct with values (for telemetry and stuff)
//     estimate.x = X(0);
//     estimate.y = X(1);
//     estimate.z = X(2);
//     estimate.vx = X(3);
//     estimate.vy = X(4);
//     estimate.vz = X(5);

//     // Reset status (measurements has been used!)
//     // data.status.flow = 0;
//     // data.status.lidar = 0;
//     // data.status.imu = 0;
//     // data.status.pos = 0;
// }
//     void rotate_to_world( float * vector )
//     {

//          float p = this->data.roll;
//          float q = this->data.pitch;
//          float u = this->data.yaw;
//         distance = p;

//         Matrix<3,1> in = { vector[0], vector[1], vector[2] };
//         Matrix<3,1> out = {0,0,0};

//         Matrix<3,3> R = {   cos(q)*cos(u), sin(p)*sin(q)*cos(u)-cos(p)*sin(u), cos(p)*sin(q)*cos(u)+sin(p)*sin(u) ,
//                             cos(q)*sin(u), sin(p)*sin(q)*sin(u)+cos(p)*cos(u), cos(p)*sin(q)*sin(u)-sin(p)*cos(u) ,
//                             -sin(q),       sin(p)*cos(q),                      cos(p)*cos(q)                      };

//         out = R * in;

//         vector[0] = out(0);
//         vector[1] = out(1);
//         vector[2] = out(2);

//     }

//     float IIR(float newSample, float prevOutput, float alpha)
//     {
//         return ( (1.0f-alpha)*newSample + alpha * prevOutput);
//     }




private:





        // Estimator matrixes
    Matrix<6,6> A = {   1,  0,  0,  DT_SEC, 0,  0,
                        0,  1,  0,  0,  DT_SEC, 0,
                        0,  0,  1,  0,  0,  DT_SEC,
                        0,  0,  0,  1,  0,  0,
                        0,  0,  0,  0,  1,  0, 
                        0,  0,  0,  0,  0,  1 };

    Matrix<6,3> B = {   0.5*pow(DT_SEC,2),  0,              0,         
                        0,              0.5*pow(DT_SEC,2),  0,         
                        0,              0,              0.5*pow(DT_SEC,2),  
                        DT_SEC,             0,              0,         
                        0,              DT_SEC,             0,         
                        0,              0,              DT_SEC };
 
    Matrix<6,6> H; 

    // State vector
    Matrix<6,1> X = {0,0,0,0,0,0};

    // Prediction vector
    Matrix<6,1> Xpre;

    // Measurement vector
    Matrix<6,1> Z = {0,0,0,0,0,0};

    // Input vector
    Matrix<3,1> U = {0,0,0};

    // Estimator gain
    /* Matrix<6,6> Kf = {  0.0001,    0.0000,    0.0000,    0.0095,   -0.0000,    0.0000,
                        0.0000,    0.0001,   -0.0000,    0.0000,    0.0095,   -0.0000,
                        0.0000,   -0.0000,    0.1547,    0.0000,   -0.0000,    0.0000,
                        0.0000,    0.0000,    0.0000,    0.1057,   -0.0000,    0.0000,
                       -0.0000,    0.0000,   -0.0000,   -0.0000,    0.1057,   -0.0000,
                        0.0000,   -0.0000,    1.3001,    0.0000,   -0.0000,    0.0000  }; */

    /* Matrix<6,6> Kf = {  0.0345,    0.0000,   0.0000,    0.0019,    0.0000,    0.0000,
                        0.0000,    0.0345,   0.0000,    0.0000,    0.0019,    0.0000,
                        0.0000,    0.0000,   0.0274,    0.0000,    0.0000,    0.0001,
                        0.1495,    0.0000,   0.0000,    0.0216,    0.0000,    0.0000,
                        0.0000,    0.1495,   0.0000,    0.0000,    0.0216,    0.0000,
                        0.0000,    0.0000,   0.0767,    0.0000,    0.0000,    0.0004    }; */

    Matrix<6,6> Kf = {  0.618520, 0.000000, 0.000000, 0.000330, 0.000000, 0.000000,
                        0.000000, 0.618520, 0.000000, 0.000000, 0.000330, 0.000000,
                        0.000000, 0.000000, 0.318880, 0.000000, 0.000000, 0.000420,
                        0.162570, 0.000000, 0.000000, 0.131210, 0.000000, 0.000000,
                        0.000000, 0.162570, 0.000000, 0.000000, 0.131210, 0.000000,
                        0.000000, 0.000000, 4.190280, 0.000000, 0.000000, 0.045440   };

};


#endif
