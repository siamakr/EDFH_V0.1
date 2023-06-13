///// Sensors.h /////

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
#define DT_SEC (0.01)
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
    float x, y, z; //this will be removed when imu_data_t is envoked
    float evx, evy, evz;
    float ex, ey, ez;
    byte linAccuracy{0};
    byte gyroAccuracy{0};
    //byte magAccuracy{0};
    float quatRadianAccuracy{0};
    byte quatAccuracy{0}; 
    
    struct{ // Bitfield, using 1 byte, to represent if new measurements are available
        uint8_t imu     : 1;
        uint8_t flow    : 1;
        uint8_t lidar   : 1;
        uint8_t pos     : 1;
    } status;
    
} fsm_data_t;

typedef struct
{
    float x, y, z;
    float vx, vy, vz; 

} estimator_data_t;



//...... Class Definition .....//
class Sensors
{    
    //..... Object Definition .....//



public:

    fsm_data_t data;
    estimator_data_t estimate;
    BNO080 fsm;
    LIDARLite_v3HP garmin;
    uint16_t distance;

    float yaw_origin;
    
    Matrix<6,6> A = {   1.00f, 0.00f, 0.00f, 0.01f, 0.00f, 0.00f,   
                        0.00f, 1.00f, 0.00f, 0.00f, 0.01f, 0.00f, 
                        0.00f, 0.00f, 1.00f, 0.00f, 0.00f, 0.01f, 
                        0.00f, 0.00f, 0.00f, 1.00f, 0.00f, 0.00f, 
                        0.00f, 0.00f, 0.00f, 0.00f, 1.00f, 0.00f, 
                        0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 1.00f}; 

    Matrix<6,3> B = {   0.00005f, 0.00f, 0.00f,   
                        0.00f, 0.00005f, 0.00f, 
                        0.00f, 0.00f, 0.00005f, 
                        0.01f, 0.00f, 0.00f, 
                        0.00f, 0.01f, 0.00f, 
                        0.00f, 0.00f, 0.01f}; 
 
    Matrix<6,6> H = {   0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f,   
                        0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 
                        0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 
                        0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 
                        0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 
                        0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f}; 



    // State vector
    Matrix<6,1> Xe= {01.00f,01.00f,01.00f,01.00f,01.00f,01.00f} ;

    // Prediction vector
    Matrix<6,1> Xpre = {0.00f,0.00f,0.00f,0.00f,0.00f,0.00f};  

    // Measurement vector
    Matrix<6,1> Z = {0.00f,0.00f,0.00f,0.00f,0.00f,0.00f};

    // Input vector
    Matrix<3,1> U_est = {0.00f,0.00f,0.00f};

    // Estimator gain
    /* Matrix<6,6> Kf = {  0.0001,    0.0000,    0.0000,    0.0095,   -0.0000,    0.0000,
                        0.0000,    0.0001,   -0.0000,    0.0000,    0.0095,   -0.0000,
                        0.0000,   -0.0000,    0.1547,    0.0000,   -0.0000,    0.0000,
                        0.0000,    0.0000,    0.0000,    0.1057,   -0.0000,    0.0000,
                       -0.0000,    0.0000,   -0.0000,   -0.0000,    0.1057,   -0.0000,
                        0.0000,   -0.0000,    1.3001,    0.0000,   -0.0000,    0.0000  }; */


     Matrix<6,6> Kf = {  0.0345,    0.0000,   0.0000,    0.0019,    0.0000,    0.0000,
                        0.0000,    0.0345,   0.0000,    0.0000,    0.0019,    0.0000,
                        0.0000,    0.0000,   0.0274,    0.0000,    0.0000,    0.0001,
                        0.1495,    0.0000,   0.0000,    0.0216,    0.0000,    0.0000,
                        0.0000,    0.1495,   0.0000,    0.0000,    0.0216,    0.0000,
                        0.0000,    0.0000,   0.0767,    0.0000,    0.0000,    0.0004    };





    
    Sensors(void);

    void init(void);

    void fsm_init(void);

    void lidar_init(void);

    void flow_init(void);

    void save_calibrate_fsm(void);
 
    void sample_fsm(void);

    void print_fsm(void);

    void print_fsm_calibration(void);

    void print_estimator(void);

    void sample_lidar(void);

    void run_estimator(void);

    void rotate_to_world( float * vector );

    float rotate_yaw(float yaw);
    
    float IIR(float newSample, float prevOutput, float alpha);





private:

};









#endif
