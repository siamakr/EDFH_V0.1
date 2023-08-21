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
#define DT_MSEC 5.00f
#define DT_SEC (0.0050)
#define d2r (PI/180.00f)
#define r2d (180.00f/PI)

#define FSM_PITCH_OFFSET_RAD (d2r * 1.3772)
#define FSM_ROLL_OFFSET_RAD (d2r * 0.6578f)
#define FSM_YAW_OFFSET_RAD (d2r * 0.00f)
// #define FSM_ROLL_OFFSET_RAD (d2r * -1.28331)
// #define FSM_PITCH_OFFSET_RAD (d2r * -2.18253)


//..... SPI Pin Definitons .....//
const byte imuCSPin = 10;
const byte imuWAKPin = 7;
const byte imuINTPin = 8;
const byte imuRSTPin = 9;

const float roll_offset{d2r*1.3772};
const float pitch_offset{d2r*0.6578f};

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
 
    Matrix<6,6> H = {   0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f,   
                        0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 
                        0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 
                        0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 
                        0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 
                        0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f}; 



    // State vector
    Matrix<6,1> Xe= {0.00f,0.00f,0.00f,0.00f,0.00f,0.00f} ;

    // Prediction vector
    Matrix<6,1> Xpre = {0.00f,0.00f,0.00f,0.00f,0.00f,0.00f};  

    // Measurement vector
    Matrix<6,1> Z = {0.00f,0.00f,0.00f,0.00f,0.00f,0.00f};

    // Input vector
    Matrix<3,1> U_est = {0.00f,0.00f,0.00f};

    // Estimator gain
     Matrix<6,6> Kf = {  0.0001,    0.0000,    0.0000,    0.0095,   -0.0000,    0.0000,
                        0.0000,    0.0001,   -0.0000,    0.0000,    0.0095,   -0.0000,
                        0.0000,   -0.0000,    0.1547,    0.0000,   -0.0000,    0.0000,
                        0.0000,    0.0000,    0.0000,    0.1057,   -0.0000,    0.0000,
                       -0.0000,    0.0000,   -0.0000,   -0.0000,    0.1057,   -0.0000,
                        0.0000,   -0.0000,    1.3001,    0.0000,   -0.0000,    0.0000  }; 

    //  Matrix<6,6> Kf = {  0.0345,    0.0000,   0.0000,    0.0019,    0.0000,    0.0000,
    //                     0.0000,    0.0345,   0.0000,    0.0000,    0.0019,    0.0000,
    //                     0.0000,    0.0000,   0.0274,    0.0000,    0.0000,    0.0001,
    //                     0.1495,    0.0000,   0.0000,    0.0216,    0.0000,    0.0000,
    //                     0.0000,    0.1495,   0.0000,    0.0000,    0.0216,    0.0000,
    //                     0.0000,    0.0000,   0.0767,    0.0000,    0.0000,    0.0004    }; 

    // Matrix<6,6> Kf = {  0.618520, 0.000000, 0.000000, 0.000330, 0.000000, 0.000000,
    //                     0.000000, 0.618520, 0.000000, 0.000000, 0.000330, 0.000000,
    //                     0.000000, 0.000000, 0.318880, 0.000000, 0.000000, 0.000420,
    //                     0.162570, 0.000000, 0.000000, 0.131210, 0.000000, 0.000000,
    //                     0.000000, 0.162570, 0.000000, 0.000000, 0.131210, 0.000000,
    //                     0.000000, 0.000000, 4.190280, 0.000000, 0.000000, 0.045440   };

    
        //FILTER PARAMS
    float _alpha_gyro{0.10};                //GYROSCOPE FILTER ALPHA
    float _alpha_servo{0.07};               //SERVO ACTUATOR SIGNAL FILTER ALPHA 
    float _alpha_accel{0.05};               //ACCELEROMETER SIGNAL FILTER ALPHA 





    
    Sensors(void);

    void init(void);

    void fsm_init(void);

    void lidar_init(void);

    void flow_init(void);

    void save_calibrate_fsm(void);
 
    void sample_fsm(void);

    void sample_lidar(void);

    void run_estimator(void);

    void rotate_to_world( float * vector );

    void set_origin(void);
    
    void update_pos( float x, float y );

    float rotate_yaw(float yaw);
    
    float IIR(float newSample, float prevOutput, float alpha);

    void print_debug(void);

    void print_imu(void);

    void print_fsm(void);

    void print_estimator(void);





private:

};









#endif
