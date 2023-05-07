///// Sensors.h /////

#ifndef _EDFH_SENSORS_H
#define _EDFH_SENSORS_H

// #include <Wire.h>
// #include <SPI.h>
#include <Math.h>
#include <BasicLinearAlgebra.h>
#include <stdint.h>
#include "BNO080.h"

using namespace BLA;

//..... Defines .....//
#define DT_MSEC 10.00f
#define d2r (PI/180.00f)
#define r2d (180.00f/PI)


//..... SPI Pin Definitons .....//
byte imuCSPin = 10;
byte imuWAKPin = 7;
byte imuINTPin = 8;
byte imuRSTPin = 9;

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

//..... Object Definition .....//
fsm_data_t data; 
BNO080 * fsm;


//..... Timing Vars.....//
float sensor_timer, print_timer;

//...... Class Definition .....//
class Sensors
{
    
public:

    Sensors( void );
    
    // Init the sensor objects
    void init( void );

    //... FSM305 Functions Begin ...//
    void sample_fsm(void);
    
    void print_fsm(void);
    
    void save_calibrate_fsm(void); 

    void print_fsm_calibration(void);

    //... FSM305 Functions End ...//


private:

    
    // adafruit_bno055_offsets_t bnoOffset;

    // float IIR( float newSample, float prevOutput, float alpha);

    // void rotate_to_world( float * vector );

    


};


#endif