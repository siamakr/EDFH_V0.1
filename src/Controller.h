///// Controller.h /////

#ifndef _EDFH_CONTROLLER_H
#define _EDFH_CONTROLLER_H

#include "Servo.h"
#include <Math.h>
#include <BasicLinearAlgebra.h>
#include <stdint.h>

// Teensy 4.1 Pin assignments
#define RW_PIN 37
#define EDF_PIN 36
#define YSERVO_PIN 33
#define XSERVO_PIN 14

//// Vehicle Specs + General Constants
#define COM_TO_TVC 0.1335
#define MASS 3.008                    //Kg
#define G 9.87

//// Vehicle's Minimums and Maximums 
#define MAX_ANGLE_SERVO 8.00f           //Deg
#define SERVO_MAX_SEC_PER_DEG 0.003333f      //dt/.003333 |(dt=0.1) = 3º
#define SERVO_MAX_SEC_PER_RAD 0.0095496f      //dt/.003333 |(dt=0.1) = 3º
#define SERVO_MAX_DEGREE_PER_DT 1.2
#define MAX_VEHICLE_ANGLE_DEG 35.00f
#define DEADBAND_ANGLE_DEG 0.001f
#define SERVO_ANG_TO_TVC_ANG 3.00f

//// EDF Params 
#define EDF_OFF_PWM 900                 //uSec
#define EDF_MIN_PWM 1500                //uSec
#define EDF_MAX_PWM 2000                //uSec
#define EDF_MAX_SUSTAINED_PWM 1730      //uSec
#define EDF_IDLE_PWM 1600               //uSec

//VALUES SET FOR +-15º GIMBAL ANGLE FOR BOTH X AND Y
#define SERVO_X_CENTER_US 1422
#define SERVO_Y_CENTER_US 1576
#define SERVO_X_MIN_US 1026
#define SERVO_Y_MIN_US 1026
#define SERVO_X_MAX_US 1900
#define SERVO_Y_MAX_US 1900

////////// Version 4 //////////
#define X_P1 0.1174
#define X_P2 33.20
#define X_P3 1422

#define Y_P1 -0.2077
#define Y_P2 -26.89
#define Y_P3 1576

////////// Version 3 //////////
//#define X_P1 0.1243
//#define X_P2 33.11
//#define X_P3 1428
//
//#define Y_P1 -0.2112
//#define Y_P2 -26.83
//#define Y_P3 1596

//EDF MOTOR RPM TO THRUST TO PWM TRANSFORMATORS
#define RAD2N_P1 0.018566536813619f    //Newtons to radians/s 
#define RAD2N_P2 -22.506778362213f     //Newtons to Radians/s
#define RAD2PWM_P1 0.2719786528784f    //Radians/s to PWM(us)  
#define RAD2PWM_P2 1010.29617153703f   //Radians/s to PWM(us)
#define RPM_TO_OMEGA (2.0f*PI/60.0f)        //RPM to Radians/s
#define OMEGA_TO_RPM (60.0f/(2.0f*PI))      //Radians/s to RPM
#define GRAMS_TO_NEWTONS (9.80f / 1000.00f) //Grams(g) to Newtons(N)

//MASS-MOMENT-OF-INERTIA OF VEHICLE
#define V_JXX 0.0058595f
#define V_JYY 0.0058595f
#define V_JZZ 0.01202768f
#define EDF_JZZ 0.0001744f      //MASS-MOMENT-OF-INERTIA OF EDF-PROP/MOTOR
#define RW_JZZ 0.00174245f      //MASS-MOMENT-OF-INERTIA OF REACTION WHEEL

using namespace BLA;

//..... Defines .....//
#define DT_MSEC 10.00f
#define d2r (PI/180.00f)
#define r2d (180.00f/PI)

typedef struct 
{
    int pwmx, pwmy, pwmedf, pwmrw;
    float angle_x, angle_xx, angle_y, angle_yy, Tm, Tx, Ty, Tz, Tedf;
    float trq_rw, trq_x, trq_y; 
    Matrix<8,1> e = {0.00f};
    Matrix<4,1> u = {0.00f};
    //add more data points as needed for debugging
}controller_data_t;


//...... Class Definition .....//
class Controller
{
    

public:

    controller_data_t cd; 


    Controller( void );
    
    void init(void);


    void hover(float r, float p, float y, float gx, float gy, float gz, float z, float vz);

    void actuate( void );


    void init_servos(void);


    void init_edf(void);


    void zero_servos();


    void prime_edf(void);



    void edf_shutdown(void);


    // to shut everything down if we go past max set angle
    void emergency_check(float r, float p);


    void suspend(void);


private:

    Servo sx; 
    Servo sy; 
    Servo edf;
    //Servo rw;
    
    //Write to X servo
    void writeXservo(float angle);

    //write to Y servo
    void writeYservo(float angle);

    //to write command to EDF ESC
    void writeEDF(float Ft);

    float limit(float value, float min, float max);

    float IIRF(float newSample, float prevOutput, float alpha);

    //TESTING VARIABLES //
    float int_gain{0};
    float int_z_gain{0};
    float n_gain_x{0.5100};     //ROLL GAIN
    float n_gain_y{0.5100};     //PITCH GAIN
    float g_gain_x{0.1230};     //GX GAIN
    float g_gain_y{0.1230};     //GY GAIN 
    float g_alpha{0.18};        //GYROSCOPE FILTER ALPHA
    float u_alpha{0.05};        //SERVO ACTUATOR SIGNAL FILTER ALPHA 

    float xsetpoint{0.00f};
    float ysetpoint{0.00f};
    float tinterval1{2.50f};
    float tinterval2{1.25f};


    Matrix<4,8> K = { n_gain_x,   -0.0000,    0.0000,  g_gain_x,   -0.0000,    0.0000,    0.0000,    0.0000,
                        0.0000,  n_gain_y,    0.0000,   -0.0000,  g_gain_y,   -0.0000,    0.0000,    0.0000,
                       -0.0000,    0.0000,   -0.0316,   -0.0000,    0.0000,   -0.0561,    0.0000,    0.0000,
                        0.0000,    0.0000,    0.0000,   -0.0000,    0.0000,    0.0000,     5.42,   3.0942};

};


#endif
