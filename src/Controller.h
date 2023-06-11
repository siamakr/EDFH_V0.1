///// Controller.h /////

#ifndef _EDFH_CONTROLLER_H
#define _EDFH_CONTROLLER_H

#include "Actuator.h"
#include "Servo.h"
#include "Sensors.h"
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
#define ledf .050
#define MASS 2.757                    //Kg
#define MAX_TVC_DEFLECTION_DEG 10.00f
#define MAX_TVC_DEFLECTION_RAD (d2r * MAX_TVC_DEFLECTION_DEG)
#define G 9.87


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
    Matrix<12,1> e_int = {0.00f};
    Matrix<4,1> u = {0.00f};
    //add more data points as needed for debugging
}controller_data_t;

typedef enum{
    CONTROL_STATUS_STATIONARY = 0,
    CONTROL_STATUS_FLYING,
    CONTROL_STATUS_LANDING,
} control_status_t; 

typedef enum{
    SETPOINT_X = 0,
    SETPOINT_Y,
    SETPOINT_Z,
    SETPOINT_ROLL,
    SETPOINT_PITCH,
    SETPOINT_YAW
} control_setpoint_t;


//...... Class Definition .....//
class Controller
{
public:
    Actuator act;
    controller_data_t cd;
    control_setpoint_t setpoint; 
        Matrix<8,1> SP_hover = {0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00};    //Desired Reference
    Matrix<12,1> SP_hover_int = {0.00};    //Desired Reference


    Controller( void );
    
    void init(void);

    void edf_startup(int seconds);

    void lqr(float r, float p, float y, float gx, float gy, float gz, float z, float vz);
    
    void lqr_int(float r, float p, float y, float gx, float gy, float gz, float z, float vz);

    void actuate( float angle_x_rad, float angle_y_rad, float thrust_force_newton );

    void actuate_servos(float angle_x_rad, float angle_y_rad);

    void actuate_edf( float thrust_force_newton );

    void set_reference(control_setpoint_t cs, float value);




private:


    float limit(float value, float min, float max);

    float IIRF(float newSample, float prevOutput, float alpha);

    //LQR Gains 
    //Use these for gain scheduling //
    float _gain_roll{0.5100};               //ROLL GAIN
    float _gain_pitch{_gain_roll};          //PITCH GAIN
    float _gain_yaw{.0316};                 //YAW GAIN

    float _gain_gx{0.1230};                 //GX GAIN
    float _gain_gy{_gain_gx};               //GY GAIN 
    float _gain_gz{.0561};                  //GZ GAIN

    float _gain_vz{5.42};                   //ALT VELOCITY
    float _gain_z{3.0942};                  //ALTITUDE
    float _gain_z_int{.1};                    //ALTITUDE INTEGRAL GAIN

    float _gain_roll_int{.1};                 //ROLL INTEGRAL GAIN
    float _gain_pitch_int{.1};                //PITCH INTEGRAL GAIN       
    float _gain_yaw_int{.1};                  //YAW INTEGRAL GAIN

    float _int_bound_att{d2r * 8.00f};
    float _int_bound_alt{0.850f};


    //FILTER PARAMS
    float _alpha_gyro{0.18};                //GYROSCOPE FILTER ALPHA
    float _alpha_actuator{0.05};            //SERVO ACTUATOR SIGNAL FILTER ALPHA 

    float xsetpoint{0.00f};
    float ysetpoint{0.00f};
    float tinterval1{2.50f};
    float tinterval2{1.25f};



    //Attitude Only LQR gain Matrix "K" 
    // Matrix<4,8> K = { n_gain_x,   -0.0000,    0.0000,  g_gain_x,   -0.0000,    0.0000,    0.0000,    0.0000,
    //                     0.0000,  n_gain_y,    0.0000,   -0.0000,  g_gain_y,   -0.0000,    0.0000,    0.0000,
    //                    -0.0000,    0.0000,   -0.0316,   -0.0000,    0.0000,   -0.0561,    0.0000,    0.0000,
    //                     0.0000,    0.0000,    0.0000,   -0.0000,    0.0000,    0.0000,     5.42,   3.0942};

    Matrix<4,8> K = {       _gain_roll,     -0.0000,        0.0000,     _gain_gx,   -0.0000,    0.0000,     0.0000,     0.0000,
                            0.0000,         _gain_pitch,    0.0000,     -0.0000,    _gain_gy,   -0.0000,    0.0000,     0.0000,
                           -0.0000,         0.0000,         _gain_yaw,  -0.0000,    0.0000,     _gain_gz,   0.0000,     0.0000,
                            0.0000,         0.0000,         0.0000,     -0.0000,    0.0000,     0.0000,     _gain_vz,   _gain_z};


    //Attitude+Altitude LQR gain matrix "K_int" with full state integral action (for each output "U")
                    //      ROLL        PITCH       YAW     gx          gy          gz          vz          z       z_int     Roll_i     Pitch_i     Yaw_i          
    Matrix<4,12> K_int = { _gain_roll,      -0.0000,        0.0000,     _gain_gx,   -0.0000,    0.0000,     0.0000,     0.0000,     -0.0000,        _gain_roll_int, 0.0000,             0.0000,
                            0.0000,         _gain_pitch,    0.0000,     -0.0000,    _gain_gy,   -0.0000,    0.0000,     0.0000,     -0.0000,        0.0000,         _gain_pitch_int,    0.0000,
                           -0.0000,         0.0000,         _gain_yaw,  -0.0000,    0.0000,     _gain_gz,   0.0000,     0.0000,     -0.0000,        0.0000,         0.0000,             _gain_yaw_int,
                            0.0000,         0.0000,         0.0000,     -0.0000,    0.0000,     0.0000,     _gain_vz,   _gain_z,    _gain_z_int,    0.0000,         0.0000,             0.0000};

};


#endif
