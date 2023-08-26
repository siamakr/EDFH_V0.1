///// Controller.h /////

#ifndef _EDFH_CONTROLLER_H
#define _EDFH_CONTROLLER_H

#include "Actuator.h"
#include "Servo.h"
#include "Sensors.h"
#include <Math.h>
#include <BasicLinearAlgebra.h>
#include <stdint.h>


//// Vehicle Specs + General Constants
#define COM_TO_TVC 0.1335                                       //m
#define ledf .050                                               //m
#define MASS_EDF .700                                           //Kg
#define MASS 2.17                                              //Kg
//#define MASS 2.9                                              //Kg
#define MAX_TVC_DEFLECTION_DEG 5.00f                           //deg
#define MAX_TVC_DEFLECTION_RAD (d2r * MAX_TVC_DEFLECTION_DEG)   //rad
#define G 9.87                                                  //m/s^2


//MASS-MOMENT-OF-INERTIA OF VEHICLE
#define V_JXX 0.0058595f
#define V_JYY 0.0058595f
#define V_JZZ 0.01202768f
#define EDF_JZZ 0.0001744f      //MASS-MOMENT-OF-INERTIA OF EDF-PROP/MOTOR
#define RW_JZZ 0.00174245f      //MASS-MOMENT-OF-INERTIA OF REACTION WHEEL

using namespace BLA;

//..... Defines .....//
//#define DT_MSEC 5.00f
#define d2r (PI/180.00f)
#define r2d (180.00f/PI)

typedef struct 
{
    int pwmx, pwmy, pwmedf, pwmrw;
    float angle_x, angle_xx, angle_y, angle_yy, Tm, Tx, Ty, Tz, Tedf;
    float delta_x, delta_y, delta_xx, delta_yy;
    float trq_rw, trq_x, trq_y; 
    Matrix<8,1> e = {0.00f};
    Matrix<12,1> e_int = {0.00f};
    Matrix<4,1> u = {0.00f};
    Matrix<4,1> u_output = {0.00f};
    //add more data points as needed for debugging
}controller_data_t;

typedef enum{
    CONTROL_STATUS_STATIONARY = 0,
    CONTROL_STATUS_EDF_PRIMING,
    CONTROL_STATUS_FLYING,
    CONTROL_STATUS_LANDING,
    CONTROL_STATUS_IMU_CALIBRATION
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
    Matrix<8,1> SP_hover = {0.00,0.00,0.00,0.00,0.00,0.00,0.60,0.00};    //Desired Reference
    Matrix<12,1> SP_hover_int = {0.00f,0.00f,0.00f,  0.00f,0.00f,0.00f,  0.650f,0.00f,0.00f,  0.00f,0.00f,0.00f};    //Desired Reference
    control_status_t status;


    Controller( void );
    
    void init(void);

    bool edf_startup(int seconds);

    void lqr(float r, float p, float y, float gx, float gy, float gz, float z, float vz);
    
    void lqr_int(float r, float p, float y, float gx, float gy, float gz, float z, float vz);

    void lqr_pos( float x, float y, float vx, float vy, float yaw );

    void actuate( float angle_x_rad, float angle_y_rad, float thrust_force_newton );

    void actuate_servos(float angle_x_rad, float angle_y_rad);

    void actuate_edf( float thrust_force_newton );

    void set_reference(control_setpoint_t cs, float value);

    void print_debug(void);

    // Position state vector
    Matrix<6,1> X_pos = {0,0,0,0,0,0};

    // Setpoints for position controller (x, y, vx, vy, xint, yint)
    Matrix<6,1> SP_pos = {0,0,0,0,0,0};

    // Output from position controller
    Matrix<2,1> U_pos = {0,0};


private:


    float limit(float value, float min, float max);
    void LIMIT(float & value, float min, float max);

    float IIRF(float newSample, float prevOutput, float alpha);

    void IIR(float & new_sample, float prev_output, float alpha);
    //Feedforward gains
    volatile float _gain_ff_roll{-0.00005};
    volatile float _gain_ff_pitch{-0.00005};
    //LQR Gains 
    //Use these for gain scheduling //
    volatile float _gain_roll{0.300};               //ROLL GAIN
    volatile float _gain_pitch{_gain_roll};          //PITCH GAIN
    volatile float _gain_yaw{.116};                 //YAW GAIN

    volatile float _gain_gx{0.1100};                 //GX GAIN
    volatile float _gain_gy{_gain_gx};               //GY GAIN 
    volatile float _gain_gz{.361};                  //GZ GAIN

    volatile float _gain_z{10.10};                   //ALT VELOCITY
    volatile float _gain_vz{23.6942};                  //ALTITUDE
    volatile float _gain_z_int{0.00f};                  //ALTITUDE INTEGRAL GAIN

    volatile float _gain_roll_int{0.5};              //ROLL INTEGRAL GAIN
    volatile float _gain_pitch_int{0.5};             //PITCH INTEGRAL GAIN       
    volatile float _gain_yaw_int{-.0001};               //YAW INTEGRAL GAIN

    volatile float _int_bound_att{d2r * 2.00f};
    volatile float _int_bound_alt{0.050f};
    volatile float _max_int_def{d2r*2.00f};

    volatile float _alpha_servo{0.080};               //SERVO ACTUATOR SIGNAL FILTER ALPHA 

    float error_integral_x{0};
    float error_integral_y{0};



    /* Matrix<2,6> K_pos = {     0.0000,  -0.1368,   0.0000,  -0.1744,   0.0000,  -0.0250,
                              0.1368,   0.0000,   0.1744,   0.0000,   0.0250,   0.0000,  }; */

    Matrix<2,6> K_pos = {   0.0000,  0.1531,   0.0000,  0.1700,   0.0000,  0.0500,
                            -0.1531,   0.0000,   -0.1700,   0.0000,   0.0500,   0.0000, };
                            // x      // y      // vx     // vy     // xint   // yint


    Matrix<12,1> _gain_matrix = {   _gain_roll, _gain_pitch, _gain_yaw, 
                                    _gain_gx, _gain_gy, _gain_gz, 
                                    _gain_vz, _gain_z, _gain_z_int,
                                    _gain_roll_int, _gain_pitch_int, _gain_yaw_int};
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
                            0.0000,         0.0000,         0.0000,     -0.0000,    0.0000,     0.0000,     _gain_z,   _gain_vz,    _gain_z_int,    0.0000,         0.0000,             0.0000};


    //These are debug matricies to use its matrix print function
    Matrix<20> debug ;

};


#endif
