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
#define MASS 2.757                    //Kg
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


    Controller( void );
    
    void init(void);

    void hover(fsm_data_t id, estimater_data_t ed);

    void hover(float r, float p, float y, float gx, float gy, float gz, float z, float vz);

    void actuate( void );

    void actuate_servos(void);

    void actuate_edf(void);

    void set_reference(control_setpoint_t cs, float value);




private:


    float limit(float value, float min, float max);

    float IIRF(float newSample, float prevOutput, float alpha);

    Matrix<8,1> SP_hover = {0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00};    //Desired Reference

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
