/* This is a custom Actuation class to controll the 2 MKS servo-motors
* and the Mr. Madd (90mm) Electric Ducted Fan (EDF) motor to be used 
* by Controller and Teststand classes. 
* This class handles the conversion between desired TVC angles and the
* regression curves that describes the mapping of TVC angle to servo 
* PWM values. 
* Similarly, the EDF motor also has this conversion handled here including all 
* initializations.  
*/

#ifndef _EDFH_ACTUATOR_H
#define _EDFH_ACTUATOR_H

#include <Arduino.h>
#include "Servo.h"
#include <math.h>

//define which board being used 
//#define UNO
#define TEENSY 


#ifdef TEENSY
// Teensy 4.1 Pin assignments
#define RW_PIN 37
#define EDF_PIN 36
#define YSERVO_PIN 33
#define XSERVO_PIN 14
#endif

#ifdef UNO 
#define RW_PIN 37
#define EDF_PIN 6
#define YSERVO_PIN 33
#define XSERVO_PIN 14
#endif

//// EDF Params 
#define EDF_OFF_PWM 900                 //uSec
#define EDF_MIN_PWM 1500                //uSec
#define EDF_MAX_PWM 2000                //uSec
#define EDF_MAX_SUSTAINED_PWM 1730      //uSec
#define EDF_IDLE_PWM 1600               //uSec

//EDF Regression Polynomial Coeffs
#define EDF_P1 -.00025151
#define EDF_P2 0.96511
#define EDF_P3 -891.54

//VALUES SET FOR +-15º GIMBAL ANGLE FOR BOTH X AND Y
#define SERVO_X_CENTER_US 1422          //µs
#define SERVO_Y_CENTER_US 1576          //µs
#define SERVO_X_MIN_US 1026             //µs
#define SERVO_Y_MIN_US 1026             //µs
#define SERVO_X_MAX_US 1900             //µs
#define SERVO_Y_MAX_US 1900             //µs

///// MKS Servo TVC Regression Coeffs

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


//  -.00025151   0.96511  -891.54
//Config params 
#define MAX_VEHICLE_ANGLE_DEG 35.00f

#define d2r (PI/180.00f)
#define r2d (180.00f/PI)

typedef struct 
{
    int pwmx, pwmy, pwmedf, pwmrw;
    float ang_x, ang_y, f_thrust, omega_rw;

}actuator_data_t;


class Actuator{
public:
    actuator_data_t ad;

    Actuator();

    //...Initialiaztion Functions...// 
    //Initializes all actuators 
    void init(void);

    //Initialize MKS Servo motors 
    void init_servos(void);

    //Initialize Mr.Madd EDF motor 
    void init_edf(void);

    //Bring TVC system to centre
    void zero_servos();

    //Take EDF to 50% and hold for 5 seconds 
    void prime_edf(void);

    //Take EDF to 50% and hold for desired time
    void prime_edf(int delay_time_ms);

    //Shuts EDF motor down 
    void edf_shutdown(void);

    bool servo_dance_x(float start_angle, int delay_time_ms);


    //... Actuation Functions ...//
    
    //Write to X servo in degrees
    void writeXservo(float angle);

    //Write to X Servo in PWM µs
    void writeXservo(int pwm);

    //write to Y servo in degrees
    void writeYservo(float angle);

    //write to Y servo in PMW µs
    void writeYservo(int pwm);

    //Write to EDF with Thrust Force (Newtons)
    void writeEDF(float Ft);
    
    //Write to EDF with PWM(µs)
    void writeEDF(int pwm);

    //Checks vehicles Roll/Pitch for exceed in max threshold
    void emergency_check(float & r, float & p);

    //Shuts EDF motor down and goes into wait
    //a restart will be required before implementation of 
    //state machine
    void suspend(void);

private:

    Servo sx; 
    Servo sy; 
    Servo edf;
    //Servo rw;


};







#endif