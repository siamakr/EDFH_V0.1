///// Actuator.h /////

#ifndef _EDFH_ACTUATOR_H
#define _EDFH_ACTUATOR_H

#include "Servo.h"
#include <math.h>


//..... Defines .....//
#define DT_MSEC 10.00f
#define d2r (PI/180.00f)
#define r2d (180.00f/PI)

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

typedef struct
{
    int pwmx, pwmy, pwmedf, pwmrw;
}servo_pwm_data_t;

//...... Class Definition .....//
class Actuator
{
public:
    Servo sx; 
    Servo sy; 
    Servo edf;
    //Servo rw;
    //servo_pwm_data_t pwmdata;

    Actuator( void )
    {

    }

    void init(void)
    {
        //attach servo pins
        sx.attach(XSERVO_PIN);
        delay(100);
        sy.attach(YSERVO_PIN);
        delay(100);
    }
    
    void init_servos(void)
    {

        // rw.attach(RW_PIN);
        // delay(200);
        zero_servos();
        delay(1000);
    }

    void init_edf(void)
    {
        edf.attach(EDF_PIN, 1000, 2000);
        delay(200);    
        edf.writeMicroseconds(EDF_OFF_PWM);
        delay(1000);  
    }

    void zero_servos()
    {
        //Zero Servos
        writeXservo(0);
        writeYservo(0);
        delay(1000);
    }

    void prime_edf(void)
    {
        //go to 1500 and wait 5 seconds
        edf.writeMicroseconds(EDF_MIN_PWM+30);
        delay(4000);
    }

    //Write to X servo
    void writeXservo(float angle)
    {
        //map angle in degrees to pwm value for servo
        int pwmX{round( X_P1 * pow(angle,2) + X_P2 * angle + X_P3 ) };
        sx.writeMicroseconds(pwmX);
        //pwmdata.pwmx = pwmX;
    }

    //write to Y servo
    void writeYservo(float angle)
    {
        //map angle in degrees to pwm value for servo
        int pwmY{ round(Y_P1 * pow(angle,2) + Y_P2 * (angle) + Y_P3 ) };      // using polynomial regression coefficients to map tvc angle to pwm vals
        // int pwmY{ round(Y_P1 * pow(angle,2) - Y_P2 * (angle) + Y_P3 ) };      // true regression equations
        sy.writeMicroseconds(pwmY);
        //pwmdata.pwmy = pwmY;
    }

    //to write command to EDF ESC
    void writeEDF(float Ft)
    {
        float omega{(Ft - RAD2N_P2)/RAD2N_P1};
        int pwm{round(omega * RAD2PWM_P1 + RAD2PWM_P2)};
        //Serial.print(pwm);
        edf.writeMicroseconds(pwm);
        //pwmdata.pwmedf = pwm; 
    }

    // to shut everything down if we go past max set angle
    void emergency_check(float r, float p)
    {
        if(r >= MAX_VEHICLE_ANGLE_DEG || r <= -MAX_VEHICLE_ANGLE_DEG || p >= MAX_VEHICLE_ANGLE_DEG || p <= -MAX_VEHICLE_ANGLE_DEG)
        {
            writeEDF(0);
            writeXservo(0);
            writeYservo(0);
            Serial.println("Max attitude angle reached....  ");
            Serial.println("Vehicle is in SAFE-MODE... must restart....");
            while(1);
        }
    }

    void suspend(void)
    {
        //turn off EDF motor
        writeEDF(0);
        //zero TVC actuators.
        writeXservo(0);
        writeYservo(0);
        //Serial.println("Max attitude angle reached....  ");
        //Serial.println("Vehicle is in SAFE-MODE... must restart....");
        while(1);
    }


private:

};


#endif
