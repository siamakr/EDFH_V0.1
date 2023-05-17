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
    Servo sx; 
    Servo sy; 
    Servo edf;
    //Servo rw;

    Controller( void )
    {
        
    }
    
    void init(void)
    {
        //... Servo Initialization ...//
        Serial.println("initializing Servos..");
        init_servos();
        zero_servos();
        Serial.println("Servo Init done...");
        delay(500);

        //... EDF initialization/priming ...//
        //Serial.println("EDF Prime begin ...");
        //act.init_edf();
        //act.prime_edf();
        //Serial.println("EDF Priming done... ");
        
    }

    void hover(float r, float p, float y, float gx, float gy, float gz, float z, float vz)
    {
        //Envoke Matricies on the function stack and initialize them
        Matrix<4,1> U = {0.00,0.00,0.00,0.00}; // Output vector
        Matrix<8,1> error {0.00,0.00,0.00,0.00,0.00,0.00,0.00,0}; // State error vector
        Matrix<8,1> REF = {0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00};
        Matrix<8,1> Xs = {0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00};

          //load state vecotr
        Xs = {r, p, y, gx, gy, gz, 0, 0};

        //run controller
        error = Xs - REF;
        U = -K * error;
        //update thrust with vehicle mass 
        //must have LIDAR turned on for below to be uncommented
        //U(3) += MASS * G;
        //Use below for static hold-down tests 
        U(3) = MASS * G;

        //calculate integral terms if within integeral bounds
        //U(0) += (error(0) <= d2r*3.00f || error(0) >= d2r*-3.00f) ? int_gain * (-(cd.e(0) - error(0)) * dt) : 0.00f;
        //U(1) += (error(1) <= d2r*3.00f || error(1) >= d2r*-3.00f) ? int_gain * (-(cd.e(1) - error(1)) * dt) : 0.00f;
        //U(3) += int_z_gain * (error(6) - cd.e(6)) ;


        //load new Thrust Vector from desired torque
        float Tx{ U(3) * sin(U(0)) };
        float Ty{ U(3) * sin(U(1)) * cos(U(0)) };
        float Tz{ U(3) * cos(U(1)) * cos(U(0)) };          

            // Forces including COM change of gimballing EDF (adding force components per mass force)
            //  float Tx{ (U(3) * sin(U(0)) };
            //  float Ty{ U(3) * sin(U(1)) * cos(U(0)) };
            //  float Tz{  U(3) *cos(U(1)) * cos(U(0))  }; 


        float Tm = sqrt(pow(Tx,2) + pow(Ty,2) + pow(Tz,2));
        //save the value straight out of the controller before nomalizing. 
        //U(3) = Tm;

        cd.angle_xx = asin(Tx/(Tm - pow(Ty,2)));
        cd.angle_yy = asin(Ty/Tm);

        // U(1) = U(1) * cos(U(0));

        //filter servo angles, the more filtering, the bigger the delay
        //  cd.angle_x = IIRF(U(0), cd.u(0), 0.08);
        //  cd.angle_y = IIRF(U(1), cd.u(1), 0.08);


        //limit servo angles to +-8º
        //filtering and limiting in one line 
        // cd.angle_x = limit(IIRF(cd.angle_xx, cd.u(0), 0.08), d2r * -8.00f, d2r * 8.00f);
        // cd.angle_y = limit(IIRF(cd.angle_yy, cd.u(1), 0.08), d2r * -8.00f, d2r * 8.00f);

        cd.angle_x = limit(IIRF(U(0), cd.u(0), 0.08), d2r * -8.00f, d2r * 8.00f);
        cd.angle_y = limit(IIRF(U(1), cd.u(1), 0.08), d2r * -8.00f, d2r * 8.00f);
        cd.Tedf = limit(Tm, 15.00f, 32.00f);
        
        //Actuate servos/edf motor 
        writeXservo(r2d * cd.angle_x);
        writeYservo(r2d * cd.angle_y);
        //act.edf.write(Tm);

        //Store debug/filtering data into struct

        cd.u = U;
        cd.e = error; 
        cd.Tm = Tm;
        
        cd.Tx = Tx; 
        cd.Ty = Ty; 
        cd.Tz = Tz;



    }

    void init_servos(void)
    {
        //attach servo pins
        sx.attach(XSERVO_PIN);
        delay(100);
        sy.attach(YSERVO_PIN);
        delay(100);
        // rw.attach(RW_PIN);
        // delay(200);
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
        writeXservo(0.00);
        writeYservo(0.00);
        delay(1000);
    }

    void prime_edf(void)
    {
        //go to 1500 and wait 5 seconds
        edf.writeMicroseconds(EDF_MIN_PWM+30);
        delay(5000);
    }

    //Write to X servo
    void writeXservo(float angle)
    {
        //map angle in degrees to pwm value for servo
        int pwmX{round( X_P1 * pow(angle,2) + X_P2 * angle + X_P3 ) };
        cd.pwmx = pwmX;
        sx.writeMicroseconds(pwmX);  
    }

    //write to Y servo
    void writeYservo(float angle)
    {
        //map angle in degrees to pwm value for servo
        int pwmY{ round(Y_P1 * pow(angle,2) + Y_P2 * (angle) + Y_P3 ) };      // using polynomial regression coefficients to map tvc angle to pwm vals
        cd.pwmy = pwmY;
        sy.writeMicroseconds(pwmY);
    }

    //to write command to EDF ESC
    void writeEDF(float Ft)
    {
        float omega{(Ft - RAD2N_P2)/RAD2N_P1};
        int pwm{round(omega * RAD2PWM_P1 + RAD2PWM_P2)};
        cd.pwmedf = pwm; 
        edf.writeMicroseconds(pwm);   
    }

    void edf_shutdown(void)
    {
        edf.writeMicroseconds(EDF_OFF_PWM);
    }

    // to shut everything down if we go past max set angle
    void emergency_check(float r, float p)
    {
        if(r >= MAX_VEHICLE_ANGLE_DEG || r <= -MAX_VEHICLE_ANGLE_DEG || p >= MAX_VEHICLE_ANGLE_DEG || p <= -MAX_VEHICLE_ANGLE_DEG)
        {
            suspend();
        }
    }

    void suspend(void)
    {
        edf_shutdown();
        zero_servos();
        Serial.println("Max attitude angle reached....  ");
        Serial.println("Vehicle is in SAFE-MODE... must restart....");
        while(1);
    }
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


    float limit(float value, float min, float max)
    {
        if(value >= max ) value = max;
        if(value <= min ) value = min;

        return value;
    }

    float IIRF(float newSample, float prevOutput, float alpha)
    {
        return ( ( (1.0f-alpha)*newSample ) + ( alpha*prevOutput ) );
    }


private:



};


#endif
