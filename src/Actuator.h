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
#define RW_PIN 3
#define EDF_PIN 4
#define YSERVO_PIN 5
#define XSERVO_PIN 6

//// CONSTANTS  ///////
#define COM_TO_TVC 0.1985
#define MASS 3.033                    //Kg
#define MASS_EDF .700 
#define G 9.87
#define MAX_ANGLE_SERVO 8           //Deg
#define ROLL_OFFSET (0.533 * PI / 180)
#define PITCH_OFFSET (-11.16 * PI / 180)
//#define SERVO_MAX_SEC_PER_DEG 0.001667      //dt/.001667 |(dt=0.1) = 6º
#define SERVO_MAX_SEC_PER_DEG 0.003333f      //dt/.003333 |(dt=0.1) = 3º
#define SERVO_MAX_SEC_PER_RAD 0.0095496f      //dt/.003333 |(dt=0.1) = 3º
//#define DT .01
#define SERVO_MAX_DEGREE_PER_DT 12
#define MAX_VEHICLE_ANGLE_DEG 35.00f
#define DEADBAND_ANGLE_DEG 0.001f
#define SERVO_ANG_TO_TVC_ANG 3.00f
#define LIDAR_MOUNT_OFFSET_M 0.145

#define PITCH_TVC_CENTER_PWM 1462     //uSec
#define PITCH_TVC_MAX_PWM 1950        //uSec
#define PITCH_TVC_MIN_PWM 1020        //uSec
#define ROLL_TVC_CENTER_PWM 1500       //uSec
#define ROLL_TVC_MAX_PWM 1885          //uSec
#define ROLL_TVC_MIN_PWM 1060          //uSec
#define EDF_OFF_PWM 900              //uSec
#define EDF_MIN_PWM 1500              //uSec
#define EDF_MAX_PWM 2000              //uSec
#define EDF_MAX_SUSTAINED_PWM 1730    //uSec
#define EDF_IDLE_PWM 1600             //uSec

//SERVO-ANGLE TO TVC ANGLE POLYNOMIAL TRANSOFORMATOR
#define X_P1 0.0
#define X_P2 32.07
#define X_P3 1422

#define Y_P1 0
#define Y_P2 -25.68
//#define X_P2 -22.12         //real value of the regression test, can be reversed
#define Y_P3 1518

//
//#define Y_P1 0
//#define Y_P2 -25.68
////#define X_P2 -22.12         //real value of the regression test, can be reversed
//#define Y_P3 1518
//#define X_P1 0.0
//#define X_P2 32.07
//#define X_P3 1452
//#define YY_P1 -.204054981741083f
//#define YY_P2 1530.81204806643f


//EDF MOTOR RPM TO THRUST TO PWM TRANSFORMATORS
#define RAD2N_P1 0.018566536813619f    //Newtons to radians/s
#define RAD2N_P2 -22.506778362213f     //Newtons to Radians/s
#define RAD2PWM_P1 0.2719786528784f    //Radians/s to PWM(us)
#define RAD2PWM_P2 1010.29617153703f   //Radians/s to PWM(us)
#define RPM_TO_OMEGA (2.0f*PI/60.0f)        //RPM to Radians/s
#define OMEGA_TO_RPM (60.0f/(2.0f*PI))      //Radians/s to RPM
#define GRAMS_TO_NEWTONS (9.80f / 1000.00f) //Grams(g) to Newtons(N)

//...... Class Definition .....//
class Actuator
{
    

public:
    Servo sx; 
    Servo sy; 
    Servo edf;
    Servo rw;

    Actuator( void )
    {

    }
    
    void init_servos(void)
    {
        //attach servo pins
        sx.attach(XSERVO_PIN);
        sy.attach(YSERVO_PIN);
        rw.attach(RW_PIN);
        delay(200);

        zero_servos();
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
    }

    //write to Y servo
    void writeYservo(float angle)
    {
        //map angle in degrees to pwm value for servo
        int pwmY{ round(Y_P1 * pow(angle,2) + Y_P2 * (angle) + Y_P3 ) };      // using polynomial regression coefficients to map tvc angle to pwm vals
        // int pwmY{ round(Y_P1 * pow(angle,2) - Y_P2 * (angle) + Y_P3 ) };      // true regression equations
        sy.writeMicroseconds(pwmY);
    }

    //to write command to EDF ESC
    void writeEDF(float Ft)
    {
        float omega{(Ft - RAD2N_P2)/RAD2N_P1};
        int pwm{round(omega * RAD2PWM_P1 + RAD2PWM_P2)};
        //Serial.print(pwm);
        edf.writeMicroseconds(pwm);
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
    // void servo_calibration(void)
    // {
    //     if(Serial.available())
    //     {
    //         byte incoming = Serial.read();

    //         switch (incoming)
    //         {
    //         case '1': // display IMU angles
    //             display_imu();
    //         break;

    //         case '2': // begin both servo calibrations
    //             x_servo_calib();
    //             delay(3000);
    //             Serial.println("\n\n");
    //             y_servo_calib();
    //         break;

    //         case '3':
    //             manual_servo_control();
    //         break;

    //         default:
    //         break;
    //         }
    //     }
    // }

    // void x_servo_calib(void)
    // {
    //     //center both servos
    //     sx.writeMicroseconds(SERVO_X_CENTER_US);
    //     sy.writeMicroseconds(SERVO_Y_CENTER_US);
    //     delay(1000);
    //     //save starting IMU values (unadjusted)
    //     samplebno();
    //     float r_raw{r2d(data.roll)};
    //     float p_raw{r2d(data.pitch)};
    //     sx.writeMicroseconds(SERVO_X_MIN_US); 
    //     delay(1000);

    //     for (int i = SERVO_X_MIN_US; i <  SERVO_X_MAX_US; i++)
    //     {
    //         sx.writeMicroseconds(i);
    //         delay(200);
    //         samplebno();
    //         Serial.print(i);
    //         Serial.print(",");
    //         Serial.print(r2d(data.roll),5);
    //         Serial.print(",");
    //         Serial.println(d2r(data.roll) - r_raw,5);
    //     }
    //     delay(500);
    // }

    // void y_servo_calib(void)
    // {
    //     //center both servos
    //     sx.writeMicroseconds(SERVO_X_CENTER_US);
    //     sy.writeMicroseconds(SERVO_Y_CENTER_US);
    //     delay(1000);
    //     //save starting IMU values (unadjusted)
    //     samplebno();
    //     float r_raw{r2d(data.roll)};
    //     float p_raw{r2d(data.pitch)};
    //     sy.writeMicroseconds(SERVO_Y_MIN_US); 
    //     delay(1000);

    //     for (int i = SERVO_Y_MIN_US; i <  SERVO_Y_MAX_US; i++)
    //     {
    //         sy.writeMicroseconds(i);
    //         delay(200);
    //         samplebno();
    //         Serial.print(i);
    //         Serial.print(",");
    //         Serial.print(r2d(data.pitch),5);
    //         Serial.print(",");
    //         Serial.println(r2d(data.pitch) + p_raw,5);
    //     }
    //     delay(500);
    // }

    // void manual_servo_control(void){
    
    // //if (Serial.available())
    // // {
    //     byte incoming = Serial.read();

    //     switch (incoming)
    //     {
    //     case 'm':     //set both servos to mid-point
    //     pwmx = SERVO_X_CENTER_US;
    //     pwmy = SERVO_Y_CENTER_US;
    //     break;
    //     case 'z':     //decrement by 10us
    //     pwmx +=1;
    //     break;
    //     case 'c':     //increment by 10us
    //     pwmx -=1;
    //     break;
    //     case 'x':     //center x servo to 1500
    //     pwmx = SERVO_X_CENTER_US;
    //     break;
    //     case 't':     //decrement by 10us
    //     pwmy +=1;
    //     break;
    //     case 'u':     //increment by 10us
    //     pwmy -=1;
    //     break;
    //     case 'y':     //center x servo to 1500
    //     pwmy = SERVO_Y_CENTER_US;
    //     break;

    //     default:
    //     break;
    //     }
    // // }
    // sx.writeMicroseconds(pwmx);
    // sy.writeMicroseconds(pwmy);
    // delay(200);
    // samplebno();
    // samplebno();
    // char text[40];
    // sprintf(text, "%i,  %.5f  ,%i,  %.5f", pwmx, r2d(data.roll), pwmy, r2d(data.pitch)); 
    // Serial.println(text);



    // }

    // void display_imu(void)
    // {
    // char text[50]; 
    // samplebno(); 
    // sprintf(text, "%.5f, %.5f %.5f", r2d(data.roll), r2d(data.pitch), r2d(data.yaw)); 
    // Serial.println(text); 

    // }

private:

};


#endif
