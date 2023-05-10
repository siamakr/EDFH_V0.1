///// Controller.h /////

#ifndef _EDFH_CONTROLLER_H
#define _EDFH_CONTROLLER_H

#include "Actuator.h"
#include <Math.h>
#include <BasicLinearAlgebra.h>
#include <stdint.h>


using namespace BLA;

//..... Defines .....//
#define DT_MSEC 10.00f
#define d2r (PI/180.00f)
#define r2d (180.00f/PI)

typedef struct 
{
    int pwmx, pwmy, pwmedf, pwmrw;
    float angle_x, angle_y, Thrust, torque_rw, torque_x, torque_y; 
    float e[8] = {0.00f};
    //add more data points as needed for debugging
}controller_data_t;


//...... Class Definition .....//
class Controller
{
    

public:
    Actuator act; 
    controller_data_t cd; 

    Controller( void )
    {
        
    }
    
    void init(void)
    {
        act.init_servos();
        act.init_edf();
        act.prime_edf();
        
    }

    void hover(float r, float p, float y, float gx, float gy, float gz, float z, float vz)
    {
        Matrix<4,1> U_hov = {0.00,0.00,0.00,0.00}; // Output vector
        Matrix<8,1> error_hov {0.00,0.00,0.00,0.00,0.00,0.00,0.00,0}; // State error vector
        Matrix<8,1> REF_hov = {0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00};
        Matrix<8,1> Xs_hov = {0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00};

          //load state vecotr
        Xs_hov = {r, p, y, gx, gy, gz, 0, 0};

        //run controller
        error_hov = Xs_hov-REF_hov;
        U_hov = -K_hov * error_hov;
        //U_hov(3) += MASS * G;
        U_hov(3) = MASS * G;

        //calculate integral terms if within bounds
        //U_hov(0) += (error_hov(0) <= d2r(3) || error_hov(0) >= d2r(-3)) ? int_gain * (-(e_roll_prev - error_hov(0)) * dt) : 0.00f;
        //U_hov(1) += (error_hov(1) <= d2r(3) || error_hov(1) >= d2r(-3)) ? int_gain * (-(e_pitch_prev - error_hov(1)) * dt) : 0.00f;
        //U_hov(3) += int_z_gain * (error_hov(6) - e_z_prev) ;


        //load new Thrust Vector from desired torque
        float Tx{ U_hov(3) * sin(U_hov(0)) };
        float Ty{ U_hov(3) * sin(U_hov(1)) * cos(U_hov(0)) };
        float Tz{ U_hov(3) * cos(U_hov(1)) * cos(U_hov(0)) };          

            // Forces including COM change of gimballing EDF (adding force components per mass force)
            //  float Tx{ (U_hov(3) * sin(U_hov(0)) };
            //  float Ty{ U_hov(3) * sin(U_hov(1)) * cos(U_hov(0)) };
            //  float Tz{  U_hov(3) *cos(U_hov(1)) * cos(U_hov(0))  }; 


        float Tm = sqrt(pow(Tx,2) + pow(Ty,2) + pow(Tz,2));
        U_hov(3) = Tm;

        //U_hov(0) = asin(Tx/(Tm- pow(Ty,2)));
        //U_hov(1) = asin(Ty/Tm);

        // U_hov(1) = U_hov(1) * cos(U_hov(0));

        //filter servo angles, the more filtering, the bigger the delay
        U_hov(0) = IIR(U_hov(0), cd.angle_x, u_alpha);
        U_hov(1) = IIR(U_hov(1), cd.angle_y, u_alpha);

        //limit servo angles to +-15º
        U_hov(0) = limit(U_hov(0), d2r * -8, d2r * 8);
        U_hov(1) = limit(U_hov(1), d2r * -8, d2r * 8);
        U_hov(3) = limit(U_hov(3), 15.00f, 32.00f);
        
        //Actuate servos/edf motor 
        act.sx.write(r2d * U_hov(0));
        act.sy.write(r2d * U_hov(1));
        act.edf.write(U_hov(3));

        //Store debug/filtering data into struct
        cd.angle_x = U_hov(0);
        cd.angle_y = U_hov(1);
        cd.Thrust = U_hov(3);

    }

private:

    //TESTING VARIABLES //
    float int_gain{0};
    float int_z_gain{0};
    float n_gain_x{0.5100};
    float n_gain_y{0.5100};
    float g_gain_x{0.1230};
    float g_gain_y{0.1230};
    float g_alpha{0.18};
    float u_alpha{0.05};

    float xsetpoint{0.00f};
    float ysetpoint{0.00f};
    float tinterval1{2.50f};
    float tinterval2{1.25f};


    Matrix<4,8> K_hov = { n_gain_x,   -0.0000,    0.0000,  g_gain_x,   -0.0000,    0.0000,    0.0000,    0.0000,
                        0.0000,  n_gain_y,    0.0000,   -0.0000,  g_gain_y,   -0.0000,    0.0000,    0.0000,
                       -0.0000,    0.0000,   -0.0316,   -0.0000,    0.0000,   -0.0561,    0.0000,    0.0000,
                        0.0000,    0.0000,    0.0000,   -0.0000,    0.0000,    0.0000,     5.42,   3.0942};


    float limit(float value, float min, float max)
    {
        if(value >= max ) value = max;
        if(value <= min ) value = min;

        return value;
    }

    float IIR(float newSample, float prevOutput, float alpha)
    {
        return ( (1.0f-alpha)*newSample + alpha * prevOutput);
    }

};


#endif
