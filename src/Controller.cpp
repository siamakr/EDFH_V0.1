
#include "Controller.h"
#include "Sensors.h"


    Controller::Controller( void )
    {
        
    }
    
    void Controller::init(void)
    {
        //attach actuator pins
        act.init_servos();
        act.init_edf();
        delay(100);
        //center TVC 
        act.zero_servos();
        delay(100);
        
        
    }

    void Controller::edf_startup(int seconds)
    {
        //Make sure the edf motor is off
        act.edf.writeMicroseconds(EDF_OFF_PWM);
        delay(10);

        float timer{millis()};

        while(millis() - timer <= seconds * 1000)
        {
            act.edf.writeMicroseconds(EDF_MIN_PWM);
        }
    }



    void Controller::lqr(float r, float p, float y, float gx, float gy, float gz, float z, float vz)
    {
        //Envoke Matricies on the function stack and initialize them
        Matrix<4,1> U = {0.00,0.00,0.00,0.00}; // Control Vector
        Matrix<8,1> error {0.00,0.00,0.00,0.00,0.00,0.00,0.00,0}; // State error vector
        Matrix<8,1> REF = SP_hover;         //store desired setpoint (either by user or from Position Controller Output)
        Matrix<8,1> Xs = {0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00};     //State Vector

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

        // cd.angle_xx = asin(Tx/(Tm));
        // cd.angle_yy = asin(Ty/(Tm - pow(Tx,2)));

        // U(1) = U(1) * cos(U(0));

        //filter servo angles, the more filtering, the bigger the delay
        //  cd.angle_x = IIRF(U(0), cd.u(0), 0.08);
        //  cd.angle_y = IIRF(U(1), cd.u(1), 0.08);


        //limit servo angles to +-8º
        //filtering and limiting in one line 
        // cd.angle_x = limit(IIRF(cd.angle_xx, cd.u(0), 0.08), d2r * -8.00f, d2r * 8.00f);
        // cd.angle_y = limit(IIRF(cd.angle_yy, cd.u(1), 0.08), d2r * -8.00f, d2r * 8.00f);

        cd.angle_x = limit(IIRF(U(0), cd.u(0), 0.08), -1 * MAX_TVC_DEFLECTION_RAD, MAX_TVC_DEFLECTION_RAD);
        cd.angle_y = limit(IIRF(U(1), cd.u(1), 0.08), -1 * MAX_TVC_DEFLECTION_RAD, MAX_TVC_DEFLECTION_RAD);
        cd.Tedf = limit(Tm, 15.00f, 31.00f);
        
        //Actuate servos/edf motor 
        act.writeXservo((float) (r2d * -cd.angle_x));
        act.writeYservo((float) (r2d * -cd.angle_y));
        act.writeEDF((float) cd.Tedf);

        //Store debug/filtering data into struct

        cd.u = U;
        cd.e = error; 
        cd.Tm = Tm;
        
        cd.Tx = Tx; 
        cd.Ty = Ty; 
        cd.Tz = Tz;
    }

    //This method repeats the lqr method but adds integral action for each output 
    // ie; for roll(xservo), pitch(yservo), yaw(reaction wheel), Ft (edf motor)
    // each integral action is added into the state vector Xs 
    void Controller::lqr_int(float r, float p, float y, float gx, float gy, float gz, float z, float vz)
    {
        //Envoke Matricies on the function stack and initialize them
        Matrix<4,1> U = {0.00}; // Control Vector
        Matrix<12,1> error {0.00}; // State error vector
        Matrix<12,1> REF = SP_hover_int;         //store desired setpoint (either by user or from Position Controller Output)
        Matrix<12,1> Xs = {0.00};     //State Vector

          //load state vecotr
        Xs = {r, p, y, gx, gy, gz, 0, 0};

        //calculate reference error
        error = REF - Xs;

        //Clamp the integral action to a +- x-degrees neighborhood of the desired attitude. 
        //Calculate integral action and put updated error values back into error matrix
        error(8)  = ( ( error(7) < (-1 * _int_bound_att) ) || ( error(7) > _int_bound_att ) ) ? error(7) * DT_SEC : 0.00f;       
        error(9)  = ( ( error(1) < (-1 * _int_bound_att) ) || ( error(1) > _int_bound_att ) ) ? error(7) * DT_SEC : 0.00f;       
        error(10) = ( ( error(2) < (-1 * _int_bound_att) ) || ( error(2) > _int_bound_att ) ) ? error(7) * DT_SEC : 0.00f;       
        error(11) = ( ( error(3) < (-1 * _int_bound_alt) ) || ( error(3) > _int_bound_alt ) ) ? error(7) * DT_SEC : 0.00f;       
       
       
    
        //Run LQR Controller + full integral action
        U = K_int * error;

        //Update the EDF motor control signal with Vehicle weight
        //U(3) += MASS * G;         //Normal Mode
        U(3) = MASS * G;            //Hold-down Gimbal test Mode

        //Thrust in each axis with corrected torque term 
        //due to the change in J when the EDF motor is actuated. 
        //the 2nd term for each thrust component subtracts the static torque induced by the gimballed 
        //edf motor.
        //Calculate each component of the Thrust Vector
        float Tx{ U(3) * sin(U(0))- (COM_TO_TVC + ledf * cos(U(0))) };
        float Ty{ U(3) * sin(U(1)) * cos(U(0)) - (COM_TO_TVC + ledf * cos(U(1))) };
        float Tz{ U(3) * cos(U(1)) * cos(U(0)) };           

        //Get the magnitude of the thrust vector components 
        float Tm = sqrt(pow(Tx,2) + pow(Ty,2) + pow(Tz,2));
        //save the value straight out of the controller before nomalizing. 
        //U(3) = Tm;
        
        //Using the  EmboRockETH paper's outline of attaining gimbal angles 
        //from each thrust vector component and magnitude of thrust. 
        cd.angle_xx = limit(asin(Tx/(Tm )), -1 * MAX_TVC_DEFLECTION_RAD , MAX_TVC_DEFLECTION_RAD);
        cd.angle_yy = limit(asin(Ty/(Tm)), -1 * MAX_TVC_DEFLECTION_RAD, MAX_TVC_DEFLECTION_RAD);
        // cd.angle_xx = limit(asin(Tx/(Tm - pow(Ty,2))), -1 * MAX_TVC_DEFLECTION_RAD , MAX_TVC_DEFLECTION_RAD);
        // cd.angle_yy = limit(asin(Ty/Tm), -1 * MAX_TVC_DEFLECTION_RAD, MAX_TVC_DEFLECTION_RAD);

        // cd.angle_xx = asin(Tx/(Tm));
        // cd.angle_yy = asin(Ty/(Tm - pow(Tx,2)));

        //filter servo angles, the more filtering, the bigger the delay
        //  cd.angle_x = IIRF(U(0), cd.u(0), 0.08);
        //  cd.angle_y = IIRF(U(1), cd.u(1), 0.08);


        //limit servo angles to +-8º
        //filtering and limiting in one line 
        // cd.angle_x = limit(IIRF(cd.angle_xx, cd.u(0), 0.08), d2r * -8.00f, d2r * 8.00f);
        // cd.angle_y = limit(IIRF(cd.angle_yy, cd.u(1), 0.08), d2r * -8.00f, d2r * 8.00f);

        cd.angle_x = limit(IIRF(U(0), cd.u(0), 0.08), -1 * MAX_TVC_DEFLECTION_RAD, MAX_TVC_DEFLECTION_RAD);
        cd.angle_y = limit(IIRF(U(1), cd.u(1), 0.08), -1 * MAX_TVC_DEFLECTION_RAD, MAX_TVC_DEFLECTION_RAD);
        cd.Tedf = limit(Tm, 15.00f, 31.00f);
        
        //Actuate servos/edf motor 
        act.writeXservo((float) (r2d * -cd.angle_xx));
        act.writeYservo((float) (r2d * -cd.angle_yy));
        act.writeEDF((float) cd.Tedf);

        //Store debug/filtering data into struct

        cd.u = U;
        cd.e_int = error; 
        cd.Tm = Tm;
        
        cd.Tx = Tx; 
        cd.Ty = Ty; 
        cd.Tz = Tz;
    }

    void Controller::actuate( float angle_x_rad, float angle_y_rad, float thrust_force_newton )
    {
        //angles are switched due to calibration imu axis change, will change this
        //in the polynomial regression definition of the servo angles to tvc angle
        
        act.writeXservo((float) r2d * -angle_x_rad);
        act.writeYservo((float) r2d * -angle_y_rad);
        act.writeEDF((float) thrust_force_newton);
    }

    void Controller::actuate_servos(float angle_x_rad, float angle_y_rad)
    {
        act.writeXservo((float) r2d * -angle_x_rad);
        act.writeYservo((float) r2d * -angle_y_rad);
    }

    void Controller::actuate_edf(float thrust_force_newton)
    {
        act.writeEDF((float) thrust_force_newton);
    }

    void Controller::set_reference( control_setpoint_t cs, float value ){
        value = value * d2r;

        switch( cs ){
            // case SETPOINT_X: SP_pos(0) = value; break;
            // case SETPOINT_Y: SP_pos(1) = value; break;
            // case SETPOINT_Z: SP_hover(6) = value; break;
            case SETPOINT_ROLL: SP_hover(0) = value; break;
            case SETPOINT_PITCH: SP_hover(1) = value; break;
            case SETPOINT_YAW: SP_hover(2) = value; break;
        }

    }




    float Controller::limit(float value, float min, float max)
    {
        return value < min ? min : (value > max ? max : value); 
    }

    float Controller::IIRF(float newSample, float prevOutput, float alpha)
    {
        return ( ( (1.00f - alpha ) * newSample ) + ( alpha * prevOutput ) );
    }