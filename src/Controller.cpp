
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

    bool Controller::edf_startup(int seconds)
    {
        //Make sure the edf motor is off
        act.edf.writeMicroseconds(EDF_OFF_PWM);

        float timer{millis()};

        while(millis() - timer <= seconds * 1000)
        {
            act.edf.writeMicroseconds(EDF_MIN_PWM+50);
        }

        return true;
    }



    void Controller::lqr(float r, float p, float y, float gx, float gy, float gz, float z, float vz)
    {
        //Envoke Matricies on the function stack and initialize them
        Matrix<4,1> U = {0.00}; // Control Vector
        Matrix<12,1> error {0.00}; // State error vector
        Matrix<4,12> K = K_int;
        Matrix<12,1> REF = SP_hover_int;         //store desired setpoint (either by user or from Position Controller Output)
        Matrix<12,1> Xs = {0.00};     //State Vector

        //load state vecotr
        Xs = {r, p, y, gx, gy, gz, z, vz};

        //calculate reference error
        error = Xs - REF;

        //altitude integral action 
        error(8)  = ( ( error(6) >= (-1 * _int_bound_alt) ) || ( error(6) <= _int_bound_alt ) ) ? cd.u(8) + ( _gain_z_int     * error(7) * DT_SEC) : 0.00f;       
        //attitude integral actions
        error(9)  = ( ( error(1) >= (-1 * _int_bound_att) ) && ( error(1) <= _int_bound_att ) ) ? cd.u(9) + ( _gain_roll_int  * error(1) * DT_SEC) : 0.00f;       
        error(10) = ( ( error(2) >= (-1 * _int_bound_att) ) && ( error(2) <= _int_bound_att ) ) ? cd.u(10) + ( _gain_pitch_int * error(2) * DT_SEC) : 0.00f;       
        error(11) = ( ( error(3) >= (-1 * _int_bound_att) ) && ( error(3) <= _int_bound_att ) ) ? cd.u(11) + ( _gain_yaw_int   * error(3) * DT_SEC) : 0.00f;  

        LIMIT(error(9) , -1 * _max_int_def, _max_int_def);
        LIMIT(error(10), -1 * _max_int_def, _max_int_def);
        LIMIT(error(11), -1 * _max_int_def, _max_int_def);

        //Run LQR Controller + full integral action
        U = -K * error;

        //Update the EDF motor control signal with Vehicle weight
        U(3) += MASS * G;         //Normal Mode

        //Calculate each component of the Thrust Vector
        float Tx{ U(3) * sin(U(0)) - (MASS_EDF * sin(U(0)))};
        float Ty{ U(3) * sin(U(1)) * cos(U(0)) - (MASS_EDF * sin(U(1))) };
        float Tz{ U(3) * cos(U(1)) * cos(U(0)) };   

        //Get the magnitude of the thrust vector components 
        float Tm{sqrt(pow(Tx,2) + pow(Ty,2) + pow(Tz,2))};
        float delta_xx{asin(Tx/(Tm))};
        float delta_yy{asin(Ty/(Tm))};

        //-----------delete below//debugging only-----------//
        cd.delta_x = delta_xx;
        cd.delta_y = delta_yy;
        LIMIT(cd.delta_x, -1 * MAX_TVC_DEFLECTION_RAD, MAX_TVC_DEFLECTION_RAD );
        LIMIT(cd.delta_y, -1 * MAX_TVC_DEFLECTION_RAD, MAX_TVC_DEFLECTION_RAD );
        //-----------delete above/debugging only-----------//

        //Feedforward
        delta_xx -= error(0) * _gain_ff_roll;
        delta_yy -= error(1) * _gain_ff_pitch;

        //Filter
        IIR(delta_xx, cd.delta_xx, _alpha_servo);
        IIR(delta_yy, cd.delta_yy, _alpha_servo);

        //Limit
        LIMIT(delta_xx, -1 * MAX_TVC_DEFLECTION_RAD, MAX_TVC_DEFLECTION_RAD );
        LIMIT(delta_yy, -1 * MAX_TVC_DEFLECTION_RAD, MAX_TVC_DEFLECTION_RAD );
        LIMIT(Tm, 21.00f, 31.00f);

        //Actuate
        actuate(delta_xx, delta_yy, Tm);

        //Store data for next iteration 
        cd.delta_xx = delta_xx;
        cd.delta_yy = delta_yy;
        cd.e_int = error;
        cd.u(0) = delta_xx;
        cd.u(1) = delta_yy;
        cd.u(2) = U(2);
        cd.u(3) = Tm;



    }

    //This method repeats the lqr method but adds integral action for each output 
    // ie; for roll(xservo), pitch(yservo), yaw(reaction wheel), Ft (edf motor)
    // each integral action is added into the state vector Xs 
    void Controller::lqr_int(float r, float p, float y, float gx, float gy, float gz, float z, float vz)
    {
        //Envoke Matricies on the function stack and initialize them
        Matrix<4,1> U = {0.00}; // Control Vector
        Matrix<12,1> error {0.00}; // State error vector
        Matrix<4,12> K = K_int;
        Matrix<12,1> REF = SP_hover_int;         //store desired setpoint (either by user or from Position Controller Output)
        Matrix<12,1> Xs = {0.00};     //State Vector
        Matrix<4,1> U_out = {0.00f};

          //load state vecotr
        Xs = {r, p, y, gx, gy, gz, z, vz};

        //calculate reference error
        error = Xs - REF;

        //Clamp the integral action to a +- x-degrees neighborhood of the desired attitude. 
        //Calculate integral action and put updated error values back into error matrix
        //the cd.e_int(•) term is the previous error in the integral positions in error vector
        //altitude integral action 
        //error(8)  = ( ( error(7) >= (-1 * _int_bound_alt) ) || ( error(7) <= _int_bound_alt ) ) ? error(8) + ( _gain_z_int     * error(7) * DT_SEC) : 0.00f;       
        //attitude integral actions
        error(9)  = ( ( error(1) >= (-1 * _int_bound_att) ) && ( error(1) <= _int_bound_att ) ) ? cd.u(9) + ( _gain_roll_int  * error(1) * DT_SEC) : 0.00f;       
        error(10) = ( ( error(2) >= (-1 * _int_bound_att) ) && ( error(2) <= _int_bound_att ) ) ? cd.u(10) + ( _gain_pitch_int * error(2) * DT_SEC) : 0.00f;       
        error(11) = ( ( error(3) >= (-1 * _int_bound_att) ) && ( error(3) <= _int_bound_att ) ) ? cd.u(11) + ( _gain_yaw_int   * error(3) * DT_SEC) : 0.00f;  

        error(9) = limit(error(9), -_max_int_def, _max_int_def);     
        error(10) = limit(error(10), -_max_int_def, _max_int_def);     
        error(11) = limit(error(11), -_max_int_def, _max_int_def);     
       

    
        //Run LQR Controller + full integral action
        U = -K * error;

        //Update the EDF motor control signal with Vehicle weight
        U(3) += MASS * G;         //Normal Mode
        //U(3) = MASS * G;            //Hold-down Gimbal test Mode

        //Thrust in each axis with corrected torque term 
        //due to the change in J when the EDF motor is actuated. 
        //the 2nd term for each thrust component subtracts the static torque induced by the gimballed 
        //edf motor.

        //Calculate each component of the Thrust Vector
        // float Tx{ U(3) * sin(U(0)) - (MASS_EDF * sin(U(0)))};
        // float Ty{ U(3) * sin(U(1)) * cos(U(0)) - (MASS_EDF * sin(U(1))) };
        // float Tz{ U(3) * cos(U(1)) * cos(U(0)) };   

        //Calculate each component of the Thrust Vector
        float Tx{ U(3) * sin(U(0)) };
        float Ty{ U(3) * sin(U(1)) * cos(U(0)) };
        float Tz{ U(3) * cos(U(1)) * cos(U(0)) };   



        //Get the magnitude of the thrust vector components 
        float Tm{sqrt(pow(Tx,2) + pow(Ty,2) + pow(Tz,2))};
        // float delta_xx{asin(Tx/(Tm))};
        // float delta_yy{asin(Ty/(Tm))};

        // //Feedforward
        // delta_xx += error(0) * _gain_ff_roll;
        // delta_yy += error(1) * _gain_ff_pitch;
        // //Filter
        // IIR(delta_xx, cd.delta_xx, _alpha_servo);
        // IIR(delta_yy, cd.delta_yy, _alpha_servo);
        // //Limit
        // LIMIT(delta_xx, -1 * MAX_TVC_DEFLECTION_RAD, MAX_TVC_DEFLECTION_RAD );
        // LIMIT(delta_yy, -1 * MAX_TVC_DEFLECTION_RAD, MAX_TVC_DEFLECTION_RAD );
        // LIMIT(Tm, 21.00f, 31.00f);
        // //Actuate
        // actuate(delta_xx, delta_yy, Tm);
        // //Store data for next iteration 
        // cd.delta_xx = delta_xx;
        // cd.delta_yy = delta_yy;

        //Using the  EmboRockETH paper's outline of attaining gimbal angles 
        //from each thrust vector component and magnitude of thrust. 
        cd.angle_xx = limit(IIRF(asin(Tx/(Tm)),             cd.u_output(3), _alpha_servo), -1 * MAX_TVC_DEFLECTION_RAD, MAX_TVC_DEFLECTION_RAD);
        cd.angle_yy = limit(IIRF(asin(Ty/(Tm)),             cd.u_output(4), _alpha_servo), -1 * MAX_TVC_DEFLECTION_RAD, MAX_TVC_DEFLECTION_RAD);

        //Feedforward 
        cd.angle_xx -= error(0) * _gain_ff_roll;
        cd.angle_yy -= error(1) * _gain_ff_pitch;




    




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
        float u0 = U(0);
        float u1 = U(1);

        cd.angle_x = limit(IIRF(u0, cd.u_output(0), _alpha_servo), -1 * MAX_TVC_DEFLECTION_RAD, MAX_TVC_DEFLECTION_RAD);
        cd.angle_y = limit(IIRF(u1, cd.u_output(1), _alpha_servo), -1 * MAX_TVC_DEFLECTION_RAD, MAX_TVC_DEFLECTION_RAD);
        cd.Tedf = limit(Tm, 21.00f, 31.00f);

        if(isnan(cd.angle_x)) cd.angle_x = 0.00f;
        if(isnan(cd.angle_y)) cd.angle_y = 0.00f;


        
        //Actuate servos/edf motor 
        act.writeXservo((float) (r2d * -cd.angle_xx));
        act.writeYservo((float) (r2d * -cd.angle_yy));
        act.writeEDF((float) cd.Tedf);

        //Store debug/filtering data into struct
        cd.u_output(0) = cd.angle_x;
        cd.u_output(1) = cd.angle_y;
        cd.u_output(2) = cd.angle_xx;
        cd.u_output(3) = cd.angle_yy;

        
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
        
        act.writeXservo((float)  -angle_x_rad);
        act.writeYservo((float)  -angle_y_rad);
        act.writeEDF((float) thrust_force_newton);
    }

    void Controller::actuate_servos(float angle_x_rad, float angle_y_rad)
    {
        act.writeXservo((float)  -angle_x_rad);
        act.writeYservo((float)  -angle_y_rad);
    }

    void Controller::actuate_edf(float thrust_force_newton)
    {
        act.writeEDF((float) thrust_force_newton);
    }

    void Controller::set_reference( control_setpoint_t cs, float value ){
        value = value * d2r;

        // switch( cs ){
        //     // case SETPOINT_X: SP_pos(0) = value; break;
        //     // case SETPOINT_Y: SP_pos(1) = value; break;
        //     // case SETPOINT_Z: SP_hover(6) = value; break;
        //     case SETPOINT_ROLL: SP_hover_int(0) = value; break;
        //     case SETPOINT_PITCH: SP_hover_int(1) = value; break;
        //     case SETPOINT_YAW: SP_hover_int(2) = value; break;
        // }

        switch( cs ){
            // case SETPOINT_X: SP_pos(0) = value; break;
            // case SETPOINT_Y: SP_pos(1) = value; break;
             case SETPOINT_Z: SP_hover_int(6) = value; break;
            case SETPOINT_ROLL: SP_hover_int(0) = value; break;
            case SETPOINT_PITCH: SP_hover_int(1) = value; break;
            case SETPOINT_YAW: SP_hover_int(2) = value; break;
        }

    }




    float Controller::limit(float value, float min, float max)
    {
        return value <= min ? min : (value >= max ? max : value); 
    }

    void Controller::LIMIT(float & value, float min, float max )
    {
        value = ( (value <= min) ? min : (value >= max ? max : value) );
    }

    float Controller::IIRF(float newSample, float prevOutput, float alpha)
    {
        return ( ( (1.00f - alpha ) * newSample ) + ( alpha * prevOutput ) );
    }

    void Controller::IIR(float & new_sample, float prev_output, float alpha)
    {
        new_sample = ( ( (1.00f - alpha ) * new_sample ) + ( alpha * prev_output ) );
    }

    void Controller::print_debug(void)
    {
        Serial << debug << "\n" ;
    }