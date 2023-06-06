
#include "Controller.h"
#include "Sensors.h"


    Controller::Controller( void )
    {
        
    }
    
    void Controller::init(void)
    {
        // //... Servo Initialization ...//
        // Serial.println("initializing Servos..");
        // init_servos();
        // zero_servos();
        // Serial.println("Servo Init done...");
        // delay(500);

       // act.init();

        //Initialize individually 
        act.init_servos();
        act.zero_servos();

        act.init_edf();
        act.edf_shutdown();
        

        
    }

    void Controller::hover(fsm_data_t id, estimater_data_t ed)
    {
        //Envoke Matricies on the function stack and initialize them
        Matrix<4,1> U = {0.00,0.00,0.00,0.00}; // Control Vector
        Matrix<8,1> error {0.00,0.00,0.00,0.00,0.00,0.00,0.00,0}; // State error vector
        Matrix<8,1> REF = SP_hover;
        Matrix<8,1> Xs = {0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00};     //State Vector

          //load state vecotr
        Xs = {id.roll, id.pitch, id.yaw, id.gx, id.gy, id.gz, ed.vz, ed.z};

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
        //writeXservo(r2d * -cd.angle_y);
        //writeYservo(r2d * -cd.angle_x);
        //edf.write(Tm);

        //Store debug/filtering data into struct

        cd.u = U;
        cd.e = error; 
        cd.Tm = Tm;
        
        cd.Tx = Tx; 
        cd.Ty = Ty; 
        cd.Tz = Tz;
    }

    void Controller::hover(float r, float p, float y, float gx, float gy, float gz, float z, float vz)
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

    void Controller::actuate( void )
    {
        //angles are switched due to calibration imu axis change, will change this
        //in the polynomial regression definition of the servo angles to tvc angle
        
        act.writeXservo((float) r2d * -cd.angle_x);
        act.writeYservo((float) r2d * -cd.angle_y);
        act.writeEDF((float) cd.Tedf);
    }

    void Controller::actuate_servos(void)
    {
        act.writeXservo((float) r2d * -cd.angle_x);
        act.writeYservo((float) r2d * -cd.angle_y);
    }

    void Controller::actuate_edf(void)
    {
        act.writeEDF(cd.Tedf);
    }

    void Controller::set_reference( control_setpoint_t cs, float value ){

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
        // if(value > max ) value = max;
        // if(value < min ) value = min;

        //return value;
        return value < min ? min : (value > max ? max : value); 

    }

    float Controller::IIRF(float newSample, float prevOutput, float alpha)
    {
        return ( ( (1.0f-alpha)*newSample ) + ( alpha*prevOutput ) );
    }