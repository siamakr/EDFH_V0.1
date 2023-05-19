
#include "Controller.h"

    Controller::Controller( void )
    {
        
    }
    
    void Controller::init(void)
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

    void Controller::hover(float r, float p, float y, float gx, float gy, float gz, float z, float vz)
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

    void Controller::actuate( void )
    {
        //angles are switched due to calibration imu axis change, will change this
        //in the polynomial regression definition of the servo angles to tvc angle
        writeXservo(r2d * -cd.angle_y);
        writeYservo(r2d * -cd.angle_x);
        writeEDF(cd.Tedf);
    }

    void Controller::init_servos(void)
    {
        //attach servo pins
        sx.attach(XSERVO_PIN);
        delay(100);
        sy.attach(YSERVO_PIN);
        delay(100);
        // rw.attach(RW_PIN);
        // delay(200);
    }

    void Controller::init_edf(void)
    {
        edf.attach(EDF_PIN, 1000, 2000);
        delay(200);    
        edf.writeMicroseconds(EDF_OFF_PWM);
        delay(1000);  
    }

    void Controller::zero_servos()
    {
        //Zero Servos
        writeXservo(0.00);
        writeYservo(0.00);
        delay(1000);
    }

    void Controller::prime_edf(void)
    {
        //go to 1500 and wait 5 seconds
        edf.writeMicroseconds(EDF_MIN_PWM+30);
        delay(5000);
    }

    //Write to X servo
    void Controller::writeXservo(float angle)
    {
        //map angle in degrees to pwm value for servo
        int pwmX{round( X_P1 * pow(angle,2) + X_P2 * angle + X_P3 ) };
        cd.pwmx = pwmX;
        sx.writeMicroseconds(pwmX);  
    }

    //write to Y servo
    void Controller::writeYservo(float angle)
    {
        //map angle in degrees to pwm value for servo
        int pwmY{ round(Y_P1 * pow(angle,2) + Y_P2 * (angle) + Y_P3 ) };      // using polynomial regression coefficients to map tvc angle to pwm vals
        cd.pwmy = pwmY;
        sy.writeMicroseconds(pwmY);
    }

    //to write command to EDF ESC
    void Controller::writeEDF(float Ft)
    {
        float omega{(Ft - RAD2N_P2)/RAD2N_P1};
        int pwm{round(omega * RAD2PWM_P1 + RAD2PWM_P2)};
        cd.pwmedf = pwm; 
        edf.writeMicroseconds(pwm);   
    }

    void Controller::edf_shutdown(void)
    {
        edf.writeMicroseconds(EDF_OFF_PWM);
    }

    // to shut everything down if we go past max set angle
    void Controller::emergency_check(float r, float p)
    {
        if(r >= MAX_VEHICLE_ANGLE_DEG || r <= -MAX_VEHICLE_ANGLE_DEG || p >= MAX_VEHICLE_ANGLE_DEG || p <= -MAX_VEHICLE_ANGLE_DEG)
        {
            suspend();
        }
    }

    void Controller::suspend(void)
    {
        edf_shutdown();
        zero_servos();
        Serial.println("Max attitude angle reached....  ");
        Serial.println("Vehicle is in SAFE-MODE... must restart....");
        while(1);
    }



    float Controller::limit(float value, float min, float max)
    {
        if(value >= max ) value = max;
        if(value <= min ) value = min;

        return value;
    }

    float Controller::IIRF(float newSample, float prevOutput, float alpha)
    {
        return ( ( (1.0f-alpha)*newSample ) + ( alpha*prevOutput ) );
    }