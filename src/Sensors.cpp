#include "Sensors.h"

    Sensors::Sensors(void){

    }

    void Sensors::init(void)
    {
        lidar_init();
        delay(100);
        fsm_init();

        //flow_init();
    }
        //... GARMIN LIDAR functions end ...//
    void Sensors::fsm_init(void)
    {
                //..... FSM305/BNO080 Init .....//
        Serial.print("FSM Init start..."); 
        //delay(300);

        if (fsm.beginSPI(imuCSPin, imuWAKPin, imuINTPin, imuRSTPin, 3000000) == false)
        {
            Serial.println("BNO080 over SPI not detected. Are you sure you have all 6 connections? Freezing...");
            while(1);
        }

        fsm.calibrateAll();
        fsm.enableLinearAccelerometer(DT_MSEC);  // m/s^2 no gravity
        fsm.enableRotationVector(DT_MSEC);  // quat
        //fsm.enableGameRotationVector(DT_MSEC);
        fsm.enableGyro(DT_MSEC);  // rad/s
        //fsm.enableGyroIntegratedRotationVector(DT_MSEC);
        //fsm.enableMagnetometer(DT_MSEC);  // cannot be enabled at the same time as RotationVector (will not produce data)
        
        Serial.println("FSM Init Finished..."); 
        delay(300);
    }

    void Sensors::lidar_init(void)
    {   
        Wire.begin();
        delay(100);
        // "2" = short-range, fast speed
        // "0" = normal operation 
        garmin.configure(0, LIDARLITE_ADDR_DEFAULT);
        delay(200);
    }

    void Sensors::flow_init(void)
    {

    }

    ///.......... FSM305/BNO080 FUNCTIONS ..........////

    void Sensors::save_calibrate_fsm(void)
    {
        if(Serial.available())
        {
            byte incoming = Serial.read();

            if(incoming == 's')
            {
                fsm.saveCalibration(); //Saves the current dynamic calibration data (DCD) to memory
                fsm.requestCalibrationStatus(); //Sends command to get the latest calibration status

                //Wait for calibration response, timeout if no response
                int counter = 100;
            
                while(1)
                {
                    if(--counter == 0) break;
                    if(fsm.dataAvailable() == true)
                {
                    //The IMU can report many different things. We must wait
                    //for the ME Calibration Response Status byte to go to zero
                    if(fsm.calibrationComplete() == true)
                    {
                        Serial.println("Calibration data successfully stored");
                        delay(1000);
                        break;
                    }
                }

                delay(1);
            }

            if(counter == 0) Serial.println("Calibration data failed to store. Please try again.");

            //fsm.endCalibration(); //Turns off all calibration
            //In general, calibration should be left on at all times. The BNO080
            //auto-calibrates and auto-records cal data roughly every 5 minutes
            }
        } 
    } 


    void Sensors::sample_fsm(void)
    {
        if(fsm.dataAvailable() == true)
        {
            //..... Sample IMU .....//
            //... Linear Accel ...//
            //fsm.getLinAccel(data.ax, data.ay, data.az, data.linAccuracy);

            data.linAccuracy = fsm.getLinAccelAccuracy();
            data.ay = IIR(fsm.getLinAccelX(), data.ax, .10);
            data.ax = IIR(fsm.getLinAccelY(), data.ay, .10);
            data.az = IIR(fsm.getLinAccelZ(), data.az, .10);

            //... Gyro ...//
           // fsm.getGyro(data.gx, data.gy, data.gz, data.gyroAccuracy);

            // data.gx = fsm.getFastGyroX();
            // data.gy = fsm.getFastGyroY();
            // data.gz = fsm.getFastGyroZ();
            data.gyroAccuracy = fsm.getGyroAccuracy();
            data.gy = IIR(fsm.getGyroX(), data.gx, .08);
            data.gx = IIR(fsm.getGyroY(), data.gy, .08);
            data.gz = IIR(fsm.getGyroZ(), data.gz, .08);
            
            //... Rotation Vector ...//
            fsm.getQuat(data.qi, data.qj, data.qk, data.qw, data.quatRadianAccuracy, data.quatAccuracy);

            // data.qi = fsm.getQuatI();
            // data.qj = fsm.getQuatJ();
            // data.qk = fsm.getQuatK();
            // data.qw = fsm.getQuatReal();


            //... Euler Angle Representation ...//
            data.pitch = fsm.getRoll();
            data.roll = fsm.getPitch();
            data.yaw = fsm.getYaw();    

        }
    }


    void Sensors::print_fsm(void)
    {
        char text[200];
        sprintf(text, "%0.5f, %0.5f, %0.5f,\t  %0.5f, %0.5f, %0.5f, \t  %0.5f, %0.5f, %0.5f    ",
        r2d*data.roll,
        r2d*data.pitch,
        r2d*data.yaw,
        r2d*data.gx,
        r2d*data.gy,
        r2d*data.gz,
        data.ax,
        data.ay,
        data.az);

        Serial.println(text);
    }

    void Sensors::print_fsm_calibration(void)
    {
        char text[250];

        sprintf(text, "%0.5f, %0.5f, %0.5f,\t   %0.5f, %0.5f, %0.5f, %0.5f,\t  %0.5f, %0.5f, %0.5f, \t  %0.5f, %0.5f, %0.5f    ",
        r2d*data.roll,
        r2d*data.pitch,
        r2d*data.yaw,
        data.qi,
        data.qj,
        data.qk,
        data.qw,
        r2d*data.gx,
        r2d*data.gy,
        r2d*data.gz,
        data.ax,
        data.ay,
        data.az);

        Serial.print(text);
        Serial.print(data.linAccuracy);
        Serial.print(", ");
        Serial.print(data.gyroAccuracy);
        Serial.print(", ");
        Serial.print(data.quatAccuracy);
        Serial.println(", ");
    }
    
    void Sensors::print_estimator(void)
    {
        char text[250];

        sprintf(text, "%0.5f, %0.5f, %0.5f, %0.5f, \t   %0.5f, %0.5f, %0.5f,\t  %0.5f, %0.5f, %0.5f, \t  %0.5f, %0.5f, %0.5f, \t  %i, %i, %i ",
        r2d*data.roll,
        r2d*data.pitch,
        r2d*data.yaw,
        data.z,

        estimate.x,
        estimate.y,
        estimate.z,

        estimate.vx,
        estimate.vy,
        estimate.vz,

        data.ax,
        data.ay,
        data.az,

        data.linAccuracy,
        data.gyroAccuracy,
        data.quatAccuracy);

        Serial.println(text);
    }

    //... FSM305 Functions End ...//
    
    //... GARMIN LIDAR functions Begin ...//
    void Sensors::sample_lidar(void)
    {
        //uint8_t newDistance = 0;
        //uint16_t distance{0};
    
        if (garmin.getBusyFlag() == 0)
        {
            // Trigger the next range measurement
            garmin.takeRange();

            // Read new distance data from device registers
            data.z = (float)(garmin.readDistance()/100.00f);

            // Report to calling function that we have new data
            data.status.lidar = 1;
        }
        //convert from cm to m
       // data.z = (float)distance/100.00f;
    }

    void Sensors::run_estimator(void){

    /* ---- Sensor processing ---- */
    float p[3] = {0}; // Position vector (z body to world)
    //float v[3] = {0}; // Velocity vector (vx, vy to world)
    float a[3] = {0}; // Acceleration vector (ax, ay, az to world)

    // Rotate lidar measurement to world frame
    p[2] = data.z;
    rotate_to_world( p );

    // Perform gyrocompensation on flow and rotate to world frame.
    // v[0] = data.vx * p[2] + data.gy * p[2];
    // v[1] = data.vy * p[2] - data.gx * p[2]; 
    // rotate_to_world( v );

    // Rotate acceleration to world frame
    a[0] = data.ax; a[1] = data.ay; a[2] = data.az;
    rotate_to_world( a );


    /* ---- Estimation ---- */
    // Fill input vector with acceleration
    H.Fill(0);
    Z.Fill(0);
    U = { a[0], a[1], a[2] };

    // Fill measurement vector with data
   // if( data.status.pos == 1 ){
        // H(0,0) = 1; H(1,1) = 1;
        // Z(0) = data.x;
        // Z(1) = data.y;
    //}

    if( data.status.lidar == 1){
        H(2,2) = 1;
        Z(2) = p[2]; // p[2]: z
    }

    // if( data.status.flow == 1){
    //     H(3,3) = 1; H(4,4) = 1;
    //     Z(3) = v[0]; // vx
    //     Z(4) = v[1]; // vy
    // }

    // Prediction, based on previous state and current input
    Xpre = A*X + B*U; 

    // Update prediction with update using measurements 
    X = Xpre + Kf*(Z - H*Xpre); 
    
    // Fill estimate struct with values (for telemetry and stuff)
    estimate.x = X(0);
    estimate.y = X(1);
    estimate.z = X(2);
    estimate.vx = X(3);
    estimate.vy = X(4);
    estimate.vz = X(5);

    // Reset status (measurements has been used!)
    // data.status.flow = 0;
     data.status.lidar = 0;
    // data.status.imu = 0;
    // data.status.pos = 0;
}
    void Sensors::rotate_to_world( float * vector )
    {
        //*************************** PROBLEM IS HERE ***************************

        //When using dummy variables for pqu, it works!
        //this means data.roll etc. are not being read into pqu for some reason
        //using "distance" i confirmed that data isn't being read into this 
        //function for some reason 
        //  float p = 0.23;
        //  float q = -0.23;
        //  float u = 0.01;

         float p = data.roll;
         float q = data.pitch;
         float u = data.yaw;

        //testing public/private
        //distance = p;
        //distance = data.roll;

        Matrix<3,1> in = { vector[0], vector[1], vector[2] };
        Matrix<3,1> out = {0,0,0};

        Matrix<3,3> R = {   cos(q)*cos(u), sin(p)*sin(q)*cos(u)-cos(p)*sin(u), cos(p)*sin(q)*cos(u)+sin(p)*sin(u) ,
                            cos(q)*sin(u), sin(p)*sin(q)*sin(u)+cos(p)*cos(u), cos(p)*sin(q)*sin(u)-sin(p)*cos(u) ,
                            -sin(q),       sin(p)*cos(q),                      cos(p)*cos(q)                      };

        out = R * in;

        vector[0] = out(0);
        vector[1] = out(1);
        vector[2] = out(2);

    }

    float Sensors::IIR(float newSample, float prevOutput, float alpha)
    {
        return ( (1.0f-alpha)*newSample + alpha * prevOutput);
    }