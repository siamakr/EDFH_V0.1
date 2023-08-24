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

        if (fsm.beginSPI(imuCSPin, imuWAKPin, imuINTPin, imuRSTPin, 4000000) == false)
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
        sample_fsm();
        // while(!(data.gyroAccuracy == 3 && data.linAccuracy == 3 && data.quatAccuracy == 3 ) )
        // {
        //     sample_fsm();
        //     delay(250);
        //     print_fsm();
        //     Serial.println("move vehicle to calibrate IMU ");
        // }
        //save current position of yaw as the zero. 
        yaw_origin = data.yaw;
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

    void Sensors::flow_init(void){
    // Initiate flow sensor
    if ( flow.begin() ) {
        flow.setLed(false);
        Serial.println("Flow connected successfully"); 
    }else{
        Serial.println("Flow failed to connected, check connections");
        while(1); 
    }

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

            save_calibrate_fsm();
            //..... Sample IMU .....//
            //... Linear Accel ...//
            //fsm.getLinAccel(data.ax, data.ay, data.az, data.linAccuracy);

            data.linAccuracy = fsm.getLinAccelAccuracy();
            data.ay = IIR(fsm.getLinAccelX(), data.ay, _alpha_accel);
            data.ax = IIR(-fsm.getLinAccelY(), data.ax, _alpha_accel);
            data.az = IIR(-fsm.getLinAccelZ(), data.az, _alpha_accel);

            //... Gyro ...//
           // fsm.getGyro(data.gx, data.gy, data.gz, data.gyroAccuracy);

            // data.gx = fsm.getFastGyroX();
            // data.gy = fsm.getFastGyroY();
            // data.gz = fsm.getFastGyroZ();
            data.gyroAccuracy = fsm.getGyroAccuracy();
            data.gy = IIR(fsm.getGyroX(), data.gy, _alpha_gyro);
            data.gx = IIR(fsm.getGyroY(), data.gx, _alpha_gyro);
            data.gz = IIR(fsm.getGyroZ(), data.gz, _alpha_gyro);
            
            //... Rotation Vector ...//
            fsm.getQuat(data.qi, data.qj, data.qk, data.qw, data.quatRadianAccuracy, data.quatAccuracy);

            // data.qi = fsm.getQuatI();
            // data.qj = fsm.getQuatJ();
            // data.qk = fsm.getQuatK();
            // data.qw = fsm.getQuatReal();
            // data.quatAccuracy = fsm.getQuatAccuracy();
            // data.quatRadianAccuracy = fsm.getQuatRadianAccuracy();


            //... Euler Angle Representation ...//
            data.pitch = fsm.getRoll() + d2r* 0.6578f;
            data.roll = fsm.getPitch() + d2r* 1.3772;
            data.yaw = fsm.getYaw() + FSM_YAW_OFFSET_RAD;    
            //data.yaw = 0.00f;

            
        }
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

    void Sensors::sample_flow(){
        //sample the raw velocity (Must be altitude/attitude corrected)
        // uint16_t vx, vy;
        // flow.readMotionCount( &vx, &vy );

        // data.vx = (float) vx;
        // data.vy = (float) vy;

        // data.status.flow = 1;

    static uint32_t last_sample;
    int16_t dx, dy;
    float ofx, ofy;

    float dt = (micros() - last_sample);
    dt = dt/1000000;
    
    last_sample = micros();
    
    // Here dy and dx are flipped, to match orientation of IMU (which is placed to align with the body frame)
    flow.readMotionCount( &dx, &dy );

    // Convert change in pixels to unitless velocity 1/s
    ofx = ((float)dx / dt ) / PMW3901_FOCAL; // Focal length (in px) found experimentally
    ofy = ((float)dy / dt ) / PMW3901_FOCAL;

    // Return the unitless velocity, which can be scaled by height above ground (z)
    data.vx = ofx; // pixels / second
    data.vy = ofy; // pixels / second

    data.status.flow = 1;


    }


    void Sensors::run_estimator(void){


    /* ---- Sensor processing ---- */
    float p[3] = {0.00f}; // Position vector (z body to world)
    float v[3] = {0.00f}; // Velocity vector (vx, vy to world)
    float a[3] = {0.00f}; // Acceleration vector (ax, ay, az to world)

    // Rotate lidar measurement to world frame
    p[2] = data.z;
    rotate_to_world( p );
    data.evz = (data.ez - p[2]) / DT_SEC;
    data.ez = p[2];

    // Perform gyrocompensation on flow and rotate to world frame.
    v[0] = data.vx * p[2] - data.gy * p[2];
    v[1] = data.vy * p[2] - data.gx * p[2]; 
    rotate_to_world( v );

    // Rotate acceleration to world frame
    a[0] = data.ax; a[1] = data.ay; a[2] = data.az;
    rotate_to_world( a );

    //integrate az to get vz from accelerometer for testing 
    data.evz_accel = a[2] * DT_SEC;

    //update the world frame accel values 
    data.axw = a[0];
    data.ayw = a[1];
    data.azw = a[2];

    /* ---- Estimation ---- */
    // Fill input vector with acceleration
    H.Fill(0.00f);
    Z.Fill(0.00f);
    U_est = { a[0], a[1], a[2] };


    // Fill measurement vector with data
   // if( data.status.pos == 1 ){
        // H(0,0) = 1; H(1,1) = 1;
        // Z(0) = data.x;
        // Z(1) = data.y;
    //}

    if( data.status.lidar == 1){
        H(2,2) = 1.0f;
        Z(2,1) = p[2]; // p[2]: z
    }

    if( data.status.flow == 1){
        H(3,3) = 1; H(4,4) = 1;
        Z(3) = v[0]; // vx
        Z(4) = v[1]; // vy
    }

    // Matrix <6,1> Xpre_t1 = A * Xe;
    // Matrix <6,1> Xpre_t2 = B * U_est;
    // Matrix <6,1> Xpre_temp = Xpre_t1 + Xpre_t2;

    Xpre = (A * Xe) + (B * U_est); 

    Xe = Xpre + Kf*(Z - H*Xpre); 

    if(isnan(Xe(0,0))) Xe.Fill(0.00f);
    // Serial << " Xeafter: " <<  Xe << "\n";
    // Fil estimate struct with values (for telemetry and stuff)

    estimate.x = Xe(0);
    estimate.y = Xe(1);
    estimate.z = Xe(2);
    estimate.vx = Xe(3);
    estimate.vy = Xe(4);
    estimate.vz = Xe(5);

    // Reset status (measurements has been used!)
     data.status.flow = 0;
     data.status.lidar = 0;
    // data.status.imu = 0;
    // data.status.pos = 0;
}

void Sensors::set_origin(){
    yaw_origin = data.yaw;

   // X.Fill(0);
}


void Sensors::update_pos( float x, float y ){

    data.x = x;
    data.y = y;

    data.status.pos = 1;
}

// Rotate yaw to align with origin / home
float Sensors::rotate_yaw( float yaw ){

    float rotated = yaw - yaw_origin;

    if( rotated > PI )
        rotated -= TWO_PI;
    else if( rotated < -PI )
        rotated += TWO_PI;
    
    return rotated;

}
    void Sensors::rotate_to_world( float * vector )
    {

        float p = data.roll;
        float q = data.pitch;
         float u = data.yaw;
        //float u = 0.00f;
        Matrix<3,1> in = { vector[0], vector[1], vector[2] };
        Matrix<3,1> out = {0,0,0};

        // Matrix<3,3> R = {   cos(data.pitch)*cos(data.yaw), sin(data.roll)*sin(data.pitch)*cos(data.yaw)-cos(data.roll)*sin(data.yaw), cos(data.roll)*sin(data.pitch)*cos(data.yaw)+sin(data.roll)*sin(data.yaw) ,
        //                     cos(data.pitch)*sin(data.yaw), sin(data.roll)*sin(data.pitch)*sin(data.yaw)+cos(data.roll)*cos(data.yaw), cos(data.roll)*sin(data.pitch)*sin(data.yaw)-sin(data.roll)*cos(data.yaw) ,
        //                     -sin(data.pitch),       sin(data.roll)*cos(data.pitch),                      cos(data.roll)*cos(data.pitch)                      };
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


    void Sensors::print_imu(void)
    {
        char text[250];

        sprintf(text, "%0.5f, %0.5f, %0.5f, \t   %0.5f, %0.5f, %0.5f,\t   %0.5f, %0.5f, %0.5f,\t   %0.5f, %0.5f, %0.5f, \t  %0.5f, %0.5f, %0.5f, \t  %i, %i, %i ",
        r2d*data.roll,
        r2d*data.pitch,
        r2d*data.yaw,

        r2d*data.gx,
        r2d*data.gy,
        r2d*data.gz,

        data.ax,
        data.ay,
        data.az,

        estimate.x,
        estimate.y,
        estimate.z,

        estimate.vx,
        estimate.vy,
        estimate.vz,

        data.linAccuracy,
        data.gyroAccuracy,
        data.quatAccuracy);

        Serial.println(text);
    }

    void Sensors::print_fsm(void)
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