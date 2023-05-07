///// Sensors.cpp /////

#include "Sensors.h"

//BNO080 * fsm = new BNO080(imuCSPin, imuWAKPin, imuINTPin, imuRSTPin);


Sensors::Sensors(void){
 //fsm = new BNO080(imuCSPin, imuWAKPin, imuINTPin, imuRSTPin);
}

void Sensors::init(void)
{
      // Disable all SPI devices  before setup
    pinMode( imuCSPin, OUTPUT );
    digitalWrite( imuCSPin, HIGH );

    delay(100);

  //..... FSM305/BNO080 Init .....//
  Serial.print("FSM Init start..."); 
  delay(300);

  if (fsm->beginSPI(imuCSPin, imuWAKPin, imuINTPin, imuRSTPin, 3000000) == false)
  {
    Serial.println("BNO080 over SPI not detected. Are you sure you have all 6 connections? Freezing...");
    while(1);
  }

    fsm->calibrateAll();
    fsm->enableLinearAccelerometer(DT_MSEC);  // m/s^2 no gravity
    fsm->enableRotationVector(DT_MSEC);  // quat
    //fsm->enableGameRotationVector(DT_MSEC);
    fsm->enableGyro(DT_MSEC);  // rad/s
    //fsm->enableGyroIntegratedRotationVector(DT_MSEC);
    //fsm->enableMagnetometer(DT_MSEC);  // cannot be enabled at the same time as RotationVector (will not produce data)
  
  Serial.println("FSM Init Finished..."); 
  delay(300);

  //..... PMW3901 Init .....//


}

///.......... FSM305/BNO080 FUNCTIONS ..........////

void save_calibrate_fsm(void)
{
  if(Serial.available())
  {
    byte incoming = Serial.read();

    if(incoming == 's')
    {
      fsm->saveCalibration(); //Saves the current dynamic calibration data (DCD) to memory
      fsm->requestCalibrationStatus(); //Sends command to get the latest calibration status

      //Wait for calibration response, timeout if no response
      int counter = 100;
      
      while(1)
      {
        if(--counter == 0) break;
        if(fsm->dataAvailable() == true)
        {
          //The IMU can report many different things. We must wait
          //for the ME Calibration Response Status byte to go to zero
          if(fsm->calibrationComplete() == true)
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

void sample_fsm(void)
{
  if(fsm->dataAvailable() == true)
  {
    //..... Sample IMU .....//
    //... Linear Accel ...//
    fsm->getLinAccel(data.ax, data.ay, data.az, data.linAccuracy);

    // data.ax = fsm.getAccelX();
    // data.ay = fsm.getAccelY();
    // data.az = fsm.getAccelZ();

    //... Gyro ...//
    fsm->getGyro(data.gx, data.gy, data.gz, data.gyroAccuracy);

    // data.gx = fsm.getFastGyroX();
    // data.gy = fsm.getFastGyroY();
    // data.gz = fsm.getFastGyroZ();

    // data.gx = fsm.getGyroX();
    // data.gy = fsm.getGyroY();
    // data.gz = fsm.getGyroZ();
    
    //... Rotation Vector ...//
    fsm->getQuat(data.qi, data.qj, data.qk, data.qw, data.quatRadianAccuracy, data.quatAccuracy);

    // data.qi = fsm.getQuatI();
    // data.qj = fsm.getQuatJ();
    // data.qk = fsm.getQuatK();
    // data.qw = fsm.getQuatReal();


    //... Euler Angle Representation ...//
    data.roll = fsm->getRoll();
    data.pitch = fsm->getPitch();
    data.yaw = fsm->getYaw();    

  }

}


void print_fsm(void)
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

void print_fsm_calibration(void)
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

// void Sensors::sampleLidar(void)
// {
//   uint8_t newDistance = 0;
//   uint16_t  distance;
  
//   if (garmin.getBusyFlag() == 0)
//   {
//       // Trigger the next range measurement
//       garmin.takeRange();

//       // Read new distance data from device registers
//       distance = garmin.readDistance();

//       // Report to calling function that we have new data
//       newDistance = 1;
//   }
//   data.zb = distance/100.00f;
// }





// float Sensors::IIR(float newSample, float prevOutput, float alpha)
// {
//   return ( (1.0f-alpha)*newSample + alpha * prevOutput);
// }


// ///////////////// Private Functions Begin /////////////////



// void Sensors::rotate_to_world( float * vector ){

//     float p = data.roll;
//     float q = data.pitch;
//     float u = data.yaw;

//     Matrix<3,1> in = { vector[0], vector[1], vector[2] };
//     Matrix<3,1> out = {0,0,0};

//     Matrix<3,3> R = {   cos(q)*cos(u), sin(p)*sin(q)*cos(u)-cos(p)*sin(u), cos(p)*sin(q)*cos(u)+sin(p)*sin(u) ,
//                         cos(q)*sin(u), sin(p)*sin(q)*sin(u)+cos(p)*cos(u), cos(p)*sin(q)*sin(u)-sin(p)*cos(u) ,
//                         -sin(q),       sin(p)*cos(q),                      cos(p)*cos(q)                      };

//     out = R * in;

//     vector[0] = out(0);
//     vector[1] = out(1);
//     vector[2] = out(2);

// }


// float Sensors::IIR( float newSample, float prevOutput, float alpha)
// {
//   return ( (1.0f-alpha)*newSample + alpha * prevOutput);
// }
