#include <Arduino.h>
#include "Sensors.h"
#include "Controller.h"




//#define STATEMACHINE
#define QUICKTESTENV

//Envoking Class objects 
Sensors sensor; 
Controller control; 




//..... State Machine Vars .....// 
//... Timing Vars ...//
float sensor_timer, print_timer, estimator_timer;
//... Flags ...//
bool is_calibrated{false};  


void print_control_imu(void);

#ifdef STATEMACHINE
void setup() {
    //..... Begins .....//
  Serial.begin(115200);
  //Wire.begin();
  Serial.print("setup started");
  //.......... Inits ...........//
  //allocate sensor object holding all sensor func/vars
  //sensor.init();     //initializes all sensors at once
  control.init();
  delay(100);
  //first function call
  sensor.init();     //initializes all sensors at once
  //sensor.fsm_init();
  //sensor.lidar_init();
  delay(100);
  


}


void loop() {
  //sample BNO080 as fast as possible allowing for .isAvailable() to catch 
  //when imu is not ready with new readings. 
  
  sensor.sample_fsm();
  //..... Sensor Timer .....//
  if(millis() - sensor_timer >= DT_MSEC)
  {
    sensor_timer = millis();
    sensor.sample_lidar();
    sensor.run_estimator();
    ///*
    control.hover(sensor.data.roll, 
                  sensor.data.pitch, 
                  sensor.data.yaw, 
                  sensor.data.gx, 
                  sensor.data.gy, 
                  sensor.data.gz, 
                  sensor.estimate.z, 
                  sensor.estimate.vz);
    //*/
  }

  //..... Estimator Timer .....//   
  if(millis() - estimator_timer >= DT_MSEC *1.50f )
  {
    estimator_timer = millis();
    //sensor.run_estimator();
  }

  //..... Print Timer .....//
  if(millis() - print_timer >= (DT_MSEC) * 4 ) //DT_MSEC * 4 = 40mS
  {
    print_timer = millis();

    //print_control_imu();
    sensor.print_estimator();

  }
  
  
}

#endif

//TODO: This function should go in Controller!!!
void print_control_imu(void)
{
  char text[250];
  //              Roll  pitch   yaw       gx      gy    gz        ax    ay      az        cdax   cdaxx   pwmx   cday   cdayy  pwmy  Tm    pwmedf
  sprintf(text, "%0.5f, %0.5f, %0.5f,\t   %0.5f, %0.5f, %0.5f,\t  %0.5f, %0.5f, %0.5f,\t   %0.5f,  %0.5f,  %i,  %0.5f, %05f,  %i,   %0.5f, %i   ",
    r2d*sensor.data.roll,
    r2d*sensor.data.pitch,
    r2d*sensor.data.yaw,
    r2d*sensor.data.gx,
    r2d*sensor.data.gy,
    r2d*sensor.data.gz,
    sensor.data.ax,
    sensor.data.ay,
    sensor.data.az,
    r2d*control.cd.angle_x,
    r2d*control.cd.angle_xx,
    control.cd.pwmx,
    r2d*control.cd.angle_y,
    r2d*control.cd.angle_yy,
    control.cd.pwmy,
    control.cd.Tedf,
    control.cd.pwmedf
  );

  Serial.print(text);
  Serial.print(sensor.data.linAccuracy);
  Serial.print(", ");
  Serial.print(sensor.data.gyroAccuracy);
  Serial.print(", ");
  Serial.print(sensor.data.quatAccuracy);
  Serial.print(", ");
  Serial.print(sensor.data.z);
  Serial.println(", ");
}


#ifdef QUICKTESTENV

#include "Actuator.h" 

Actuator a;
bool flag{false};

void setup(){
  Serial.begin(115200);
  a.init_servos();
}

void loop(){
    a.servo_dance_x(12.00f, 15);
   // delay(2000);
    a.servo_dance_x(10.00f, 10);
   // delay(2000);
    a.servo_dance_x(8.00f, 7);
    a.servo_dance_x(7.00f, 5);
    a.servo_dance_x(5.00f, 3);
    a.servo_dance_x(3.00f, 2);
    a.servo_dance_x(2.00f, 1);
    a.zero_servos();
   // delay(2000);


    delay(10000);
}
#endif