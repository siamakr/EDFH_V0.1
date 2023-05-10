#include <Arduino.h>
#include "Sensors.h"
#include "Controller.h"



Sensors sensor; 
//Controller control; 
//..... Timing Vars.....//
float sensor_timer, print_timer;



void setup() {
    //..... Begins .....//
  Serial.begin(115200);
  Serial.print("setup started");
  //.......... Inits ...........//
  //allocate sensor object holding all sensor func/vars
  
  delay(100);
  //first function call
  sensor.init();     //initializes all sensors at once
  //sensor.fsm_init();
  //sensor.lidar_init();
  delay(100);
  //control.init();


}

void loop() {

  sensor.sample_fsm();
  //..... Sensor Timer .....//
  if(millis() - sensor_timer >= DT_MSEC)
  {
    sensor_timer = millis();
    //sensor.sample_lidar();
    //control.hover(sensor.data.roll, sensor.data.pitch, sensor.data.yaw, sensor.data.gx, sensor.data.gy, sensor.data.gz, sensor.data.z, sensor.data.vz);
  }

  //..... Print Timer .....//
  if(millis() - print_timer >= DT_MSEC * 4) //DT_MSEC * 4 = 40mS
  {
    print_timer = millis();
    sensor.print_sensors();
  }
  
  // sensor.save_calibrate_fsm();
  //  sensor.sample_fsm(); 
  //  sensor.print_fsm_calibration(); 
}
