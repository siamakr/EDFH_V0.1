#include <Arduino.h>
#include "./sensors.h"
// #include "Servo.h"



Sensors sensor; 

//..... Timing Vars.....//
float sensor_timer, print_timer;



void setup() {
    //..... Begins .....//
  Serial.begin(115200);

  //.......... Inits ...........//
  //allocate sensor object holding all sensor func/vars
  
  delay(100);
  //first function call
  sensor.init();     //arg=true when you want to calibrate IMU
  delay(100);

}

void loop() {
  Serial.print("lll");
  // //..... Sensor Timer .....//
  // if(millis() - sensor_timer >= DT_MSEC)
  // {
  //   sensor_timer = millis();
  //   sensor.sample_fsm();
  // }

  // //..... Print Timer .....//S
  // if(millis() - print_timer >= DT_MSEC * 4) //DT_MSEC * 4 = 40mS
  // {
  //   print_timer = millis();
  //   sensor.print_fsm();
  // }
  
  // sensor.save_calibrate_fsm();
   sensor.sample_fsm(); 
   sensor.print_fsm_calibration(); 
}
