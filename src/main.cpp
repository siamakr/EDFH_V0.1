#include <Arduino.h>
#include "Sensors.h"
#include "Controller.h"



Sensors sensor; 
Controller control; 
//..... Timing Vars.....//
float sensor_timer, print_timer;


void print_control_imu(void);

void setup() {
    //..... Begins .....//
  Serial.begin(115200);
  Serial.print("setup started");
  //.......... Inits ...........//
  //allocate sensor object holding all sensor func/vars
  control.init();
  delay(100);
  //first function call
  sensor.init();     //initializes all sensors at once
  //sensor.fsm_init();
  //sensor.lidar_init();
  delay(100);
  


}

void loop() {

  sensor.sample_fsm();
  //..... Sensor Timer .....//
  if(millis() - sensor_timer >= DT_MSEC)
  {
    sensor_timer = millis();
    //sensor.sample_lidar();
    control.hover(sensor.data.roll, 
                  sensor.data.pitch, 
                  sensor.data.yaw, 
                  sensor.data.gx, 
                  sensor.data.gy, 
                  sensor.data.gz, 
                  sensor.data.z, 
                  sensor.data.vz);
  }

  //..... Print Timer .....//
  if(millis() - print_timer >= DT_MSEC * 4) //DT_MSEC * 4 = 40mS
  {
    print_timer = millis();
    //sensor.print_fsm_calibration();
    print_control_imu();
  }
  
  // sensor.save_calibrate_fsm();
  //  sensor.sample_fsm(); 
  //  sensor.print_fsm_calibration(); 
}

    void print_control_imu(void)
    {
        char text[250];

        sprintf(text, "%0.5f, %0.5f, %0.5f,\t   %0.5f, %0.5f, %0.5f,\t  %0.5f, %0.5f, %0.5f,\t   %0.5f, %0.5f, %i, %i, %0.5f    ",
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
        r2d*control.cd.angle_y,
        control.cd.pwmx,
        control.cd.pwmy,
        control.cd.Tm);

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
