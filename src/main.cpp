#include <Arduino.h>
#include "Sensors.h"
#include "Controller.h"




#define STATEMACHINE
//#define QUICKTESTENV

//Envoking Class objects 
Sensors sensor; 
Controller control; 




//..... State Machine Vars .....// 
//... Timing Vars ...//
float sensor_timer, print_timer, estimator_timer;
float mst{0.00f};       //Mission Start Timer
//... Flags ...//
bool is_calibrated{false};  
bool start_flag{false};


void print_control_imu(void);
void step_response_state_machine(float step_interval_ms, float angle);
void print_estimator_main();

#ifdef STATEMACHINE
void setup() {

  Serial.begin(115200);
  Serial.println("Serial Started...");

  // //Servo inits
  control.act.init_servos();
  control.act.init_edf();
  delay(100);
  control.act.zero_servos();
  delay(100);

 // control.init();

  //Sensors init
  sensor.lidar_init();
  delay(100);
  sensor.fsm_init();
  delay(100);

 // sensor.init();

  //EDF init + prime

   control.act.prime_edf(5);



// mst = millis();


}


void loop() {

    if(start_flag == false)
  {
    mst = millis();
    start_flag = true;
  }
  //sample BNO080 as fast as possible allowing for .isAvailable() to catch 
  //when imu is not ready with new readings. 
  sensor.sample_fsm();
  
  //..... Sensor Timer .....//
  if(millis() - sensor_timer >= DT_MSEC)
  {
    sensor_timer = millis();
  
    sensor.sample_lidar();
    //sensor.run_estimator();
    /*
    control.lqr(sensor.data.roll, 
                  sensor.data.pitch, 
                  sensor.data.yaw, 
                  sensor.data.gx, 
                  sensor.data.gy, 
                  sensor.data.gz, 
                  sensor.estimate.z, 
                  sensor.estimate.vz);
    */
    ///*
    control.lqr(sensor.data.roll, 
                  sensor.data.pitch, 
                  sensor.data.yaw, 
                  sensor.data.gx, 
                  sensor.data.gy, 
                  sensor.data.gz, 
                  sensor.estimate.z, 
                  sensor.estimate.vz);
    //*/

    //control.hover(sensor.data, sensor.estimate);
  }

  //..... Estimator Timer .....//   
  if(millis() - estimator_timer >= DT_MSEC )
  {
    estimator_timer = millis();
   // sensor.run_estimator();
    //control.actuate();
    //control.actuate_servos();
    //control.actuate_edf()
  }

  //..... Print Timer .....//
  if(millis() - print_timer >= (DT_MSEC * 4)  ) //DT_MSEC * 4 = 40mS
  {
    print_timer = millis();

    print_control_imu();
    //sensor.print_estimator();
   // print_estimator_main();

  }

  step_response_state_machine(3000, 6.00f);



}



void step_response_state_machine(float step_interval_ms, float angle)
{
  float elapsed_time{millis() - mst};

  if(elapsed_time >= (step_interval_ms * 1) && elapsed_time < (step_interval_ms * 2) )
  {
    control.set_reference(SETPOINT_PITCH, 0.00f);
    control.set_reference(SETPOINT_ROLL , 0.00f);
  }

  if(elapsed_time >= ( step_interval_ms * 2) && elapsed_time < (step_interval_ms * 3))
  {
    control.set_reference(SETPOINT_PITCH, angle);
    control.set_reference(SETPOINT_ROLL , 0.00f);
  }

  if(elapsed_time >= ( step_interval_ms * 3 ) && elapsed_time < (step_interval_ms * 4))
  {
    control.set_reference(SETPOINT_PITCH, -angle);
    control.set_reference(SETPOINT_ROLL , 0.00f);
  }

  if(elapsed_time >= (step_interval_ms * 4) && elapsed_time <= (step_interval_ms * 5))
  {
    control.set_reference(SETPOINT_PITCH, 0.00f);
    control.set_reference(SETPOINT_ROLL , angle);
  }

  if(elapsed_time >= (step_interval_ms * 5) && elapsed_time < (step_interval_ms * 6))
  {
    control.set_reference(SETPOINT_PITCH, 0.00f);
    control.set_reference(SETPOINT_ROLL , -angle);
  }

  if(elapsed_time >= (step_interval_ms * 6) && (elapsed_time < step_interval_ms * 7))
  {
    control.set_reference(SETPOINT_PITCH, -angle);
    control.set_reference(SETPOINT_ROLL , angle);
  }

  if(elapsed_time >= (step_interval_ms * 7)) control.act.suspend();

  // Commment out below when using any of the print functions 
  // Serial.print(r2d * control.SP_hover(0));
  // Serial.print(",  ");
  // Serial.println(r2d * control.SP_hover(1));

}



//TODO: This function should go in Controller!!!
void print_control_imu(void)
{
  char text[250];
  //              Roll  RollSP pitch  pitchSP  yaw       gx      gy    gz        ax    ay      az        cdax   cdaxx   pwmx   cday   cdayy  pwmy  Tm    pwmedf
  sprintf(text, "%0.5f, %0.5f,  %0.5f, %0.5f,    %0.5f,\t   %0.5f, %0.5f, %0.5f,\t  %0.5f, %0.5f, %0.5f,\t   %0.5f,  %0.5f,  %i,  %0.5f, %05f,  %i,   %0.5f, %i   ",
    r2d*sensor.data.roll,
    r2d*control.SP_hover(0),
    r2d*sensor.data.pitch,
    r2d*control.SP_hover(1),
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
void print_estimator_main(void)
{
    char text[250];

    sprintf(text, "%0.5f, %0.5f, %0.5f, %0.5f, \t   %0.5f, %0.5f, %0.5f,\t  %0.5f, %0.5f, %0.5f, \t  %0.5f, %0.5f, %0.5f, \t  %i, %i, %i ",
    r2d*sensor.data.roll,
    r2d*sensor.data.pitch,
    r2d*sensor.data.yaw,
    sensor.data.z,

    sensor.estimate.x,
    sensor.estimate.y,
    sensor.estimate.z,

    sensor.estimate.vx,
    sensor.estimate.vy,
    sensor.estimate.vz,

    sensor.data.ax,
    sensor.data.ay,
    sensor.data.az,

    sensor.data.linAccuracy,
    sensor.data.gyroAccuracy,
    sensor.data.quatAccuracy);

    Serial.println(text);
}

#endif

#ifdef QUICKTESTENV

#include "Actuator.h" 

Actuator a;
bool flag{false};

void setup(){
  Serial.begin(115200);
  a.init_servos();
}

void loop(){
  //   a.servo_dance_x(12.00f, 15);
  //  // delay(2000);
  //   a.servo_dance_x(10.00f, 10);
  //  // delay(2000);
  //   a.servo_dance_x(8.00f, 7);
  //   a.servo_dance_x(7.00f, 5);
  //   a.servo_dance_x(5.00f, 3);
  //   a.servo_dance_x(3.00f, 2);
  //   a.servo_dance_x(2.00f, 1);
  //   a.zero_servos();
  //  // delay(2000);


  //   delay(10000);

  while(a.step_response_x_servo(5));
  delay(1000); 
  a.zero_servos();
  while(a.step_response_y_servo(5));
  delay(1000);
  a.zero_servos();

}
#endif