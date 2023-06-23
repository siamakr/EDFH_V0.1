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
volatile float sensor_timer{0};
volatile float print_timer{0.00f};
volatile float estimator_timer{0.00f};
volatile float mst{0.00f};       //Mission Start Timer
volatile float init_timer{0.00f};
//... Flags ...//
volatile bool is_calibrated{false};  
volatile bool start_flag{false};
volatile bool prime_start_flag{false};


void print_control_imu(void);
void step_response_state_machine(float step_interval_ms, float angle);
void run_hover_program(void);
void print_control_imu_estimater(void);
void print_estimator_main();
void user_read_int(int & value, String message);
void user_read_anykey(String message);
void user_read_float(float & value, String message);

#ifdef STATEMACHINE
void setup() {

  Serial.begin(115200);
  // Serial.println("Serial Started...");

  // //Servo inits
  control.act.init_servos();
  control.act.init_edf();
  delay(100);
  control.act.zero_servos();
  delay(100);
  //control.edf_startup(5);
  delay(200);
 // control.init();

  //Sensors init
  sensor.lidar_init();
  delay(100);
  sensor.fsm_init();
  delay(200);

   // control.edf_startup(5);
  // sensor.fsm_init();
  control.status = CONTROL_STATUS_EDF_PRIMING;
  init_timer = millis();
}

// void loop() {

//   switch (control.status)
//   {
//     case CONTROL_STATUS_STATIONARY:
//       // Wait for user input to run program
//       user_read_anykey("waiting for user input...");
//       // Change next state to EDF priming 
//       control.status = CONTROL_STATUS_EDF_PRIMING;
//       // Initialize/zero program parameters/timers
//       start_flag = false;
//       init_timer = millis();
//     break;
    
//     case CONTROL_STATUS_EDF_PRIMING:
//       //prime EDF for 5 seconds, and after enter flying mode 
//       if((millis() - init_timer) <= 5000){ 
//         control.act.edf.writeMicroseconds(1500);
//       }else{ 
//         control.status = CONTROL_STATUS_FLYING;
//         mst = millis();
//       }
//     break;

//     case CONTROL_STATUS_FLYING:
//       run_hover_program();
//       step_response_state_machine(2500, 3.00f);
//     break;
    
//     case CONTROL_STATUS_LANDING:
//     //TODO: Change params for landing
//     break;

//     case CONTROL_STATUS_IMU_CALIBRATION:
//     sensor.sample_fsm();
//         while(!(sensor.data.gyroAccuracy == 3 && sensor.data.linAccuracy == 3 && sensor.data.quatAccuracy == 3 ) )
//         {
//             sensor.sample_fsm();
//             delay(250);
//             sensor.print_fsm();
//            // Serial.println("move vehicle to calibrate IMU ");
//         }
//     break;
    
//     default:
//       control.status = CONTROL_STATUS_STATIONARY;
//     break;
//   }

// }


void loop() {
  sensor.sample_fsm();

  if(control.status == CONTROL_STATUS_STATIONARY)
  {
    if(Serial.available() == true){
      start_flag = false;     //this will reset mst once edf priming is done 
      init_timer = millis();  //resets edf priming timer
      control.status = CONTROL_STATUS_EDF_PRIMING;  //changes state to edf priming on next state
    }
  }else if(control.status == CONTROL_STATUS_EDF_PRIMING)
  {
    control.act.writeEDF((int) 1700);
    if(millis() - init_timer <= 6000)
    {
      control.status = CONTROL_STATUS_EDF_PRIMING;
    }else{
      control.status = CONTROL_STATUS_FLYING; 
    }
  }else if(control.status == CONTROL_STATUS_FLYING){

    if(start_flag == false)
    {
      mst = millis();
      start_flag = true;
    }
    //sample BNO080 as fast as possible allowing for .isAvailable() to catch 
    //when imu is not ready with new readings. 
    //sensor.sample_fsm();
    
    //..... Sensor Timer .....//
    if(millis() - sensor_timer >= DT_MSEC)
    {
      sensor_timer = millis();
    
      sensor.sample_lidar();
      //sensor.run_estimator();

      control.lqr_int(sensor.data.roll, 
                    sensor.data.pitch, 
                    sensor.data.yaw, 
                    sensor.data.gx, 
                    sensor.data.gy, 
                    sensor.data.gz, 
                    sensor.estimate.z, 
                    sensor.estimate.vz);

    }

    //..... Estimator Timer .....//   
    if(millis() - estimator_timer >= DT_MSEC )
    {
      estimator_timer = millis();
      sensor.run_estimator();
      //control.actuate();
      //control.actuate_servos();
      //control.actuate_edf()
    }

    //..... Print Timer .....//
    if(millis() - print_timer >= (DT_MSEC * 5)  ) //DT_MSEC * 4 = 40mS
    {
      print_timer = millis();
      //control.print_debug();
      //print_control_imu();
      print_control_imu_estimater();
      //sensor.print_estimator();
    // print_estimator_main();

    }

    step_response_state_machine(2000, 3.00f);
  }

  // }else{
  //   Serial.println("waiting for priming");
  //   delay(1000);
  // }

}

void run_hover_program(void){
    if(start_flag == false)
    {
      mst = millis();
      start_flag = !start_flag;
    }
    //sample BNO080 as fast as possible allowing for .isAvailable() to catch 
    //when imu is not ready with new readings. 
    sensor.sample_fsm();
    
    //..... Sensor Timer .....//
    if(millis() - sensor_timer >= DT_MSEC)
    {
      sensor_timer = millis();
    
      //sensor.sample_lidar();
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
      control.lqr_int(sensor.data.roll, 
                    sensor.data.pitch, 
                    sensor.data.yaw, 
                    sensor.data.gx, 
                    sensor.data.gy, 
                    sensor.data.gz, 
                    sensor.estimate.z, 
                    sensor.estimate.vz);
    // */

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
    if(millis() - print_timer >= (DT_MSEC * 10)  ) //DT_MSEC * 4 = 40mS
    {
      print_timer = millis();
      //control.print_debug();
      print_control_imu();
      //sensor.print_estimator();
    // print_estimator_main();

    }

    //step_response_state_machine(2500, 3.00f);
}



void step_response_state_machine(float step_interval_ms, float angle)
{
  float elapsed_time{millis() - mst};

  if(elapsed_time >= (step_interval_ms * 1) && elapsed_time < (step_interval_ms * 2) )
  {
    control.set_reference(SETPOINT_PITCH, 0.00f);
    control.set_reference(SETPOINT_ROLL , 0.00f);
  }

  else if(elapsed_time >= ( step_interval_ms * 2) && elapsed_time < (step_interval_ms * 3))
  {
    control.set_reference(SETPOINT_PITCH, -angle);
    control.set_reference(SETPOINT_ROLL , 0.00f);
  }

  else if(elapsed_time >= ( step_interval_ms * 3 ) && elapsed_time < (step_interval_ms * 4))
  {
    control.set_reference(SETPOINT_PITCH, angle);
    control.set_reference(SETPOINT_ROLL , 0.00f);
  }

  else if(elapsed_time >= (step_interval_ms * 4) && elapsed_time <= (step_interval_ms * 5))
  {
    control.set_reference(SETPOINT_PITCH, 0.00f);
    control.set_reference(SETPOINT_ROLL , -angle);
  }

  else if(elapsed_time >= (step_interval_ms * 5) && elapsed_time < (step_interval_ms * 6))
  {
    control.set_reference(SETPOINT_PITCH, 0.00f);
    control.set_reference(SETPOINT_ROLL , angle);
  }

  else if(elapsed_time >= (step_interval_ms * 6) && (elapsed_time < step_interval_ms * 7))
  {
    control.set_reference(SETPOINT_PITCH, angle);
    control.set_reference(SETPOINT_ROLL , -angle);
  }

  else if(elapsed_time >= (step_interval_ms * 7) && (elapsed_time < step_interval_ms * 8))
  {
    control.set_reference(SETPOINT_PITCH, angle);
    control.set_reference(SETPOINT_ROLL , -angle);
  }

  // if(elapsed_time >= (step_interval_ms * 8) && (elapsed_time < step_interval_ms * 9))
  // {
  //   control.set_reference(SETPOINT_PITCH, -angle);
  //   control.set_reference(SETPOINT_ROLL , angle);
  // }

  // if(elapsed_time >= (step_interval_ms * 9) && (elapsed_time < step_interval_ms * 10))
  // {
  //   control.set_reference(SETPOINT_PITCH, -angle);
  //   control.set_reference(SETPOINT_ROLL , angle);
  // }

  // if(elapsed_time >= (step_interval_ms * 10) && (elapsed_time < step_interval_ms * 11))
  // {
  //   control.set_reference(SETPOINT_PITCH, angle);
  //   control.set_reference(SETPOINT_ROLL , angle);
  // }

  // if(elapsed_time >= (step_interval_ms * 11) && (elapsed_time < step_interval_ms * 12))
  // {
  //   control.set_reference(SETPOINT_PITCH, 0.00f);
  //   control.set_reference(SETPOINT_ROLL , 0.00f);
  // }

  // if(elapsed_time >= (step_interval_ms * 12) && (elapsed_time < step_interval_ms * 13))
  // {
  //   control.set_reference(SETPOINT_PITCH, 2*angle);
  //   control.set_reference(SETPOINT_ROLL , 2*angle);
  // }

  // if(elapsed_time >= (step_interval_ms * 13) && (elapsed_time < step_interval_ms * 14))
  // {
  //   control.set_reference(SETPOINT_PITCH, -2*angle);
  //   control.set_reference(SETPOINT_ROLL , -angle);
  // }

  // if(elapsed_time >= (step_interval_ms * 15) && (elapsed_time < step_interval_ms * 16))
  // {
  //   control.set_reference(SETPOINT_PITCH, 0.00f);
  //   control.set_reference(SETPOINT_ROLL , 0.00f);
  // }

  else if(elapsed_time >= (step_interval_ms * 8))
  {
    control.act.edf_shutdown();
    control.act.zero_servos();
    //while(1);
    control.status = CONTROL_STATUS_STATIONARY;
  }



}



//TODO: This function should go in Controller!!!
void print_control_imu(void)
{
  char text[250];
  //              Roll  RollSP pitch  pitchSP  yaw       gx      gy    gz        ax    ay      az        cdax   cdaxx   pwmx   cday   cdayy  pwmy  Tm    pwmedf
  sprintf(text, "%0.5f, %0.5f,\t %0.5f, %0.5f,\t %0.5f,\t %0.5f, %0.5f, %0.5f,\t %0.5f, %0.5f, %0.5f,\t %0.5f, %0.5f,%i,\t %0.5f, %05f, %i,\t %0.5f, %i,\t %i, %i, %i ",
    r2d*sensor.data.roll,
      r2d*control.SP_hover_int(0),
    r2d*sensor.data.pitch,
      r2d*control.SP_hover_int(1),
    r2d*sensor.data.yaw,

    r2d*sensor.data.gx,
    r2d*sensor.data.gy,
    r2d*sensor.data.gz,

    sensor.data.ax,
    sensor.data.ay,
    sensor.data.az,

    r2d*control.cd.angle_x,
    r2d*control.cd.angle_xx,
      control.act.ad.pwmx,

    r2d*control.cd.angle_y,
    r2d*control.cd.angle_yy,
      control.act.ad.pwmy,

    control.cd.Tedf,
      control.act.ad.pwmedf,

    sensor.data.linAccuracy,
    sensor.data.gyroAccuracy,
    sensor.data.quatAccuracy);

  Serial.println(text);

}

void print_control_imu_estimater(void)
{
  char text[250];
  //              Roll  RollSP pitch  pitchSP  yaw       gx      gy    gz        ax    ay      az        cdax   cdaxx   pwmx   cday   cdayy  pwmy  Tm    pwmedf
  sprintf(text, "%0.5f, %0.5f,\t %0.5f, %0.5f,\t %0.5f,\t %0.5f, %0.5f, %0.5f,\t %0.5f, %0.5f, %0.5f, \t %0.5f, %0.5f,%i,\t %0.5f, %05f, %i,\t %0.5f, %i,\t %i, %i, %i ",
    r2d*sensor.data.roll,
      r2d*control.SP_hover_int(0),
    r2d*sensor.data.pitch,
      r2d*control.SP_hover_int(1),
    r2d*sensor.data.yaw,

    sensor.estimate.x,
    sensor.estimate.y,
    sensor.estimate.z,

    sensor.estimate.vx,
    sensor.estimate.vy,
    sensor.estimate.vz,
    //sensor.data.z,

    r2d*control.cd.angle_x,
    r2d*control.cd.angle_xx,
      control.act.ad.pwmx,

    r2d*control.cd.angle_y,
    r2d*control.cd.angle_yy,
      control.act.ad.pwmy,

    control.cd.Tedf,
      control.act.ad.pwmedf,

    sensor.data.linAccuracy,
    sensor.data.gyroAccuracy,
    sensor.data.quatAccuracy);

  Serial.println(text);

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

void user_read_int(int & value, String message)
{
        //get start microseconds from user and store
    Serial.print(message);

    while (Serial.available()) Serial.read(); //Clear anything in RX buffer
    while (Serial.available() == 0) delay(10); //Wait for user to press key

    //Read user input
    value = (int)Serial.parseInt();
    Serial.println();  
}

void user_read_float(float & value, String message)
{
        //get start microseconds from user and store
    Serial.print(message);

    while (Serial.available()) Serial.read(); //Clear anything in RX buffer
    while (Serial.available() == 0) delay(10); //Wait for user to press key

    //Read user input
    value = Serial.parseFloat();
    Serial.println();  
}

void user_read_anykey(String message)
{
    Serial.println(message);
    while (Serial.available()) Serial.read(); //Clear anything in RX buffer
    while (Serial.available() == 0) delay(10); //Wait for user to press key
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