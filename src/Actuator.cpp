
#include "Actuator.h"


Actuator::Actuator(void)
{

}

void Actuator::init(void)
{
    init_servos();
    init_edf();
    init_rw();
    delay(100);
    zero_servos();
    zero_rw();
    edf_shutdown();
}
    
void Actuator::init_servos(void)
{
    //attach servo pins
    //sx.attach(XSERVO_PIN, SERVO_X_MIN_US, SERVO_X_MAX_US);
    sx.attach(XSERVO_PIN);
    delay(10);
    //sy.attach(YSERVO_PIN, SERVO_Y_MIN_US, SERVO_Y_MAX_US);
    sy.attach(YSERVO_PIN);
    delay(10);
    // rw.attach(RW_PIN);
    // delay(200);
}

void Actuator::init_rw(void)
{
    rw.attach(RW_PIN);
    delay(200);
}

void Actuator::init_edf(void)
{
    edf.attach(EDF_PIN);
    delay(20);    
    edf.writeMicroseconds(EDF_OFF_PWM);
    delay(100);  
}


void Actuator::zero_servos()
{
    //Zero Servos
    writeXservo(0.00f);
    writeYservo(0.00f);
    delay(10);
}



void Actuator::prime_edf(void)
{
    //go to 1500 and wait 5 seconds
    edf.writeMicroseconds(EDF_MIN_PWM);
    delay(5000);
}

void Actuator::prime_edf(int delay_time_ms)
{
    //go to 1500 and wait 5 seconds
    edf.writeMicroseconds(EDF_MIN_PWM);
    delay(delay_time_ms);
}

bool Actuator::prime_edf(int delay_time_ms, float start_timer)
{
    edf.writeMicroseconds(EDF_MIN_PWM);
    if((millis() - start_timer) < delay_time_ms)
    {
        return true;
    }else{
        return false;
    }
}

bool Actuator::servo_dance_x(float max_angle, int delay_time_ms)
{
    //max_angle refers to the radius that parametrization 
    //of the circle to be "drawn" by the tip of the TVC motor
    //Ex. max_angle = 5 referes that the maximum TVC angle that 
    //is actuated by the servos is 5 degrees.

    // writeYservo(max_angle);
    // delay(500);
    
    for(int i{0}; i <= 360; i++)
    {
        writeXservo((float) max_angle * (float) sin( i * d2r) );
        writeYservo((float) max_angle * (float) cos( i * d2r ) );
        
        //for debugging/testing
        Serial.print(max_angle * sin( (float) i * d2r ));
        Serial.print("\t");
        Serial.println(max_angle * cos( (float) i * d2r ));

        delay(delay_time_ms);
    }
    
    // delay(1000);

    // zero_servos();

    return true;
}

//Write to X servo
void Actuator::writeXservo(float angle)
{
    //map angle in degrees to pwm value for servo
    int pwmX{round( X_P1 * pow(angle,2) + X_P2 * angle + X_P3 ) };
    ad.pwmx = pwmX;
    ad.ang_x = angle;
    sx.writeMicroseconds(pwmX);  
}

void Actuator::writeXservo(int pwm)
{
    ad.pwmx = pwm;
    sx.writeMicroseconds(pwm);  
}

//write to Y servo
void Actuator::writeYservo(float angle)
{
    //Map TVC angle (degrees) to TVC PWM(µs) 
    int pwmY{ round(Y_P1 * pow(angle,2) + Y_P2 * (angle) + Y_P3 ) };      // using polynomial regression coefficients to map tvc angle to pwm vals
    sy.writeMicroseconds(pwmY);
    ad.pwmy = pwmY;
    ad.ang_y = angle;
}

void Actuator::writeYservo(int pwm)
{
    ad.pwmy = pwm;
    sy.writeMicroseconds(pwm);  
}

void Actuator::writeEDF(float Ft)
{
    int pwm{ round( (EDF_P1 * pow(Ft,2)) + EDF_P2 * Ft + EDF_P3 )};
    edf.writeMicroseconds(pwm);   
    ad.pwmedf = pwm; 
}

void Actuator::writeEDF(int pwm)
{
    ad.pwmedf = pwm; 
    edf.writeMicroseconds(pwm);   
}

void Actuator::writeRW(float omega){
    int pwm{round(RW_P1 * omega + RW_P2)};
    ad.pwmrw = pwm;
    ad.omega_rw = omega;
    rw.writeMicroseconds(pwm);
}


void Actuator::edf_shutdown(void)
{
    edf.writeMicroseconds(EDF_OFF_PWM);
}

// to shut everything down if we go past max set angle
void Actuator::emergency_check(float & r, float & p)
{
    if(r > MAX_VEHICLE_ANGLE_DEG || r < -MAX_VEHICLE_ANGLE_DEG || p > MAX_VEHICLE_ANGLE_DEG || p < -MAX_VEHICLE_ANGLE_DEG)
    {
        edf_shutdown();
        zero_servos();
        while(1);       //This should return the state machine into hold state
    }
}

void Actuator::suspend(void)
{
    edf_shutdown();
    zero_servos();
    while(1);       //This should return the state machine into hold state
}

bool Actuator::step_response_x_servo(int angle_deg)
{
    zero_servos();
    delay(1000);

    writeXservo(angle_deg);
    delay(1000);

    int start_pwm{ad.pwmx};
    float timer{millis()};
    writeXservo(-angle_deg); 

    while(ad.pwmx >= -1 * start_pwm);
    float elapsed_time = millis() - timer;

    Serial.println(elapsed_time);

    return true;
}

bool Actuator::step_response_y_servo(int angle_deg)
{
    zero_servos();
    delay(1000);

    writeYservo(angle_deg);
    delay(1000);

    int start_pwm{ad.pwmy};
    float timer{millis()};
    writeYservo(-angle_deg); 

    while(ad.pwmy >= -1 * start_pwm);
    float elapsed_time = millis() - timer;

    Serial.println(elapsed_time);

    return true;
}