#include "MPU9250.h"

#include<ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>


#define rm_pwm_pin 2
#define rm_run_stop 3
#define rm_dir_pin 4
//#define rm_brake_pin 5


#define lm_pwm_pin 6
#define lm_run_stop 7
#define lm_dir_pin 8
//#define lm_brake_pin 9



int rm_pwm=0;
int lm_pwm=0;
float rm_analog_val=0;
float lm_analog_val=0;

//ROS Node Handle
ros::NodeHandle nh;
/*******************************IMU SPECIFIC************************************/
//IMU Publisher
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("/imu/data_raw", &imu_msg);
long sequence = 0;


// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x69
MPU9250 IMU(Wire,0x69);
int status;
/*******************************************************************************/

void rm_pwm_callback(const std_msgs::Int16 &msg){
  rm_pwm=msg.data;
  if(rm_pwm==0){
    //digitalWrite(rm_run_stop,LOW);
    rm_run(0);
  }
}
void lm_pwm_callback(const std_msgs::Int16 &msg){
  lm_pwm=msg.data;
  if(lm_pwm==0){
    //digitalWrite(lm_run_stop,LOW);
    lm_run(0);
  }
}


ros::Subscriber<std_msgs::Int16>rm_pwm_sub("/rm_pwm",&rm_pwm_callback);
ros::Subscriber<std_msgs::Int16>lm_pwm_sub("/lm_pwm",&lm_pwm_callback);


void setup() {
    // serial to display data
    Serial.begin(57600);
    while(!Serial) {}
    
    nh.initNode();
    nh.advertise(imu_pub);
    nh.subscribe(rm_pwm_sub);
    nh.subscribe(lm_pwm_sub);
    
    pinMode(rm_pwm_pin,OUTPUT);
    pinMode(rm_run_stop,OUTPUT);
    pinMode(rm_dir_pin,OUTPUT);
    //pinMode(rm_brake_pin,OUTPUT);

    pinMode(lm_pwm_pin,OUTPUT);
    pinMode(lm_run_stop,OUTPUT);
    pinMode(lm_dir_pin,OUTPUT);
    //pinMode(lm_brake_pin,OUTPUT);

    digitalWrite(rm_run_stop,HIGH);
    digitalWrite(lm_run_stop,HIGH);

/*PWM Frequency Changer for Motor Driver*/

    TCCR3B = TCCR3B & B11111000 | B00000010;  // for PWM frequency of 3921.16 Hz Pins :2,3,5
    TCCR4B = TCCR4B & B11111000 | B00000010;  // for PWM frequency of 3921.16 Hz Pins:6,7,8
    //TCCR2B = TCCR2B & B11111000 | B00000010;  // for  PWM frequency of 3921.16 Hz Pins:9,10

    // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }

}

void rm_run(bool x){
  digitalWrite(rm_run_stop,x);

}
void lm_run(bool y){
  digitalWrite(lm_run_stop,y);
}

void loop() {
   // read the sensor
   IMU.readSensor();
  
   rm_analog_val= int(2.55*rm_pwm) ;
   lm_analog_val= int(2.55*lm_pwm) ;
   


    if (rm_pwm<0){
      rm_run(1);
      digitalWrite(rm_dir_pin,0);
      analogWrite(rm_pwm_pin,-(rm_analog_val));
    }
    else if (rm_pwm>0){
      rm_run(1);
      digitalWrite(rm_dir_pin,1);
      analogWrite(rm_pwm_pin,rm_analog_val);
    }
    if (lm_pwm<0){
      lm_run(1);
      digitalWrite(lm_dir_pin,0);
      analogWrite(lm_pwm_pin,map(abs(lm_pwm),0,101,0,256));
    }
    else if (lm_pwm>0){
      lm_run(1);
      digitalWrite(lm_dir_pin,1);
      analogWrite(lm_pwm_pin,map(abs(lm_pwm),0,101,0,256));
    }

    //Header
  imu_msg.header.seq = sequence++;
  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id  ="imu";

  //Linear Acceleration 
  imu_msg.linear_acceleration.x = IMU.getAccelX_mss() ;
  imu_msg.linear_acceleration.y = IMU.getAccelY_mss() ;
  imu_msg.linear_acceleration.z = IMU.getAccelZ_mss() ;
  
  //Angular Velocity
  imu_msg.angular_velocity.x = IMU.getGyroX_rads() ;
  imu_msg.angular_velocity.y = IMU.getGyroY_rads() ; 
  imu_msg.angular_velocity.z = IMU.getGyroZ_rads() ;  

  //Orientation
  imu_msg.orientation.x = 0 ;
  imu_msg.orientation.y = 0 ;
  imu_msg.orientation.z = 0 ;
  imu_msg.orientation.w = 0 ;

  imu_pub.publish(&imu_msg);
  nh.spinOnce();

  delay(10);
}
