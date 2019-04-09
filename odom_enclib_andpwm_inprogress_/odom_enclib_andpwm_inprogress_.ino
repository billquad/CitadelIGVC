
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <Encoder.h>

Encoder right(3, 19);
Encoder left(2, 18); 

std_msgs::Int16 left_ticks_number;
std_msgs::Int16 right_ticks_number;
std_msgs::Int16 pwm_Right;
std_msgs::Int16 pwm_Left;
// Init the publishers
ros::Publisher left_pub("left_ticks", &left_ticks_number);
ros::Publisher right_pub("right_ticks", &right_ticks_number);
ros::Publisher left_pwd("pwm_l", &pwm_Left);
ros::Publisher right_pwd("pwm_r", &pwm_Right);

const int Left_Motor = 10;
const int Right_Motor = 9;
const int Green_Light = 6;
int duty_vel_R;
int duty_vel_L;
int pwm_R =155;
int pwm_L =155;

long positionLeft  = -999;
long positionRight = -999;

ros::NodeHandle nh_arduino; // Initiated the node handle

void duty_left (const std_msgs::Float32& duty_cycle_L){
    duty_vel_L = duty_cycle_L.data;
  if (duty_vel_L == 0)
    {pwm_L = 155;}
  else
    {pwm_L = duty_vel_L*20+155.25;
    pwm_Left.data=pwm_L;
    left_pwd.publish(&pwm_Left);
    }
    
  analogWrite(Left_Motor, pwm_L);
  delay(200);// accessign the data within the 
  // UInt16 wrapper
}
void duty_right (const std_msgs::Float32& duty_cycle_R){
  duty_vel_R = duty_cycle_R.data;
  if (duty_vel_R == 0)
    {pwm_R = 155;}
  else
    {pwm_R = duty_vel_R*10+155.25;
    }
    pwm_Right.data=pwm_R;
    right_pwd.publish(&pwm_Right);

  analogWrite(Right_Motor, pwm_R);
  delay(200);
}

void light (const std_msgs::Int16& light){
  if( light.data == 1 ){
    digitalWrite(Green_Light, HIGH);
  }
  else digitalWrite(Green_Light, LOW);
}
ros::Subscriber<std_msgs::Float32> sub_D_L("Duty_Cycle_Left", duty_left);
ros::Subscriber<std_msgs::Float32> sub_D_R("Duty_Cycle_Right", duty_right);
ros::Subscriber<std_msgs::Int16> sub_L("Light", light);
void setup() {
  pinMode(Left_Motor, OUTPUT);
  pinMode(Right_Motor, OUTPUT);
  pinMode(Green_Light, OUTPUT);
  digitalWrite(Green_Light, HIGH);

  nh_arduino.initNode();
  nh_arduino.subscribe(sub_D_L);
  nh_arduino.subscribe(sub_D_R);
  nh_arduino.subscribe(sub_L);
  nh_arduino.advertise(left_pub);
  nh_arduino.advertise(right_pub);
  nh_arduino.advertise(left_pwd);
  nh_arduino.advertise(right_pwd);
}

void loop() {
  long newLeft, newRight;
  newLeft = left.read();
  newRight = right.read();
  if (newLeft != positionLeft || newRight != positionRight) {
    left_ticks_number.data = newLeft;
    right_ticks_number.data = newRight;
    positionLeft = newLeft;
    positionRight = newRight;
  }
  
  left_pub.publish(&left_ticks_number);
  right_pub.publish(&right_ticks_number);
  nh_arduino.spinOnce();
}
