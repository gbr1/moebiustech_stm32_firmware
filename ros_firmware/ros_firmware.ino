#include "board_pins.h"
#include "motorcontroller.h"
#include <ros.h>
#include <moebiustech_stm32_ros/Drive.h>
#include <moebiustech_stm32_ros/Feedback.h>

// ROS
ros::NodeHandle  nh;

// timers
uint16 timer_joint=0;
uint16 timer_led=0;
uint16 timer_motors=0;


bool led_status = false;
uint16 led_period = 1000;

// joints
MotorController motorD(MOTOR_D_PWM,MOTOR_D_IN1,MOTOR_D_IN2,MOTOR_D_TIM, COUNT_BOTH_CHANNELS,MOTOR_D_CH1,MOTOR_D_CH2,true,MOTOR_RATIO,10.0); //left
MotorController motorC(MOTOR_C_PWM,MOTOR_C_IN1,MOTOR_C_IN2,MOTOR_C_TIM, COUNT_BOTH_CHANNELS,MOTOR_C_CH1,MOTOR_C_CH2,true,MOTOR_RATIO,10.0); //right

//subscriber for controlling joints
void motors(const moebiustech_stm32_ros::Drive& cmd_msg);
ros::Subscriber<moebiustech_stm32_ros::Drive> jointdrive("/moebiustech_stm32/cmd_drive", motors);

//publisher for joints feedback
moebiustech_stm32_ros::Feedback robot_state;
ros::Publisher jointstate("/moebiustech_stm32/feedback", &robot_state);

void setup() {
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);
  serial_port.begin(115200);

  nh.initNode();
  nh.subscribe(jointdrive);
  nh.advertise(jointstate);
  


  systick_attach_callback(tick);
}


void loop() {
  motorC.init();
  motorD.init();
  //waiting rosserial connection
  led_period=1000;
  while(!nh.connected()){
    updateLed();
    nh.spinOnce();
  }
  //connection is ok
  nh.loginfo("MoebiusTech STM32 board is successfully connected!");
  led_period=100;
  //run
  while (nh.connected()){
    updateLed();
    updateMotors();
    publishJoints();    
    nh.spinOnce();
    //delay(1);
  }
}

void updateLed(){
  if (led_status){
    if (timer_led>=100){
      digitalWrite(LED_BUILTIN,HIGH);
      led_status=false;
      timer_led=0;
    }
  }
  else{
    if (timer_led>=led_period){
      digitalWrite(LED_BUILTIN,LOW);
      led_status=true;
      timer_led=0;
    }
  }
}

void updateMotors(){
  if (timer_motors>=10){
    motorC.update();
    motorD.update();
    timer_motors=0;  
  }
}

void motors(const moebiustech_stm32_ros::Drive& cmd_msg){
  systick_attach_callback(NULL);
  motorD.setReference(cmd_msg.drivers[0]); //left wheel
  motorC.setReference(cmd_msg.drivers[1]); //right wheel
  systick_attach_callback(tick);
}

void publishJoints(){
  if (timer_joint>20){  //50hz for ros control

    robot_state.header.frame_id = "base_link";
    robot_state.header.stamp=nh.now();

    robot_state.drivers[0].measured_velocity = motorD.getRadAtS(); //right
    robot_state.drivers[0].measured_travel = motorD.getTravel();

    robot_state.drivers[1].measured_velocity = motorC.getRadAtS(); //left
    robot_state.drivers[1].measured_travel = motorC.getTravel();

    jointstate.publish(&robot_state);

    timer_joint=0;
  }
}

void tick(void){
  timer_led++;
  timer_joint++;
  timer_motors++;
}
