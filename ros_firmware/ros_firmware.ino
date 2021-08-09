#include "board_pins.h"
#include "motorcontroller.h"
#include <ros.h>
#include <moebiustech_stm32_ros/Drive.h>

// ROS
ros::NodeHandle  nh;

// timers
int i=3000;
unsigned long t=0;
uint16 timer_joint=0;
uint16 timer_led=0;
float x=0;
int32_t vel=65535;


bool led_status = false;
uint16 led_period = 1000;

// joints
MotorController motorC(MOTOR_C_PWM,MOTOR_C_IN1,MOTOR_C_IN2,MOTOR_C_TIM, COUNT_BOTH_CHANNELS,MOTOR_C_CH1,MOTOR_C_CH2,true,MOTOR_RATIO,10.0);
MotorController motorD(MOTOR_D_PWM,MOTOR_D_IN1,MOTOR_D_IN2,MOTOR_D_TIM, COUNT_BOTH_CHANNELS,MOTOR_D_CH1,MOTOR_D_CH2,true,MOTOR_RATIO,10.0);

//subscriber for controlling joints
void motors(const moebiustech_stm32_ros::Drive& cmd_msg);
ros::Subscriber<moebiustech_stm32_ros::Drive> jointdrive("/moebiustech_stm32/cmd_drive", motors);

int ctrl = 1;
int kk=0;
void setup() {
  pinMode(LED_BUILTIN,OUTPUT);
  serial_port.begin(115200);




  nh.initNode();
  nh.subscribe(jointdrive);


  systick_attach_callback(tick);
  t=millis();
  motorC.clearMemory();
  motorC.setReference(6.28);
}


void loop() {
  led_period=1000;
  while(!nh.connected()){
    updateLed();
    nh.spinOnce();
  }
   //connection is ok
  nh.loginfo("MoebiusTech STM32 board is successfully connected!");
  led_period=100;

  while (nh.connected()){
    updateLed();
    nh.spinOnce();
    //delay(1);
  }


  /*



  if (k>=1000){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    k=0;
  }

  if (kk>=20){
    //motorC.setReference(6.28*(x/5.0-floor(x/5.0)));
    motorC.setReference(6.28*sin(x));
    motorD.setReference(6.28*sin(x));

    x+=0.02;
    kk=0;
  }
  if (timer_joint>=10){
    motorC.update();
    motorD.update();
    timer_joint=0;
  }
  */
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

void tick(void){
  timer_led++;
  kk++;
  timer_joint++;
}
