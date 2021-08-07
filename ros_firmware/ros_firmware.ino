#include "encoder.h"
#include "board_pins.h"
#include "dcmotor.h"
#include "motorcontroller.h"

int i=3000;
unsigned long t=0;
uint16 timer_joint=0;
int k=0;
float x=0;
int32_t vel=65535;
/*
DCMotor dcmotor_C(MOTOR_C_PWM,MOTOR_C_IN1,MOTOR_C_IN2);
DCMotor dcmotor_D(MOTOR_D_PWM,MOTOR_D_IN1,MOTOR_D_IN2);
Encoder encoder_motor_D(MOTOR_D_TIM, COUNT_BOTH_CHANNELS,MOTOR_D_CH1,MOTOR_D_CH2);
Encoder encoder_motor_C(MOTOR_C_TIM, COUNT_BOTH_CHANNELS,MOTOR_C_CH1,MOTOR_C_CH2);
*/
MotorController motorC(MOTOR_C_PWM,MOTOR_C_IN1,MOTOR_C_IN2,MOTOR_C_TIM, COUNT_BOTH_CHANNELS,MOTOR_C_CH1,MOTOR_C_CH2,true,MOTOR_RATIO,10.0);
MotorController motorD(MOTOR_D_PWM,MOTOR_D_IN1,MOTOR_D_IN2,MOTOR_D_TIM, COUNT_BOTH_CHANNELS,MOTOR_D_CH1,MOTOR_D_CH2,true,MOTOR_RATIO,10.0);

int ctrl = 1;
int kk=0;
void setup() {
  pinMode(LED_BUILTIN,OUTPUT);
  Serial1.begin(115200);
  systick_attach_callback(tick);
  t=millis();
  motorC.clearMemory();
  motorC.setReference(6.28);
}


void loop() {
  /*
  if (millis()-t>5000){
    i=-i;
    t=millis();
  }
  */
  
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
  /*

  if (timer_joint>=10){
    Serial1.print(encoder_motor_D.getCount());
    Serial1.print("   ");
    Serial1.println(encoder_motor_C.getCount());
    encoder_motor_C.reset();
    encoder_motor_D.reset();
    timer_joint=0;
  }*/
  /*
  while(encoder_motor_C.getCount()<=(44*30)){
    dcmotor_C.setSpeed(-i);
  }
  dcmotor_C.setStop();
  encoder_motor_C.reset();
  /*
  dcmotor_C.setSpeed(i);
  dcmotor_D.setSpeed(i);
  */
  if (timer_joint>=10){
    motorC.update();
    motorD.update();
    timer_joint=0;
  }
}

void tick(void){
  k++;
  kk++;
  timer_joint++;
}
