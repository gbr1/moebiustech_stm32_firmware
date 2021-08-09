#ifndef __BOARD_PINS_H__
#define __BOARD_PINS_H__


#define LED_BUILTIN PC13
#define serial_port Serial1
#define USE_STM32_HW_SERIAL //is Serial1

//motor A encoder is associated to TIM5, must be added afio to switch the mode (conflict with motorD TIM2)
#define MOTOR_A_PWM PC9
#define MOTOR_A_IN1 PD2
#define MOTOR_A_IN2 PC12
#define MOTOR_A_CH1 PA0
#define MOTOR_A_CH2 PA1
#define MOTOR_A_TIM TIMER5

//motor B encoder is associated to TIM3, PB5 can go in conflict with TIM3CH1
#define MOTOR_B_PWM PC8
#define MOTOR_B_IN1 PB4
#define MOTOR_B_IN2 PB5
#define MOTOR_B_CH1 PA6
#define MOTOR_B_CH2 PA7
#define MOTOR_B_TIM TIMER3

//motor C encoder is associated to TIM4
#define MOTOR_C_PWM PC7
#define MOTOR_C_IN1 PC5
#define MOTOR_C_IN2 PC4
#define MOTOR_C_CH1 PB6
#define MOTOR_C_CH2 PB7
#define MOTOR_C_TIM TIMER4

//motor D encoder is associated to TIM2
#define MOTOR_D_PWM PC6
#define MOTOR_D_IN1 PB0
#define MOTOR_D_IN2 PB1
#define MOTOR_D_CH1 PA15
#define MOTOR_D_CH2 PB3
#define MOTOR_D_TIM TIMER2


#endif