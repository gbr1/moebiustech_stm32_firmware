#ifndef __MOTOR_SPECS_H__
#define __MOTOR_SPECS_H__

const float MOTOR_CPR = 44.0;  // probably 11ppr so 44cpr
const float MOTOR_GR = 30.0;   // from a datasheet 333rpm -> 30:1
const float MOTOR_RATIO = MOTOR_CPR * MOTOR_GR; //from motor encoder shaft to wheel shaft
const float MOTOR_LIMIT = 34.5; // maximum rad/s, 333rpm -> 34.871678415 rad/s

#endif