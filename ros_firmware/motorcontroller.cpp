#include "motorcontroller.h"

MotorController::MotorController(uint8 _pwm, uint8 _in1, uint8 _in2,
                                 timer_dev* _timenc, unsigned char _modec, uint8 _ch1, uint8 _ch2, bool _invert,
                                 float _ratio, float _controller_freq):
                                      DCMotor(_pwm, _in1, _in2),
                                      Encoder(_timenc, _modec, _ch1, _ch2){

  force_stop();
  invertPolarity(_invert);
  reset();
  ratio=_ratio;
  reference=0.0;
  error=0.0;
  controller_freq=_controller_freq;
  rad_factor=1000.0*(2*PI/ratio)/controller_freq;

  clearMemory();
  id_memory=0;

  p_error=0;
  c_i=0;

  measure=0.0;
  travel=0.0;

}

void MotorController::addMemory(float _val){
  id_memory++;
  if (id_memory>=MEM_SIZE){
    id_memory=0;
  }
  measure_memory[id_memory]=_val;
}

float MotorController::meanMemory(){
  float mean=0.0;
  for (uint8_t i=0; i<MEM_SIZE; i++){
    mean+=measure_memory[i];
  }
  return mean/float(MEM_SIZE);
}

void MotorController::clearMemory(){
  for (uint8_t i=0; i<MEM_SIZE; i++){
    measure_memory[i]=0.0;
  }
}


float MotorController::checkLimit(float _val){
  if (_val<-MOTOR_LIMIT){
    return -MOTOR_LIMIT;
  }
  if (_val>MOTOR_LIMIT){
    return MOTOR_LIMIT;
  }
  return _val;
}

void MotorController::setRadAtS(float _val){
  //convert rad/s to pwm and move the motor
  setSpeed(65535.0*_val/MOTOR_LIMIT);  
}

void MotorController::update(){
  // get measure
  measure=getCount()*rad_factor;
  //  reset encoder
  reset();

  //  add to the filter
  addMemory(measure);
  
  //get the moving average
  measure=meanMemory();
  travel+=measure*controller_freq/1000.0;
    
  //error
  error=reference-measure;

  //pid
  c_p = kp*error;
  c_i = checkLimit(c_i+ki*error);
  c_d = kd*(error-p_error);

  //output
  actuation=checkLimit(c_p+c_i+c_d);
 
  // set rad/s output
  setRadAtS(actuation);

  p_error=error;
}

void MotorController::init(){
  setReference(0.0);
  force_stop();
  clearMemory();
  reset();
  resetTravel();
}