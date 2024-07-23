#include "ControllerGeral.h"

float ControllerGeral::saturate(float input){
  if(input < lowerLimit) input = lowerLimit;
  if(input > upperLimit) input = upperLimit;

  return input;
}

void ControllerGeral::setRegulationGains(Matrix<1, systemOrderGeral> gains){
   this->K = gains;
};

void ControllerGeral::setTrackingGains(Matrix<systemOrderGeral+1, 1> gains){

  for(int i = 0; i < systemOrderGeral; i++) {
    this->Ktracking(i) = gains(i, 0);
  }
  this->integratorGain = gains(systemOrderGeral, 0);
}

float ControllerGeral::controlLaw(Matrix<systemOrderGeral,1> currentState){
  
  return saturate((this->K * currentState)(0));
}

float ControllerGeral::controlLawTracking(Matrix<systemOrderGeral,1> currentState, float setPoint){

  float v = this->v_z1 + setPoint - currentState(0, 0);
  
  this->v_z1 = v;

  // limitation
  // if(v_z1 > 10000)
  //   v_z1 = 10000;
  // else if(v_z1 < -10000)
  //   v_z1 = -10000;
  
  return (this->Ktracking * currentState + this->integratorGain * v)(0);
}
