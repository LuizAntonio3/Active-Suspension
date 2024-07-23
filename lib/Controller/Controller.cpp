#include "Controller.h"

float Controller::saturate(float input){
  if(input < lowerLimit) input = lowerLimit;
  if(input > upperLimit) input = upperLimit;

  return input;
}

void Controller::setRegulationGains(Matrix<1, systemOrder> gains){
   this->K = gains;
};

void Controller::setTrackingGains(Matrix<systemOrder+1, 1> gains){

  for(int i = 0; i < systemOrder; i++) {
    this->Ktracking(i) = gains(i, 0);
  }
  this->integratorGain = gains(systemOrder, 0);
}

float Controller::controlLaw(Matrix<systemOrder,1> currentState){
  
  return saturate((this->K * currentState)(0));
}

float Controller::controlLawTracking(Matrix<systemOrder,1> currentState, float setPoint){

  float v = this->v_z1 + setPoint - currentState(0, 0);
  
  this->v_z1 = v;

  // limitation
  // if(v_z1 > 10000)
  //   v_z1 = 10000;
  // else if(v_z1 < -10000)
  //   v_z1 = -10000;
  
  return (this->Ktracking * currentState + this->integratorGain * v)(0);
}
