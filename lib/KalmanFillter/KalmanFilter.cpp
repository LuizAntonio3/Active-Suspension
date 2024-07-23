#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(Matrix<systemOrder, systemOrder> A, Matrix<systemOrder,1> B, Matrix<1, systemOrder> C, float qValue, float rValue){
  
  // for(int i = 0; i < systemOrder; i++) {
  //   for(int j = 0; j < systemOrder; j++) {
  //     if(i == j) {
  //       // U(i, j) = 1;
  //       this->Q = qValue;
  //     }
  //     else{
  //       // U(i, j) = 0;
  //       this->Q = 0;
  //     }
  //   }
  // }

  this->A = A;
  this->B = B;
  this->C = C;
  
  // this->Q = {
  //   qValue, 0,
  //   0, qValue/10
  // };
  this->Q = {qValue};
  this->R = {rValue};
  this->U = {1, 0};
};

KalmanFilter::~KalmanFilter(){};

Matrix<KalmanFilter::systemOrder, 1> KalmanFilter::kalman(float input, float output){

  //a priori
  this->x_priori = this->A * this->x_hat + this->B * input;
  this->P_priori = this->A * this->P_hat * ~this->A + this->U * this->Q * ~this->U;
  
  //kalman gain
  this->K_kalman = this->P_priori * ~this->C * Inverse(this->C * this->P_priori * ~this->C + this->R);

  //a posteriori
  this->x_hat = this->x_priori + this->K_kalman * (output - (this->C * this->x_priori)(0,0));
  this->P_hat = this->P_priori - this->K_kalman * this->C * this->P_priori;
  
  return this->x_hat;
};

float KalmanFilter::getPTrace(){

  float trace = this->P_hat(0) + this->P_hat(2);

  return trace;
}
