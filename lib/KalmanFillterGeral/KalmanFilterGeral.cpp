#include "KalmanFilterGeral.h"

KalmanFilterGeral::KalmanFilterGeral(Matrix<systemOrderGeral, systemOrderGeral> A, Matrix<systemOrderGeral,1> B, Matrix<1, systemOrderGeral> C, float qValue, float rValue){
  
  // for(int i = 0; i < systemOrderGeral; i++) {
  //   for(int j = 0; j < systemOrderGeral; j++) {
  //     if(i == j) {
  //       this->Q = qValue;
  //     }
  //     else{
  //       this->Q = 0;
  //     }
  //   }
  // }

  this->A = A;
  this->B = B;
  this->C = C;
  
  this->Q = {
    qValue, 0, 0, 0, 0,
    0, qValue*10, 0, 0, 0,
    0, 0, qValue*10, 0, 0,
    0, 0, 0, qValue*10, 0,
    0, 0, 0, 0, qValue*10
  };

  this->R = {rValue};
  this->P_hat.Fill(0);
  this->P_priori.Fill(0);
};

KalmanFilterGeral::~KalmanFilterGeral(){};

Matrix<KalmanFilterGeral::systemOrderGeral, 1> KalmanFilterGeral::kalman(float input, float output){

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

float KalmanFilterGeral::getPTrace(){

  float trace = this->P_hat(0) + this->P_hat(2);

  return trace;
}
