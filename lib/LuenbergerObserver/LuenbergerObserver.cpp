#include "LuenbergerObserver.h"

LuenbergerObserver::LuenbergerObserver(Matrix<systemOrder, systemOrder> A, Matrix<systemOrder,1> B, Matrix<1, systemOrder> C, Matrix<systemOrder, 1> L){

  this->A = A;
  this->B = B;
  this->C = C;
  this->L = L;
  
};

LuenbergerObserver::~LuenbergerObserver(){};

Matrix<LuenbergerObserver::systemOrder, 1> LuenbergerObserver::estimate(float input, float output){
  x_hat = A * x_hat + B*input + L * (output - (C * x_hat)(0,0));
  
  return x_hat;
}