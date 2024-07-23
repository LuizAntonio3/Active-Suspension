#ifndef KALMANFILTER_GERAL_H
#define KALMANFILTER_GERAL_H

#include <BasicLinearAlgebra.h>
using namespace BLA;

class KalmanFilterGeral{

  public:
    static const int systemOrderGeral = 5;
    // static const int saidas = 1;

  private:

    Matrix<systemOrderGeral, systemOrderGeral> A;
    Matrix<systemOrderGeral,1> B;
    Matrix<1, systemOrderGeral> C;
    
    Matrix<systemOrderGeral, systemOrderGeral> P_priori;
    Matrix<systemOrderGeral, systemOrderGeral> P_hat;
    
    Matrix<systemOrderGeral, 1> x_hat;
    Matrix<systemOrderGeral, 1> x_priori;

    Matrix<systemOrderGeral, 1> K_kalman;
    
    //Identity matrix
    Eye<systemOrderGeral, systemOrderGeral> U;

    Matrix<systemOrderGeral, systemOrderGeral> Q;
    Matrix<1,1> R;
  
  public:

    KalmanFilterGeral(Matrix<systemOrderGeral, systemOrderGeral> A, Matrix<systemOrderGeral,1> B, Matrix<1, systemOrderGeral> C, float qValue, float rValue);
    ~KalmanFilterGeral();
    float getPTrace();

    Matrix<systemOrderGeral, 1> kalman(float input, float output);  
};

#endif
