#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <BasicLinearAlgebra.h>
using namespace BLA;

class KalmanFilter{

  private:

    static const int systemOrder = 2;

    Matrix<systemOrder, systemOrder> A;
    Matrix<systemOrder,1> B;
    Matrix<1, systemOrder> C;
    
    Matrix<systemOrder, systemOrder> P_priori;
    Matrix<systemOrder, systemOrder> P_hat;
    
    Matrix<systemOrder, 1> x_hat;
    Matrix<systemOrder, 1> x_priori;

    Matrix<systemOrder, 1> K_kalman;
    
    //Identity matrix
    Matrix<systemOrder, 1> U;

    Matrix<1, 1> Q;
    Matrix<1,1> R;
  
  public:

    KalmanFilter(Matrix<systemOrder, systemOrder> A, Matrix<systemOrder,1> B, Matrix<1, systemOrder> C, float qValue, float rValue);
    ~KalmanFilter();
    float getPTrace();

    Matrix<systemOrder, 1> kalman(float input, float output);  
};

#endif
