#ifndef LUENBERGEROBSERVER_H
#define LUENBERGEROBSERVER_H

#include <BasicLinearAlgebra.h>
using namespace BLA;

class LuenbergerObserver{

  private:

    static const int systemOrder = 2;

    Matrix<systemOrder, systemOrder> A;
    Matrix<systemOrder,1> B;
    Matrix<1, systemOrder> C; 
    
    Matrix<systemOrder, 1> x_hat;
    Matrix<systemOrder, 1> L;
  
  public:

    LuenbergerObserver(Matrix<systemOrder, systemOrder> A, Matrix<systemOrder,1> B, Matrix<1, systemOrder> C, Matrix<systemOrder, 1> L);
    ~LuenbergerObserver();
    float getPTrace();

    Matrix<systemOrder, 1> estimate(float input, float output);
};

#endif
