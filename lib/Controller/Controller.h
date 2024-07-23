#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <BasicLinearAlgebra.h>
using namespace BLA;

class Controller{

  private:
    static const int systemOrder = 2;
    
    Matrix<1, systemOrder> K;
    Matrix<1, systemOrder> Ktracking;

    // Matrix<1, maxControllerOrder> K;
    // Matrix<1, maxControllerOrder> Ktracking;

    float integratorGain;
    float v_z1 = 0;
    float lowerLimit;
    float upperLimit;
    int controllerOrder;

  public:
    void setOrder(int order) { this->controllerOrder = order; }
    float controlLaw(Matrix<systemOrder, 1> currentState);
    void setSaturationLimits(float lowerLimit, float upperLimit) { this->lowerLimit = lowerLimit; this->upperLimit = upperLimit; };
    float saturate(float input);
    void setRegulationGains(Matrix<1, systemOrder> gains);
    void setTrackingGains(Matrix<systemOrder+1, 1> gains);
    float controlLawTracking(Matrix<systemOrder,1> currentState, float setpoint);
};
#endif
