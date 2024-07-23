#ifndef CONTROLLER_GERAL_H
#define CONTROLLER_GERAL_H

#include <BasicLinearAlgebra.h>
using namespace BLA;

class ControllerGeral{

  public:
    static const int systemOrderGeral = 5;

  private:
    
    Matrix<1, systemOrderGeral> K;
    Matrix<1, systemOrderGeral> Ktracking;

    // Matrix<1, maxControllerOrder> K;
    // Matrix<1, maxControllerOrder> Ktracking;

    float integratorGain;
    float v_z1 = 0;
    float lowerLimit;
    float upperLimit;
    int controllerOrder;

  public:
    void setOrder(int order) { this->controllerOrder = order; }
    float controlLaw(Matrix<systemOrderGeral, 1> currentState);
    void setSaturationLimits(float lowerLimit, float upperLimit) { this->lowerLimit = lowerLimit; this->upperLimit = upperLimit; };
    float saturate(float input);
    void setRegulationGains(Matrix<1, systemOrderGeral> gains);
    void setTrackingGains(Matrix<systemOrderGeral+1, 1> gains);
    float controlLawTracking(Matrix<systemOrderGeral,1> currentState, float setpoint);
};
#endif
