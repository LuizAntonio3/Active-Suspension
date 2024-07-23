#ifndef UNSPRUNG_MASS_SENSOR
#define UNSPRUNG_MASS_SENSOR

#include <ThreePhaseEncoder.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;

class UnsprungMassSensor {
    private:
        Matrix<2, 1> unsprungMassLocation;
        Matrix<2, 1> unsprungMassPreviousLocation;
        double unsprungMassVelocity;
        double unsprungMassPreviousVelocity;
        double unsprungMassAceleration;
        
        unsigned long int previousTime; // for derivative calculation

        ThreePhaseEncoder* encoderArm;
    public:
        UnsprungMassSensor(ThreePhaseEncoder* encoderArm) { this->encoderArm = encoderArm; }
        double getUnsprungMassAceleration() { return this->unsprungMassAceleration; }
        void update();
};

#endif