#ifndef DAMPER_SENSOR
#define DAMPER_SENSOR

#include <ThreePhaseEncoder.h>

#define SINGLE_LINK_LENGTH 10e-3 // meters

class DamperSensor {
    private:
        float zero;
        double deflection, previousDeflection = 0;;
        double deflectionSpeed;
        double linearDeflection, previousLinearDeflection = 0;
        double linearDeflectionSpeed = 0;
        float locationLowerFixation;
        float locationUpperFixation;

        unsigned long int previousTime; // for derivative calculation

        ThreePhaseEncoder* encoderArm;
        ThreePhaseEncoder* encoderSingleLink;
    public:
        DamperSensor(ThreePhaseEncoder* encoderArm, ThreePhaseEncoder* encoderSingleLink) {
            this->encoderArm = encoderArm;
            this->encoderSingleLink = encoderSingleLink;
        };
        double getDeflection() { return deflection; }
        double getDeflectionSpeed() { return deflectionSpeed; }
        double getLinearActuatorDeflection() { return linearDeflection; }
        double getLinearActuatorDeflectionSpeed() { return linearDeflectionSpeed; }
        void updateZero();
        void update();
};

#endif