#include "ThreePhaseEncoder.h"

ThreePhaseEncoder::ThreePhaseEncoder(uint8_t phaseA, uint8_t phaseB, uint8_t phaseZ){
    this->phaseA = phaseA;
    this->phaseB = phaseB;
    this->phaseZ = phaseZ;
}

void ThreePhaseEncoder::incrementPulse() {
    pulses += rotationDirection;
    deltaPulses += rotationDirection;
}

// calculate the mean speed
void ThreePhaseEncoder::updateSpeed() {
    if(millis() - lastCalculation > MIN_PERIOD) {
        angularSpeedInPulses = deltaPulses / ((millis() - lastCalculation)*1e-3);
        deltaPulses = 0;
        lastCalculation = millis();
    }
}

void ThreePhaseEncoder::phase_A_Interrupt() {
    if (digitalRead(this->phaseA)) {
        // RISING TRANSITION
        if (digitalRead(this->phaseB) == LOW) {
            // CW ROTATION
            rotationDirection = DIR_CW;
        }
        else {
            // CCW ROTATION
            rotationDirection = DIR_CCW;
        }
    }
    else {
        if (digitalRead(this->phaseB)) {
            // CW ROTATION
            rotationDirection = DIR_CW;
        }
        else {
            // CCW ROTATION
            rotationDirection = DIR_CCW;
        }
    }

    incrementPulse();
}

void ThreePhaseEncoder::phase_B_Interrupt() {
    if (digitalRead(this->phaseB)) {
        // RISING TRANSITION
        if (digitalRead(this->phaseA) == LOW) {
            // CCW ROTATION
            rotationDirection = DIR_CCW;
        }
        else {
            // CW ROTATION
            rotationDirection = DIR_CW;
        }
    }
    else {
        if (digitalRead(this->phaseA)) {
            // CCW ROTATION
            rotationDirection = DIR_CCW;
        }
        else {
            // CW ROTATION
            rotationDirection = DIR_CW;
        }
    }

    incrementPulse();
}

void ThreePhaseEncoder::phase_Z_Interrupt() {
    double aux = getSRPulsesInDegrees() / 360;

    if(aux > 1)
        aux = 1;
    else if(aux < -1)
        aux = -1;

    fullRotations += aux; 

    if(abs(getPulsesDegresWithCorrection()) > maxAngle) {
        fullRotations = 0;
    }

    this->pulses = 0;
}