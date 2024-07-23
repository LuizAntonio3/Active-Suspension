#ifndef ENCODER_3
#define ENCODER_3

#include <Arduino.h>
#define PULSES_PER_ROTATION 4000.0
#define DEG_PER_PULSE 360.0/PULSES_PER_ROTATION
#define MIN_PERIOD 5 // ms

enum Direction { DIR_STILL = 0, DIR_CW = 1, DIR_CCW = -1};

class ThreePhaseEncoder {
    
    private:
        static ThreePhaseEncoder *intances[2];

        uint8_t phaseA, phaseB, phaseZ;
        volatile long int pulses = 0; // POSITIVE -> CW - NEGATIVE -> CCW
        volatile long int deltaPulses = 0;
        volatile double fullRotations = 0; // POSITIVE -> FULL ROTATION IN CW / NEGATIVE -> FULL ROTATION IN CCW
        Direction rotationDirection = DIR_STILL;
        double maxAngle = 360; // default to 360 deg
        double angularSpeedInPulses = 0; // pulses per second
        double correctionFactor = 1;

        // volatile long int timePulseCW = 0;
        // volatile long int timePulseCCW = 0;
        volatile long int lastCalculation = 0;

        void incrementPulse();

    public:
        void IRAM_ATTR phase_A_Interrupt();
        void IRAM_ATTR phase_B_Interrupt();
        void IRAM_ATTR phase_Z_Interrupt();

        ThreePhaseEncoder(uint8_t phase_a, uint8_t phase_b, uint8_t phase_z);
        void setCorrectionFactor(double factor) { correctionFactor = factor; }

        // double getPulses() { return pulses; }
        double getSRPulses() { return pulses; }
        double getSRPulsesInDegrees() { return getSRPulses() * DEG_PER_PULSE; }
        double getPulses() { return fullRotations * PULSES_PER_ROTATION + pulses; }
        double getPulsesDegrees() { return getPulses() * DEG_PER_PULSE; }
        double getPulsesRad() { return getPulsesDegrees() * 0.01745329252; }
        double getPulsesDegresWithCorrection() {return getPulses() * DEG_PER_PULSE / correctionFactor; }
        double getPulsesRadWithCorrection() { return getPulsesDegrees() * 0.01745329252 / correctionFactor; }
        double getAngularSpeedInPulses() { return angularSpeedInPulses; }
        double getAngularSpeedInDegrees() { return angularSpeedInPulses * DEG_PER_PULSE; }
        double getAngularSpeedInRad() { return getAngularSpeedInDegrees() * 0.01745329252; } // check if this conversion is correct
        double getAngularSpeedInRadCorrection() { return getAngularSpeedInRad() / correctionFactor; }
        double updateZero() { return this->pulses = 0; }

        // Outside logic can set the angle in degrees wich the value will reset
        void setMaxAngle(double max) { maxAngle = max; }
        void updateSpeed();

        uint8_t pA() { return phaseA; }
        uint8_t pB() { return phaseB; }
        uint8_t pZ() { return phaseZ; }
};

#endif