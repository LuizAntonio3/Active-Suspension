#ifndef CUSTOM_SERVO
#define CUSTOM_SERVO

#include <Arduino.h>
// #include <QuickPID.h>
#include <Controller.h>
#include <BasicLinearAlgebra.h>
#include "KalmanFilter.h"

using namespace BLA;

#define SOURCE_VOLTAGE 12
#define MOTOR_VOLTAGE 12
#define DEAD_ZONE 250
#define REVERSING_DELAY 1 // ms

class CustomServo {
    private:
        static const int systemOrder = 2;

        uint8_t in1, in2;
        int enable;
        int in1PwmChannel;
        int in2PwmChannel;
        int enablePwmChannel;
        // int controllerOrder;

        bool servoEnabled;

        float target;
        float currentRead;
        int stationaryCount;
        float pwmOutput = 0;
        float lastPwmOutput = 0;

        float minPwm;
        float maxPwm;

        Controller controller;
    public:
        /*
            parameters: uint8_t in1Pin, uint8_t in2Pin, int in1PwmChannel, int in2PwmChannel, int pwmFrequency,
                        int pwmResolution, float Kp, float Ki, float Kd, float targetSpeed = 0, int samplingTimeUs = 20000
        */
        CustomServo(uint8_t enable, uint8_t in1Pin, uint8_t in2Pin, int in1PwmChannel, int in2PwmChannel, int pwmFrequency,
                    int pwmResolution, float target = 0);

        void enableServo() { this->servoEnabled = true; };
        void disableServo() { 
            this->servoEnabled = false; 
            ledcWrite(this->enablePwmChannel, 0);
            digitalWrite(this->in1PwmChannel, LOW);
            digitalWrite(this->in2PwmChannel, LOW);
        };
        
        float getPwmOutput() { return this->pwmOutput; }
        float getTarget() { return this->target; }
        float getMaxPWM() { return this->maxPwm; }
        
        // new speed in rad/s
        void setMaxPwm(float newMaxPWM) { this->maxPwm = newMaxPWM; this->minPwm = -newMaxPWM; }
        void setTarget(float newTarget) { this->target = newTarget; };
        void setTrackingGains(Matrix<systemOrder+1, 1> gains) { controller.setTrackingGains(gains); }
        
        void brake();
        void applyPwmToMotor(float pwmValue);
        void update(float currentSpeed, KalmanFilter* kalmanFilter);
};

#endif