#include "CustomServo.h"

CustomServo::CustomServo(uint8_t enable, uint8_t in1Pin, uint8_t in2Pin, int in1PwmChannel, int in2PwmChannel, int pwmFrequency,
                         int pwmResolution, float target) {
    
    this->target = target;
    // this->controllerOrder = controllerOrder;
    enableServo();

    // // PWM init
    this->enablePwmChannel = in1PwmChannel;
    this->in1PwmChannel = in1Pin;
    this->in2PwmChannel = in2Pin;

    // this->in1PwmChannel = in1PwmChannel;
    // this->in2PwmChannel = in2PwmChannel;

    ledcSetup(in1PwmChannel, pwmFrequency, pwmResolution);
    ledcAttachPin(enable, in1PwmChannel);

    pinMode(this->in1PwmChannel, OUTPUT);
    pinMode(this->in2PwmChannel, OUTPUT);

    // ledcSetup(in1PwmChannel, pwmFrequency, pwmResolution);
    // ledcSetup(in2PwmChannel, pwmFrequency, pwmResolution);
    // ledcAttachPin(in1Pin, in1PwmChannel);
    // ledcAttachPin(in2Pin, in2PwmChannel);

    // // PID init
    maxPwm = pow(2, pwmResolution) * MOTOR_VOLTAGE / SOURCE_VOLTAGE;
    minPwm = -maxPwm;

    // controller.setOrder(this->controllerOrder);
    controller.setSaturationLimits(minPwm, maxPwm);
}

void CustomServo::applyPwmToMotor(float pwmValue) {
    this->pwmOutput = pwmValue;

    if(stationaryCount >= 10){
        pwmOutput = 200;
        digitalWrite(this->in1PwmChannel, HIGH);
        digitalWrite(this->in2PwmChannel, HIGH);
        ledcWrite(this->enablePwmChannel, pwmOutput);
    }
    else if(pwmOutput >= 0) {
        // CW
        if (pwmOutput > maxPwm) 
            pwmOutput = maxPwm;

        // if (lastPwmOutput < 0) {
        //     digitalWrite(this->in1PwmChannel, LOW);
        //     digitalWrite(this->in2PwmChannel, LOW);
        //     if (pwmOutput > 800)
        //         pwmOutput = 800;
        // }

        // else if (pwmOutput <= DEAD_ZONE)
        //     pwmOutput = 0;
        
        // ledcWrite(this->in1PwmChannel, pwmOutput);
        // ledcWrite(this->in2PwmChannel, 0);

        digitalWrite(this->in1PwmChannel, LOW);
        digitalWrite(this->in2PwmChannel, HIGH);
        ledcWrite(this->enablePwmChannel, pwmOutput);
    }
    else {
        // CCW
        if (pwmOutput < minPwm) 
            pwmOutput = minPwm;

        // if (lastPwmOutput > 0) {
        //     digitalWrite(this->in1PwmChannel, LOW);
        //     digitalWrite(this->in2PwmChannel, LOW);
        //     if (pwmOutput < -800)
        //         pwmOutput = -800;
        // }
        // else if(pwmOutput >= -DEAD_ZONE)
        //     pwmOutput = 0;
            
        // ledcWrite(this->in1PwmChannel, 0);
        // ledcWrite(this->in2PwmChannel, -pwmOutput);

        digitalWrite(this->in1PwmChannel, HIGH);
        digitalWrite(this->in2PwmChannel, LOW);
        ledcWrite(this->enablePwmChannel, -pwmOutput);
    }

    // this->lastPwmOutput = this->pwmOutput;
}

void CustomServo::brake() {
    ledcWrite(this->in1PwmChannel, 0);
    ledcWrite(this->in2PwmChannel, 0);
}

void CustomServo::update(float currentRead, KalmanFilter* kalmanFilter) {
    if(!servoEnabled) 
        return;

    this->currentRead = currentRead;

    Matrix<systemOrder,1> states;
    states = kalmanFilter->kalman(this->getPwmOutput(), currentRead);
    // Serial.print(String(currentRead) + ", ");
    // Serial.print(String(states(0,0), 5) + ", ");
    // Serial.print(String(states(1,0), 5) + "\n");
    
    states(0, 0) = currentRead;

    if(abs(this->getTarget() - currentRead) / abs(this->getTarget() + 0.0001) * 100 <= 1 and stationaryCount <= 50){
        this->stationaryCount++;
    }
    else
        this->stationaryCount = 0;

    pwmOutput = controller.controlLawTracking(states, this->target);

    applyPwmToMotor(this->pwmOutput);
}