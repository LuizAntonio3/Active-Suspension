// #include <Arduino.h>
// #include <pins.h>
// #include <ThreePhaseEncoder.h>
// // #include <CustomMPU6050.h>
// #include <DamperSensor.h>
// #include <UnsprungMassSensor.h>
// #include <CustomServo.h>
// #include <prbs.h>
// #include <BasicLinearAlgebra.h>
// #include <system.h>
// #include <geralSystem.h>
// #include <KalmanFilter.h>
// // #include <TinyMPU6050.h>
// #include <MPU6050_light.h>
// #include <KalmanFilterGeral.h>
// #include <ControllerGeral.h>

// using namespace BLA;

// // tasks related constants
// #define PRINT_SENSOR_DATA
// #define USE_MPU6050
// #define USE_ENCODERS
// #define ENABLE_SERVO_CONTROLLER
// #define TEST_SERVO
// // #define ENABLE_GERAL_CONTROLLER
// #define UPDATE_SENSORS
// // #define ENABLE_OVERRIDE_CONTROLLER
// // #define USE_MANUAL_SETPOINT
// // #define MANUAL_PWM
// // #define IDENTIFY_MOTOR
// // #define IDENTIFY_WITH_MOTOR_CONTROLLER

// #define SENSOR_UPDATE_TIME 19 // ms
// #define SAMPLING_TIME_SERVO 1 // ms - for the controller that works but is slow
// #define SAMPLING_TIME 19 // ms
// #define PRBS_HOLDING_FACTOR_MOTOR 50
// #define PRBS_HOLDING_FACTOR 6
// #define PRBS_PERIOD 12000 // <- update this value
// #define PLANETARY_REDUCTION 26.9
// #define PWM_DIVISION_FACTOR 1.2629629 // 1.27875

// float controllerEnabled = 1;

// #ifdef USE_ENCODERS
// ThreePhaseEncoder encoderActuator(PIN_ACTUATOR_ENCODER_A, PIN_ACTUATOR_ENCODER_B, PIN_ACTUATOR_ENCODER_Z);
// ThreePhaseEncoder encoderArm(PIN_ARM_ENCODER_A, PIN_ARM_ENCODER_B, PIN_ARM_ENCODER_Z);
// DamperSensor damperSensor(&encoderArm, &encoderActuator);
// UnsprungMassSensor unsprungMassSensor(&encoderArm);
// #endif

// #ifdef USE_MPU6050
// // CustomMPU6050 sprungMassMpu(-139, 1685, 723, -47, 6, 42);
// MPU6050 sprungMassMpu(Wire);
// #endif

// CustomServo singleLinkServo(PIN_ENABLE_SINGLE_LINK, PIN_INT1_SINGLE_LINK_PWM, PIN_INT2_SINGLE_LINK_PWM, 0, 1, 30000, 10);
// KalmanFilter kalmanFilter(sys.A, sys.B, sys.C, 1.8e-5, 1e-5); // colocar os dois estados na printagem

// ControllerGeral geralController;
// KalmanFilterGeral kalmanFilterGeral(sysGeral.A, sysGeral.B, sysGeral.C, 1e-5, 1e-5); // colocar os seis estados na printagem

// Matrix<systemOrder+1,1> singleLinkGains;
// Matrix<1, systemOrderGeral> geralControllerGains;

// #if defined(IDENTIFY_MOTOR) || defined(IDENTIFY_WITH_MOTOR_CONTROLLER)
// uint16_t prbsStartingValue = 0xF6CDu;
// // uint16_t prbsStartingValue = 0xBDF1u;
// uint16_t prbsSequence[PRBS_PERIOD+1];
// uint16_t prbsBinarySequency[PRBS_PERIOD+1];
// int prbsCounter = 0;
// int lastCounterToPwm = prbsCounter;
// uint16_t prbsMinVal = 0;
// uint16_t prbsMaxVal = 0;
// #endif

// // Sprung mass MPU6050 interrupt
// // volatile bool sprungMassMpuInt = false;
// // void IRAM_ATTR sprungMassDmpDataReady() { sprungMassMpuInt = true; }

// // Encoders interrupt functions
// #ifdef USE_ENCODERS
// void IRAM_ATTR encoderActuatorPhaseA() { encoderActuator.phase_A_Interrupt(); }
// void IRAM_ATTR encoderActuatorPhaseB() { encoderActuator.phase_B_Interrupt(); }
// void IRAM_ATTR encoderActuatorPhaseZ() { encoderActuator.phase_Z_Interrupt(); }
// void IRAM_ATTR encoderArmPhaseA() { encoderArm.phase_A_Interrupt(); }
// void IRAM_ATTR encoderArmPhaseB() { encoderArm.phase_B_Interrupt(); }
// void IRAM_ATTR encoderArmPhaseZ() { encoderArm.phase_Z_Interrupt(); }
// #endif

// template< typename T> 
// T map( const T x, T in_min, T in_max, T out_min, T out_max ) 
// { 
//   if(x < in_min) return out_min;
//   if(x > in_max) return out_max;
//   return ((x - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min;
// }

// float getDisplacementByAngle(float angle) {
//   return 73.7078 * angle*pow(angle, 2)  + 219.923 * angle;
// }

// void printSensorsData(void* parameters) {
//   for(;;){ // infinite loop
//     // Serial.print("Data");
//     #ifdef USE_ENCODERS
//     // Serial.print(String(map(prbsSequence[lastCounterToPwm], prbsMinVal, prbsMaxVal, -1024, 1024)) + ",");                  // SL PWM output
//     // Serial.print(controllerEnabled > 0? 100:0);
//     // Serial.print(",");
//     Serial.print(singleLinkServo.getPwmOutput());                  // SL PWM output
//     Serial.print(",");
//     Serial.print(singleLinkServo.getTarget());                     // controller - U (SL speed)
//     Serial.print(",");
//     // Serial.print(encoderActuator.getAngularSpeedInRadCorrection());// motor speed
//     // Serial.print(",");
//     Serial.print(encoderActuator.getPulsesDegresWithCorrection());    // SL position (ok)
//     // Serial.print(",");
//     Serial.print(encoderActuator.getAngularSpeedInRadCorrection()); // SL speed
//     Serial.print(",");
//     // Serial.print(String(encoderArm.getPulsesRadWithCorrection(), 5));         // ARM position
//     // Serial.print(",");
//     // Serial.print(String(getDisplacementByAngle(encoderArm.getPulsesRadWithCorrection()), 5));     // ARM position
//     // Serial.print(",");
//     // Serial.print(String(encoderArm.getAngularSpeedInRad(), 5));               // ARM speed
//     // Serial.print(",");
//     // Serial.print(String(damperSensor.getDeflection(), 5));                    // Deflection
//     // Serial.print(",");
//     // Serial.print(String(damperSensor.getDeflectionSpeed(), 5));               // Deflection speed
//     // Serial.print(",");
//     // Serial.print(String(damperSensor.getLinearActuatorDeflection(), 5));      // Zlin position
//     // Serial.print(",");
//     // Serial.print(String(damperSensor.getLinearActuatorDeflectionSpeed(), 5)); // Zlin speed
//     // Serial.print(",");
//     // Serial.print(String(unsprungMassSensor.getUnsprungMassAceleration(), 5)); // Unsprung acceleration
//     #endif
//     #ifdef USE_MPU6050
//     // Serial.print(",");
//     // Serial.print(sprungMassMpu.getAccX()*9.81); // Sprung acceleration x m/s²
//     // Serial.print(",");
//     // Serial.print(sprungMassMpu.getAccY()*9.81); // Sprung acceleration y m/s²
//     // Serial.print(",");
//     // Serial.print(sprungMassMpu.getAccZ()*9.81);  // Sprung acceleration z m/s²
//     #endif

//     Serial.print("\n");
//     #if defined(IDENTIFY_MOTOR) && !defined(IDENTIFY_WITH_MOTOR_CONTROLLER)
//     vTaskDelay(SAMPLING_TIME_SERVO / portTICK_PERIOD_MS);
//     #endif
//     #ifndef IDENTIFY_MOTOR
//     vTaskDelay(SENSOR_UPDATE_TIME / portTICK_PERIOD_MS);
//     #endif
//   }
// }

// void updateSensorsData(void* parameters) {
//   for(;;){ // infinite loop
//     #ifdef USE_MPU6050
//     sprungMassMpu.update();
//     #endif
//     #ifdef USE_ENCODERS
//     encoderActuator.updateSpeed();
//     encoderArm.updateSpeed();
//     damperSensor.update();
//     unsprungMassSensor.update();
//     #endif

//     vTaskDelay(SENSOR_UPDATE_TIME / portTICK_PERIOD_MS);
//   }
// }

// void controlServo(void* parameters) {
//   for(;;){ // infinite loop
//     singleLinkServo.update(encoderActuator.getPulsesDegresWithCorrection(), &kalmanFilter);
//     vTaskDelay(SAMPLING_TIME_SERVO / portTICK_PERIOD_MS);
//   }
// }

// void controlAll(void* parameters) {
//   for(;;){ // infinite loop

//     if(controllerEnabled > 0) {
//       singleLinkServo.enableServo();
//       float currentRead = sprungMassMpu.getAccX() * 9.81; // m/s²

//       Matrix<ControllerGeral::systemOrderGeral,1> states;
//       states = kalmanFilterGeral.kalman(encoderActuator.getPulsesDegresWithCorrection()-90, currentRead);

//       // states(0, 0) = currentRead;
      
//       float targetPos = geralController.controlLaw(states);
//       singleLinkServo.setTarget(targetPos+90);
//     }
//     else {
//       singleLinkServo.setTarget(90);
//       // singleLinkServo.disableServo();
//     }

//     vTaskDelay(SAMPLING_TIME / portTICK_PERIOD_MS);
//   }
// }

// void runIdentification(void* parameters) {
//   for(;;){ // infinite loop
//     #ifdef IDENTIFY_MOTOR
//     float pwmToApply = 0;

//     if(prbsCounter <= PRBS_PERIOD) {
//       pwmToApply = map(prbsSequence[prbsCounter], prbsMinVal, prbsMaxVal, -singleLinkServo.getMaxPWM(), singleLinkServo.getMaxPWM());

//       if(prbsBinarySequency[prbsCounter] == 0)
//         pwmToApply = 1023;
//       else
//         pwmToApply = -1023;
      
//       lastCounterToPwm = prbsCounter;
//       prbsCounter++;
//     }
//     else if (prbsCounter == PRBS_PERIOD+1) {
//       Serial.println("End Of Motor Identification");
//       lastCounterToPwm = prbsCounter;
//       prbsCounter++;
//     }

//     // pwmToApply = 300;
//     singleLinkServo.applyPwmToMotor(pwmToApply);

//     vTaskDelay(SAMPLING_TIME_SERVO * PRBS_HOLDING_FACTOR_MOTOR / portTICK_PERIOD_MS);
//     #endif

//     #ifdef IDENTIFY_WITH_MOTOR_CONTROLLER
//     float angleToApply = 0;

//     if(prbsCounter <= PRBS_PERIOD) {
//       // Serial.println("pwm to apply: " + String(prbsSequence[prbsCounter]));
//       angleToApply = map<float>(prbsSequence[prbsCounter], prbsMinVal, prbsMaxVal, 90-60, 90+60);

//       // if(prbsBinarySequency[prbsCounter] == 0)
//       //   angleToApply = 80;
//       // else
//       //   angleToApply = 100;
      
//       lastCounterToPwm = prbsCounter;
//       prbsCounter++;
//     }
//     else if (prbsCounter == PRBS_PERIOD+1) {
//       Serial.println("End Of Motor Identification with controller");
//       angleToApply = 0;
//       lastCounterToPwm = prbsCounter;
//       prbsCounter++;
//     }

//     // angleToApply = 111;
//     singleLinkServo.setTarget(angleToApply);

//     vTaskDelay(SAMPLING_TIME * PRBS_HOLDING_FACTOR / portTICK_PERIOD_MS);
//     #endif
//   }
// }

// #ifdef USE_ENCODERS
// void encodersSetup() {
//   // Encoders max value setup
//   // taking into account the planetary gear box reduction wich is 29 + 1 (for margin)
//   encoderActuator.setMaxAngle(720*600);
//   encoderActuator.setCorrectionFactor(PLANETARY_REDUCTION);

//   // Encoders interrupt setup
//   pinMode(encoderActuator.pA(), INPUT_PULLDOWN);
//   pinMode(encoderActuator.pB(), INPUT_PULLDOWN);
//   pinMode(encoderActuator.pZ(), INPUT_PULLDOWN);
//   pinMode(encoderArm.pA(), INPUT_PULLDOWN);
//   pinMode(encoderArm.pB(), INPUT_PULLDOWN);
//   pinMode(encoderArm.pZ(), INPUT_PULLDOWN);
//   attachInterrupt(digitalPinToInterrupt(encoderActuator.pA()), encoderActuatorPhaseA, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(encoderActuator.pB()), encoderActuatorPhaseB, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(encoderActuator.pZ()), encoderActuatorPhaseZ, RISING);
//   attachInterrupt(digitalPinToInterrupt(encoderArm.pA()), encoderArmPhaseA, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(encoderArm.pB()), encoderArmPhaseB, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(encoderArm.pZ()), encoderArmPhaseZ, RISING);
// }
// #endif

// void setup() {
//   // Serial printer
//   Serial.begin(500000);

//   // Servo motor controller gains - esse ta top - alpha = 0.99 (0-180deg = 360ms)
//   // singleLinkGains(0, 0) = -83.872031585166425;
//   // singleLinkGains(1, 0) = -82.805221667698390;
//   // singleLinkGains(2, 0) = 0.411077661890745; 

//   // Servo - testar - alpha = 0.99 + modelo levemente diferente
//   // singleLinkGains(0, 0) = -19.333238951771271;
//   // singleLinkGains(1, 0) = -19.097123413587646;
//   // singleLinkGains(2, 0) = 0.095118003842203;

//   // testar se funciona certinho desse jeito
//   // singleLinkGains = {
//   //   -19.333238951771271,
//   //   -19.097123413587646,
//   //     0.095118003842203
//   // };
  
//   // Servo motor controller gains - esse ta top - alpha = 0.989 (0-180deg = 340ms)
//   // singleLinkGains(0, 0) = -1.322953938521483e2;
//   // singleLinkGains(1, 0) = -1.304921329090377e2;
//   // singleLinkGains(2, 0) =  0.006950642400832e2;

//   // Servo motor controller gains - esse ta + rapido + com sobressinal - alpha = 0.985 (0-180deg = 600ms) - causado pelo sobressinal
//   // singleLinkGains(0, 0) = ;
//   // singleLinkGains(1, 0) = ;
//   // singleLinkGains(2, 0) =  ;

//   // ** controlador em uso
//   singleLinkGains = {
//     -2.912650021672834e2,
//     -2.863040109125190e2,
//     0.019070245854254e2
//   };

//   // teste para deixar mais rapido
//   // singleLinkGains = {
//   //            -306.387144449756,
//   //        -270.999646995752,
//   //        -235.805139187033,
//   //         2.33929729515702
//   // };

//   // (30-150)-2ident-crtl-novo-40ms-2-susp-05-02-2024 17-45.csv - 2 entradas
//   geralControllerGains = {
//          0.887111297577267,
//           5.99808305895598,
//           9.17460108172961,
//           2.12318252499283,
//          -9.19686380271413,
//          -4.99135217283706
//   };

//   singleLinkServo.setMaxPwm(singleLinkServo.getMaxPWM() / PWM_DIVISION_FACTOR);
//   singleLinkServo.setTrackingGains(singleLinkGains);
//   singleLinkServo.setTarget(90);

//   geralController.setRegulationGains(geralControllerGains);
//   geralController.setSaturationLimits(-60, 60);

//   // singleLinkServo.disableServo();
//   // singleLinkServo.brake();

//   #ifdef USE_ENCODERS
//   encodersSetup();
//   #endif

//   #if defined(IDENTIFY_MOTOR) || defined(IDENTIFY_WITH_MOTOR_CONTROLLER)
//   Serial.println("Motor Identification RUN");
//   prbs(prbsStartingValue, PRBS_PERIOD, prbsSequence, prbsBinarySequency);
//   prbsMaxVal = prbsSequence[0];
//   prbsMinVal = prbsSequence[0];

//   for (int i = 0; i < (sizeof(prbsSequence) / sizeof(prbsSequence[0])); i++) {
//       if (prbsSequence[i] > prbsMaxVal) {
//          prbsMaxVal = prbsSequence[i];
//       }
//       if (prbsSequence[i] < prbsMinVal) {
//          prbsMinVal = prbsSequence[i];
//       }
//    }
//   #endif

//   #ifdef USE_MPU6050
//   // MPU6050 setup
//   Wire.begin();
//   byte status = sprungMassMpu.begin(1, 0);
//   Serial.print(F("MPU6050 status: "));
//   Serial.println(status);
//   Serial.println(F("Calculating offsets, do not move MPU6050"));
//   delay(1000);
//   sprungMassMpu.calcOffsets(true,true); // gyro and accelero
//   Serial.println("Done!\n");

//   // sprungMassMpu.Initialize();
//   // Serial.println("=====================================");
//   // Serial.println("Starting calibration...");
//   // sprungMassMpu.Calibrate();
//   // Serial.println("Calibration complete!");
//   // pinMode(PIN_SPRUNG_MASS_MPU6050_INT, INPUT);
//   // attachInterrupt(digitalPinToInterrupt(PIN_SPRUNG_MASS_MPU6050_INT), sprungMassDmpDataReady, RISING);
//   #endif

//   // #ifdef ENABLE_GERAL_CONTROLLER

//   // Zero routine for the second controller
//   singleLinkServo.setTarget(90);
//   int counter = 0;
//   while(true) {
//     Serial.println("zeroing servo in 90 deg");
//     singleLinkServo.update(encoderActuator.getPulsesDegresWithCorrection(), &kalmanFilter);

//     if(abs(singleLinkServo.getTarget() - encoderActuator.getPulsesDegresWithCorrection()) / abs(singleLinkServo.getTarget() + 0.0001) * 100 <= 5)
//       counter++;

//     if(counter >= 2000)
//       break;

//     delay(SAMPLING_TIME_SERVO);
//   }
//   delay(2000);
//   // #endif

//   // Tasks Creation
//   #ifdef PRINT_SENSOR_DATA
//   xTaskCreatePinnedToCore(
//     printSensorsData,
//     "Print",
//     5000,
//     NULL,
//     15,
//     NULL,
//     1
//   );
//   #endif

//   #ifdef UPDATE_SENSORS
//   xTaskCreatePinnedToCore(
//     updateSensorsData,
//     "Update all the sensors data",
//     5000,
//     NULL,
//     5,
//     NULL,
//     1
//   );
//   #endif

//   #if defined(IDENTIFY_MOTOR) || defined(IDENTIFY_WITH_MOTOR_CONTROLLER)
//   xTaskCreatePinnedToCore(
//     runIdentification,
//     "RunIdent",
//     3000,
//     NULL,
//     8,
//     NULL,
//     0
//   );
//   #endif

//   #if !defined(IDENTIFY_MOTOR) && defined(ENABLE_SERVO_CONTROLLER)
//   xTaskCreatePinnedToCore(
//     controlServo,
//     "UpdateServo",
//     3000,
//     NULL,
//     15,
//     NULL,
//     0
//   );
//   #endif

//   #if !defined(IDENTIFY_MOTOR) && !defined(IDENTIFY_WITH_MOTOR_CONTROLLER) && defined(ENABLE_GERAL_CONTROLLER)
//   xTaskCreatePinnedToCore(
//     controlAll,
//     "Update geral controller",
//     3000,
//     NULL,
//     10,
//     NULL,
//     0
//   );
//   #endif

//   #ifdef USE_ENCODERS
//   damperSensor.updateZero();
//   #endif


//   encoderArm.updateZero();

//   pinMode(PIN_LED, OUTPUT);
//   digitalWrite(PIN_LED, HIGH);
// }

// unsigned long int k = 0;

// void loop() {
//   #ifdef USE_MANUAL_SETPOINT
//   if (Serial.available() > 0) {
//     Serial.setTimeout(5);
//     String floatString = Serial.readString();
//     Serial.flush();
//     #ifndef MANUAL_PWM
//     float targetPosition = floatString.toFloat();

//     #ifdef ENABLE_OVERRIDE_CONTROLLER
//     controllerEnabled = targetPosition;
//     #endif
//     #ifndef ENABLE_OVERRIDE_CONTROLLER
//     singleLinkServo.setTarget(targetPosition);
//     #endif

//     #endif

//     #ifdef MANUAL_PWM
//     float targetPwm = floatString.toFloat();
//     // singleLinkServo.applyPwmToMotor(targetPwm);
//     #endif
//   }
//   #endif

//   #if defined(TEST_SERVO) && !defined(ENABLE_GERAL_CONTROLLER)

//   singleLinkServo.setTarget(90 + 60 * sin(2*PI*k*1/1000));
//   k++;
    
//   vTaskDelay(1 / portTICK_PERIOD_MS);
//   #endif
// }