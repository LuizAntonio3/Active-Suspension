#include <Arduino.h>
#include <MPU6050_light.h>
#include <BasicLinearAlgebra.h>
// #include <TinyMPU6050.h>
// #include <filters.h>

// Constants related imports
#include <pins.h>
#include <system.h>
#include <geralSystem.h>

// Custom Librarys imports
#include <ThreePhaseEncoder.h>
// #include <CustomMPU6050.h>
#include <DamperSensor.h>
#include <UnsprungMassSensor.h>
#include <CustomServo.h>
#include <prbs.h>
#include <KalmanFilter.h>
#include <KalmanFilterGeral.h>
#include <ControllerGeral.h>

using namespace BLA;

// *** TASKS RELATED CONSTANTS ***
#define USE_MPU6050
#define USE_ENCODERS
#define PRINT_SENSOR_DATA
#define UPDATE_SENSORS
#define ENABLE_SERVO_CONTROLLER
// #define TEST_SERVO
#define ENABLE_GERAL_CONTROLLER
// #define USE_MANUAL_SETPOINT
// #define ENABLE_OVERRIDE_CONTROLLER
// #define MANUAL_PWM
// #define IDENTIFY_MOTOR
// #define IDENTIFY_WITH_MOTOR_CONTROLLER

#define SENSOR_UPDATE_TIME 14 // ms
#define SAMPLING_TIME_SERVO 1 // us // ms - for the controller that works but is slow
#define SAMPLING_TIME 19 // ms
#define PRBS_HOLDING_FACTOR_MOTOR 50
#define PRBS_HOLDING_FACTOR 30 // 80 // 7 para identificaçao geral usada
#define PRBS_PERIOD 12000 // <- update this value
#define PLANETARY_REDUCTION 26.9
#define PWM_DIVISION_FACTOR 1.255 // 1.2629629 // 1.27875


// LOGIC RELATED VARIABLES
unsigned long int k = 0;
float controllerEnabled = 1;

// ultrassonic related constants
#define ULTRA_TIMEOUT 5*1000 // us
float distanceM;
// float lastDistances[5] = {0,0,0,0,0};

#ifdef USE_ENCODERS
ThreePhaseEncoder encoderActuator(PIN_ACTUATOR_ENCODER_A, PIN_ACTUATOR_ENCODER_B, PIN_ACTUATOR_ENCODER_Z);
ThreePhaseEncoder encoderArm(PIN_ARM_ENCODER_A, PIN_ARM_ENCODER_B, PIN_ARM_ENCODER_Z);
DamperSensor damperSensor(&encoderArm, &encoderActuator);
UnsprungMassSensor unsprungMassSensor(&encoderArm);
#endif

#ifdef USE_MPU6050
MPU6050 sprungMassMpu(Wire);
#endif

CustomServo singleLinkServo(PIN_ENABLE_SINGLE_LINK, PIN_INT1_SINGLE_LINK_PWM, PIN_INT2_SINGLE_LINK_PWM, 0, 1, 30000, 10);
KalmanFilter kalmanFilter(sys.A, sys.B, sys.C, 1e-5, 1e-6); // 1.8 e 1

ControllerGeral geralController;
KalmanFilterGeral kalmanFilterGeral(sysGeral.A, sysGeral.B, sysGeral.C, 1.5e-5, 2.1e-5);

Matrix<systemOrder+1,1> singleLinkGains;
Matrix<1, systemOrderGeral> geralControllerGains;

// IIR::ORDER  order  = IIR::ORDER::OD3;
// Filter lowPassFilter(20, 19*1e-3, order);

// Identification related variables
#if defined(IDENTIFY_MOTOR) || defined(IDENTIFY_WITH_MOTOR_CONTROLLER)
uint16_t prbsStartingValue = 0xF6CDu;
// uint16_t prbsStartingValue = 0xBDF1u;
uint16_t prbsSequence[PRBS_PERIOD+1];
uint16_t prbsBinarySequency[PRBS_PERIOD+1];
int prbsCounter = 0;
int lastCounterToPwm = prbsCounter;
uint16_t prbsMinVal = 0;
uint16_t prbsMaxVal = 0;
#endif

// Encoders interrupt functions
#ifdef USE_ENCODERS
void IRAM_ATTR encoderActuatorPhaseA() { encoderActuator.phase_A_Interrupt(); }
void IRAM_ATTR encoderActuatorPhaseB() { encoderActuator.phase_B_Interrupt(); }
void IRAM_ATTR encoderActuatorPhaseZ() { encoderActuator.phase_Z_Interrupt(); }
void IRAM_ATTR encoderArmPhaseA() { encoderArm.phase_A_Interrupt(); }
void IRAM_ATTR encoderArmPhaseB() { encoderArm.phase_B_Interrupt(); }
void IRAM_ATTR encoderArmPhaseZ() { encoderArm.phase_Z_Interrupt(); }
#endif

template< typename T> 
T map( const T x, T in_min, T in_max, T out_min, T out_max ) 
{ 
  if(x < in_min) return out_min;
  if(x > in_max) return out_max;
  return ((x - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min;
}

float getDisplacementByAngle(float angle) {
  return 73.7078 * angle*pow(angle, 2)  + 219.923 * angle;
}

void readUltrassonicSensor() {

  // portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;
  // taskENTER_CRITICAL(&myMutex);
  digitalWrite(PIN_TRIGGER, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIGGER, LOW);
  
  unsigned long duration = pulseIn(PIN_ECHO, HIGH, ULTRA_TIMEOUT);
  // taskEXIT_CRITICAL(&myMutex);

  // float sum = 0;
  // for (int i = 0; i < 5-1; i++) {
  //   lastDistances[i] = lastDistances[i+1];
  //   sum += lastDistances[i];
  // }

  // lastDistances[4] = duration * 0.0344 / 2 + 1.5; // duration * 1e-6 * 343 / 2.0;
  // lastDistances[4] = static_cast<float>(static_cast<int>((duration * 0.0344 / 2 + 1.5) * 10.)) / 10.;
  // sum += lastDistances[4];

  distanceM = duration * 0.0344 / 2 + 1.5; // static_cast<float>(static_cast<int>((duration * 0.0344 / 2 + 1.5) * 10.)) / 10.;
}

void printSensorsData(void* parameters) {
  for(;;){ // infinite loop
    // Serial.print("Data");
    #ifdef USE_ENCODERS
    // Serial.print(String(map(prbsSequence[lastCounterToPwm], prbsMinVal, prbsMaxVal, -1024, 1024)) + ",");                  // SL PWM output
    Serial.print(controllerEnabled > 0? 100:0);
    Serial.print(",");
    Serial.print(singleLinkServo.getPwmOutput());                  // SL PWM output
    Serial.print(",");
    Serial.print(singleLinkServo.getTarget());                     // controller - U (SL speed)
    Serial.print(",");
    // Serial.print(encoderActuator.getAngularSpeedInRadCorrection());// motor speed
    // Serial.print(",");
    Serial.print(encoderActuator.getPulsesDegresWithCorrection());    // SL position (ok)
    Serial.print(",");
    Serial.print(encoderActuator.getAngularSpeedInRadCorrection()); // SL speed
    Serial.print(",");
    Serial.print(String(encoderArm.getPulsesDegresWithCorrection(), 5));         // ARM position
    Serial.print(",");
    Serial.print(String(distanceM, 5)); // Distance
    // Serial.print(",");
    // Serial.print(String(getDisplacementByAngle(encoderArm.getPulsesRadWithCorrection()), 5));     // ARM position
    // Serial.print(",");
    // Serial.print(String(encoderArm.getAngularSpeedInRad(), 5));               // ARM speed
    // Serial.print(",");
    // Serial.print(String(damperSensor.getDeflection(), 5));                    // Deflection
    // Serial.print(",");
    // Serial.print(String(damperSensor.getDeflectionSpeed(), 5));               // Deflection speed
    // Serial.print(",");
    // Serial.print(String(damperSensor.getLinearActuatorDeflection(), 5));      // Zlin position
    // Serial.print(",");
    // Serial.print(String(damperSensor.getLinearActuatorDeflectionSpeed(), 5)); // Zlin speed
    // Serial.print(",");
    // Serial.print(String(unsprungMassSensor.getUnsprungMassAceleration(), 5)); // Unsprung acceleration
    #endif
    #ifdef USE_MPU6050
    Serial.print(",");
    Serial.print(sprungMassMpu.getAccX()*9.81); // Sprung acceleration x m/s²
    // Serial.print(",");
    // Serial.print(sprungMassMpu.getAccY()*9.81); // Sprung acceleration y m/s²
    // Serial.print(",");
    // Serial.print(sprungMassMpu.getAccZ()*9.81);  // Sprung acceleration z m/s²
    #endif

    Serial.print("\n");

    // vTaskDelay(SAMPLING_TIME_SERVO / portTICK_PERIOD_MS);

    #if defined(IDENTIFY_MOTOR) && !defined(IDENTIFY_WITH_MOTOR_CONTROLLER)
    vTaskDelay(SAMPLING_TIME_SERVO / portTICK_PERIOD_MS);
    #endif
    #ifndef IDENTIFY_MOTOR
    vTaskDelay(SENSOR_UPDATE_TIME / portTICK_PERIOD_MS);
    #endif
  }
}

void updateSensorsData(void* parameters) {
  for(;;){ // infinite loop
    #ifdef USE_MPU6050
    sprungMassMpu.update();
    #endif
    #ifdef USE_ENCODERS
    encoderActuator.updateSpeed();
    encoderArm.updateSpeed();
    // damperSensor.update();
    // unsprungMassSensor.update();
    #endif
    readUltrassonicSensor();

    vTaskDelay(SENSOR_UPDATE_TIME / portTICK_PERIOD_MS);
  }
}

void controlServo(void* parameters) {
  for(;;){ // infinite loop
    portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;
    taskENTER_CRITICAL(&myMutex);
    singleLinkServo.update(encoderActuator.getPulsesDegresWithCorrection(), &kalmanFilter);
    taskEXIT_CRITICAL(&myMutex);
    vTaskDelay(SAMPLING_TIME_SERVO / portTICK_PERIOD_MS);
  }
}

void controlAll(void* parameters) {
  for(;;){ // infinite loop

    if(controllerEnabled > 0) {
      if (millis() > 60000)
        controllerEnabled = 0;

      singleLinkServo.enableServo();

      Matrix<ControllerGeral::systemOrderGeral,1> states;
      
      portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;
      taskENTER_CRITICAL(&myMutex);
      states = kalmanFilterGeral.kalman(singleLinkServo.getTarget()-90, sprungMassMpu.getAccX() * 9.81);

      // states(0, 0) = sprungMassMpu.getAccX() * 9.81;

      float targetPos = geralController.controlLaw(states);
      taskEXIT_CRITICAL(&myMutex);

      // *** Kalman Filter Printing ***
      // Serial.print(sprungMassMpu.getAccX() * 9.81);
      // Serial.print(", ");
      // Serial.print(states(0, 0));
      // Serial.print(", ");
      // Serial.print(states(1, 0));
      // Serial.print(", ");
      // Serial.print(states(2, 0));
      // Serial.print(", ");
      // Serial.print(states(3, 0));
      // Serial.print(", ");
      // Serial.println(states(4, 0));

      singleLinkServo.setTarget(targetPos+90);
    }
    else {
      singleLinkServo.setTarget(90);
      // singleLinkServo.disableServo();
    }

    vTaskDelay(SAMPLING_TIME / portTICK_PERIOD_MS);
  }
}

void runIdentification(void* parameters) {
  for(;;){ // infinite loop
    #ifdef IDENTIFY_MOTOR
    float pwmToApply = 0;

    if(prbsCounter <= PRBS_PERIOD) {
      pwmToApply = map(prbsSequence[prbsCounter], prbsMinVal, prbsMaxVal, -singleLinkServo.getMaxPWM(), singleLinkServo.getMaxPWM());
      // pwmToApply = map(prbsSequence[prbsCounter], prbsMinVal, prbsMaxVal, -750, 750);

      // if(prbsBinarySequency[prbsCounter] == 0)
      //   pwmToApply = 600;
      // else
      //   pwmToApply = -600;
      
      lastCounterToPwm = prbsCounter;
      prbsCounter++;
    }
    else if (prbsCounter == PRBS_PERIOD+1) {
      Serial.println("End Of Motor Identification");
      lastCounterToPwm = prbsCounter;
      prbsCounter++;
    }

    // pwmToApply = 300;
    singleLinkServo.applyPwmToMotor(pwmToApply);

    vTaskDelay(SAMPLING_TIME_SERVO * PRBS_HOLDING_FACTOR_MOTOR / portTICK_PERIOD_MS);
    // delayMicroseconds(SAMPLING_TIME_SERVO * PRBS_HOLDING_FACTOR_MOTOR);
    #endif

    #ifdef IDENTIFY_WITH_MOTOR_CONTROLLER
    float angleToApply = 0;

    if(prbsCounter <= PRBS_PERIOD) {
      // Serial.println("pwm to apply: " + String(prbsSequence[prbsCounter]));
      angleToApply = map<float>(prbsSequence[prbsCounter], prbsMinVal, prbsMaxVal, 90-65, 90+65);

      // if(prbsBinarySequency[prbsCounter] == 0)
      //   angleToApply = 80;
      // else
      //   angleToApply = 100;
      
      lastCounterToPwm = prbsCounter;
      prbsCounter++;
    }
    else if (prbsCounter == PRBS_PERIOD+1) {
      Serial.println("End Of Motor Identification with controller");
      angleToApply = 0;
      lastCounterToPwm = prbsCounter;
      prbsCounter++;
    }

    // angleToApply = 111;
    singleLinkServo.setTarget(angleToApply);

    vTaskDelay(SAMPLING_TIME * PRBS_HOLDING_FACTOR / portTICK_PERIOD_MS);
    #endif
  }
}

#ifdef USE_ENCODERS
void encodersSetup() {
  // Encoders max value setup
  // taking into account the planetary gear box reduction wich is 29 + 1 (for margin)
  encoderActuator.setMaxAngle(720*600);
  encoderActuator.setCorrectionFactor(PLANETARY_REDUCTION);

  // Encoders interrupt setup
  pinMode(encoderActuator.pA(), INPUT_PULLDOWN);
  pinMode(encoderActuator.pB(), INPUT_PULLDOWN);
  pinMode(encoderActuator.pZ(), INPUT_PULLDOWN);
  pinMode(encoderArm.pA(), INPUT_PULLDOWN);
  pinMode(encoderArm.pB(), INPUT_PULLDOWN);
  pinMode(encoderArm.pZ(), INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(encoderActuator.pA()), encoderActuatorPhaseA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderActuator.pB()), encoderActuatorPhaseB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderActuator.pZ()), encoderActuatorPhaseZ, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderArm.pA()), encoderArmPhaseA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderArm.pB()), encoderArmPhaseB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderArm.pZ()), encoderArmPhaseZ, RISING);
}
#endif

void setup() {
  // Serial printer
  Serial.begin(500000);

  pinMode(PIN_TRIGGER, OUTPUT); // Sets the trigPin as an Output
  pinMode(PIN_ECHO, INPUT);

  digitalWrite(PIN_TRIGGER, LOW);
  delayMicroseconds(2);

  // ** controlador h infinito lento modelo 1
  // singleLinkGains = {
  //   -2.912650021672834e2,
  //   -2.863040109125190e2,
  //   0.019070245854254e2
  // };

  // *** controlador h infinito em uso modelo 2 (funcional) - alpha = .989 (eu acho que foi esse) - sem saturaçao + realimentaçao ponderada do sinal de controle
  // singleLinkGains = {
  //   -13820.4336200527,
  //   -8844.78794378311,
  //   102.607461999986
  // };

  // MODELO MOLA FORTE - teste - controlador h2 - alpha = 0.91 - rapido - TESTES REALIZADOS COM ESSE
  // singleLinkGains = {
  //        -15241.2813862522,
  //        -13304.1346732993,
  //         117.863149281938
  // };

  // MODELO MOLA FORTE - h2 - lmis-separadas
  singleLinkGains = { // - 217.465227308753
          -12980.832502934,
      -10945.7475046691,
          148.194277791022
  };

  // MODELO MOLA MEDIA - teste - controlador h2 - alpha = 0.91 - rapido - verificando
  // singleLinkGains = {
  //        -14073.6226797766,
  //         -11692.074575118,
  //         163.915886595754
  // };

  // // *** controlador h2 em uso (funcional) - alpha = .979
  // geralControllerGains = {
  //   -1.65407038666072,
  //   -2.84757445409209,
  //   0.479010693411724,
  //   0.471766362424482,
  //   0.461119314914191
  // };

  // teste - controlador h2 - alpha = .95 - TESTES REALIZADOS COM ESSE
  // geralControllerGains = {
  //   -1.34551114353039,
  //   -2.21496785526688,
  //    1.07680486946358,
  //    1.01076484738942,
  //   0.934436035316188
  // };

  // teste controlador h2 - alpha = .95 - lmis isoladas
  geralControllerGains = {
         -1.34216705215132,
          -2.2099873300152,
          1.06310444625557,
         0.995545083167062,
          0.92790872284339
  };

  // modelo mola media
  // geralControllerGains = {
  //         26.0170447634785,
  //         18.0230860775384,
  //         44.6840189241488,
  //         44.4548384068325,
  //         44.2081951111191
  // };

  singleLinkServo.setMaxPwm(singleLinkServo.getMaxPWM() / PWM_DIVISION_FACTOR);
  singleLinkServo.setTrackingGains(singleLinkGains);
  singleLinkServo.setTarget(90);

  geralController.setRegulationGains(geralControllerGains);
  geralController.setSaturationLimits(-60, 60);

  #ifdef USE_ENCODERS
  encodersSetup();
  #endif

  #if defined(IDENTIFY_MOTOR) || defined(IDENTIFY_WITH_MOTOR_CONTROLLER)
  Serial.println("Motor Identification RUN");
  prbs(prbsStartingValue, PRBS_PERIOD, prbsSequence, prbsBinarySequency);
  prbsMaxVal = prbsSequence[0];
  prbsMinVal = prbsSequence[0];

  for (int i = 0; i < (sizeof(prbsSequence) / sizeof(prbsSequence[0])); i++) {
      if (prbsSequence[i] > prbsMaxVal) {
         prbsMaxVal = prbsSequence[i];
      }
      if (prbsSequence[i] < prbsMinVal) {
         prbsMinVal = prbsSequence[i];
      }
   }
  #endif

  // MPU6050 setup
  #ifdef USE_MPU6050
  Wire.begin();
  byte status = sprungMassMpu.begin(1, 0);
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  sprungMassMpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
  #endif

  // Zero routine for the second controller
  #ifdef ENABLE_SERVO_CONTROLLER
  singleLinkServo.setTarget(90);
  int counter = 0;
  while(true) {
    Serial.println("zeroing servo in 90 deg");
    singleLinkServo.update(encoderActuator.getPulsesDegresWithCorrection(), &kalmanFilter);

    if(abs(singleLinkServo.getTarget() - encoderActuator.getPulsesDegresWithCorrection()) / abs(singleLinkServo.getTarget() + 0.0001) * 100 <= 2)
      counter++;

    if(counter >= 1000)
      break;

    delay(SAMPLING_TIME_SERVO);
  }
  #endif

  // Tasks Creation
  #ifdef PRINT_SENSOR_DATA
  xTaskCreatePinnedToCore(
    printSensorsData,
    "Print",
    5000,
    NULL,
    15,
    NULL,
    1
  );
  #endif

  #ifdef UPDATE_SENSORS
  xTaskCreatePinnedToCore(
    updateSensorsData,
    "Update all the sensors data",
    5000,
    NULL,
    5,
    NULL,
    1
  );
  #endif

  #if defined(IDENTIFY_MOTOR) || defined(IDENTIFY_WITH_MOTOR_CONTROLLER)
  xTaskCreatePinnedToCore(
    runIdentification,
    "RunIdent",
    3000,
    NULL,
    8,
    NULL,
    0
  );
  #endif

  #if !defined(IDENTIFY_MOTOR) && defined(ENABLE_SERVO_CONTROLLER)
  xTaskCreatePinnedToCore(
    controlServo,
    "UpdateServo",
    3000,
    NULL,
    15,
    NULL,
    0
  );
  #endif

  #if defined(ENABLE_GERAL_CONTROLLER) // && !defined(IDENTIFY_MOTOR) && !defined(IDENTIFY_WITH_MOTOR_CONTROLLER) && 
  xTaskCreatePinnedToCore(
    controlAll,
    "Update geral controller",
    6000,
    NULL,
    15,
    NULL,
    0
  );
  #endif

  #ifdef USE_ENCODERS
  damperSensor.updateZero();
  #endif

  encoderArm.updateZero();

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
}

void loop() {

  #ifdef USE_MANUAL_SETPOINT
  if (Serial.available() > 0) {
    Serial.setTimeout(5);
    String floatString = Serial.readString();
    Serial.flush();
    #ifndef MANUAL_PWM
    float targetPosition = floatString.toFloat();

    #ifdef ENABLE_OVERRIDE_CONTROLLER
    controllerEnabled = targetPosition;
    #endif
    #ifndef ENABLE_OVERRIDE_CONTROLLER
    singleLinkServo.setTarget(targetPosition);
    #endif

    #endif

    #ifdef MANUAL_PWM
    float targetPwm = floatString.toFloat();
    // singleLinkServo.applyPwmToMotor(targetPwm);
    #endif
  }
  #endif

  // sinosoidal input for servo controller test
  #if defined(TEST_SERVO) && !defined(ENABLE_GERAL_CONTROLLER)

  singleLinkServo.setTarget(90 + 60 * sin(2*PI*k*1/1000));
  k++;
    
  vTaskDelay(1 / portTICK_PERIOD_MS);
  #endif
}