// #include "I2Cdev.h"
// #include "MPU6050_6Axis_MotionApps20.h"

// #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//     #include "Wire.h"
// #endif

// MPU6050 mpu;

// #define OUTPUT_READABLE_YAWPITCHROLL // reading
// #define OUTPUT_READABLE_REALACCEL // reading

// #define INTERRUPT_PIN 34  // use pin 2 on Arduino Uno & most boards

// #define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
// bool blinkState = false;

// // MPU control/status vars
// bool dmpReady = false;  // set true if DMP init was successful
// uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
// uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
// uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
// uint16_t fifoCount;     // count of all bytes currently in FIFO
// uint8_t fifoBuffer[64]; // FIFO storage buffer

// // orientation/motion vars
// Quaternion q;           // [w, x, y, z]         quaternion container
// VectorInt16 aa;         // [x, y, z]            accel sensor measurements
// VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
// VectorFloat gravity;    // [x, y, z]            gravity vector
// float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
// void IRAM_ATTR dmpDataReady() {
//     mpuInterrupt = true;
// }

// // ================================================================
// // ===                      INITIAL SETUP                       ===
// // ================================================================

// void setup() {
//     // join I2C bus (I2Cdev library doesn't do this automatically)
//     #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//         Wire.begin();
//         Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
//     #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
//         Fastwire::setup(400, true);
//     #endif

//     Serial.begin(500000);

//     // initialize device
//     Serial.println(F("Initializing I2C devices..."));
//     mpu.initialize();
//     pinMode(INTERRUPT_PIN, INPUT);

//     // verify connection
//     Serial.println(F("Testing device connections..."));
//     Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

//     // wait for ready
//     // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//     // while (Serial.available() && Serial.read()); // empty buffer
//     // while (!Serial.available());                 // wait for data
//     // while (Serial.available() && Serial.read()); // empty buffer again

//     // load and configure the DMP
//     Serial.println(F("Initializing DMP..."));
//     devStatus = mpu.dmpInitialize();

//     // supply your own gyro offsets here, scaled for min sensitivity
//     mpu.setXAccelOffset(-529);
//     mpu.setYAccelOffset(1605);
//     mpu.setZAccelOffset(713);
//     mpu.setXGyroOffset(-43);
//     mpu.setYGyroOffset(6);
//     mpu.setZGyroOffset(44);
    

//     // make sure it worked (returns 0 if so)
//     if (devStatus == 0) {
//         // Calibration Time: generate offsets and calibrate our MPU6050
//         mpu.CalibrateAccel(6);
//         mpu.CalibrateGyro(6);
//         mpu.PrintActiveOffsets();
//         // turn on the DMP, now that it's ready
//         Serial.println(F("Enabling DMP..."));
//         mpu.setDMPEnabled(true);

//         // enable Arduino interrupt detection
//         Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
//         Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
//         Serial.println(F(")..."));
//         attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
//         mpuIntStatus = mpu.getIntStatus();

//         // set our DMP Ready flag so the main loop() function knows it's okay to use it
//         Serial.println(F("DMP ready! Waiting for first interrupt..."));
//         dmpReady = true;

//         // get expected DMP packet size for later comparison
//         packetSize = mpu.dmpGetFIFOPacketSize();
//     } else {
//         // ERROR!
//         // 1 = initial memory load failed
//         // 2 = DMP configuration updates failed
//         // (if it's going to break, usually the code will be 1)
//         Serial.print(F("DMP Initialization failed (code "));
//         Serial.print(devStatus);
//         Serial.println(F(")"));
//     }

//     // configure LED for output
//     pinMode(LED_PIN, OUTPUT);
// }

// void loop() {
//     // if programming failed, don't try to do anything
//     if (!dmpReady) return;
//     // read a packet from FIFO
//     if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

//         #ifdef OUTPUT_READABLE_YAWPITCHROLL
//             // display Euler angles in degrees
//             mpu.dmpGetQuaternion(&q, fifoBuffer);
//             mpu.dmpGetGravity(&gravity, &q);
//             mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//             Serial.print("ypr\t");
//             Serial.print(ypr[0] * 180/M_PI);
//             Serial.print("\t");
//             Serial.print(ypr[1] * 180/M_PI);
//             Serial.print("\t");
//             Serial.println(ypr[2] * 180/M_PI);
//         #endif

//         #ifdef OUTPUT_READABLE_REALACCEL
//             // display real acceleration, adjusted to remove gravity
//             mpu.dmpGetQuaternion(&q, fifoBuffer);
//             mpu.dmpGetAccel(&aa, fifoBuffer);
//             mpu.dmpGetGravity(&gravity, &q);
//             mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//             Serial.print("gravity\t");
//             Serial.print(gravity.x);
//             Serial.print("\t");
//             Serial.print(gravity.y);
//             Serial.print("\t");
//             Serial.print(gravity.z);
//             Serial.print("\t");
//             Serial.print("areal\t");
//             Serial.print(aaReal.x * 9.81 / 8192.0);
//             Serial.print("\t");
//             Serial.print(aaReal.y * 9.81 / 8192.0);
//             Serial.print("\t");
//             Serial.println(aaReal.z * 9.81 / 8192.0);
//         #endif

//         // blink LED to indicate activity
//         blinkState = !blinkState;
//         digitalWrite(LED_PIN, blinkState);
//     }
// }
