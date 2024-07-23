#ifndef CUSTOM_MPU6050
#define CUSTOM_MPU6050

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

class CustomMPU6050 {
    private:
        MPU6050 mpu;
        bool dmpReady = false;  // set true if DMP init was successful
        uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
        uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
        uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
        uint16_t fifoCount;     // count of all bytes currently in FIFO
        uint8_t fifoBuffer[64]; // FIFO storage buffer

        Quaternion q;           // [w, x, y, z]         quaternion container
        VectorInt16 aa;         // [x, y, z]            accel sensor measurements
        VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
        VectorFloat gravity;    // [x, y, z]            gravity vector
        float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

        int16_t xAccOffset, yAccOffset, zAccOffset;
        int16_t xGyroOffset, yGyroOffset, zGyroOffset;

    public:
        CustomMPU6050(int16_t xAccOffset, int16_t yAccOffset, int16_t zAccOffset,
                      int16_t xGyroOffset, int16_t yGyroOffset, int16_t zGyroOffset);

        void initialize();
        bool update();

        float xAcc() { return this->aaReal.x * 9.81 / 8192.0; }
        float yAcc() { return this->aaReal.y * 9.81 / 8192.0; }
        float zAcc() { return this->aaReal.z * 9.81 / 8192.0; }

        float xGyro() { return this->ypr[0] * 180/M_PI; }
        float yGyro() { return this->ypr[1] * 180/M_PI; }
        float zGyro() { return this->ypr[2] * 180/M_PI; }
};

#endif