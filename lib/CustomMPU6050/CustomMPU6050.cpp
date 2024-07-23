#include "CustomMPU6050.h"

CustomMPU6050::CustomMPU6050(int16_t xAccOffset, int16_t yAccOffset, int16_t zAccOffset,
                             int16_t xGyroOffset, int16_t yGyroOffset, int16_t zGyroOffset) {
    xAccOffset = xAccOffset;
    yAccOffset = yAccOffset;
    zAccOffset = zAccOffset;
    xGyroOffset = xGyroOffset;
    yGyroOffset = yGyroOffset;
    zGyroOffset = zGyroOffset;
}

void CustomMPU6050::initialize() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // Serial.begin(115200);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(xAccOffset);
    mpu.setYAccelOffset(yAccOffset);
    mpu.setZAccelOffset(zAccOffset);
    mpu.setXGyroOffset(xGyroOffset);
    mpu.setYGyroOffset(yGyroOffset);
    mpu.setZGyroOffset(zGyroOffset);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(100);
        mpu.CalibrateGyro(100);
        mpu.PrintActiveOffsets();
        
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

bool CustomMPU6050::update() {
    if(!dmpReady) { return false; }
    
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        // Serial.print("ypr\t");
        // Serial.print(ypr[0] * 180/M_PI);
        // Serial.print("\t");
        // Serial.print(ypr[1] * 180/M_PI);
        // Serial.print("\t");
        // Serial.println(ypr[2] * 180/M_PI);

        // display real acceleration, adjusted to remove gravity
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        
        // Serial.print("gravity\t");
        // Serial.print(gravity.x);
        // Serial.print("\t");
        // Serial.print(gravity.y);
        // Serial.print("\t");
        // Serial.print(gravity.z);
        // Serial.print("\t");
        // Serial.print("areal\t");
        // Serial.print(aaReal.x * 9.81 / 8192.0);
        // Serial.print("\t");
        // Serial.print(aaReal.y * 9.81 / 8192.0);
        // Serial.print("\t");
        // Serial.println(aaReal.z * 9.81 / 8192.0);

        return true;
    }

    return false;
}