#include <Wire.h>

#define MPU6050_ADDR 0x68 // MPU6050 device address
#define PWR_MGMT_1   0x6B // Power management register
#define ACCEL_XOUT_H 0x3B // Accelerometer X-axis high byte
#define GYRO_XOUT_H  0x43 // Gyroscope X-axis high byte

class MPU6050 {
private:
    uint8_t sda;
    uint8_t scl;
public:
    MPU6050(uint8_t sda, uint8_t scl){
        this->sda = sda;
        this->scl = scl;
    }

    void Initialize() {
        Wire.setPins(sda, scl);
        Wire.begin();
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(PWR_MGMT_1);
        Wire.write(0); // Wake up MPU6050
        Wire.endTransmission(true);
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(0x1B); // Gyroscope configuration register
        Wire.write(0x10); // Set full scale range to ±1000 deg/s
        Wire.endTransmission(true);
    }

    void ReadAccel(int16_t &ax, int16_t &ay, int16_t &az) {
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(ACCEL_XOUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU6050_ADDR, 6, 1);

        ax = (Wire.read() << 8 | Wire.read());
        ay = (Wire.read() << 8 | Wire.read());
        az = (Wire.read() << 8 | Wire.read());
    }

    void ReadGyro(int16_t &gx, int16_t &gy, int16_t &gz) {
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(GYRO_XOUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU6050_ADDR, 6, 1);

        gx = (Wire.read() << 8 | Wire.read());
        gy = (Wire.read() << 8 | Wire.read());
        gz = (Wire.read() << 8 | Wire.read());
    }
};