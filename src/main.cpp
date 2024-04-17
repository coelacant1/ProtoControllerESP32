#include <Arduino.h>
#include "ESP32C3SuperMini.h"
#include "MotionControl.h"

MPU6050 mpu = MPU6050(8, 9);
MotionControl mC = MotionControl(mpu, 4);

void setup() {
    Serial.begin(9600);

    mpu.Initialize();
    mC.Initialize();
}

void loop() {
    mC.Update();
}
