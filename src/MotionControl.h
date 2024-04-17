// MotionControl.h
#ifndef MotionControl_h
#define MotionControl_h

#include <Arduino.h>
#include "MPU6050.h"

class MotionControl {
private:
    MPU6050& mpu;
    int buttonPin;
    volatile bool buttonPressed = false;
    float velocity[3] = {0, 0, 0}; // Current velocity for X, Y, Z
    float maxVelocity[3] = {0, 0, 0}; // Maximum velocity for X, Y, Z
    float angularVelocity[3] = {0, 0, 0}; // Current velocity for X, Y, Z
    float maxAngularVelocity[3] = {0, 0, 0}; // Maximum velocity for X, Y, Z
    unsigned long sampleCount = 0;
    unsigned long lastSampleTime = 0; // Stores the last sample time
    int16_t pax, pay, paz;
    int16_t pgx, pgy, pgz;

public:
    MotionControl(MPU6050& mpuSensor, int btnPin) : mpu(mpuSensor), buttonPin(btnPin) {}

    void Initialize() {
        pinMode(buttonPin, INPUT_PULLUP); // Assuming active-low button
        mpu.Initialize();
    }

    void Update() {
        if (!digitalRead(buttonPin)) {
            if(!buttonPressed) {
                lastSampleTime = millis();
                // Reset velocities and max velocities when button is first pressed
                Reset();
                sampleCount = 0;
                buttonPressed = true;
            }

            CheckMotion();
        }
        else if(buttonPressed){
            buttonPressed = false;

            // Output the max velocity result when button is released
            PrintMotion();
        }
    }

private:
    void CheckMotion() {
        unsigned long currentTime = millis();
        unsigned long deltaTime = currentTime - lastSampleTime;
        lastSampleTime = currentTime;

        if (deltaTime == 0) {
            return; // Prevent division by zero if called in rapid succession
        }

        int16_t ax, ay, az;
        int16_t gx, gy, gz;
        mpu.ReadAccel(ax, ay, az);
        mpu.ReadGyro(gx, gy, gz);

        ax = abs(ax);
        ay = abs(ay);
        az = abs(az);
        gx = abs(gx);
        gy = abs(gy);
        gz = abs(gz);

        if(sampleCount == 0){
            pax = ax;
            pay = ay;
            paz = az;
            pgx = gx;
            pgy = gy;
            pgz = gz;
        }

        // Convert to 'g' assuming 16384 is 1g for MPU6050 at default settings (2g range)
        float gForceX = (ax - pax) / 16384.0f;
        float gForceY = (ay - pay) / 16384.0f;
        float gForceZ = (az - paz) / 16384.0f;


        // Integrate acceleration to get velocity (v = u + at)
        velocity[0] += gForceX * (deltaTime / 10.0f);
        velocity[1] += gForceY * (deltaTime / 10.0f);
        velocity[2] += gForceZ * (deltaTime / 10.0f);

        angularVelocity[0] += (gx - pgx) / 32.8f;
        angularVelocity[1] += (gy - pgy) / 32.8f;
        angularVelocity[2] += (gz - pgz) / 32.8f;

        pax = ax;
        pay = ay;
        paz = az;
        pgx = gx;
        pgy = gy;
        pgz = gz;

        // Update max velocity if current velocity is greater
        for (int i = 0; i < 3; i++) {
            if (abs(velocity[i]) > abs(maxVelocity[i])) {
                maxVelocity[i] = velocity[i];
            }

            if (abs(angularVelocity[i]) > abs(maxAngularVelocity[i])) {
                maxAngularVelocity[i] = angularVelocity[i];
            }
        }

        sampleCount++;
    }

    void PrintMotion() {
        // Determine the axis with the greatest magnitude of velocity
        int maxVelIndex = 0; // Index of the axis with the greatest velocity magnitude
        int maxAngIndex = 0; // Index of the axis with the greatest velocity magnitude
        float maxVel = maxVelocity[0]; // Start with the magnitude of the X-axis velocity
        float maxAng = maxAngularVelocity[0];

        angularVelocity[0] /= float(sampleCount);
        angularVelocity[1] /= float(sampleCount);
        angularVelocity[2] /= float(sampleCount);

        // Check Y and Z axes
        for (int i = 1; i < 3; i++) {
            if (abs(maxVelocity[i]) > abs(maxVel)) {
                maxVel = maxVelocity[i];
                maxVelIndex = i;
            }

            if (abs(maxAngularVelocity[i]) > abs(maxAng)) {
                maxAng = maxAngularVelocity[i];
                maxAngIndex = i;
            }
        }


        if(abs(maxAngularVelocity[maxAngIndex]) > 350.0f){
            // Print the maximum velocity and the axis information
            switch (maxAngIndex) {
                case 0: Serial.print("A     "); break;
                case 1: Serial.print(" B    "); break;
                case 2: Serial.print("  C   "); break;
            }

            Serial.print('\t');
            Serial.println(maxAng);
        }
        else if (abs(maxVelocity[maxVelIndex]) > 0.08f){
            // Print the maximum velocity and the axis information
            switch (maxVelIndex) {
                case 0: Serial.print("   D  "); break;
                case 1: Serial.print("    E "); break;
                case 2: Serial.print("     F"); break;
            }
            
            Serial.print('\t');
            Serial.println(maxVel);
        }
        else if (sampleCount > 200){
            Serial.println("      G");
        }
        else{
            Serial.println("       H");
        }
    }

    void Reset() {
        for (int i = 0; i < 3; i++) {
            velocity[i] = 0;
            maxVelocity[i] = 0;
            angularVelocity[i] = 0;
            maxAngularVelocity[i] = 0;
        }
    }

};

#endif // MotionControl_h
