#pragma once

#include <Arduino.h>
#include <Wire.h>

class HardwareTest{
private:
    static void PrintAddress(uint8_t address, const String& text){
        Serial.print(address, HEX);
        Serial.print('\t');
        Serial.println(text);
    }

    static void PrintAddressValue(uint8_t address, const String& text, int value){
        Serial.print(address, HEX);
        Serial.print('\t');
        Serial.print(text);
        Serial.print('\t');
        Serial.println(value);
    }

    static bool TestI2CConnection(uint8_t address, String name){
        Wire.setPins(8, 9);

        Wire.begin();

        Wire.setClock(400000);//for longer range transmissions
        
        Wire.beginTransmission(address);

        uint8_t error = Wire.endTransmission();
        
        switch (error) {
            case 0:
                PrintAddress(address, " " + name + " Found!");
                return true;
                break;
            case 1:
                PrintAddress(address, " " + name + ":\tData too long to fit in transmit buffer.");
                break;
            case 2:
                PrintAddress(address, " " + name + ":\tReceived NACK on transmit of address.");
                break;
            case 3:
                PrintAddress(address, " " + name + ":\tReceived NACK on transmit of data.");
                break;
            case 4:
                PrintAddress(address, " " + name + ":\tOther error.");
                break;
            default:
                PrintAddress(address, " " + name + " not found. Unknown error.");
                break;
        }
        
        Wire.end();

        return false;
    }

public:
    static bool ScanDevice(uint8_t address) {//timeout in milliseconds and threshold is minimum for detection (0 is far away, 255 is touching)
        return TestI2CConnection(address, "MPU6050");
    }

    static void ScanDevices() {//timeout in milliseconds and threshold is minimum for detection (0 is far away, 255 is touching)
        Wire.setPins(8, 9);

        Wire.begin();

        Wire.setClock(100000);//for longer range transmissions

        uint8_t numDevices = 0;

        for (uint8_t i = 0; i < 127; i++){
            Wire.beginTransmission(i);

            uint8_t error = Wire.endTransmission();

            if(error == 0){// Device Found
                Serial.print("Device on address: ");
                Serial.println(i, HEX);
                numDevices++;
            }
        }

        Serial.print("Number of Devices Found: ");
        Serial.println(numDevices);
		
        Wire.end();
    }


    static void ResetI2CBus() {
        Wire.end();  // Disable the I2C hardware
        delay(10);   // Wait a bit
        Wire.begin();  // Re-enable the I2C hardware
    }

};
