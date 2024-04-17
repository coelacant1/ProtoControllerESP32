#include <Arduino.h>

//Temporary Fix to map pins to board supported by PIO

class ESP32C3SuperMini{
public:
    static uint8_t ConvertXiao(uint8_t input){
        switch(input){
            case 0: return -1; break;
            case 1: return -1; break;
            case 2: return 0; break;
            case 3: return 1; break;
            case 4: return 2; break;
            case 5: return 3; break;
            case 6: return 4; break;
            case 7: return 5; break;
            case 8: return 8; break;
            case 9: return 9; break;
            case 10: return 10; break;
            case 20: return 7; break;
            case 21: return 6; break;
            default: return -1; break;
            
        }
    }
};