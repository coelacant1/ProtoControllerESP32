#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SerialTransfer.h>
#include <Adafruit_APDS9960.h>
#include "MorphDecoder.h"

#define PROTOCONTROLLERV1_1

MorphDecoder morphDecoder; 

Adafruit_APDS9960 apds;

uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

static struct ESP32Data {
    float oW;
    float oX;
    float oY;
    float oZ;
    uint16_t r;
    uint16_t g;
    uint16_t b;
    uint16_t c;
    uint8_t m;
    uint8_t d;
    float ratio;
    uint8_t mode;
} e32Data;

const uint8_t button1 = 32;
long previousMillis = 0;
bool wasUsed = false;
bool useApds = false;
bool useBno = false;

void IRAM_ATTR isr() {
  if(millis() - previousMillis > 250){
    if(!wasUsed){
      e32Data.mode += 1;
      if (e32Data.mode > 8) e32Data.mode = 0;
  
      previousMillis = millis();
      wasUsed = true;
    }
    else{
      wasUsed = false;
    }
  }
}

SerialTransfer dataTransfer;

void setup(void){
  Wire.begin(23, 22);
  Serial.begin(115200);

  #ifdef PROTOCONTROLLERV1_1
  Serial1.begin(9600, SERIAL_8N1, 12, 13);//For V1-1
  #else
  Serial1.begin(9600, SERIAL_8N1, 12, 19);
  #endif
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  pinMode(button1, INPUT_PULLUP);
  attachInterrupt(button1, isr, RISING);
  
  dataTransfer.begin(Serial1);

  if (!bno.begin()){
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  else{
    Serial.println("BNO initialized!");
    useBno = true;
  }
  
  if(!apds.begin()){
    Serial.println("Failed to initialize APDS! Please check your wiring.");
  }
  else{
    Serial.println("APDS initialized!");
    useApds = true;
    
    //enable color sensign mode
    apds.enableColor(true);
    apds.enableProximity(true);
  }

  delay(1000);
}

void loop(void){
  if(useBno){
    imu::Quaternion quat = bno.getQuat();
    
    e32Data.oW = quat.w();
    e32Data.oX = quat.x();
    e32Data.oY = quat.y();
    e32Data.oZ = quat.z();
  }

  if(useApds){
    apds.getColorData(&e32Data.r, &e32Data.g, &e32Data.b, &e32Data.c);
    e32Data.d = apds.readProximity();
  }
  

  e32Data.ratio = (float)(millis() % 5000) / 5000.0f;

  Serial.print(e32Data.ratio); Serial.print('\t'); Serial.println(e32Data.mode);
  morphDecoder.Fetch();
 
  uint16_t sendSize = 0;

  sendSize = dataTransfer.txObj(e32Data, sendSize);
  dataTransfer.sendData(sendSize);

  delay(20);
}
