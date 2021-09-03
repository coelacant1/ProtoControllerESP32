#pragma once

#include <arduinoFFT.h>
#include "KalmanFilter.h"

class MorphDecoder{
public:
  enum MouthShape{
    EE,
    E,
    AE,
    AH,
    OO,
    OPEN
  };
  
private:
  static const uint16_t sampleFreqency = 5000;
  static const uint16_t samplePeriod = round(1000000 * (1.0f / sampleFreqency));
  static const uint16_t samples = 512;//Multiple of 2 only
  static const uint8_t micPin = 34;
  uint16_t lowThreshold = 50;
  
  arduinoFFT FFT = arduinoFFT();
  SemaphoreHandle_t fftMutex;
  double noisData[samples];
  double realData[samples];
  double imagData[samples];
  unsigned long previousTime = 0;

  void SampleMicrophone(){
    for(uint16_t i = 0; i < samples; i++){
      realData[i] = analogRead(micPin);
      imagData[i] = 0;
    }
    
    previousTime = micros();
    
    while (micros() < (previousTime + samplePeriod)){ delay(0); }
  }

  void CalculateFFT(){
    FFT.DCRemoval(realData, samples);
    FFT.Windowing(realData, samples, FFT_WIN_TYP_FLT_TOP, FFT_FORWARD);
    FFT.Compute(realData, imagData, samples, FFT_FORWARD);
    FFT.ComplexToMagnitude(realData, imagData, samples);
  }

  void RemoveNoise(){
    for(uint16_t i = lowThreshold; i < samples / 2; i++){
      noisData[i] = noisData[i] * 0.95 + realData[i] * 0.05;
      realData[i] = realData[i] - noisData[i];
      realData[i] = realData[i] < 0 ? 0 : realData[i];
    }
  }

  void SmoothFFT(){
    KalmanFilter kf = KalmanFilter(0.05, 20);

    for(uint16_t i = lowThreshold; i < samples / 2; i++){
      realData[i] = kf.Filter(realData[i]);
    }
  }

  //void EstimateFormant
  
  void FFTUpdate(){
    while(true){ // infinite loop
      if(xSemaphoreTake(fftMutex, (TickType_t)5) == pdTRUE){
        SampleMicrophone();
        //CalculateFFT();
        //CombineFFT();
        //RemoveNoise();
        //SmoothFFT();

        xSemaphoreGive(fftMutex);
      }
      
      vTaskDelay(50 / portTICK_PERIOD_MS);//5ms delay
    }
    
    vTaskDelete(NULL);
  }
  
  static void FFTWrapper(void* _this){
    ((MorphDecoder*)_this)->FFTUpdate();
  }

  void CreateFFTTask(){
    xTaskCreate(this->FFTWrapper, "Update FFT", 16384, this, 1, NULL);
  }
  
public:
  MorphDecoder(){
    pinMode(micPin, INPUT);
    fftMutex = xSemaphoreCreateMutex();
    
    CreateFFTTask();
  }

  void Fetch(){
    if(xSemaphoreTake(fftMutex, (TickType_t)5) == pdTRUE){
      for(uint16_t i = lowThreshold; i < samples / 2; i++){
        //Serial.println(realData[i]);
      }
      
      xSemaphoreGive(fftMutex);
    }
  }
};
