#include "SerialInterpreter.h"

void setup() {
  Serial.begin(115200);

  SerialInterpreter::Initialize();
}

void loop() {
  SerialInterpreter::Update();

  String dataString = SerialInterpreter::GetQuaternion() + "," + SerialInterpreter::GetColor() + "," + SerialInterpreter::GetRatio() + "," + SerialInterpreter::GetMode() + "," + SerialInterpreter::GetMorph();
  
  Serial.println(dataString);
}
