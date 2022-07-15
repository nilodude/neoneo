#include "Arduino.h"
#include "Input.h"
Input::Input(int audioPin, int switchPin, int offset) {
  audioPin= audioPin;
  switchPin = switchPin;
  offset = offset;
  raw = 0;
  audioMode = LOW;
  controlSign = LOW;
  audNorm = 0;
  aux = 0;
}




//int raw, boolean audioMode, boolean controlSign, long audNorm, long aux 
