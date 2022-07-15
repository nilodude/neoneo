#include "Arduino.h"
#include "Input.h"
Input::Input(int audioPin, int switchPin, int offset) {
  _audioPin= audioPin;
  _switchPin = switchPin;
  _offset = offset;
  _raw = 0;
  _audioMode = LOW;
  _controlSign = LOW;
  _audNorm = 0;
  _aux = 0;
}




//int raw, boolean audioMode, boolean controlSign, long audNorm, long aux 
