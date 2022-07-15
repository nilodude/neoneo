#ifndef Input_h
#define Input_h
#include "Arduino.h" 
class Input {
public:
  Input(int audioPin, int switchPin, int offset);
  int audioPin;
  int switchPin;
  int offset;
  int raw;
  boolean audioMode;
  boolean controlSign;
  long audNorm;
  long aux;
};
#endif
