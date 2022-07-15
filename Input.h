#ifndef Input_h
#define Input_h
#include "Arduino.h" 
class Input {
public:
  Input(int audioPin, int switchPin, int offset);
private:
  int _audioPin;
  int _switchPin;
  int _offset;
  int _raw;
  boolean _audioMode;
  boolean _controlSign;
  long _audNorm;
  long _aux;
};
#endif
