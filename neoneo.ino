#include <Adafruit_NeoPixel.h>
//#include "Input.h"

#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define SW1 A5
#define SW2 A3
#define SW3 A4
#define IN1 A2
#define IN2 A0
#define IN3 A1
#define OFF1 0
#define OFF2 20
#define OFF3 40

#define LED_PIN    6
#define LED_COUNT 60
#define MEAN (512)
#define NUM_SAMPLES (32)
#define NUM_SAMPLES_CTRL (1)
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

int debug = 0;

int raw1 = 0;
int raw2 = 0;
int raw3 = 0;

boolean startup = HIGH;

float dropFactor = .89;

struct Map {
  float dbValue;
  long ledNumber;
};

struct Input {
  int audioPin;
  int switchPin;
  int offset;
  int adc;
  int raw;
  boolean audioMode;
  boolean controlSign;
  long audNorm;
  int top;
  int bottom;
  long last;
};

Input input1 = {IN1, SW1,OFF1,0,0,LOW,LOW,0,0,0,0};
Input input2 = {IN2, SW2, OFF2,0,0,LOW,LOW,0,0,0,0};
Input input3 = {IN3, SW3, OFF3,0,0,LOW,LOW,0,0,0,0};

const Map lut[] = {
  { -30.5, 1},  { -30, 2},  { -30, 3},  { -26, 4},  { -25, 5}, { -24, 6},    { -23, 7 },    { -22, 8 },   { -21, 9},  { -20, 10},
  { -19, 11}, { -18, 12}, { -16, 13}, { -14, 14},  { -12, 15},  { -10, 16},   { -8, 17}, { -6, 18},  { -4, 19},  { -2, 20}
};

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

uint32_t red = strip.Color(255, 0, 0);
uint32_t green = strip.Color(0, 255, 0);
uint32_t blue = strip.Color(0, 0, 255);
uint32_t snakeColor = strip.Color(255, 30, 255);

long start = 0;

void setup() {
  if(debug) Serial.begin(9600);
  strip.begin();
  strip.show();
  strip.setBrightness(4);
  if (startup) {
    startUpAnimation();
  }
  measureMode(&input1);
  measureMode(&input2);
  measureMode(&input3);

  //Serial.println(input1.audioPin);
}

void loop() {
  
  measureSignal2(&input1);
  measureSignal2(&input2);
  measureSignal2(&input3);

  if(millis() - start > 30){
    strip.clear();
    start = millis();  
  }
  
  colorWipe(input1.audioMode, input1.bottom, input1.top, input1.controlSign, input1.audNorm, input1.offset);
  colorWipe(input2.audioMode, input2.bottom, input2.top, input2.controlSign, input2.audNorm, input2.offset);
  colorWipe(input3.audioMode, input3.bottom, input3.top, input3.controlSign, input3.audNorm, input3.offset);
  strip.show();
  
  
  measureMode(&input1);
  measureMode(&input2);
  measureMode(&input3);

  input1.last = input1.audNorm;
  input2.last = input2.audNorm;
  input3.last = input3.audNorm;

  if(debug) printValues();
  
}

void measureMode(Input *input) {
  int thres = 606;
  input->raw = analoggRead(input->switchPin);
  input->audioMode = input->raw > thres;
  input->controlSign = input->raw > 1;
}

void measureSignal2(Input *input) {
  int adc = 0, amp = 0, audNorm = 0, mean = 0;
  long rms = 0;
  float dB = 0;
  int numsamples = input->audioMode ? NUM_SAMPLES : NUM_SAMPLES_CTRL;
  for (int i = 0; i < numsamples; i++)  {
    adc = 1023 - analoggRead(input->audioPin);
    amp = abs(adc - MEAN);
    rms += (long(amp) * amp);
    mean += adc;
    if(adc > input->top)
      input->top = adc;
    
    if(adc < input->bottom)
      input->bottom = adc;
  }
  mean /= NUM_SAMPLES_CTRL;
  rms /= numsamples;
  dB = 20.0 * log10(sqrt(rms) / MEAN);

  if (input->audioMode) { 
    audNorm = db2led(dB, input->last);
  } else {
    audNorm = (40 * ((float)adc / 1024) - 20);
  }
  
  input->audNorm = audNorm > 40 ? 40 : audNorm;
  input->adc = adc;
  
}

int analoggRead(uint8_t pin) {
  uint8_t analog_reference = DEFAULT;
  uint8_t low, high;

  if (pin >= 14) pin -= 14;

  ADMUX = (analog_reference << 6) | (pin & 0x07);

  sbi(ADCSRA, ADSC);

  while (bit_is_set(ADCSRA, ADSC));

  low  = ADCL;
  high = ADCH;

  return (high << 8) | low;
}

void colorWipe(boolean audioMode, int bottom, int top , boolean controlSign, long audNorm, int offset) {
  if (audioMode) {
    audioWipe(audNorm, offset);
  } else {
    controlWipe(audNorm, bottom, top, offset, controlSign);
  }
}

void audioWipe(int value, int offset) {
  for (int i = 1 + offset; i <= value + offset; i++)
    strip.setPixelColor(i - 1, greenRedFade(i - offset));
}

void controlWipe(int value, int top, int bottom, int offset, boolean controlSign) {
  if (value + offset < 0 + offset) {
    for (int i = 20 + offset; i > 20 - abs(value) + offset; i--)
      strip.setPixelColor(i - 1, red);
  } else {
    for (int i = 1 + offset; i <= value + offset; i++)
      strip.setPixelColor(i, green);
  }
  strip.setPixelColor(offset, controlSign ? green : red);
}

long db2led(float db, long last) {
  int low = 0;
  int up = sizeof(lut) / sizeof(Map) - 1;
  int index = (low + up) / 2;
  int led = 0;
  while ((round(lut[index].dbValue) != round(db)) && (low <= up)) {
    if (lut[index].dbValue > db) up = index - 1;
    else low = index + 1;
    index = (low + up) / 2;
  }
  led = lut[index].ledNumber;
  if (led < last) {
    if (led < 18 && led > 8) {
      led = ceil(last * dropFactor);
    } else if (led >= 18) {
      led = last * 0.95;
    } else {
      led = last * dropFactor;
    }
  }

  return led;
}

uint32_t greenRedFade(long i) {
  int r = min(255, i * 10);
  int g = max(0, 90);
  return i > 15 ? i == 20 ? red : strip.Color(r , g , 0) : green;
}

void startUpAnimation() {
  snake(70);
}

void snake(int wait) {
  strip.clear();
  int tail = 5;
  for (int i = 50; i < 55; i++) {
    strip.clear();
    for (int j = i; j < i + tail; j++) {
      strip.setPixelColor(j, snakeColor);
    }
    strip.show();
    delay(wait);
  }

  for (int i = 26; i > 20; i--) {
    strip.clear();
    for (int j = i; j < i + tail; j++) {
      strip.setPixelColor(j, snakeColor);
    }
    strip.show();
    delay(wait);
  }
  for (int i = 11; i < 16; i++) {
    strip.clear();
    for (int j = i; j < i + tail; j++) {
      strip.setPixelColor(j, snakeColor);
    }
    strip.show();
    delay(wait);
  }
  startup = LOW;
}

void printValues(){
  printInput(&input1);
  printInput(&input2);
  printInput(&input3);
  Serial.println();
}

void printInput(Input *input) {
  switch (input->audioPin) {
    case 16:
      Serial.print("|IN1");
      break;
    case 14:
      Serial.print("\t|IN2");
      break;
    case 15:
      Serial.print("\t|IN3");
      break;
    default:
      Serial.print("");
  }

  Serial.print(" ");
  if (input->audioMode) {
    Serial.print("aud");
  } else {
    Serial.print("ctl");
    Serial.print(input->controlSign ? "+" : "-" );
  }
  Serial.print("\t");
  Serial.print(input->adc);
  Serial.print("\t");
  Serial.print(input->audNorm);
}
