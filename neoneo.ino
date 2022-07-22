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
boolean ud = LOW;
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
  long last;
};

Input input1 = {IN1, SW1,OFF1,0,0,LOW,LOW,0,0};
Input input2 = {IN2, SW2, OFF2,0,0,LOW,LOW,0,0};
Input input3 = {IN3, SW3, OFF3,0,0,LOW,LOW,0,0};

const Map lut[] = {
  { -30.5, 1},  { -30, 2},  { -30, 3},  { -26, 4},  { -25, 5}, { -24, 6},    { -23, 7 },    { -22, 8 },   { -21, 9},  { -20, 10},
  { -19, 11}, { -18, 12}, { -16, 13}, { -14, 14},  { -12, 15},  { -10, 16},   { -8, 17}, { -6, 18},  { -4, 19},  { -2, 20}
};

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel prevStrip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

uint32_t red = strip.Color(255, 0, 0);
uint32_t green = strip.Color(0, 255, 0);
uint32_t blue = strip.Color(0, 0, 255);
uint32_t snakeColor = strip.Color(255, 30, 255);

uint32_t previous[60] = {};
int fadeCount = 1;

uint32_t fadeMillis = 0;
uint32_t lastFadeMillis = 0;


uint32_t currentMillis = 0;
uint32_t lastMillis = 0;


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

  
  
  //strip.clear();
  colorWipe(input1.audioMode, input1.controlSign, input1.audNorm, input1.offset);
  colorWipe(input2.audioMode, input2.controlSign, input2.audNorm, input2.offset);
  colorWipe(input3.audioMode, input3.controlSign, input3.audNorm, input3.offset);
  
  strip.show();
  
  
  measureMode(&input1);
  measureMode(&input2);
  measureMode(&input3);

  input1.last = input1.audNorm;
  input2.last = input2.audNorm;
  input3.last = input3.audNorm;

  if(debug==1) {
    printValues();
  }else if(debug ==2){
    plotValues();
  }
  
}

void measureMode(Input *input) {
  int thres = 606;
  input->raw = analoggRead(input->switchPin);
  input->audioMode = input->raw > thres;
  input->controlSign = input->raw > 1;
}

void measureSignal2(Input *input) {
  long adc = 0, amp = 0, audNorm = 0, mean = 0;
  long rms = 0;
  float dB = 0;
  int numsamples = input->audioMode ? NUM_SAMPLES : NUM_SAMPLES_CTRL;
  for (int i = 0; i < numsamples; i++)  {
    adc = 1023 - analoggRead(input->audioPin);
    amp = abs(adc - MEAN);
    rms += (long(amp) * amp);
    mean += adc;
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

void colorWipe(boolean audioMode, boolean controlSign, long audNorm, int offset) {
  if (audioMode) {
    audioWipe(audNorm, offset);
  } else {
    //if "its time" to update, update, if not, smoothly substract brightness from last value
    //if(shouldUpdate) controlWipe();
    //else dropSmoothly();
    //va por buen camino pero es necesario comprobar que se esta accediendo realmente a la posicion correcta del array
    
      if(fadeCount > 3){
        fadeCount = 0;
        controlWipe(audNorm, offset, controlSign);
        lastFadeMillis = fadeMillis;
      }else{
        fadeCount++;
        for(int i= 1+offset; i<=offset +19;i++){
          if(prevStrip.getPixelColor(i-1) > 0){
            uint32_t currentColor = prevStrip.getPixelColor(i-1);
            uint8_t currentRed = Red(currentColor);
            uint8_t currentGreen = Green(currentColor);
  
            //uint32_t newColor = controlSign ? strip.Color(0,currentGreen - 2,0) : strip.Color(0,currentRed - 2,0);
          
            strip.setPixelColor(i-1, strip.Color(0,currentGreen - 5,0));
            prevStrip.setPixelColor(i-1, strip.Color(0,currentGreen - 5,0));
          }else{
            strip.setPixelColor(i-1, 0);
            //prevStrip.setPixelColor(i-1, 0);
          }
        }
      }
    
  }
}

uint8_t Red(uint32_t color){
  return (color >> 16) & 0xFF;
}

uint8_t Green(uint32_t color){
   return (color >> 8) & 0xFF;
}

uint8_t Blue(uint32_t color)
{
  return color & 0xFF;
}


void audioWipe(int value, int offset) {
  for (int i = 1 + offset; i <= value + offset; i++)
    strip.setPixelColor(i - 1, greenRedFade(i - offset));
}

void controlWipe(int value, int offset, boolean controlSign) {
  if (value + offset < 0 + offset) {
    for (int i = 20 + offset; i > 20 - abs(value) + offset; i--){
      strip.setPixelColor(i - 1, red);
      prevStrip.setPixelColor(i-1, red);
      //try to store pixel values in a different AdaFruit_Neopixel variable "previousStrip"
      //previous[i-1] = red;
    }
  } else {
    for (int i = 1 + offset; i <= value + offset; i++){
      strip.setPixelColor(i, green);
      prevStrip.setPixelColor(i-1, green);
      //previous[i-1] = green;
    }
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

uint32_t redFade(int b){
  return strip.Color(b,0,0);
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

void plotValues(){
  plotInput(&input1);
  plotInput(&input2);
  plotInput(&input3);
  Serial.println();
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

void plotInput(Input *input){
  Serial.print(input->adc);  
}
