#include <Adafruit_NeoPixel.h>
#include "adc_freerunner.h"

#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define LED_PIN    6
#define LED_COUNT 60
#define AmpMax (1024 / 2)
#define MicSamples (1024*3)
#define VolumeGainFactorBits 0

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

long aud, audNorm, maxi = 0;
long soundVolAvg = 0, soundVolMax = 0, soundVolRMS = 0,t0=0;
long amp=0,k = 0;
boolean red = HIGH;
boolean audioMode = LOW;
long aux1 = 0;
float dB = 0;

struct Map {
  float dbValue;
  long ledNumber;
};

const Map lut[] ={
  {-32,1},    {-29,2},    {-26,3},    {-23,4},     {-22.5,5}, {-22,6},    {-21,7 },    {-20.5,8 },   {-20,9},  {-19.5,10},
  {-19,11},   {-18.5,12}, {-18,13},   {-17.5,14},  {-17,15},  {-16.5,16},   {-16,17}, {-15.5,18},  {-15,19},  {-14.5,20},
  {-14,21},   {-13.5,22}, {-13,23},   {-12.17,24}, {-12.35,25}, {-11.52,26},  {-10.70,27},  {-9.88,28 },  {-9.05,29}, {-8.23,30},
  {-7.41,31}, {-6,32}, {-5.5,33}, {-5,34},  {-4.5,35}, {-4,36}, {-3.5,37},    {-3,38},  {-2.5,39}, {-2,40}};

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(9600);
  strip.begin();
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(5); // Set BRIGHTNESS to about 1/25.5 (max = 255)
  audioMode = digitalRead(A3);

  ADMUX |= (1 << REFS0); // Set ADC reference to AVCC
  ADCSRA = 0xe0 + 4;
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  sbi(ADCSRA, ADPS0);

}

void loop() {
  //aud = analogRead(A2);

  aud = MeasureVolume();
  //0hz -> -3dB, 20kHz -> -13dB
  //min= -34.1dB ; max= -2.59
  
  if(audioMode){
    //AUDIO
    
    audNorm = db2led(dB);
    red= audNorm > 20;
  }else{
    //CONTROL
    audNorm = (41L * k / 1024L) - 20;
    red = audNorm < 0;
    audNorm=abs(audNorm);
  }

  // dibujar en papel los saltos de db y hacer una LUT pa los leds
  // 

  colorWipe(audNorm,red);

  printGraph();
  aux1 = aud;

  audioMode = digitalRead(A3);
  strip.clear();
}

long MeasureVolume() {
  soundVolAvg = 0, soundVolMax = 0, soundVolRMS = 0, t0 = millis();

  for (int i = 0; i < MicSamples; i++)  {
    while (!(ADCSRA & /*0x10*/_BV(ADIF))); // wait for adc to be ready (ADIF)
    sbi(ADCSRA, ADIF); // restart adc
    byte m = ADCL; // fetch adc data
    byte j = ADCH;
    k = ((int)j << 8) | m; // form into an int
    amp = abs(k - AmpMax);
    amp <<= VolumeGainFactorBits;
    soundVolMax = max(soundVolMax, amp);
    soundVolAvg += amp;
    soundVolRMS += (long(amp) * amp);
  }
  soundVolAvg /= MicSamples;
  soundVolRMS /= MicSamples;
  float soundVolRMSflt = sqrt(soundVolRMS);

  dB = 20.0 * log10(soundVolRMSflt / AmpMax);

  // convert from 0 to 100
  soundVolAvg = 100 * soundVolAvg / AmpMax;
//  soundVolMax = 100 * soundVolMax / AmpMax;
  soundVolRMSflt = 100 *soundVolRMSflt / AmpMax;
  soundVolRMS = 10 * soundVolRMSflt / 7; // RMS to estimate peak (RMS is 0.7 of the peak in sin)

  // print
//  Serial.print(millis() - t0);
//  Serial.print("\t");
//  Serial.print(soundVolMax);
//  Serial.print("\t");
  //Serial.print(soundVolAvg);
 // Serial.print("\t");
  //Serial.print((long)dB);
  //Serial.print("\t");
  //Serial.println(dB);
  return long(dB);
}

void colorWipe(long value, boolean red) {
  uint32_t redColor = strip.Color(255, 0, 0);
  uint32_t greenColor = strip.Color(0, 255, 0);
  
  uint32_t color = red ?  redColor : greenColor;
  value = value > 40 ? 40 : value;
  if (audioMode) {
    //AUDIO
    if (value > 20) {
      value = value - 20;
      for (int i = 1; i <= 20; i++) {
        strip.setPixelColor(i - 1, greenColor);
      }
    }
    for (int i = 1; i <= value; i++) {
      strip.setPixelColor(i - 1, color);
    }
    //strip.setPixelColor(value-1, color);
    strip.show();

  } else {
    // CONTROL
    if (red) { //RED
      for (int i = 20; i > 20 - 1  - value; i--) {
        strip.setPixelColor(i - 1, redColor);
      }
      strip.show();
    } else { //GREEN
      for (int i = 1; i <= value + 1; i++) {
        strip.setPixelColor(i - 1, greenColor);
      }
      strip.show();
    }
  }
}

long db2led(float db){
  int low = 0;
  int up = sizeof(lut)/sizeof(Map) - 1;
  int index = (low+up)/2;
//  boolean upper = abs(lut[index].dbValue - dB) <= abs(lut[index-1].dbValue - dB);
  
  
  while((lut[index].dbValue != db) && (low <=up)){
    if(lut[index].dbValue > dB) up = index-1;
    else low = index +1;
    index = (low+up)/2;
  }
  return lut[index].ledNumber;
  //long led = pled == 0 ? pled : pled -1;
  
  //if(db < -

  
}

void printGraph() {
  //Serial.print(dB);
  //Serial.print("\t");
  Serial.print(aud);
  Serial.print("\t");
  Serial.println(audNorm);
}
