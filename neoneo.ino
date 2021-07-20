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
long soundVolAvg = 0, soundVolMax = 0, soundVolRMS = 0, t0 = 0;
long amp = 0, k = 0;
boolean red = HIGH;
boolean audioMode = LOW;
long aux1 = 0;
float dB = 0;

int dropTime = 0;
int dropDelay = 0;      
float dropFactor = .87;

struct Map {
  float dbValue;
  long ledNumber;
};

const Map lut[] = {
  { -30.5, 1},  { -30, 2},  { -30, 3},  { -29, 4},  { -29, 5}, { -29, 6},    { -28, 7 },    { -28, 8 },   { -27, 9},  { -27, 10},
  { -26, 11}, { -26, 12}, { -25, 13}, { -25, 14},  { -24, 15},  { -24, 16},   { -23, 17}, { -23, 18},  { -22, 19},  { -22, 20},
  { -21, 21}, { -21, 22}, { -20.5, 23}, { -20, 24}, { -19, 25}, { -18, 26},  { -17, 27},  { -16, 28 },  { -15, 29}, { -14, 30},
  { -13, 31}, { -12, 32}, { -11, 33}, { -10, 34}, { -9, 35}, { -8, 36}, { -7, 37},    { -5, 38},  { -4, 39}, { -2, 40}
};

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

  if (audioMode) {
    //AUDIO
    audNorm = db2led(dB);

    if(audNorm < aux1){
      dropTime++;
      if(dropTime > dropDelay){
        audNorm = aux1 * dropFactor;
        dropTime = 0;
      }else{
        audNorm = aux1;
      }
    }
    
    red = audNorm > 20;
  } else {
    //CONTROL
    audNorm = (41L * k / 1024L) - 20;
    red = audNorm < 0;
    audNorm = abs(audNorm);
  }

  // dibujar en papel los saltos de db y hacer una LUT pa los leds
  //

  colorWipe(audNorm, red);

  //uint32_t off = strip.Color(0, 0, 0, 0);
  //for (int i = audNorm; i > 0; i--) {
  //  strip.setPixelColor(i - 1, off);
  //}

  printGraph();
  aux1 = aud;

  audioMode = digitalRead(A3);
  //strip.clear();
  aux1 = audNorm;
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
  soundVolRMSflt = 100 * soundVolRMSflt / AmpMax;
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

void colorWipe(long value, boolean shouldBeRed) {
  uint32_t red = strip.Color(255, 0, 0);
  uint32_t green = strip.Color(0, 255, 0);
  uint32_t orange = strip.Color(255,255,0);

  value = value > 40 ? 40 : value;
  if (audioMode) {
    //AUDIO
    if (value > 20 && value <= 31) {
      value = value - 20;
      for (int i = 1; i <= 20; i++) {
        strip.setPixelColor(i - 1, green);
      }
      for (int i = 1; i <= value; i++) {
        strip.setPixelColor(i - 1, strip.Color(min(40*i,255),max(127,255-value*i),0));
      }
      strip.show();
    } else if( value > 31 ){ 
      value = value - 20;
      for (int i = 1; i <= 20; i++) {
        strip.setPixelColor(i - 1, green);
      }
      for (int i = 1; i <= 10; i++) {
        strip.setPixelColor(i - 1, strip.Color(min(40*i,255),max(127,255-value*i),0));
      }
      for (int i = 10; i <= value; i++) {
        strip.setPixelColor(i - 1, strip.Color(255,max(0,127-25*(i-9)),0));
      }
      strip.show();
    }else {
      strip.clear();
      for (int i = 1; i <= value; i++) {
        strip.setPixelColor(i - 1, green);
      }
      strip.show();
    }
    //strip.show();

  } else {
    // CONTROL
    if (red) { //RED
      for (int i = 20; i > 20 - 1  - value; i--) {
        strip.setPixelColor(i - 1, red);
      }
      //strip.show();
    } else { //GREEN
      for (int i = 1; i <= value + 1; i++) {
        strip.setPixelColor(i - 1, green);
      }
      //strip.show();
    }
  }
  //strip.show();
}

long db2led(float db) {
  int low = 0;
  int up = sizeof(lut) / sizeof(Map) - 1;
  int index = (low + up) / 2;
  //  boolean upper = abs(lut[index].dbValue - dB) <= abs(lut[index-1].dbValue - dB);


  while ((round(lut[index].dbValue) != round(db)) && (low <= up)) {
    if (lut[index].dbValue > dB) up = index - 1;
    else low = index + 1;
    index = (low + up) / 2;
  }
  return lut[index].ledNumber;
  //long led = pled == 0 ? pled : pled -1;

  //if(db < -


}

void printGraph() {
  Serial.print(dB);
  Serial.print("\t");
  Serial.print(aud);
  Serial.print("\t");
  Serial.println(audNorm);
}
