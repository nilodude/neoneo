#include <Adafruit_NeoPixel.h>
#include "adc_freerunner.h"

#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define LED_PIN    6
#define LED_COUNT 60
#define AmpMax (1024 / 2)
#define NUM_SAMPLES (1024*2)

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

long aud=0, audNorm=0, amp = 0, adc = 0,soundVolRMS = 0;

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
  { -26, 11}, { -26, 12}, { -25, 13}, { -25, 14},  { -25, 15},  { -24, 16},   { -24, 17}, { -24, 18},  { -23, 19},  { -22, 20},
  { -21, 21}, { -21, 22}, { -20.5, 23}, { -20, 24}, { -19, 25}, { -18, 26},  { -17, 27},  { -16, 28 },  { -15, 29}, { -14, 30},
  { -13, 31}, { -12, 32}, { -11, 33}, { -10, 34}, { -9, 35}, { -8, 36}, { -7, 37},    { -6, 38},  { -5, 39}, { -4, 40}
};

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(9600);
  strip.begin();
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(3); // Set BRIGHTNESS to about 1/25.5 (max = 255)
  audioMode = digitalRead(A3);

  ADMUX |= (1 << REFS0);
  ADCSRA = 0xe0 + 4;
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  sbi(ADCSRA, ADPS0);

}

void loop() {

  aud = MeasureVolume();
  
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
    audNorm = (41L * adc / 1024L) - 20;
  }

  colorWipe(audNorm);

  printGraph();

  audioMode = digitalRead(A3);
  aux1 = audNorm;
}

long MeasureVolume() {
  soundVolRMS = 0;

  for (int i = 0; i < NUM_SAMPLES; i++)  {
    while (!(ADCSRA & _BV(ADIF))); 
    sbi(ADCSRA, ADIF); 
    byte adcl = ADCL; 
    byte adch = ADCH;
    adc = ((int)adch << 8) | adcl;
    amp = abs(adc - AmpMax);
    soundVolRMS += (long(amp) * amp);
  }
  
  soundVolRMS /= NUM_SAMPLES;
  float soundVolRMSflt = sqrt(soundVolRMS);

  dB = 20.0 * log10(soundVolRMSflt / AmpMax);

  return long(dB);
}

void colorWipe(long value) {
  uint32_t red = strip.Color(255, 0, 0);
  uint32_t green = strip.Color(0, 255, 0);
  uint32_t greenT = strip.Color(0, 0, 0,map(value,0,40,255,0));
  uint32_t orange = strip.Color(255,255,0);

  value = value > 40 ? 40 : value;
  //value = 36;
  if (audioMode) {
    //AUDIO
    if (value > 20 && value <= 31) {
      value = value - 20;
      for (int i = 1; i <= 20; i++) {
        strip.setPixelColor(i - 1, green);
      }
      
      for (int i = 1; i <= value; i++) {
        strip.setPixelColor(i - 1, strip.Color(min(23*i,255),max(50,255-10*i),0));
      }
//      for (int i = value+1; i <= min(20,value+1); i++) {
//        strip.setPixelColor(i - 1, greenT);
//        //strip.setPixelColor(i, greenT);
//      }
      strip.show();
    } else if( value > 31 ){ 
      value = value - 20;
      for (int i = 1; i <= 20; i++) {
        strip.setPixelColor(i - 1, green);
      }
      for (int i = 1; i <= 10; i++) {
        strip.setPixelColor(i - 1, strip.Color(min(23*i,255),max(50,255-10*i),0));
      }
      for (int i = 11; i <= value; i++) {
        strip.setPixelColor(i - 1, strip.Color(255,max(0,115-20*(i-10)),0));
      }
//      for (int i = value+1; i <= min(20,value+1); i++) {
//        strip.setPixelColor(i - 1, greenT);
//      }
      strip.show();
    }else {
      strip.clear();
      for (int i = 1; i <= value; i++) {
        strip.setPixelColor(i - 1, green);
      }
      strip.show();
    }
  } else {
    strip.clear();
    // CONTROL
    if (value <= 0) {
      //RED
      value=abs(value);
      for (int i = 20; i > 20 - 1  - value; i--) {
        strip.setPixelColor(i - 1, red);
      }
    } else { //GREEN
      for (int i = 1; i <= value + 1; i++) {
        strip.setPixelColor(i - 1, green);
      }
    }
    strip.show();
  }
  //strip.show();
}

long db2led(float db) {
  //0hz -> -3dB, 20kHz -> -13dB
  //min= -34.1dB ; max= -2.59
  int low = 0;
  int up = sizeof(lut) / sizeof(Map) - 1;
  int index = (low + up) / 2;
  
  while ((round(lut[index].dbValue) != round(db)) && (low <= up)) {
    if (lut[index].dbValue > dB) up = index - 1;
    else low = index + 1;
    index = (low + up) / 2;
  }
  return lut[index].ledNumber;
}

void printGraph() {
  Serial.print(adc);
  Serial.print("\t");
  Serial.print(aud);
  Serial.print("\t");
  Serial.println(audNorm);
}
