#include <Adafruit_NeoPixel.h>
#include "adc_freerunner.h"

#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define SW1 2
#define SW2 4
#define SW3 3
#define IN1 2
#define IN2 0
#define IN3 1
#define LED_PIN    6
#define LED_COUNT 60
#define MEAN (512)
#define NUM_SAMPLES (64)

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

long adc = 0, amp = 0, rms = 0, audNorm = 0, led = 0;
float dB = 0;
long audNorm1 = 0;
long audNorm2 = 0;
long audNorm3 = 0;
boolean audioMode1 = LOW;
boolean audioMode2 = LOW;
boolean audioMode3 = LOW;
long aux1 = 0;
long aux2 = 0;
long aux3 = 0;


float dropFactor = .89;

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

uint32_t red = strip.Color(255, 0, 0);
uint32_t green = strip.Color(0, 255, 0);

int analoggRead(uint8_t pin);

void setup() {
  Serial.begin(9600);
  strip.begin();
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(2); // Set BRIGHTNESS to about 1/25.5 (max = 255)
  audioMode1 = digitalRead(SW1);
  audioMode2 = digitalRead(SW2);
  audioMode3 = digitalRead(SW3);
}

void loop() {
  
  audNorm1 = measure(IN1, audioMode1);
  audNorm2 = measure(IN2, audioMode2);
  audNorm3 = measure(IN3, audioMode3);
   
  colorWipe(audNorm1);
  
  audioMode1 = digitalRead(SW1);
  audioMode2 = digitalRead(SW2);
  audioMode3 = digitalRead(SW3); 
   
  aux1 = audNorm1;
  aux2 = audNorm3;
  aux3 = audNorm3;
  
  printValues();
}
long measure(int channel, boolean audioMode) {
  rms = 0;
  int numsamples = audioMode ? NUM_SAMPLES : 1;
  for (int i = 0; i < numsamples; i++)  {
    adc = analoggRead(channel)+28;  
    amp = abs(adc - MEAN);
    rms += (long(amp) * amp);
  }
  rms /= numsamples;
  dB = 20.0 * log10(sqrt(rms) / MEAN);

  if (audioMode) {
    //AUDIO MODE
    audNorm = db2led(dB);
  } else {
    //CONTROL MODE
    audNorm = -((41L * adc / 1024L) - 20);
  }
  return audNorm;  
}
//LAPUTACLAVE:
//https://garretlab.web.fc2.com/en/arduino/inside/hardware/arduino/avr/cores/arduino/wiring_analog.c/analogRead.html
int analoggRead(uint8_t pin)
{
    uint8_t analog_reference = DEFAULT;
    uint8_t low, high;
 
    if (pin >= 14) pin -= 14; // allow for channel or pin numbers
 
    // set the analog reference (high two bits of ADMUX) and select the
    // channel (low 4 bits).  this also sets ADLAR (left-adjust result)
    // to 0 (the default).
    ADMUX = (analog_reference << 6) | (pin & 0x07);
 
    // without a delay, we seem to read from the wrong channel
    //delay(1);
 
    // start the conversion
    sbi(ADCSRA, ADSC);
 
    // ADSC is cleared when the conversion finishes
    while (bit_is_set(ADCSRA, ADSC));
 
    // we have to read ADCL first; doing so locks both ADCL
    // and ADCH until ADCH is read.  reading ADCL second would
    // cause the results of each conversion to be discarded,
    // as ADCL and ADCH would be locked when it completed.
    low  = ADCL;
    high = ADCH;
 
    // combine the two bytes
    return (high << 8) | low;
}



void colorWipe(long value) {
  value = value > 40 ? 40 : value;
  //value = 36;
  if (audioMode1) { // AUDIO MODE
    strip.clear();
    if (value <= 20) {
      for (int i = 1; i <= value; i++)
        strip.setPixelColor(i - 1, green);
    } else {
      if (value > 20) {
        value = value - 20;
        for (int i = value; i <= 20; i++)
          strip.setPixelColor(i - 1, green);
        for (int i = 1; i <= value; i++)
          strip.setPixelColor(i - 1, red);
      }
      //    strip.setPixelColor(value+1, 0);
    }
  } else { // CONTROL MODE
    strip.clear();
    if (value <= 0) {
      for (int i = 20; i > 20 - 1  - abs(value); i--)
        strip.setPixelColor(i - 1, red);
    } else {
      for (int i = 1; i <= value + 1; i++)
        strip.setPixelColor(i - 1, green);
    }
  }
  strip.show();
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
  led = lut[index].ledNumber;
  if (led < aux1)
    led = aux1 * dropFactor;
    
  return led;
}

uint32_t greenRedFade(long i) {
  int r = min(255, 40 + i * 25);
  int g = max(0, 240 - i * 15);
  return strip.Color(r , g , 0);
}

void printValues() {
  Serial.print(adc);
  Serial.print("\t");
  Serial.print(audioMode1 ? "audio" : "control");
  Serial.print("\t");
  Serial.print(dB);
  Serial.print("\t");
  Serial.println(audNorm);
}
