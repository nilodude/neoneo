#include <Adafruit_NeoPixel.h>
#include "adc_freerunner.h"

#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define LED_PIN    6
#define LED_COUNT 60
#define AmpMax (1024 / 2)
#define MicSamples (1024*2)
#define VolumeGainFactorBits 0

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

long aud, audNorm, maxi = 0;
boolean red = HIGH;
boolean audioMode = LOW;
long aux1 = 0;
float dB = 0;
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(9600);
  strip.begin();
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(10); // Set BRIGHTNESS to about 1/25.5 (max = 255)
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
    audNorm = map(dB,-35.0,-2.00,0,40);
    red= audNorm > 20;
  }else{
    //CONTROL
    audNorm = (41L * aud / 1024L) - 20;
    red = audNorm < 0;
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
  long soundVolAvg = 0, soundVolMax = 0, soundVolRMS = 0, t0 = millis();

  for (int i = 0; i < MicSamples; i++)  {
    while (!(ADCSRA & /*0x10*/_BV(ADIF))); // wait for adc to be ready (ADIF)
    sbi(ADCSRA, ADIF); // restart adc
    byte m = ADCL; // fetch adc data
    byte j = ADCH;
    int k = ((int)j << 8) | m; // form into an int
    int amp = abs(k - AmpMax);
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
//  soundVolAvg = 100 * soundVolAvg / AmpMax;
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


void printGraph() {

  //maxi = aud > maxi ? aud : maxi;
  //Serial.print(maxi);
 // Serial.print("\t");
  Serial.print(dB);
  Serial.print("\t");
  Serial.print(aud);
  Serial.print("\t");
  Serial.println(audNorm);

}
