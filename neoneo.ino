#include <Adafruit_NeoPixel.h>
#include "adc_freerunner.h"

#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define LED_PIN    6
#define LED_COUNT 60
#define AVG_SAMPLES 4

int16_t volatile avg_values[AVG_SAMPLES];
int16_t volatile avg = 0;

long aud1, aud1Offset = 0;
boolean red = HIGH;
boolean audioMode = LOW;
long aux1 = 0;
float lambda = .875;

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(9600);
  strip.begin();
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(10); // Set BRIGHTNESS to about 1/25.5 (max = 255)
  audioMode = digitalRead(A3);
  setup_adc();
}

void loop() {

  aud1 = audioMode ? abs(adc_values[0] - 512) : adc_values[0];
  avg = lambda * aud1 + (1 - lambda) * avg;
   const int sampleWindow = 70; 
   int maximum = 210;
   int val= map(val, 0, 1023, -10, 10);
   unsigned int sample;
   unsigned long startMillis= millis();  // Start of sample window
   unsigned int peakToPeak = 0;   // peak-to-peak level

   unsigned int signalMax = 0;
   unsigned int signalMin = 200;

   // collect data for 50 mS
   while (millis() - startMillis < sampleWindow)
   {
      sample = adc_values[0];

       if(val<0){
        sample=sample/(val*(-1));
        }
       if(val>0){
        sample=sample*val;
        }
      
      if (sample < 1024)  // toss out spurious readings
      {
         if (sample > signalMax)
         {
            signalMax = sample;  // save just the max levels
         }
         else if (sample < signalMin)
         {
            signalMin = sample;  // save just the min levels
         }
      }
   }
   peakToPeak = signalMax - signalMin; 
   int led = map(peakToPeak, 0, maximum, 0, strip.numPixels());
  
 // avg = (3 * avg + 1 * aud1) /4 ; // lambda 3/4 = 0.75
 // avg = (7 * avg + aud1) >> 3 ; // lambda 7/8 = 0.875
  avg = 41 * (avg+512) /1024;
  aud1Offset = (41L * aud1 / 1024L) - 20;
  red = audioMode ? led > 20 : aud1Offset < 0;

  colorWipe(audioMode? led : abs(aud1Offset), red, audioMode, 2);

  printTable();
  aux1 = aud1;

  audioMode = digitalRead(A3);
  strip.clear();
}


void colorWipe(long value, boolean red, boolean audioMode, int wait) {
  uint32_t redColor = strip.Color(255, 0, 0);
  uint32_t greenColor = strip.Color(0, 255, 0);

  uint32_t color = red ?  redColor : greenColor;

  if (audioMode) {
    //AUDIO
    if (value > 20) {
      value = value - 20;
      for (int i = 1; i <= strip.numPixels(); i++) {
        strip.setPixelColor(i - 1, greenColor);
      }
    }
    for (int i = 1; i <= value + 1; i++) {
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




void printTable() {
  // if (aud1 != aux1) {
  Serial.print("avg\taud1Offset\tcolor\tmode");
  Serial.print("\n\n");
  Serial.print(avg);
  Serial.print("\t");
  Serial.print(aud1Offset);
  Serial.print("\t\t");
  Serial.print(red ? "red" : "green");
  Serial.print("\t");
  Serial.print(audioMode ? "audio" : "control");
  Serial.print("\n\n");
  //}
}
