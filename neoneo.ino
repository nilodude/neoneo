

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif


#define LED_PIN    6

#define LED_COUNT 60

long aud1, aud1Norm = 0;
boolean red = HIGH;
boolean audioMode = LOW;
long aux1 = 0;
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);


// setup() function -- runs once at startup --------------------------------

void setup() {
  Serial.begin(9600);
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(10); // Set BRIGHTNESS to about 1/5 (max = 255)
  audioMode = digitalRead(A3);
}


// loop() function -- runs repeatedly as long as board is on ---------------

void loop() {
  aud1 = 41L * analogRead(A2) / 1024L;
  aud1Norm = audioMode ? aud1 : aud1 - 20;
  red = audioMode ? aud1Norm > 20 : aud1Norm < 0;

  colorWipe(abs(aud1Norm), red, audioMode, 2); // Red
  
  if (aux1 != aud1) {
    printTable();
  }
  aux1 = aud1;
  audioMode = digitalRead(A3);
  strip.clear();
}


void colorWipe(long value, boolean red, boolean audioMode, int wait) {
  uint32_t color = red ? strip.Color(255, 0, 0) : strip.Color(0, 255, 0);

  if (red) { //RED
    for (int i = 20; i > 20 -1  - value; i--) {
      strip.setPixelColor(i - 1, color);
    }
    strip.show();
    
  } else { //GREEN
    for (int i = 1; i <= value; i++) {
      strip.setPixelColor(i - 1, color);
    }
    strip.show();
  }
  
}
void printTable() {
  if (aud1 != aux1) {
    Serial.print("aud1\taud1Norm\tcolor\tmode");
    Serial.print("\n\n");
    Serial.print(aud1);
    Serial.print("\t");
    Serial.print(aud1Norm);
    Serial.print("\t\t");
    Serial.print(red ? "red" : "green");
    Serial.print("\t");
    Serial.print(audioMode ? "audio" : "control");
    Serial.print("\n\n");
  }
}
