#include <Adafruit_NeoPixel.h>

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
#define NUM_SAMPLES (64)

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

long audNorm1 = 0;
long audNorm2 = 0;
long audNorm3 = 0;

boolean audioMode1 = LOW;
boolean controlSign1 = LOW;
boolean audioMode2 = LOW;
boolean controlSign2 = LOW;
boolean audioMode3 = LOW;
boolean controlSign3 = LOW;

long aux1 = 0;
long aux2 = 0;
long aux3 = 0;
long led1 = 0;
long led2 = 0;
long led3 = 0;

boolean startup = HIGH;

float dropFactor = .89;

struct Map {
  float dbValue;
  long ledNumber;
};

//con el oscilador a pelo deberia encenderse hasta el led 15, con A = oscilador, B=2, C=0 --> A*B+C debe encender todo (oscilador *2)
const Map lut[] = {
  { -30.5, 1},  { -30, 2},  { -30, 3},  { -26, 4},  { -25, 5}, { -24, 6},    { -23, 7 },    { -22, 8 },   { -21, 9},  { -20, 10},
  { -19, 11}, { -18, 12}, { -16, 13}, { -14, 14},  { -12, 15},  { -10, 16},   { -8, 17}, { -6, 18},  { -4, 19},  { -2, 20}
};

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

uint32_t red = strip.Color(255, 0, 0);
uint32_t green = strip.Color(0, 255, 0);
uint32_t blue = strip.Color(0, 0, 255);
uint32_t snakeColor = strip.Color(255, 30, 255);

void setup() {
  Serial.begin(9600);
  strip.begin();
  strip.show();
  strip.setBrightness(2);

  measureMode();
}

void loop() {

  if (startup) {
    startUpAnimation();
  }
  audNorm1 = measureSignal(IN1, audioMode1, aux1, controlSign1);
  audNorm2 = measureSignal(IN2, audioMode2, aux2, controlSign2);
  audNorm3 = measureSignal(IN3, audioMode3, aux3, controlSign3);

  Serial.println("");

  colorWipe();

  measureMode();

  aux1 = audNorm1;
  aux2 = audNorm2;
  aux3 = audNorm3;
}

void measureMode() {
  long raw1 = analoggRead(SW1);
  audioMode1 = raw1 > 513;
  controlSign1 = raw1 > 1;

  long raw2 = analoggRead(SW2);
  audioMode2 = raw2 > 513;
  controlSign2 = raw2 > 1;

  long raw3 = analoggRead(SW3);
  audioMode3 = raw3 > 513;
  controlSign3 = raw3 > 1;
}


long measureSignal(int channel, boolean audioMode, long aux, boolean controlSign) {
  long adc = 0, amp = 0, rms = 0, audNorm = 0;
  float dB = 0;
  int numsamples = audioMode ? NUM_SAMPLES : 1;
  for (int i = 0; i < numsamples; i++)  {
    adc = 1022 - analoggRead(channel) + 2;
    amp = abs(adc - MEAN);
    rms += (long(amp) * amp);
  }
  rms /= numsamples;
  dB = 20.0 * log10(sqrt(rms) / MEAN);

  if (audioMode) {
    //AUDIO MODE
    audNorm = db2led(dB, aux);
  } else {
    //CONTROL MODE
    audNorm = (40L * adc / 1024L) - 20;
  }

  //printValues(channel, audioMode, controlSign, adc, dB, audNorm);

  return audNorm > 40 ? 40 : audNorm;
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

void colorWipe() {
  strip.clear();

  if (audioMode1) { // AUDIO MODE
    audioWipe(audNorm1, OFF1);
  } else { // CONTROL MODE
    controlWipe(audNorm1, OFF1, controlSign1);
  }
  if (audioMode2) { // AUDIO MODE
    audioWipe(audNorm2, OFF2);
  } else { // CONTROL MODE
    controlWipe(audNorm2, OFF2, controlSign2);
  }
  if (audioMode3) { // AUDIO MODE
    audioWipe(audNorm3, OFF3);
  } else { // CONTROL MODE
    controlWipe(audNorm3, OFF3, controlSign3);
  }
  strip.show();
}

void audioWipe(int value, int offset) {
  for (int i = 1 + offset; i <= value + offset; i++)
    strip.setPixelColor(i - 1, greenRedFade(i - offset));
}

void controlWipe(int value, int offset, boolean controlSign) {
  if (value + offset < 0 + offset) {
    for (int i = 20 + offset; i > 20 - abs(value) + offset; i--)
      strip.setPixelColor(i - 1, red);
  } else {
    for (int i = 1 + offset; i <= value + 1 + offset; i++) {
      if (value + offset == 0 + offset) {
        strip.setPixelColor(i - 1, controlSign ? green : red);
      } else if (i > 1 + offset) {
         if(i != 21 + offset){
          strip.setPixelColor(i - 1, green);
         }
      }
    }
    if (value + offset == 20 + offset) {
      strip.setPixelColor(offset, green);
    }
  }
}

long db2led(float db, long aux) {
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
  if (led < aux){
    if(led < 18 && led > 8){
      led = ceil(aux * dropFactor);
    }else if(led >=18){
      led = aux * 0.95;
    }else{
      led = aux * dropFactor;
    }
  }
  
  return led;
}

uint32_t greenRedFade(long i) {
  int r = min(255, i * 10);
  int g = max(0, 90);
  return i > 15 ? i == 20 ? red : strip.Color(r , g , 0) : green;
}

void printValues(int channel, boolean audioMode, boolean controlSign, long adc, float dB, long audNorm) {
  switch (channel) {
    case 16:
      Serial.print("|INPUT1");
      break;
    case 14:
      Serial.print("\t|INPUT2");
      break;
    case 15:
      Serial.print("\t|INPUT3");
      break;
    default:
      Serial.print("");
  }

  Serial.print(" ");
  if (audioMode) {
    //Serial.print("aud");
  } else {
    //Serial.print("ctl");
    //Serial.print(controlSign ? "+" : "-" );
  }
  Serial.print("\t");
  Serial.print(adc);
  //Serial.print("\t");
  //  Serial.print(rms);
  //  Serial.print("\t");
  //Serial.print(dB);
  //Serial.print(" ");
  //Serial.print(audNorm);
}

void startUpAnimation() {
  //rainbow(1);
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

void rainbow(int wait) {
  for (long firstPixelHue = 0; firstPixelHue < 5 * 65536; firstPixelHue += 256) {
    for (int i = 0; i < strip.numPixels(); i++) { // For each pixel in strip...
      int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
    }
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
    startup = LOW;
    strip.clear();
  }
}
