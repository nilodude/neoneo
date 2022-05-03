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
#define NUM_SAMPLES_CTRL (1)
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

long start = 0;
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
  strip.begin();
  strip.show();
  strip.setBrightness(4);
  if (startup) {
    startUpAnimation();
  }
  measureMode();
}

void loop() {

  audNorm1 = measureSignal(IN1, audioMode1, aux1, controlSign1);
  audNorm2 = measureSignal(IN2, audioMode2, aux2, controlSign2);
  audNorm3 = measureSignal(IN3, audioMode3, aux3, controlSign3);

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
  long adc = 0, amp = 0, rms = 0, audNorm = 0, mean = 0;
  float dB = 0;
  int numsamples = audioMode ? NUM_SAMPLES : NUM_SAMPLES_CTRL;
  for (int i = 0; i < numsamples; i++)  {
    adc = 1022 - analoggRead(channel) + 2;
    amp = abs(adc - MEAN);
    rms += (long(amp) * amp);
    mean += adc;
  }
  mean /= NUM_SAMPLES_CTRL;
  rms /= numsamples;
  dB = 20.0 * log10(sqrt(rms) / MEAN);

  if (audioMode) {
    audNorm = db2led(dB, aux);
  } else {
    audNorm = (40L * adc / 1024L) - 20;
  }
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
  if (audioMode1) {
    audioWipe(audNorm1, OFF1);
  } else {
    controlWipe(audNorm1, OFF1, controlSign1);
  }
  if (audioMode2) {
    audioWipe(audNorm2, OFF2);
  } else {
    controlWipe(audNorm2, OFF2, controlSign2);
  }
  if (audioMode3) {
    audioWipe(audNorm3, OFF3);
  } else {
    controlWipe(audNorm3, OFF3, controlSign3);
  }

  strip.show();

}

void audioWipe(int value, int offset) {
  int i = 1 + offset;
  while (i <= 20 + offset) {
    if (i <= value + offset) {
      strip.setPixelColor(i - 1, greenRedFade(i - offset));
    } else {
      strip.setPixelColor(i - 1, 0);
    }
    i++;
  }
}

void controlWipe(int value, int offset, boolean controlSign) {
  if (value + offset < 0 + offset) { //  NEGATIVE, RED
    //    for (int i = 20 + offset; i > offset; i--) {
    //      if (i > 20 - abs(value) + offset) {
    //        strip.setPixelColor(i - 1, red);
    //      } else {
    //        strip.setPixelColor(i - 1, 0);
    //      }
    //    }

    strip.fill(0, offset, 20 - abs(value) + offset);
    strip.fill(red, 20 - abs(value) + offset + 1, 20 + offset);
  } else {                          //  POSITIVE, GREEN
    for (int i = 1 + offset; i <= 20 + offset; i++)
      if (i <= value + offset) {
        if (i > 2 + offset) {
          strip.setPixelColor(i - 3, blue);
        }
        strip.setPixelColor(i - 2, strip.gamma32(strip.ColorHSV(65536 / 3, 255, 210)));
        strip.setPixelColor(i - 1, strip.gamma32(strip.ColorHSV(65536 / 3, 255, 140)));
      } else {
        strip.setPixelColor(i - 1, 0);
      }
  }
  strip.setPixelColor(offset, controlSign ? green : red);
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
  if (led < aux) {
    if (led < 18 && led > 8) {
      led = ceil(aux * dropFactor);
    } else if (led >= 18) {
      led = aux * 0.95;
    } else {
      led = aux * dropFactor;
    }
  }
  return led;
}

uint32_t greenRedFade(long i) {
  int r = min(255, i * 10);
  int g = 90;
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
