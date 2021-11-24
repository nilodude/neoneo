#include <Adafruit_NeoPixel.h>

#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define SW1 2
#define SW2 4
#define SW3 3
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
boolean audioMode2 = LOW;
boolean audioMode3 = LOW;
long aux1 = 0;
long aux2 = 0;
long aux3 = 0;
long led1 = 0;
long led2 = 0;
long led3 = 0;

float dropFactor = .89;

struct Map {
  float dbValue;
  long ledNumber;
};

//con el oscilador a pelo deberia encenderse hasta el led 9
const Map lut[] = {
  { -30.5, 1},  { -30, 2},  { -30, 3},  { -26, 4},  { -22, 5}, { -21, 6},    { -20, 7 },    { -19, 8 },   { -18, 9},  { -17, 10},
  { -16, 11}, { -15, 12}, { -14, 13}, { -12, 14},  { -10, 15},  { -8, 16},   { -6, 17}, { -4, 18},  { -2, 19},  { -0, 20}
};


Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

uint32_t red = strip.Color(255, 0, 0);
uint32_t green = strip.Color(0, 255, 0);

void setup() {
  Serial.begin(9600);
  strip.begin();
  strip.show();
  strip.setBrightness(2);
  audioMode1 = digitalRead(SW1);
  audioMode2 = digitalRead(SW2);
  audioMode3 = digitalRead(SW3);
}

void loop() {
  audNorm1 = measure(IN1, audioMode1, aux1);
  audNorm2 = measure(IN2, audioMode2, aux2);
  audNorm3 = measure(IN3, audioMode3, aux3);

  Serial.println("");

  colorWipe();

  audioMode1 = digitalRead(SW1);
  audioMode2 = digitalRead(SW2);
  audioMode3 = digitalRead(SW3);

  aux1 = audNorm1;
  aux2 = audNorm2;
  aux3 = audNorm3;
}

long measure(int channel, boolean audioMode, long aux) {
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

  printValues(channel, audioMode, adc, dB, audNorm);

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
    controlWipe(audNorm1, OFF1);
  }
  if (audioMode2) { // AUDIO MODE
    audioWipe(audNorm2, OFF2);
  } else { // CONTROL MODE
    controlWipe(audNorm2, OFF2);
  }
  if (audioMode3) { // AUDIO MODE
    audioWipe(audNorm3, OFF3);
  } else { // CONTROL MODE
    controlWipe(audNorm3, OFF3);
  }
  strip.show();
}

void audioWipe(int value, int offset) {
  for (int i = 1 + offset; i <= value + offset; i++)
    strip.setPixelColor(i - 1, greenRedFade(i - offset));
}

void controlWipe(int value, int offset) {
  if (value + offset < 0 + offset) {
    for (int i = 20 + offset; i > 20 - abs(value) + offset; i--)
      strip.setPixelColor(i - 1, red);
  } else {
    for (int i = 1 + offset; i <= value + 1 + offset; i++)
      strip.setPixelColor(i - 1, green);
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
  if (led < aux)
    led = aux * dropFactor;

  return led;
}

uint32_t greenRedFade(long i) {
  int r = min(255, i * 10);
  int g = max(0, 90);
  return i > 15 ? i == 20 ? red : strip.Color(r , g , 0) : green;
}

void printValues(int channel, boolean audioMode, long adc, float dB, long audNorm) {
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
  Serial.print(audioMode ? "aud" : "ctl");
  Serial.print("\t");
  Serial.print(adc);
  Serial.print("\t");
  //  Serial.print(rms);
  //  Serial.print("\t");
  Serial.print(dB);
  Serial.print(" ");
  Serial.print(audNorm);
}
