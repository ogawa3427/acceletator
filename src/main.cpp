#include <M5Unified.h>

// NJL7502L x8 -> ESP32-S3 ADC1 (GPIO1-10 only)
// 3V3 -[47k]- ADC node - sensor(C->E) - GND  (pull-up)
// Modulated reflection sensing: read with LED off (ambient) and on (lit),
// take (ambient - lit) to reject DC ambient (sunlight). Reflection raises
// sensor illumination -> node voltage drops -> (ambient - lit) > 0.
static const int kNumCh = 8;
static const int kAdcPins[kNumCh] = {5, 6, 7, 10, 1, 2, 8, 9};
//                                   CH1 2  3   4  5  6  7  8

// LED drive: 8 LEDs split across 2 free GPIOs, INTERLEAVED by physical position
// and fired alternately (A then B). Benefits: only 4 LEDs (~12mA) lit at once,
// and physically-adjacent LEDs are never on together -> less optical crosstalk.
// Direct drive: GPIO -[75ohm]- LED(+) , LED(-) -> GND. Active HIGH. ~3mA/LED, no MOSFET.
//   kLedPinA (M5-Bus pin15 = G18) -> LEDs of CH1,3,5,7  (even index)
//   kLedPinB (M5-Bus pin16 = G17) -> LEDs of CH2,4,6,8  (odd index)
static const int kLedPinA = 18;
static const int kLedPinB = 17;
static const bool kLedActiveHigh = true;
// channel i -> which LED pin: even index = A, odd index = B
inline int ledPinForCh(int i) { return (i & 1) ? kLedPinB : kLedPinA; }
static const int kSettleUs = 3000;  // wait after toggling LED (RC of 47k * Cnode)
static const int kAvg = 4;          // samples averaged per reading

static const int kBarFullMv = 1500;  // bar full scale for the differential signal

static const uint16_t COL_BG    = TFT_BLACK;
static const uint16_t COL_TRACK = 0x2104;
static const uint16_t COL_TEXT  = 0xC618;
static const uint16_t COL_BASE  = 0x4208;

int g_w = 320, g_h = 240;
int g_top = 18;
int g_bottom = 16;

inline void ledWrite(int pin, bool on) { digitalWrite(pin, (on == kLedActiveHigh) ? HIGH : LOW); }
inline void ledAll(bool on) { ledWrite(kLedPinA, on); ledWrite(kLedPinB, on); }
inline void ledAllOff() { ledAll(false); }

int readMv(int pin) {
  long acc = 0;
  for (int k = 0; k < kAvg; ++k) acc += analogRead(pin);
  int raw = acc / kAvg;
  return (int)(raw * 3300.0f / 4095.0f);
}

void setup() {
  auto config = M5.config();
  M5.begin(config);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  for (int i = 0; i < kNumCh; ++i) {
    pinMode(kAdcPins[i], INPUT);
    analogRead(kAdcPins[i]);
  }

  pinMode(kLedPinA, OUTPUT);
  pinMode(kLedPinB, OUTPUT);
  ledAllOff();

  g_w = M5.Lcd.width();
  g_h = M5.Lcd.height();

  M5.Lcd.fillScreen(COL_BG);
  M5.Lcd.setTextSize(1);
  const int slotW = g_w / kNumCh;
  for (int i = 0; i < kNumCh; ++i) {
    int x = i * slotW + 3;
    M5.Lcd.setTextColor(TFT_WHITE, COL_BG);
    M5.Lcd.setCursor(x, 2);
    M5.Lcd.printf("G%d", kAdcPins[i]);
  }
  M5.Lcd.drawFastHLine(0, g_h - g_bottom, g_w, COL_BASE);
}

void loop() {
  M5.update();

  int ambient[kNumCh];
  int lit[kNumCh];

  // 1) all LEDs off -> ambient
  ledAllOff();
  delayMicroseconds(kSettleUs);
  for (int i = 0; i < kNumCh; ++i) ambient[i] = readMv(kAdcPins[i]);

  // 2) all LEDs on (simultaneous) -> lit
  ledAll(true);
  delayMicroseconds(kSettleUs);
  for (int i = 0; i < kNumCh; ++i) lit[i] = readMv(kAdcPins[i]);
  ledAllOff();

  const int slotW = g_w / kNumCh;
  const int barW = slotW - 6;
  const int plotTop = g_top;
  const int plotBot = g_h - g_bottom;
  const int plotH = plotBot - plotTop;

  M5.Lcd.startWrite();
  for (int i = 0; i < kNumCh; ++i) {
    int reflect = ambient[i] - lit[i];   // reflected-light magnitude (mV)
    if (reflect < 0) reflect = 0;
    int v = reflect;
    if (v > kBarFullMv) v = kBarFullMv;

    int x = i * slotW + (slotW - barW) / 2;
    int barH = (int)((float)v / kBarFullMv * plotH);
    if (barH < 0) barH = 0;
    if (barH > plotH) barH = plotH;
    int y = plotBot - barH;

    if (barH < plotH) {
      M5.Lcd.fillRect(x, plotTop, barW, plotH - barH, COL_TRACK);
    }
    uint16_t col = M5.Lcd.color565(
        map(v, 0, kBarFullMv, 40, 230),
        map(v, 0, kBarFullMv, 230, 60),
        60);
    if (barH > 0) M5.Lcd.fillRect(x, y, barW, barH, col);

    M5.Lcd.setTextColor(COL_TEXT, COL_BG);
    M5.Lcd.setCursor(x, plotBot + 4);
    M5.Lcd.printf("%4d", reflect);

    Serial.printf("%d", reflect);
    Serial.print(i == kNumCh - 1 ? '\n' : ',');
  }
  M5.Lcd.endWrite();

  delay(30);
}
