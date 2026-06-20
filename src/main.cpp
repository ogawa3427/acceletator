#include <M5Unified.h>
#include <M5UnitSynth.h>

M5UnitSynth synth;

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
// ACTUAL wiring is paired (AABB), deduced from which channels responded.
// Moved OFF Port C (G17/G18) to free that UART port for the MIDI Unit.
//   kLedPinA -> LEDs of index 0,1,4,5  (G5,G6,G1,G2)
//   kLedPinB -> LEDs of index 2,3,6,7  (G7,G10,G8,G9)
// NOTE: swapped to 14/13 (vs 13/14) — LED A/B wires were crossed when moved to G13/G14.
static const int kLedPinA = 14;
static const int kLedPinB = 13;
static const bool kLedActiveHigh = true;
// channel i -> LED group: pairs AABB -> 0 = A, 1 = B
inline int ledGroupForCh(int i) { return (i >> 1) & 1; }
static const int kSettleUs = 3000;  // wait after toggling LED (RC of 47k * Cnode)
static const int kAvg = 4;          // samples averaged per reading

static const int kBarFullMv = 1500;  // bar full scale for the differential signal

// ---- ON/OFF detection (hysteresis) ----  TUNE these to your reflect levels
static const int kOnThreshMv  = 250;  // reflect above this -> ON
static const int kOffThreshMv = 150;  // reflect below this -> OFF
// per channel:  hand (0=left,1=right), finger (1=index..4=pinky)
// idx:    0   1    2   3    4   5    6   7
// GPIO:   G5  G6   G7  G10  G1  G2   G8  G9
// hand:   L   R    L   R    L   R    L   R
// finger: P   P    R   R    M   M    I   I
static const bool kDetect[kNumCh] = {true, true, true, true, true, true, true, true};
static const int  kHand[kNumCh]   = {0,    1,    0,   1,    0,    1,    0,    1};
static const int  kFinger[kNumCh] = {4,    4,    3,   3,    2,    2,    1,    1};
// display column order: left pinky..index | right index..pinky (keyboard layout)
static const int  kDisp[kNumCh]   = {0, 2, 4, 6,  7, 5, 3, 1};
bool g_on[kNumCh] = {false};  // persisted ON/OFF state

// ---- MIDI (M5 Unit MIDI/Synth, SAM2695) on Port C (blue) via Serial2 ----
static const int kMidiTxPin = 17;   // Port C TX (core -> unit). swap with 18 if silent
static const int kMidiRxPin = 18;   // unused for output-only MIDI
static const int kMidiBaud  = 31250;
static const int kMidiCh    = 0;     // MIDI channel 1
static const int kMidiVel   = 100;
static const int kProgTrumpet = 56;  // GM program: Trumpet
static const int kBaseNote  = 60;    // C4

// right-hand finger channel indices (in kAdcPins): trumpet-style binary add
static const int kIdxRIndex  = 7;  // G9  -> value 1
static const int kIdxRMiddle = 5;  // G2  -> value 2
static const int kIdxRRing   = 3;  // G10 -> value 4
static const int kIdxRPinky  = 1;  // G6  -> octave (+12)
// left-hand modifiers / gate
static const int kIdxLIndex  = 6;  // G8  -> released = flat
static const int kIdxLMiddle = 4;  // G1  -> pressed  = sharp
static const int kIdxLPinky  = 0;  // G5  -> note gate (sound while held)

// C major scale semitone offsets for degree 0..7
static const int kMajor[8] = {0, 2, 4, 5, 7, 9, 11, 12};
int g_curNote = -1;  // currently sounding note (-1 = none)
int  g_base[kNumCh] = {0};     // per-channel resting baseline (auto-zero)

static const uint16_t COL_BG    = TFT_BLACK;
static const uint16_t COL_TRACK = 0x2104;
static const uint16_t COL_TEXT  = 0xC618;
static const uint16_t COL_BASE  = 0x4208;

int g_w = 320, g_h = 240;
int g_top = 18;
int g_bottom = 30;  // room for mV row + note readout

inline void ledWrite(int pin, bool on) { digitalWrite(pin, (on == kLedActiveHigh) ? HIGH : LOW); }
inline void ledAll(bool on) { ledWrite(kLedPinA, on); ledWrite(kLedPinB, on); }
inline void ledAllOff() { ledAll(false); }

int readMv(int pin) {
  long acc = 0;
  for (int k = 0; k < kAvg; ++k) acc += analogRead(pin);
  int raw = acc / kAvg;
  return (int)(raw * 3300.0f / 4095.0f);
}

// Bracketed CDS for one LED group: OFF -> ON -> OFF, time-adjacent so a
// flickering ambient (100/120Hz mains light) and linear drift cancel.
// reflect = (off_before + off_after)/2 - on  (>=0 when reflection present).
void measureGroupBracket(int ledPin, int group, int* reflect) {
  int ob[kNumCh], on[kNumCh], oa[kNumCh];
  ledAllOff();
  delayMicroseconds(kSettleUs);
  for (int i = 0; i < kNumCh; ++i) if (ledGroupForCh(i) == group) ob[i] = readMv(kAdcPins[i]);

  ledWrite(ledPin, true);
  delayMicroseconds(kSettleUs);
  for (int i = 0; i < kNumCh; ++i) if (ledGroupForCh(i) == group) on[i] = readMv(kAdcPins[i]);

  ledWrite(ledPin, false);
  delayMicroseconds(kSettleUs);
  for (int i = 0; i < kNumCh; ++i) if (ledGroupForCh(i) == group) oa[i] = readMv(kAdcPins[i]);

  for (int i = 0; i < kNumCh; ++i)
    if (ledGroupForCh(i) == group) reflect[i] = (ob[i] + oa[i]) / 2 - on[i];
}

// Capture each channel's resting reflect (no finger present) as its zero point.
// Removes per-channel offsets (e.g. G8's ~250mV light leak) so one threshold fits all.
void calibrateBaseline() {
  const int N = 16;
  long acc[kNumCh] = {0};
  int tmp[kNumCh];
  for (int n = 0; n < N; ++n) {
    measureGroupBracket(kLedPinA, 0, tmp);
    measureGroupBracket(kLedPinB, 1, tmp);
    for (int i = 0; i < kNumCh; ++i) acc[i] += tmp[i];
  }
  for (int i = 0; i < kNumCh; ++i) g_base[i] = acc[i] / N;
}

// ---- MIDI via M5Unit-Synth library (SAM2695) ----
inline void midiNoteOn(int n)  { synth.setNoteOn(kMidiCh, n, kMidiVel); }
inline void midiNoteOff(int n) { synth.setNoteOff(kMidiCh, n, 0); }

void noteName(int n, char* buf) {
  static const char* nm[12] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
  sprintf(buf, "%s%d", nm[n % 12], n / 12 - 1);
}

// compute MIDI note from current finger states (-1 if no valid gate)
int computeNote() {
  int degree = (g_on[kIdxRIndex] ? 1 : 0)
             + (g_on[kIdxRMiddle] ? 2 : 0)
             + (g_on[kIdxRRing] ? 4 : 0);          // 0..7
  int oct = g_on[kIdxRPinky] ? 12 : 0;             // pinky = octave up
  int acc = (g_on[kIdxLMiddle] ? 1 : 0)            // left middle pressed = sharp
          + (g_on[kIdxLIndex] ? 0 : -1);           // left index released = flat
  int note = kBaseNote + kMajor[degree] + oct + acc;
  if (note < 0) note = 0;
  if (note > 127) note = 127;
  return note;
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

  // MIDI Unit (Port C, blue): init SAM2695 via library, set instrument to Trumpet.
  // Give the chip time after begin/reset or setInstrument gets ignored (stays piano).
  synth.begin(&Serial2, UNIT_SYNTH_BAUD, kMidiRxPin, kMidiTxPin);
  delay(100);
  synth.reset();
  delay(200);
  synth.setMasterVolume(127);
  synth.setInstrument(0, kMidiCh, Trumpet);   // enum from M5UnitSynthDef.h (=56)

  g_w = M5.Lcd.width();
  g_h = M5.Lcd.height();

  M5.Lcd.fillScreen(COL_BG);
  M5.Lcd.setTextSize(1);
  const int slotW = g_w / kNumCh;
  const char fchar[5] = {'?', 'I', 'M', 'R', 'P'};  // finger initials
  for (int p = 0; p < kNumCh; ++p) {
    int ch = kDisp[p];
    int x = p * slotW + 3;
    uint16_t hc = (kHand[ch] == 1) ? 0x07FF : 0xFD20;  // right=cyan, left=orange
    M5.Lcd.setTextColor(hc, COL_BG);
    M5.Lcd.setCursor(x, 2);
    M5.Lcd.printf("%c%d", fchar[kFinger[ch]], kAdcPins[ch]);
  }
  M5.Lcd.drawFastVLine(4 * slotW - 1, 0, g_h, COL_BASE);  // L | R hand divider
  M5.Lcd.drawFastHLine(0, g_h - g_bottom, g_w, COL_BASE);

  // auto-zero each channel at boot (keep sensors clear). Touch screen to redo.
  M5.Lcd.setTextColor(TFT_YELLOW, COL_BG);
  M5.Lcd.setCursor(40, 110);
  M5.Lcd.print("Calibrating... keep clear");
  calibrateBaseline();
  M5.Lcd.fillRect(0, 100, g_w, 30, COL_BG);
}

void loop() {
  M5.update();

  // touch screen to re-zero baselines
  if (M5.Touch.getDetail().wasPressed()) calibrateBaseline();

  int reflectMv[kNumCh];

  // Per-group bracketed CDS (OFF->ON->OFF). Group A=index0,1,4,5 ; B=index2,3,6,7.
  measureGroupBracket(kLedPinA, 0, reflectMv);
  measureGroupBracket(kLedPinB, 1, reflectMv);

  // subtract per-channel resting baseline (auto-zero)
  for (int i = 0; i < kNumCh; ++i) reflectMv[i] -= g_base[i];

  // update ON/OFF states with hysteresis
  for (int i = 0; i < kNumCh; ++i) {
    if (!kDetect[i]) continue;
    int r = reflectMv[i] < 0 ? 0 : reflectMv[i];
    if (r > kOnThreshMv) g_on[i] = true;
    else if (r < kOffThreshMv) g_on[i] = false;
  }

  // ---- MIDI: monophonic, gated by left pinky (G5) ----
  bool gate = g_on[kIdxLPinky];
  int note = computeNote();
  if (gate) {
    if (note != g_curNote) {
      if (g_curNote >= 0) midiNoteOff(g_curNote);
      midiNoteOn(note);
      g_curNote = note;
    }
  } else if (g_curNote >= 0) {
    midiNoteOff(g_curNote);
    g_curNote = -1;
  }

  const int slotW = g_w / kNumCh;
  const int barW = slotW - 6;
  const int plotTop = g_top;
  const int plotBot = g_h - g_bottom;
  const int plotH = plotBot - plotTop;

  M5.Lcd.startWrite();
  for (int p = 0; p < kNumCh; ++p) {
    int ch = kDisp[p];                   // data channel for this display column
    int reflect = reflectMv[ch];         // reflected-light magnitude (mV)
    if (reflect < 0) reflect = 0;
    int v = reflect;
    if (v > kBarFullMv) v = kBarFullMv;

    int x = p * slotW + (slotW - barW) / 2;
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

    // ON/OFF indicator strip at top: right=cyan, left=orange
    if (kDetect[ch]) {
      uint16_t onCol = (kHand[ch] == 1) ? 0x07FF : 0xFD20;
      M5.Lcd.fillRect(x, 11, barW, 5, g_on[ch] ? onCol : COL_TRACK);
    }

    M5.Lcd.setTextColor(COL_TEXT, COL_BG);
    M5.Lcd.setCursor(x, plotBot + 4);
    M5.Lcd.printf("%4d", reflect);
  }

  // note readout at the very bottom (note name + gate state)
  {
    char nb[8];
    noteName(note, nb);
    M5.Lcd.fillRect(0, g_h - 13, g_w, 13, COL_BG);
    M5.Lcd.setTextColor(gate ? TFT_GREEN : 0x7BEF, COL_BG);
    M5.Lcd.setCursor(4, g_h - 11);
    M5.Lcd.printf("%s %s  midi=%d", gate ? "PLAY" : "----", nb, note);
  }
  M5.Lcd.endWrite();

  // reflect CSV in display order (for tuning/plotting)
  for (int p = 0; p < kNumCh; ++p) {
    int r = reflectMv[kDisp[p]];
    Serial.printf("%d%c", r < 0 ? 0 : r, p == kNumCh - 1 ? '\n' : ',');
  }

  // binary state line in display order: left (P R M I) | right (I M R P)
  Serial.print("ST ");
  for (int p = 0; p < kNumCh; ++p) {
    Serial.print(g_on[kDisp[p]] ? '1' : '0');
    if (p == 3) Serial.print('|');
  }
  Serial.printf("  NOTE=%d gate=%d\n", note, gate);

  delay(15);
}
