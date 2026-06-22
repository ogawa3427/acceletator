#include <M5Unified.h>
#include <M5UnitSynth.h>

M5UnitSynth synth;

// ===== TEST MODE (StampS3 bring-up) =====
// 1 = StampS3 plays piano Do-Re-Mi-rest in a loop (sensors/MIDI logic bypassed).
// Set back to 0 to restore normal operation. CoreS3 is unaffected either way.
#define STAMP_TEST 0

#define VOLUME 127

// NJL7502L x8 -> ESP32-S3 ADC1 (GPIO1-10 only)
// 3V3 -[47k]- ADC node - sensor(C->E) - GND  (pull-up)
// Modulated reflection sensing: read with LED off (ambient) and on (lit),
// take (ambient - lit) to reject DC ambient (sunlight). Reflection raises
// sensor illumination -> node voltage drops -> (ambient - lit) > 0.
static const int kNumCh = 8;
// Sensor GPIO per finger-slot index. Physical wiring differs per board,
// so this is overwritten in detectBoard(). Default = CoreS3 prototype.
// slot:  0      1      2     3      4      5      6      7
// finger Lpnk   Rpnk   Lrng  Rrng   Lmid   Rmid   Lidx   Ridx
int kAdcPins[kNumCh] = {5, 6, 7, 10, 1, 2, 8, 9};

// LED drive: 8 LEDs split across 2 free GPIOs, INTERLEAVED by physical position
// and fired alternately (A then B). Benefits: only 4 LEDs (~12mA) lit at once,
// and physically-adjacent LEDs are never on together -> less optical crosstalk.
// Direct drive: GPIO -[75ohm]- LED(+) , LED(-) -> GND. Active HIGH. ~3mA/LED, no MOSFET.
// ACTUAL wiring is paired (AABB), deduced from which channels responded.
// Moved OFF Port C (G17/G18) to free that UART port for the MIDI Unit.
//   kLedPinA -> LEDs of index 0,1,4,5  (G5,G4,G1,G2)
//   kLedPinB -> LEDs of index 2,3,6,7  (G7,G10,G8,G9)
// Pins are board-dependent (set in detectBoard() from M5.getBoard()):
//   CoreS3 : LED A/B = G14/G13, MIDI on Port C (TX17/RX18), has LCD + touch
//   StampS3: LED A/B = G11/G12, MIDI TX=G13/RX=G14, no screen -> button + serial
int g_ledPinA = 14;   // defaults = CoreS3
int g_ledPinB = 13;
static const bool kLedActiveHigh = true;
// finger-slot -> LED group (0=A, 1=B). With kAdcPins in correct physical order,
// both boards use the AABB pattern (each finger's LED is grouped by slot pairs).
int g_ledGroup[kNumCh] = {0, 0, 1, 1, 0, 0, 1, 1};  // AABB (both boards)
inline int ledGroupForCh(int i) { return g_ledGroup[i]; }
static const int kSettleUs = 3000;  // wait after toggling LED (RC of 47k * Cnode)
static const int kAvg = 4;          // samples averaged per reading

static const int kBarFullMv = 1500;  // bar full scale for the differential signal

// ---- ON/OFF detection (hysteresis) ----  TUNE these to your reflect levels
static const int kOnThreshMv  = 250;  // reflect above this -> ON
static const int kOffThreshMv = 150;  // reflect below this -> OFF
// per channel:  hand (0=left,1=right), finger (1=index..4=pinky)
// finger-slot index (GPIO set per board in kAdcPins / detectBoard):
// idx:    0   1    2   3    4   5    6   7
// hand:   L   R    L   R    L   R    L   R
// finger: P   P    R   R    M   M    I   I
static const bool kDetect[kNumCh] = {true, true, true, true, true, true, true, true};
static const int  kHand[kNumCh]   = {0,    1,    0,   1,    0,    1,    0,    1};
static const int  kFinger[kNumCh] = {4,    4,    3,   3,    2,    2,    1,    1};
// display column order: left pinky..index | right index..pinky (keyboard layout)
static const int  kDisp[kNumCh]   = {0, 2, 4, 6,  7, 5, 3, 1};
bool g_on[kNumCh] = {false};      // persisted ON/OFF state
bool g_onPrev[kNumCh] = {false};  // last reported state (for event-based serial)

// ---- MIDI (M5 Unit MIDI/Synth, SAM2695) via Serial2 ----
int g_midiTx = 17;   // board-dependent (set in detectBoard)
int g_midiRx = 18;
static const int kMidiCh = 0;        // MIDI channel 1
// runtime board flags
bool g_hasDisplay = true;            // CoreS3 yes / StampS3 no
bool g_useTouch   = true;            // CoreS3 touch / StampS3 button
static const int kMidiVel   = 100;
static const int kProgTrumpet = 56;  // GM program: Trumpet
static const int kBaseNote  = 60;    // C4

// right-hand finger channel indices (in kAdcPins): trumpet-style binary add
static const int kIdxRIndex  = 7;  // R-index  -> value 1
static const int kIdxRMiddle = 5;  // R-middle -> value 2
static const int kIdxRRing   = 3;  // R-ring   -> value 4
static const int kIdxRPinky  = 1;  // R-pinky  -> octave (+12)
// left-hand modifiers / gate
static const int kIdxLIndex  = 6;  // L-index  -> released = flat
static const int kIdxLMiddle = 4;  // L-middle -> pressed  = sharp
static const int kIdxLPinky  = 0;  // L-pinky  -> currently unused (gate = right-hand value>0)

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
inline void ledAll(bool on) { ledWrite(g_ledPinA, on); ledWrite(g_ledPinB, on); }
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
    measureGroupBracket(g_ledPinA, 0, tmp);
    measureGroupBracket(g_ledPinB, 1, tmp);
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
// Right hand = 4-bit value (index=1, middle=2, ring=4, pinky=8) -> 0..15.
//   value 0      = rest (silence)
//   value 1..15  = major-scale degree 0..14 over TWO octaves from C4
//                  (index-only = value 1 = degree 0 = C4 = Do)
// Left hand accidentals applied on RELEASE: L-middle released = sharp, L-index released = flat.
// Returns -1 for rest.
int computeNote() {
  int value = (g_on[kIdxRIndex] ? 1 : 0)
            + (g_on[kIdxRMiddle] ? 2 : 0)
            + (g_on[kIdxRRing] ? 4 : 0)
            + (g_on[kIdxRPinky] ? 8 : 0);          // 0..15
  if (value == 0) return -1;                       // rest
  int d = value - 1;                               // degree 0..14
  int acc = (g_on[kIdxLMiddle] ? 0 : 1)            // L-middle RELEASED = sharp (+1)
          + (g_on[kIdxLIndex]  ? 0 : -1);          // L-index  RELEASED = flat  (-1)
  int note = kBaseNote + kMajor[d % 7] + 12 * (d / 7) + acc;
  if (note < 0) note = 0;
  if (note > 127) note = 127;
  return note;
}

#if STAMP_TEST
// Bring-up test: piano plays Do-Re-Mi then a rest, repeating. Non-blocking.
void stampMidiTest() {
  static const uint8_t seq[4] = {NOTE_C4, NOTE_D4, NOTE_E4, 0};  // 0 = rest (休符)
  static int idx = 0;
  static uint32_t t = 0;
  static int playing = -1;
  if (millis() - t < 500) return;   // 500 ms per step
  t = millis();
  if (playing >= 0) { synth.setNoteOff(kMidiCh, playing, 0); playing = -1; }
  uint8_t n = seq[idx];
  if (n != 0) { synth.setNoteOn(kMidiCh, n, 100); playing = n; }
  Serial.printf("TEST step=%d note=%d\n", idx, n);
  idx = (idx + 1) % 4;
}

// Diagnostic: per channel show raw(LED off) and response to BOTH LED groups.
// Read the table to tell solder-fail from group-mismatch:
//   raw~3300 & dA~0 & dB~0  -> sensor open (solder fail): pull-up rails high, no light response
//   raw mid  & big delta on its OWN grp     -> OK
//   raw mid  & big delta on the OTHER grp   -> GROUP MISMATCH (wire/grouping wrong)
//   raw mid  & dA~0 & dB~0                  -> that channel's LED dead (LED-side fault)
//   raw~0                                   -> short to GND / different fault
void stampDiag() {
  int off[kNumCh], a[kNumCh], b[kNumCh];
  ledAllOff();                 delayMicroseconds(kSettleUs);
  for (int i = 0; i < kNumCh; ++i) off[i] = readMv(kAdcPins[i]);
  ledWrite(g_ledPinA, true);   delayMicroseconds(kSettleUs);
  for (int i = 0; i < kNumCh; ++i) a[i] = readMv(kAdcPins[i]);
  ledWrite(g_ledPinA, false);
  ledWrite(g_ledPinB, true);   delayMicroseconds(kSettleUs);
  for (int i = 0; i < kNumCh; ++i) b[i] = readMv(kAdcPins[i]);
  ledWrite(g_ledPinB, false);
  for (int i = 0; i < kNumCh; ++i)
    Serial.printf("G%-2d raw=%4d dA=%5d dB=%5d own=%c\n",
                  kAdcPins[i], off[i], off[i] - a[i], off[i] - b[i],
                  ledGroupForCh(i) == 0 ? 'A' : 'B');
}
#endif

// pick board-dependent pins / features at runtime
void detectBoard() {
  switch (M5.getBoard()) {
    case m5::board_t::board_M5StampS3: {
      g_ledPinA = 11; g_ledPinB = 12;   // StampS3
      g_midiTx  = 13; g_midiRx  = 14;
      // measured sensor wiring: physical finger-slot -> GPIO (from bring-up test)
      // slot: Lpnk Rpnk Lrng Rrng Lmid Rmid Lidx Ridx
      static const int stampPins[kNumCh] = {1, 2, 5, 4, 7, 8, 10, 9};
      for (int i = 0; i < kNumCh; ++i) kAdcPins[i] = stampPins[i];
      // with this order, g_ledGroup stays the default AABB (no override needed)
      break;
    }
    case m5::board_t::board_M5StackCoreS3:
    default:
      g_ledPinA = 14; g_ledPinB = 13;   // CoreS3 (default)
      g_midiTx  = 17; g_midiRx  = 18;
      break;
  }
  g_hasDisplay = (M5.Display.width() > 0);  // StampS3 has no screen
  g_useTouch   = M5.Touch.isEnabled();      // CoreS3 touch / else button
}

void setup() {
  auto config = M5.config();
  M5.begin(config);
  detectBoard();

  Serial.begin(115200);   // USB CDC (works via CDC_ON_BOOT, but be explicit)

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  for (int i = 0; i < kNumCh; ++i) {
    pinMode(kAdcPins[i], INPUT);
    analogRead(kAdcPins[i]);
  }

  pinMode(g_ledPinA, OUTPUT);
  pinMode(g_ledPinB, OUTPUT);
  ledAllOff();

  // MIDI Unit (Port C, blue): init SAM2695 via library, set instrument to Trumpet.
  // Give the chip time after begin/reset or setInstrument gets ignored (stays piano).
  synth.begin(&Serial2, UNIT_SYNTH_BAUD, g_midiRx, g_midiTx);
  delay(100);
  synth.reset();
  delay(200);
  synth.setMasterVolume(VOLUME);
  synth.setInstrument(0, kMidiCh, Trumpet);   // enum from M5UnitSynthDef.h (=56)
#if STAMP_TEST
  if (!g_hasDisplay) synth.setInstrument(0, kMidiCh, GrandPiano_1);  // test = piano
#endif

  if (g_hasDisplay) {
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

    M5.Lcd.setTextColor(TFT_YELLOW, COL_BG);
    M5.Lcd.setCursor(40, 110);
    M5.Lcd.print("Calibrating... keep clear");
  }

  // auto-zero each channel at boot (keep sensors clear). Touch/button to redo.
  calibrateBaseline();

  if (g_hasDisplay) M5.Lcd.fillRect(0, 100, g_w, 30, COL_BG);
}

void loop() {
  M5.update();

#if STAMP_TEST
  // StampS3 bring-up: loop piano Do-Re-Mi-rest + dump sensor values once per second.
  if (!g_hasDisplay) {
    stampMidiTest();
    static uint32_t tsens = 0;
    if (millis() - tsens >= 1000) {
      tsens = millis();
      stampDiag();          // per-channel raw + response to both LED groups
    }
    return;
  }
#endif

  // re-zero baselines: touch (CoreS3) or button (StampS3)
  bool recal = g_useTouch ? M5.Touch.getDetail().wasPressed() : M5.BtnA.wasPressed();
  if (recal) calibrateBaseline();

  int reflectMv[kNumCh];

  // Per-group bracketed CDS (OFF->ON->OFF). Group A=index0,1,4,5 ; B=index2,3,6,7.
  measureGroupBracket(g_ledPinA, 0, reflectMv);
  measureGroupBracket(g_ledPinB, 1, reflectMv);

  // subtract per-channel resting baseline (auto-zero)
  for (int i = 0; i < kNumCh; ++i) reflectMv[i] -= g_base[i];

  // update ON/OFF states with hysteresis
  for (int i = 0; i < kNumCh; ++i) {
    if (!kDetect[i]) continue;
    int r = reflectMv[i] < 0 ? 0 : reflectMv[i];
    if (r > kOnThreshMv) g_on[i] = true;
    else if (r < kOffThreshMv) g_on[i] = false;
  }

  // ---- MIDI: monophonic. Right-hand value 0 (no fingers) = rest/silence ----
  int note = computeNote();          // -1 = rest
  bool gate = (note >= 0);           // sounding? (for display/serial)
  if (note >= 0) {
    if (note != g_curNote) {
      if (g_curNote >= 0) midiNoteOff(g_curNote);
      midiNoteOn(note);
      g_curNote = note;
    }
  } else if (g_curNote >= 0) {
    midiNoteOff(g_curNote);
    g_curNote = -1;
  }

  if (g_hasDisplay) {
    // ===== CoreS3: live bar graph + continuous CSV/ST serial =====
    const int slotW = g_w / kNumCh;
    const int barW = slotW - 6;
    const int plotTop = g_top;
    const int plotBot = g_h - g_bottom;
    const int plotH = plotBot - plotTop;

    M5.Lcd.startWrite();
    for (int p = 0; p < kNumCh; ++p) {
      int ch = kDisp[p];
      int reflect = reflectMv[ch];
      if (reflect < 0) reflect = 0;
      int v = reflect;
      if (v > kBarFullMv) v = kBarFullMv;

      int x = p * slotW + (slotW - barW) / 2;
      int barH = (int)((float)v / kBarFullMv * plotH);
      if (barH < 0) barH = 0;
      if (barH > plotH) barH = plotH;
      int y = plotBot - barH;

      if (barH < plotH) M5.Lcd.fillRect(x, plotTop, barW, plotH - barH, COL_TRACK);
      uint16_t col = M5.Lcd.color565(
          map(v, 0, kBarFullMv, 40, 230), map(v, 0, kBarFullMv, 230, 60), 60);
      if (barH > 0) M5.Lcd.fillRect(x, y, barW, barH, col);

      if (kDetect[ch]) {
        uint16_t onCol = (kHand[ch] == 1) ? 0x07FF : 0xFD20;
        M5.Lcd.fillRect(x, 11, barW, 5, g_on[ch] ? onCol : COL_TRACK);
      }
      M5.Lcd.setTextColor(COL_TEXT, COL_BG);
      M5.Lcd.setCursor(x, plotBot + 4);
      M5.Lcd.printf("%4d", reflect);
    }
    char nb[8];
    if (gate) noteName(note, nb); else strcpy(nb, "rest");
    M5.Lcd.fillRect(0, g_h - 13, g_w, 13, COL_BG);
    M5.Lcd.setTextColor(gate ? TFT_GREEN : 0x7BEF, COL_BG);
    M5.Lcd.setCursor(4, g_h - 11);
    M5.Lcd.printf("%s %s  midi=%d", gate ? "PLAY" : "----", nb, note);
    M5.Lcd.endWrite();

    for (int p = 0; p < kNumCh; ++p) {
      int r = reflectMv[kDisp[p]];
      Serial.printf("%d%c", r < 0 ? 0 : r, p == kNumCh - 1 ? '\n' : ',');
    }
    Serial.print("ST ");
    for (int p = 0; p < kNumCh; ++p) {
      Serial.print(g_on[kDisp[p]] ? '1' : '0');
      if (p == 3) Serial.print('|');
    }
    Serial.printf("  NOTE=%d gate=%d\n", note, gate);
  } else {
    // ===== StampS3 (no screen): print ONLY on threshold crossings =====
    static const char fchar[6] = "?IMRP";
    char nb[8];
    if (note >= 0) noteName(note, nb); else strcpy(nb, "rest");
    for (int i = 0; i < kNumCh; ++i) {
      if (!kDetect[i]) continue;
      if (g_on[i] != g_onPrev[i]) {
        Serial.printf("%s %c%c G%-2d  reflect=%4d  NOTE=%s gate=%d\n",
                      g_on[i] ? "ON " : "off",
                      kHand[i] == 1 ? 'R' : 'L', fchar[kFinger[i]],
                      kAdcPins[i], reflectMv[i] < 0 ? 0 : reflectMv[i],
                      nb, gate);
        g_onPrev[i] = g_on[i];
      }
    }
  }

  delay(15);
}
