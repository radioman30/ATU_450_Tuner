/*
 * ATU-450 Tuner Controller v4.0
 * Optimizat pentru Arduino MEGA 2560
 *
 * Hardware:
 *   - Display OLED 1.3" SH1106 (I2C pe pinii 20=SDA, 21=SCL)
 *   - Rotary Encoder (INT pe pinii 2 si 3)
 *   - BU2152FS shift register -> 24 relee (8L + 8C1 + 8C2)
 *   - SWR bridge pe A0 (FWD) si A1 (REV)
 *   - 2 butoane extra (BTN_1=5, BTN_2=6)
 *
 * MEGA 2560 - pini SPI software:
 *   SDATA -> pin 11  (poate fi orice pin digital)
 *   SCLK  -> pin 13
 *   LE    -> pin 10
 *
 * EEPROM:
 *   - Salveaza ultimele setari folosite la power-off
 *   - Salveaza pana la 20 de memori utilizator (banda + L + C1 + C2)
 *
 * Auto-tune v2:
 *   - Hill-climbing complet pe 3 axe (L, C1, C2)
 *   - Iteratie multipla pana la convergenta sau timeout
 *   - Porneste din presetul benzii curente
 *
 * Comenzi seriale:
 *   TEST, OFF, MODE, TUNE, SWR, AUTOTUNE, INFO
 *   SAVE <slot 0-19>    - salveaza setarile curente in memorie
 *   LOAD <slot 0-19>    - incarca memorie
 *   MEM                 - listeaza toate memoriile salvate
 *   LCC <L> <C1> <C2>  - control direct (0-255)
 *   PRESET <0-10>       - seteaza preset banda direct
 *   RESET               - sterge toate memoriile EEPROM
 */

#include <U8g2lib.h>
#include <Wire.h>
#include <EEPROM.h>

// ============================================================
// DISPLAY - Mega 2560: I2C pe 20(SDA) 21(SCL)
// ============================================================
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ============================================================
// PINI
// ============================================================
#define SDATA_PIN   11
#define SCLK_PIN    13
#define LE_TU_PIN   10

#define ENC_CLK     2     // INT0 pe Mega
#define ENC_DT      3     // INT1 pe Mega
#define ENC_SW      4

#define BTN_1       5     // Schimbare mod
#define BTN_2       6     // Actiune rapida / OFF

#define FWD_PIN     A0
#define REV_PIN     A1

// ============================================================
// CONSTANTE
// ============================================================
#define SWR_ALERT_THRESHOLD   2.0f
#define SWR_GOOD_THRESHOLD    1.5f
#define SWR_SAMPLES           12
#define AUTOTUNE_TIMEOUT      8000UL   // 8 secunde
#define AUTOTUNE_MIN_FWD      40       // ADC minim pentru semnal valid
#define AUTOTUNE_TARGET_SWR   1.15f    // Stop daca SWR sub acest prag

// EEPROM layout
#define EEPROM_MAGIC          0xA5
#define EEPROM_ADDR_MAGIC     0
#define EEPROM_ADDR_LAST_L    1
#define EEPROM_ADDR_LAST_C1   2
#define EEPROM_ADDR_LAST_C2   3
#define EEPROM_ADDR_LAST_PRE  4
#define EEPROM_ADDR_MEMORIES  10      // 20 memorii x 5 bytes = 100 bytes
#define MAX_MEMORIES          20
#define MEMORY_SIZE           5       // bytes per memorie: valid + L + C1 + C2 + preset

// ============================================================
// STRUCTURI
// ============================================================
struct Preset {
  const char* name;
  uint8_t     inductance;
  uint8_t     capacitor1;
  uint8_t     capacitor2;
  float       frequency;
};

struct Memory {
  bool    valid;
  uint8_t L;
  uint8_t C1;
  uint8_t C2;
  uint8_t presetIdx;
};

// ============================================================
// PRESETURI BANDA
// ============================================================
const Preset presets[] PROGMEM = {
  {"160m", 0b11111111, 0b11110000, 0b00000000, 1.900f},
  {"80m",  0b01111111, 0b11111000, 0b00000000, 3.750f},
  {"60m",  0b00111111, 0b01111100, 0b00001000, 5.350f},
  {"40m",  0b00011111, 0b00111110, 0b00011000, 7.150f},
  {"30m",  0b00001111, 0b00011111, 0b00111000, 10.100f},
  {"20m",  0b00000111, 0b00001111, 0b01111000, 14.200f},
  {"17m",  0b00000011, 0b00000111, 0b11110000, 18.100f},
  {"15m",  0b00000001, 0b00000011, 0b11111000, 21.200f},
  {"12m",  0b00000000, 0b00000001, 0b11111100, 24.900f},
  {"10m",  0b00000000, 0b00000000, 0b11111110, 28.500f},
  {"6m",   0b00000000, 0b00000000, 0b01111111, 50.000f}
};
const int NUM_PRESETS = sizeof(presets) / sizeof(Preset);

// Helper pentru citire din PROGMEM
Preset getPreset(int idx) {
  Preset p;
  memcpy_P(&p, &presets[idx], sizeof(Preset));
  return p;
}

// ============================================================
// VARIABILE GLOBALE
// ============================================================

// Encoder (volatile - folosite in ISR)
volatile int  encoderPos    = 0;
volatile int  lastEncoded   = 0;

int  lastEncoderPos  = 0;
int  currentPreset   = 0;

// Moduri
enum Mode {
  MODE_PRESET,
  MODE_MANUAL_L,
  MODE_MANUAL_C1,
  MODE_MANUAL_C2,
  MODE_SWR,
  MODE_MEMORY,
  MODE_TEST,
  MODE_COUNT
};
Mode currentMode = MODE_PRESET;

// Setari manuale
uint8_t manualL  = 0;
uint8_t manualC1 = 0;
uint8_t manualC2 = 0;
bool    tunerActive = false;

// Setari active curente
uint8_t activeL  = 0;
uint8_t activeC1 = 0;
uint8_t activeC2 = 0;

// SWR
float         currentSWR    = 0.0f;
float         minSWR        = 99.0f;
int           fwdRaw        = 0;
int           revRaw        = 0;
bool          swrAlert      = false;
bool          autoTuning    = false;
unsigned long lastSWRRead   = 0;

// Auto-tune
uint8_t bestL  = 0;
uint8_t bestC1 = 0;
uint8_t bestC2 = 0;

// Memorie
int     selectedMemory = 0;
Memory  memories[MAX_MEMORIES];

// ============================================================
// SETUP
// ============================================================
void setup() {
  // Pini relee
  pinMode(SDATA_PIN, OUTPUT);
  pinMode(SCLK_PIN,  OUTPUT);
  pinMode(LE_TU_PIN, OUTPUT);
  digitalWrite(SCLK_PIN,  LOW);
  digitalWrite(LE_TU_PIN, LOW);

  // Encoder si butoane
  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT,  INPUT_PULLUP);
  pinMode(ENC_SW,  INPUT_PULLUP);
  pinMode(BTN_1,   INPUT_PULLUP);
  pinMode(BTN_2,   INPUT_PULLUP);

  // SWR
  pinMode(FWD_PIN, INPUT);
  pinMode(REV_PIN, INPUT);

  // Interrupts encoder pe Mega (INT0=pin2, INT1=pin3)
  attachInterrupt(digitalPinToInterrupt(ENC_CLK), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DT),  updateEncoder, CHANGE);

  Serial.begin(115200);
  Serial.println(F("================================="));
  Serial.println(F(" ATU-450 Controller v4.0"));
  Serial.println(F(" Arduino MEGA 2560"));
  Serial.println(F(" 24 Relays: 8L + 8C1 + 8C2"));
  Serial.println(F(" EEPROM + Hill-Climb AutoTune"));
  Serial.println(F("================================="));

  // Display
  u8g2.begin();
  showSplashScreen();

  // EEPROM - incarca memorii si ultima setare
  loadEEPROM();

  delay(1800);

  // Aplica ultima setare salvata
  setRelays(activeL, activeC1, activeC2);
  if (activeL || activeC1 || activeC2) {
    tunerActive = true;
    Serial.println(F("Restored last settings from EEPROM."));
  }

  updateDisplay();

  Serial.println(F("Ready! Type ? for commands."));
}

// ============================================================
// LOOP
// ============================================================
void loop() {

  // Citire SWR la fiecare 200ms
  if (millis() - lastSWRRead > 200) {
    readSWR();
    lastSWRRead = millis();

    swrAlert = (tunerActive && currentSWR > SWR_ALERT_THRESHOLD && fwdRaw > AUTOTUNE_MIN_FWD);

    if (currentMode == MODE_SWR) {
      updateDisplay();
    } else {
      // Refresh doar statusbar in celelalte moduri
      displayStatusBar();
      u8g2.sendBuffer();
    }
  }

  // Encoder rotatie
  if (encoderPos != lastEncoderPos) {
    handleEncoderRotation();
    lastEncoderPos = encoderPos;
  }

  // Buton encoder
  static unsigned long lastEncBtn = 0;
  if (digitalRead(ENC_SW) == LOW && millis() - lastEncBtn > 250) {
    delay(40);
    if (digitalRead(ENC_SW) == LOW) {
      handleEncoderButton();
      lastEncBtn = millis();
      while (digitalRead(ENC_SW) == LOW);
    }
  }

  // BTN_1 - schimbare mod
  static unsigned long lastBtn1 = 0;
  if (digitalRead(BTN_1) == LOW && millis() - lastBtn1 > 250) {
    delay(40);
    if (digitalRead(BTN_1) == LOW) {
      changeMode();
      lastBtn1 = millis();
      while (digitalRead(BTN_1) == LOW);
    }
  }

  // BTN_2 - actiune rapida / OFF
  static unsigned long lastBtn2 = 0;
  if (digitalRead(BTN_2) == LOW && millis() - lastBtn2 > 250) {
    delay(40);
    if (digitalRead(BTN_2) == LOW) {
      quickAction();
      lastBtn2 = millis();
      while (digitalRead(BTN_2) == LOW);
    }
  }

  // Serial
  if (Serial.available()) {
    handleSerialCommand();
  }
}

// ============================================================
// SWR
// ============================================================

void readSWR() {
  long fwdSum = 0, revSum = 0;
  for (int i = 0; i < SWR_SAMPLES; i++) {
    fwdSum += analogRead(FWD_PIN);
    revSum += analogRead(REV_PIN);
    delayMicroseconds(150);
  }
  fwdRaw = fwdSum / SWR_SAMPLES;
  revRaw = revSum / SWR_SAMPLES;
  currentSWR = calculateSWR(fwdRaw, revRaw);
}

float calculateSWR(int fwd, int rev) {
  if (fwd < 10) return 0.0f;
  float vFwd = (float)fwd / 1023.0f;
  float vRev = (float)rev / 1023.0f;
  if (vFwd <= vRev) return 99.0f;
  float gamma = vRev / vFwd;
  if (gamma >= 1.0f) return 99.0f;
  float swr = (1.0f + gamma) / (1.0f - gamma);
  return (swr > 99.0f) ? 99.0f : swr;
}

// ============================================================
// AUTO-TUNE v2 - Hill Climbing complet pe 3 axe
// ============================================================

void autoTune() {
  if (fwdRaw < AUTOTUNE_MIN_FWD) {
    Serial.println(F("AutoTune: putere FWD insuficienta!"));
    showMessage("NO POWER", "Aplica RF!", 2000);
    return;
  }

  autoTuning = true;
  minSWR     = 99.0f;

  Preset p   = getPreset(currentPreset);
  bestL      = p.inductance;
  bestC1     = p.capacitor1;
  bestC2     = p.capacitor2;

  Serial.println(F("\n========= AUTO-TUNE v2 START ========="));
  Serial.print(F("Banda: ")); Serial.println(p.name);
  Serial.print(F("Punct start -> L=0x")); Serial.print(bestL, HEX);
  Serial.print(F(" C1=0x")); Serial.print(bestC1, HEX);
  Serial.print(F(" C2=0x")); Serial.println(bestC2, HEX);

  unsigned long startTime = millis();
  bool improved = true;
  int  iteration = 0;

  // Masoara SWR initial
  setRelays(bestL, bestC1, bestC2);
  delay(30);
  readSWR();
  minSWR = currentSWR;

  // Hill-climbing: repeta pana nu mai exista imbunatatire sau timeout
  while (improved && (millis() - startTime < AUTOTUNE_TIMEOUT)) {
    improved  = false;
    iteration++;

    Serial.print(F("Iteratie ")); Serial.print(iteration);
    Serial.print(F(" | SWR curent: ")); Serial.println(minSWR, 2);

    // --- Axa L ---
    for (int step = -1; step <= 1; step += 2) {
      if (millis() - startTime > AUTOTUNE_TIMEOUT) break;
      uint8_t testL = bestL + step;
      setRelays(testL, bestC1, bestC2);
      delay(25);
      readSWR();
      showAutoTuneProgress(testL, bestC1, bestC2);
      if (currentSWR < minSWR - 0.02f && currentSWR > 0.0f) {
        minSWR   = currentSWR;
        bestL    = testL;
        improved = true;
      }
      if (minSWR <= AUTOTUNE_TARGET_SWR) break;
    }

    // Gradient descent pe L (pasi mai mari daca departe de target)
    if (minSWR > 2.0f) {
      for (int dL = -8; dL <= 8; dL += 2) {
        if (millis() - startTime > AUTOTUNE_TIMEOUT) break;
        uint8_t testL = bestL + dL;
        setRelays(testL, bestC1, bestC2);
        delay(25);
        readSWR();
        showAutoTuneProgress(testL, bestC1, bestC2);
        if (currentSWR < minSWR - 0.02f && currentSWR > 0.0f) {
          minSWR   = currentSWR;
          bestL    = testL;
          improved = true;
        }
        if (minSWR <= AUTOTUNE_TARGET_SWR) break;
      }
    }

    if (minSWR <= AUTOTUNE_TARGET_SWR) break;

    // --- Axa C1 ---
    for (int step = -1; step <= 1; step += 2) {
      if (millis() - startTime > AUTOTUNE_TIMEOUT) break;
      uint8_t testC1 = bestC1 + step;
      setRelays(bestL, testC1, bestC2);
      delay(25);
      readSWR();
      showAutoTuneProgress(bestL, testC1, bestC2);
      if (currentSWR < minSWR - 0.02f && currentSWR > 0.0f) {
        minSWR   = currentSWR;
        bestC1   = testC1;
        improved = true;
      }
      if (minSWR <= AUTOTUNE_TARGET_SWR) break;
    }

    if (minSWR > 2.0f) {
      for (int dC1 = -8; dC1 <= 8; dC1 += 2) {
        if (millis() - startTime > AUTOTUNE_TIMEOUT) break;
        uint8_t testC1 = bestC1 + dC1;
        setRelays(bestL, testC1, bestC2);
        delay(25);
        readSWR();
        showAutoTuneProgress(bestL, testC1, bestC2);
        if (currentSWR < minSWR - 0.02f && currentSWR > 0.0f) {
          minSWR   = currentSWR;
          bestC1   = testC1;
          improved = true;
        }
        if (minSWR <= AUTOTUNE_TARGET_SWR) break;
      }
    }

    if (minSWR <= AUTOTUNE_TARGET_SWR) break;

    // --- Axa C2 ---
    for (int step = -1; step <= 1; step += 2) {
      if (millis() - startTime > AUTOTUNE_TIMEOUT) break;
      uint8_t testC2 = bestC2 + step;
      setRelays(bestL, bestC1, testC2);
      delay(25);
      readSWR();
      showAutoTuneProgress(bestL, bestC1, testC2);
      if (currentSWR < minSWR - 0.02f && currentSWR > 0.0f) {
        minSWR   = currentSWR;
        bestC2   = testC2;
        improved = true;
      }
      if (minSWR <= AUTOTUNE_TARGET_SWR) break;
    }

    if (minSWR > 2.0f) {
      for (int dC2 = -8; dC2 <= 8; dC2 += 2) {
        if (millis() - startTime > AUTOTUNE_TIMEOUT) break;
        uint8_t testC2 = bestC2 + dC2;
        setRelays(bestL, bestC1, testC2);
        delay(25);
        readSWR();
        showAutoTuneProgress(bestL, bestC1, testC2);
        if (currentSWR < minSWR - 0.02f && currentSWR > 0.0f) {
          minSWR   = currentSWR;
          bestC2   = testC2;
          improved = true;
        }
        if (minSWR <= AUTOTUNE_TARGET_SWR) break;
      }
    }

    if (minSWR <= AUTOTUNE_TARGET_SWR) break;
  }

  // Aplica rezultatul
  setRelays(bestL, bestC1, bestC2);
  activeL  = bestL;
  activeC1 = bestC1;
  activeC2 = bestC2;
  tunerActive = true;
  autoTuning  = false;

  // Salveaza in EEPROM
  saveLastSettings();

  unsigned long elapsed = millis() - startTime;

  Serial.println(F("========= AUTO-TUNE DONE ========="));
  Serial.print(F("Iteratii: ")); Serial.println(iteration);
  Serial.print(F("Timp: "));    Serial.print(elapsed); Serial.println(F(" ms"));
  Serial.print(F("Best SWR: ")); Serial.println(minSWR, 2);
  Serial.print(F("L=0x"));   Serial.print(bestL,  HEX);
  Serial.print(F(" C1=0x")); Serial.print(bestC1, HEX);
  Serial.print(F(" C2=0x")); Serial.println(bestC2, HEX);
  if (elapsed >= AUTOTUNE_TIMEOUT) Serial.println(F("ATENTIE: timeout atins!"));

  char swrStr[10];
  dtostrf(minSWR, 4, 2, swrStr);
  char msg1[22];
  sprintf(msg1, "SWR: %s", swrStr);
  showMessage("TUNED!", msg1, 2000);
  updateDisplay();
}

void showAutoTuneProgress(uint8_t l, uint8_t c1, uint8_t c2) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.drawStr(10, 12, "AUTO-TUNE...");

  char swrStr[12];
  if (currentSWR >= 10.0f || currentSWR == 0.0f) strcpy(swrStr, ">10");
  else dtostrf(currentSWR, 5, 2, swrStr);

  u8g2.setFont(u8g2_font_ncenB18_tr);
  u8g2.drawStr(20, 36, swrStr);

  char bestStr[8];
  dtostrf(minSWR, 4, 2, bestStr);
  char line[24];
  sprintf(line, "Best:%s", bestStr);
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(5, 50, line);

  // Progress bar (basata pe SWR: 1=plin, 5+=gol)
  int bar = (int)((5.0f - min(currentSWR, 5.0f)) / 4.0f * 80.0f);
  u8g2.drawFrame(44, 50, 82, 6);
  if (bar > 0) u8g2.drawBox(44, 50, bar, 6);

  u8g2.sendBuffer();
}

// ============================================================
// DISPLAY
// ============================================================

void updateDisplay() {
  u8g2.clearBuffer();
  switch (currentMode) {
    case MODE_PRESET:    displayPresetMode();                      break;
    case MODE_MANUAL_L:  displayManualMode("L  (P1-P8)",  manualL);  break;
    case MODE_MANUAL_C1: displayManualMode("C1(P17-24)", manualC1);  break;
    case MODE_MANUAL_C2: displayManualMode("C2 (P9-16)", manualC2);  break;
    case MODE_SWR:       displaySWRMode();                         break;
    case MODE_MEMORY:    displayMemoryMode();                      break;
    case MODE_TEST:      displayTestMode();                        break;
  }
  displayStatusBar();
  u8g2.sendBuffer();
}

void displayPresetMode() {
  Preset p = getPreset(currentPreset);

  u8g2.setFont(u8g2_font_ncenB24_tr);
  int x = (128 - u8g2.getStrWidth(p.name)) / 2;
  u8g2.drawStr(x, 30, p.name);

  char freqStr[16];
  dtostrf(p.frequency, 6, 3, freqStr);
  strcat(freqStr, "MHz");
  u8g2.setFont(u8g2_font_ncenB10_tr);
  x = (128 - u8g2.getStrWidth(freqStr)) / 2;
  u8g2.drawStr(x, 48, freqStr);

  char posStr[8];
  sprintf(posStr, "%d/%d", currentPreset + 1, NUM_PRESETS);
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(100, 10, posStr);

  if (fwdRaw > AUTOTUNE_MIN_FWD) {
    char swrStr[8];
    dtostrf(currentSWR, 4, 1, swrStr);
    u8g2.drawStr(0, 10, swrStr);
  }
}

void displaySWRMode() {
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 10, "SWR METER");

  if (fwdRaw < AUTOTUNE_MIN_FWD) {
    u8g2.setFont(u8g2_font_ncenB12_tr);
    u8g2.drawStr(10, 32, "NO SIGNAL");
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(5, 46, "Aplica RF!");
    return;
  }

  char swrStr[12];
  if (currentSWR >= 10.0f) strcpy(swrStr, ">10.0");
  else dtostrf(currentSWR, 5, 2, swrStr);

  u8g2.setFont(u8g2_font_ncenB18_tr);
  int x = (128 - u8g2.getStrWidth(swrStr)) / 2;
  u8g2.drawStr(x, 34, swrStr);

  // Bar grafic (1.0=minim, 4.0=maxim)
  int barW = (int)((currentSWR - 1.0f) / 3.0f * 100.0f);
  barW = constrain(barW, 0, 100);
  u8g2.drawFrame(14, 37, 102, 7);
  if (barW > 0) {
    if (currentSWR > SWR_ALERT_THRESHOLD) {
      u8g2.drawBox(14, 37, barW, 7);
    } else {
      u8g2.drawBox(14, 37, barW, 7);
    }
  }

  char fwrStr[20];
  sprintf(fwrStr, "F:%d  R:%d", fwdRaw, revRaw);
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(8, 52, fwrStr);

  if (currentSWR <= SWR_GOOD_THRESHOLD)       u8g2.drawStr(82, 10, "GOOD");
  else if (currentSWR <= SWR_ALERT_THRESHOLD) u8g2.drawStr(94, 10, "OK");
  else                                         u8g2.drawStr(78, 10, "HIGH!");

  u8g2.drawStr(5, 63, "[enc]=AUTOTUNE");
}

void displayManualMode(const char* label, uint8_t value) {
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 10, "MANUAL");
  u8g2.drawStr(0, 20, label);

  char hexStr[8];
  sprintf(hexStr, "0x%02X", value);
  u8g2.setFont(u8g2_font_ncenB18_tr);
  u8g2.drawStr(22, 38, hexStr);

  char binStr[12];
  sprintf(binStr, "%d%d%d%d %d%d%d%d",
    (value>>7)&1,(value>>6)&1,(value>>5)&1,(value>>4)&1,
    (value>>3)&1,(value>>2)&1,(value>>1)&1, value&1);
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(18, 50, binStr);

  if (fwdRaw > AUTOTUNE_MIN_FWD) {
    char swrStr[8];
    dtostrf(currentSWR, 4, 1, swrStr);
    u8g2.drawStr(88, 10, swrStr);
  }
}

void displayMemoryMode() {
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 10, "MEMORIES");

  char slotStr[10];
  sprintf(slotStr, "Slot: %02d", selectedMemory);
  u8g2.drawStr(0, 20, slotStr);

  if (memories[selectedMemory].valid) {
    Memory& m = memories[selectedMemory];
    Preset p  = getPreset(m.presetIdx);

    char line1[20];
    sprintf(line1, "Banda: %s", p.name);
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr(5, 32, line1);

    char line2[24];
    sprintf(line2, "L=%02X C1=%02X C2=%02X", m.L, m.C1, m.C2);
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(0, 44, line2);

    u8g2.drawStr(5, 53, "[enc]=LOAD  [btn2]=DEL");
  } else {
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr(15, 36, "-- EMPTY --");
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(5, 50, "[btn2]=SAVE HERE");
  }
}

void displayTestMode() {
  u8g2.setFont(u8g2_font_ncenB18_tr);
  u8g2.drawStr(25, 25, "TEST");
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(5, 38, "24 relays test");
  u8g2.drawStr(5, 50, "[enc]=RUN  [btn2]=SINGLE");
}

void displayStatusBar() {
  u8g2.drawHLine(0, 55, 128);
  u8g2.setFont(u8g2_font_6x10_tr);

  u8g2.drawStr(2, 64, tunerActive ? "ON" : "OFF");

  if (fwdRaw > AUTOTUNE_MIN_FWD) {
    char swrStr[10];
    dtostrf(currentSWR, 3, 1, swrStr);
    char line[12];
    sprintf(line, "S%s", swrStr);

    if (swrAlert) {
      u8g2.setDrawColor(1);
      u8g2.drawBox(24, 56, 40, 9);
      u8g2.setDrawColor(0);
      u8g2.drawStr(26, 64, line);
      u8g2.setDrawColor(1);
    } else {
      u8g2.drawStr(26, 64, line);
    }
  }

  // Indicator mod curent
  const char* modeLabels[] = {"PRE","L","C1","C2","SWR","MEM","TST"};
  if (currentMode < MODE_COUNT) {
    u8g2.drawStr(82, 64, modeLabels[currentMode]);
  }

  // Indicator EEPROM auto-save
  u8g2.drawStr(112, 64, "EE");
}

void showSplashScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB18_tr);
  u8g2.drawStr(10, 22, "ATU-450");
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.drawStr(5, 38, "24 Relay Tuner");
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 50, "8L+8C1+8C2  MEGA2560");
  u8g2.drawStr(30, 62, "v4.0 Hill-Climb");
  u8g2.sendBuffer();
}

void showMessage(const char* line1, const char* line2, int ms) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB18_tr);
  int x = (128 - u8g2.getStrWidth(line1)) / 2;
  u8g2.drawStr(x, 28, line1);
  u8g2.setFont(u8g2_font_ncenB10_tr);
  x = (128 - u8g2.getStrWidth(line2)) / 2;
  u8g2.drawStr(x, 48, line2);
  u8g2.sendBuffer();
  delay(ms);
}

// ============================================================
// ENCODER & BUTOANE
// ============================================================

void updateEncoder() {
  int MSB = digitalRead(ENC_CLK);
  int LSB = digitalRead(ENC_DT);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderPos++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderPos--;
  lastEncoded = encoded;
}

void handleEncoderRotation() {
  int dir = (encoderPos > lastEncoderPos) ? 1 : -1;

  switch (currentMode) {
    case MODE_PRESET:
      currentPreset = (currentPreset + dir + NUM_PRESETS) % NUM_PRESETS;
      break;
    case MODE_MANUAL_L:
      manualL += dir;
      break;
    case MODE_MANUAL_C1:
      manualC1 += dir;
      break;
    case MODE_MANUAL_C2:
      manualC2 += dir;
      break;
    case MODE_MEMORY:
      selectedMemory = (selectedMemory + dir + MAX_MEMORIES) % MAX_MEMORIES;
      break;
    case MODE_SWR:
    case MODE_TEST:
      break;
  }
  updateDisplay();
}

void handleEncoderButton() {
  switch (currentMode) {
    case MODE_PRESET:
      activatePreset();
      break;
    case MODE_MANUAL_L:
    case MODE_MANUAL_C1:
    case MODE_MANUAL_C2:
      applyManualSettings();
      break;
    case MODE_SWR:
      autoTune();
      break;
    case MODE_MEMORY:
      loadMemory(selectedMemory);
      break;
    case MODE_TEST:
      testAllRelays();
      break;
  }
}

void changeMode() {
  currentMode = (Mode)((currentMode + 1) % MODE_COUNT);

  // Populeaza valorile manuale cu setarile active curente
  if (currentMode >= MODE_MANUAL_L && currentMode <= MODE_MANUAL_C2) {
    manualL  = activeL;
    manualC1 = activeC1;
    manualC2 = activeC2;
  }

  updateDisplay();
  Serial.print(F("Mod: "));
  const char* modeNames[] = {"PRESET","MANUAL_L","MANUAL_C1","MANUAL_C2","SWR","MEMORY","TEST"};
  Serial.println(modeNames[currentMode]);
}

void quickAction() {
  switch (currentMode) {
    case MODE_TEST:
      testAllRelays();
      break;
    case MODE_SWR:
      autoTune();
      break;
    case MODE_MEMORY:
      // In modul memorie: btn2 = save sau delete
      if (memories[selectedMemory].valid) {
        deleteMemory(selectedMemory);
      } else {
        saveMemory(selectedMemory);
      }
      break;
    default:
      // OFF
      setRelays(0x00, 0x00, 0x00);
      activeL = activeC1 = activeC2 = 0;
      tunerActive = false;
      saveLastSettings();
      showMessage("ALL", "OFF", 800);
      updateDisplay();
      Serial.println(F("All OFF"));
      break;
  }
}

// ============================================================
// RELEE
// ============================================================

void setRelays(uint8_t inductance, uint8_t cap1, uint8_t cap2) {
  // BU2152FS pinout confirmat din schema ATU-450:
  // P1-P8  (pin5-12)  = RL6017-6024 = INDUCTANTE  <- ultimul shiftOut
  // P9-P16 (pin13-20) = RL6009-6016 = CAP2        <- al doilea shiftOut
  // P17-P24(pin21-28) = RL6001-6008 = CAP1        <- primul shiftOut
  digitalWrite(LE_TU_PIN, LOW);
  shiftOut(SDATA_PIN, SCLK_PIN, MSBFIRST, cap1);       // -> P17-P24 = RL6001-6008
  shiftOut(SDATA_PIN, SCLK_PIN, MSBFIRST, cap2);       // -> P9-P16  = RL6009-6016
  shiftOut(SDATA_PIN, SCLK_PIN, MSBFIRST, inductance); // -> P1-P8   = RL6017-6024
  digitalWrite(LE_TU_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(LE_TU_PIN, LOW);
}

void activatePreset() {
  Preset p = getPreset(currentPreset);
  setRelays(p.inductance, p.capacitor1, p.capacitor2);
  activeL  = p.inductance;
  activeC1 = p.capacitor1;
  activeC2 = p.capacitor2;
  tunerActive = true;
  saveLastSettings();

  char msg[20];
  sprintf(msg, "%s TUNED", p.name);
  showMessage("PRESET", msg, 800);
  updateDisplay();

  Serial.print(F("Preset activ: ")); Serial.print(p.name);
  Serial.print(F("  L=0x")); Serial.print(p.inductance, HEX);
  Serial.print(F(" C1=0x")); Serial.print(p.capacitor1, HEX);
  Serial.print(F(" C2=0x")); Serial.println(p.capacitor2, HEX);
}

void applyManualSettings() {
  setRelays(manualL, manualC1, manualC2);
  activeL  = manualL;
  activeC1 = manualC1;
  activeC2 = manualC2;
  tunerActive = true;
  saveLastSettings();

  showMessage("MANUAL", "APPLIED!", 800);
  updateDisplay();

  Serial.print(F("Manual: L=0x")); Serial.print(manualL,  HEX);
  Serial.print(F(" C1=0x"));       Serial.print(manualC1, HEX);
  Serial.print(F(" C2=0x"));       Serial.println(manualC2, HEX);
}

// ============================================================
// TEST 24 RELEE
// ============================================================

void testAllRelays() {
  Serial.println(F("\n=== Test 24 Relee ==="));

  const char* sections[] = {"INDUCTANTA", "CAPACITOR1", "CAPACITOR2"};

  for (int sec = 0; sec < 3; sec++) {
    for (int i = 0; i < 8; i++) {
      uint8_t pattern = (1 << i);
      int relayNum    = sec * 8 + i + 1;

      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB10_tr);
      int tx = (128 - u8g2.getStrWidth(sections[sec])) / 2;
      u8g2.drawStr(tx, 12, sections[sec]);

      char label[8];
      sprintf(label, "P%d", relayNum);
      u8g2.setFont(u8g2_font_ncenB18_tr);
      tx = (128 - u8g2.getStrWidth(label)) / 2;
      u8g2.drawStr(tx, 35, label);

      char binStr[12];
      sprintf(binStr, "%d%d%d%d %d%d%d%d",
        (pattern>>7)&1,(pattern>>6)&1,(pattern>>5)&1,(pattern>>4)&1,
        (pattern>>3)&1,(pattern>>2)&1,(pattern>>1)&1, pattern&1);
      u8g2.setFont(u8g2_font_6x10_tr);
      u8g2.drawStr(15, 50, binStr);

      // Progress bar
      int prog = (int)((float)relayNum / 24.0f * 128.0f);
      u8g2.drawFrame(0, 56, 128, 5);
      u8g2.drawBox(0, 56, prog, 5);
      u8g2.sendBuffer();

      if (sec == 0) setRelays(pattern, 0x00, 0x00);
      if (sec == 1) setRelays(0x00, pattern, 0x00);
      if (sec == 2) setRelays(0x00, 0x00, pattern);

      Serial.print(F("  P")); Serial.print(relayNum);
      Serial.print(F("  ["));
      for (int b = 7; b >= 0; b--) Serial.print((pattern >> b) & 1);
      Serial.println(F("]"));

      delay(350);
    }
  }

  setRelays(0x00, 0x00, 0x00);
  showMessage("DONE!", "24 relay OK", 2000);
  Serial.println(F("Test complet!"));
  updateDisplay();
}

// ============================================================
// EEPROM - Memorii & Last Settings
// ============================================================

void loadEEPROM() {
  // Verifica magic byte
  if (EEPROM.read(EEPROM_ADDR_MAGIC) != EEPROM_MAGIC) {
    Serial.println(F("EEPROM: prima pornire, initializare..."));
    initEEPROM();
    return;
  }

  // Citeste ultima setare
  activeL       = EEPROM.read(EEPROM_ADDR_LAST_L);
  activeC1      = EEPROM.read(EEPROM_ADDR_LAST_C1);
  activeC2      = EEPROM.read(EEPROM_ADDR_LAST_C2);
  currentPreset = EEPROM.read(EEPROM_ADDR_LAST_PRE);
  if (currentPreset >= NUM_PRESETS) currentPreset = 0;

  // Citeste memoriile utilizator
  for (int i = 0; i < MAX_MEMORIES; i++) {
    int addr = EEPROM_ADDR_MEMORIES + i * MEMORY_SIZE;
    memories[i].valid     = EEPROM.read(addr + 0);
    memories[i].L         = EEPROM.read(addr + 1);
    memories[i].C1        = EEPROM.read(addr + 2);
    memories[i].C2        = EEPROM.read(addr + 3);
    memories[i].presetIdx = EEPROM.read(addr + 4);
    if (memories[i].presetIdx >= NUM_PRESETS) memories[i].presetIdx = 0;
  }

  Serial.println(F("EEPROM: setari incarcate OK"));
  Serial.print(F("  Last: L=0x")); Serial.print(activeL, HEX);
  Serial.print(F(" C1=0x")); Serial.print(activeC1, HEX);
  Serial.print(F(" C2=0x")); Serial.println(activeC2, HEX);
}

void initEEPROM() {
  EEPROM.write(EEPROM_ADDR_MAGIC,    EEPROM_MAGIC);
  EEPROM.write(EEPROM_ADDR_LAST_L,   0x00);
  EEPROM.write(EEPROM_ADDR_LAST_C1,  0x00);
  EEPROM.write(EEPROM_ADDR_LAST_C2,  0x00);
  EEPROM.write(EEPROM_ADDR_LAST_PRE, 0x00);
  for (int i = 0; i < MAX_MEMORIES; i++) {
    int addr = EEPROM_ADDR_MEMORIES + i * MEMORY_SIZE;
    EEPROM.write(addr, 0x00); // valid = false
  }
  memset(memories, 0, sizeof(memories));
  activeL = activeC1 = activeC2 = 0;
  Serial.println(F("EEPROM initializat."));
}

void saveLastSettings() {
  EEPROM.update(EEPROM_ADDR_LAST_L,   activeL);
  EEPROM.update(EEPROM_ADDR_LAST_C1,  activeC1);
  EEPROM.update(EEPROM_ADDR_LAST_C2,  activeC2);
  EEPROM.update(EEPROM_ADDR_LAST_PRE, (uint8_t)currentPreset);
}

void saveMemory(int slot) {
  if (slot < 0 || slot >= MAX_MEMORIES) return;
  memories[slot].valid     = true;
  memories[slot].L         = activeL;
  memories[slot].C1        = activeC1;
  memories[slot].C2        = activeC2;
  memories[slot].presetIdx = currentPreset;

  int addr = EEPROM_ADDR_MEMORIES + slot * MEMORY_SIZE;
  EEPROM.update(addr + 0, 0x01);
  EEPROM.update(addr + 1, activeL);
  EEPROM.update(addr + 2, activeC1);
  EEPROM.update(addr + 3, activeC2);
  EEPROM.update(addr + 4, (uint8_t)currentPreset);

  Preset p = getPreset(currentPreset);
  Serial.print(F("Memorie salvata in slot ")); Serial.print(slot);
  Serial.print(F(" [banda=")); Serial.print(p.name);
  Serial.print(F("] L=0x")); Serial.print(activeL, HEX);
  Serial.print(F(" C1=0x")); Serial.print(activeC1, HEX);
  Serial.print(F(" C2=0x")); Serial.println(activeC2, HEX);

  char msg[12];
  sprintf(msg, "Slot %02d OK", slot);
  showMessage("SAVED", msg, 800);
  updateDisplay();
}

void loadMemory(int slot) {
  if (slot < 0 || slot >= MAX_MEMORIES) return;
  if (!memories[slot].valid) {
    showMessage("EMPTY", "No memory!", 1000);
    return;
  }

  Memory& m    = memories[slot];
  currentPreset = m.presetIdx;
  setRelays(m.L, m.C1, m.C2);
  activeL  = m.L;
  activeC1 = m.C1;
  activeC2 = m.C2;
  tunerActive = true;
  saveLastSettings();

  Preset p = getPreset(currentPreset);
  Serial.print(F("Memorie incarcata din slot ")); Serial.print(slot);
  Serial.print(F(" [banda=")); Serial.print(p.name);
  Serial.print(F("] L=0x")); Serial.print(m.L, HEX);
  Serial.print(F(" C1=0x")); Serial.print(m.C1, HEX);
  Serial.print(F(" C2=0x")); Serial.println(m.C2, HEX);

  char msg[12];
  sprintf(msg, "Slot %02d", slot);
  showMessage("LOADED", msg, 800);
  updateDisplay();
}

void deleteMemory(int slot) {
  if (slot < 0 || slot >= MAX_MEMORIES) return;
  memories[slot].valid = false;
  int addr = EEPROM_ADDR_MEMORIES + slot * MEMORY_SIZE;
  EEPROM.update(addr, 0x00);

  Serial.print(F("Memorie stearsa: slot ")); Serial.println(slot);
  char msg[12];
  sprintf(msg, "Slot %02d", slot);
  showMessage("DELETED", msg, 800);
  updateDisplay();
}

void listMemories() {
  Serial.println(F("\n=== Memorii salvate ==="));
  bool any = false;
  for (int i = 0; i < MAX_MEMORIES; i++) {
    if (memories[i].valid) {
      Preset p = getPreset(memories[i].presetIdx);
      Serial.print(F("  Slot ")); Serial.print(i);
      Serial.print(F(": [")); Serial.print(p.name); Serial.print(F("] "));
      Serial.print(F("L=0x")); Serial.print(memories[i].L, HEX);
      Serial.print(F(" C1=0x")); Serial.print(memories[i].C1, HEX);
      Serial.print(F(" C2=0x")); Serial.println(memories[i].C2, HEX);
      any = true;
    }
  }
  if (!any) Serial.println(F("  (nici o memorie salvata)"));
  Serial.println(F("========================"));
}

// ============================================================
// SERIAL
// ============================================================

void handleSerialCommand() {
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "TEST") {
    testAllRelays();

  } else if (cmd == "OFF") {
    setRelays(0x00, 0x00, 0x00);
    activeL = activeC1 = activeC2 = 0;
    tunerActive = false;
    saveLastSettings();
    updateDisplay();
    Serial.println(F("All OFF"));

  } else if (cmd == "MODE") {
    changeMode();

  } else if (cmd == "TUNE") {
    activatePreset();

  } else if (cmd == "SWR") {
    readSWR();
    Serial.print(F("SWR: "));   Serial.print(currentSWR, 2);
    Serial.print(F("  FWD: ")); Serial.print(fwdRaw);
    Serial.print(F("  REV: ")); Serial.println(revRaw);

  } else if (cmd == "AUTOTUNE") {
    autoTune();

  } else if (cmd == "MEM") {
    listMemories();

  } else if (cmd == "INFO") {
    Preset p = getPreset(currentPreset);
    Serial.println(F("\n=== Status ATU-450 v4.0 ==="));
    Serial.print(F("Mod:     ")); Serial.println(currentMode);
    Serial.print(F("Preset:  ")); Serial.print(p.name);
    Serial.print(F(" (")); Serial.print(p.frequency); Serial.println(F(" MHz)"));
    Serial.print(F("Tuner:   ")); Serial.println(tunerActive ? F("ON") : F("OFF"));
    Serial.print(F("Active:  L=0x")); Serial.print(activeL, HEX);
    Serial.print(F(" C1=0x")); Serial.print(activeC1, HEX);
    Serial.print(F(" C2=0x")); Serial.println(activeC2, HEX);
    Serial.print(F("SWR:     ")); Serial.println(currentSWR, 2);
    Serial.print(F("FWD:     ")); Serial.print(fwdRaw);
    Serial.print(F("  REV: ")); Serial.println(revRaw);
    Serial.print(F("Alert>:  ")); Serial.println(SWR_ALERT_THRESHOLD);
    Serial.print(F("RAM:     ")); Serial.print(freeMemory()); Serial.println(F(" bytes"));

  } else if (cmd == "RESET") {
    initEEPROM();
    showMessage("EEPROM", "RESET OK", 1500);
    updateDisplay();

  } else if (cmd.startsWith("SAVE ")) {
    int slot = cmd.substring(5).toInt();
    saveMemory(slot);

  } else if (cmd.startsWith("LOAD ")) {
    int slot = cmd.substring(5).toInt();
    loadMemory(slot);

  } else if (cmd.startsWith("PRESET ")) {
    int idx = cmd.substring(7).toInt();
    if (idx >= 0 && idx < NUM_PRESETS) {
      currentPreset = idx;
      activatePreset();
    } else {
      Serial.print(F("Preset invalid! (0-")); Serial.print(NUM_PRESETS-1); Serial.println(F(")"));
    }

  } else if (cmd.startsWith("LCC ")) {
    // Format: LCC 255 240 128
    String args = cmd.substring(4);
    int s1 = args.indexOf(' ');
    int s2 = args.indexOf(' ', s1 + 1);
    if (s1 > 0 && s2 > 0) {
      uint8_t l  = (uint8_t)args.substring(0, s1).toInt();
      uint8_t c1 = (uint8_t)args.substring(s1 + 1, s2).toInt();
      uint8_t c2 = (uint8_t)args.substring(s2 + 1).toInt();
      setRelays(l, c1, c2);
      activeL = l; activeC1 = c1; activeC2 = c2;
      tunerActive = true;
      saveLastSettings();
      updateDisplay();
      Serial.print(F("LCC: L=")); Serial.print(l);
      Serial.print(F(" C1=")); Serial.print(c1);
      Serial.print(F(" C2=")); Serial.println(c2);
    } else {
      Serial.println(F("Format: LCC <L 0-255> <C1 0-255> <C2 0-255>"));
      Serial.println(F("Exemplu: LCC 15 240 128"));
    }

  } else {
    // Help
    Serial.println(F("\n============ Comenzi ATU-450 v4.0 ============"));
    Serial.println(F("TEST           - test secvential 24 relee"));
    Serial.println(F("OFF            - toate releele OFF + salveaza EEPROM"));
    Serial.println(F("MODE           - schimba modul (7 moduri)"));
    Serial.println(F("TUNE           - aplica presetul benzii curente"));
    Serial.println(F("SWR            - citeste SWR acum"));
    Serial.println(F("AUTOTUNE       - hill-climbing auto-tune"));
    Serial.println(F("INFO           - status complet + RAM"));
    Serial.println(F("MEM            - listeaza memoriile salvate"));
    Serial.println(F("SAVE <0-19>    - salveaza setarile curente"));
    Serial.println(F("LOAD <0-19>    - incarca memorie"));
    Serial.println(F("PRESET <0-10>  - seteaza banda direct (0=160m ... 10=6m)"));
    Serial.println(F("LCC L C1 C2    - control direct 0-255"));
    Serial.println(F("RESET          - sterge toate memoriile EEPROM"));
    Serial.println(F("==============================================="));
  }
}

// RAM liber (AVR / Mega 2560)
int freeMemory() {
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}
