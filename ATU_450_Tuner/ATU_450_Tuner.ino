pentr mega2560/*
 * ATU-450 Tuner Controller v3.0
 * Display OLED 1.3" cu U8g2 + Rotary Encoder
 * Control BU2152FS - 24 relee (8L + 16C)
 * SWR Meter - punte externa (FWD + REV)
 * 
 * P1-P8:   Inductante
 * P9-P16:  Condensatoare Set 1
 * P17-P24: Condensatoare Set 2
 * 
 * A0: FWD (forward power)
 * A1: REV (reflected power)
 */

#include <U8g2lib.h>
#include <Wire.h>

// Display
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// Pini tuner
#define SDATA_PIN   11
#define SCLK_PIN    13
#define LE_TU_PIN   10

// Encoder
#define ENC_CLK     2
#define ENC_DT      3
#define ENC_SW      4

// Butoane
#define BTN_1       5
#define BTN_2       6

// SWR Meter
#define FWD_PIN     A0
#define REV_PIN     A1

#define SWR_ALERT_THRESHOLD  2.0   // Alerta daca SWR > 2.0
#define SWR_GOOD_THRESHOLD   1.5   // SWR bun sub 1.5
#define SWR_SAMPLES          10    // Mediere masuratori
#define AUTOTUNE_TIMEOUT     5000  // Timeout auto-tune (ms)
#define AUTOTUNE_MIN_FWD     50    // Putere minima FWD pentru auto-tune (ADC)

// Variabile encoder
volatile int encoderPos = 0;
volatile int lastEncoded = 0;

// Structura preset
struct Preset {
  const char* name;
  uint8_t inductance;
  uint8_t capacitor1;
  uint8_t capacitor2;
  float frequency;
};

Preset presets[] = {
  {"160m", 0b11111111, 0b11110000, 0b00000000, 1.900},
  {"80m",  0b01111111, 0b11111000, 0b00000000, 3.750},
  {"60m",  0b00111111, 0b01111100, 0b00001000, 5.350},
  {"40m",  0b00011111, 0b00111110, 0b00011000, 7.150},
  {"30m",  0b00001111, 0b00011111, 0b00111000, 10.100},
  {"20m",  0b00000111, 0b00001111, 0b01111000, 14.200},
  {"17m",  0b00000011, 0b00000111, 0b11110000, 18.100},
  {"15m",  0b00000001, 0b00000011, 0b11111000, 21.200},
  {"12m",  0b00000000, 0b00000001, 0b11111100, 24.900},
  {"10m",  0b00000000, 0b00000000, 0b11111110, 28.500},
  {"6m",   0b00000000, 0b00000000, 0b01111111, 50.000}
};

const int numPresets = sizeof(presets) / sizeof(Preset);
int currentPreset = 0;
int lastEncoderPos = 0;

// Moduri operare
enum Mode {
  MODE_PRESET,
  MODE_MANUAL_L,
  MODE_MANUAL_C1,
  MODE_MANUAL_C2,
  MODE_SWR,
  MODE_TEST
};

Mode currentMode = MODE_PRESET;
uint8_t manualL  = 0;
uint8_t manualC1 = 0;
uint8_t manualC2 = 0;
bool tunerActive = false;

// SWR variabile
float currentSWR   = 0.0;
float minSWR       = 99.0;
int   fwdRaw       = 0;
int   revRaw       = 0;
bool  swrAlert     = false;
bool  autoTuning   = false;
unsigned long lastSWRRead = 0;

// Auto-tune
uint8_t bestL  = 0;
uint8_t bestC1 = 0;
uint8_t bestC2 = 0;

// ============================================================
void setup() {
  // Pini tuner
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

  // SWR pini analogici
  pinMode(FWD_PIN, INPUT);
  pinMode(REV_PIN, INPUT);

  // Interrupts encoder
  attachInterrupt(digitalPinToInterrupt(ENC_CLK), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DT),  updateEncoder, CHANGE);

  Serial.begin(115200);
  Serial.println(F("ATU-450 Controller v3.0"));
  Serial.println(F("24 Relays: 8L + 16C + SWR Meter"));

  u8g2.begin();
  showSplashScreen();
  delay(2000);

  setRelays(0x00, 0x00, 0x00);
  updateDisplay();

  Serial.println(F("Ready!"));
  Serial.println(F("Commands: TEST, OFF, MODE, TUNE, SWR, AUTOTUNE, INFO"));
}

// ============================================================
void loop() {
  // Citire SWR periodic
  if (millis() - lastSWRRead > 200) {
    readSWR();
    lastSWRRead = millis();

    // Alerta SWR
    if (tunerActive && currentSWR > SWR_ALERT_THRESHOLD && fwdRaw > AUTOTUNE_MIN_FWD) {
      swrAlert = true;
    } else {
      swrAlert = false;
    }

    // Refresh display daca suntem in modul SWR
    if (currentMode == MODE_SWR) {
      updateDisplay();
    } else {
      // Actualizeaza doar status bar-ul in celelalte moduri
      displayStatusBar();
      u8g2.sendBuffer();
    }
  }

  // Encoder
  if (encoderPos != lastEncoderPos) {
    handleEncoderRotation();
    lastEncoderPos = encoderPos;
  }

  // Buton encoder
  static unsigned long lastEncBtn = 0;
  if (digitalRead(ENC_SW) == LOW && millis() - lastEncBtn > 300) {
    delay(50);
    if (digitalRead(ENC_SW) == LOW) {
      handleEncoderButton();
      lastEncBtn = millis();
      while (digitalRead(ENC_SW) == LOW);
    }
  }

  // Buton 1 - Schimbare mod
  static unsigned long lastBtn1 = 0;
  if (digitalRead(BTN_1) == LOW && millis() - lastBtn1 > 300) {
    delay(50);
    if (digitalRead(BTN_1) == LOW) {
      changeMode();
      lastBtn1 = millis();
      while (digitalRead(BTN_1) == LOW);
    }
  }

  // Buton 2 - actiune rapida / OFF
  static unsigned long lastBtn2 = 0;
  if (digitalRead(BTN_2) == LOW && millis() - lastBtn2 > 300) {
    delay(50);
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
    delayMicroseconds(200);
  }
  fwdRaw = fwdSum / SWR_SAMPLES;
  revRaw = revSum / SWR_SAMPLES;

  currentSWR = calculateSWR(fwdRaw, revRaw);
}

float calculateSWR(int fwd, int rev) {
  if (fwd < 10) return 0.0;           // Nu e semnal

  float vFwd = (float)fwd / 1023.0;
  float vRev = (float)rev / 1023.0;

  if (vFwd <= vRev) return 99.0;      // Eroare / scurtcircuit

  float gamma = vRev / vFwd;          // Coeficient de reflectie
  if (gamma >= 1.0) return 99.0;

  float swr = (1.0 + gamma) / (1.0 - gamma);
  if (swr > 99.0) swr = 99.0;

  return swr;
}

// Auto-tune: cauta combinatia L/C cu SWR minim
void autoTune() {
  if (fwdRaw < AUTOTUNE_MIN_FWD) {
    Serial.println(F("Auto-tune: putere FWD prea mica!"));
    showMessage("NO POWER", "Aplica RF!", 2000);
    return;
  }

  autoTuning = true;
  minSWR = 99.0;
  bestL  = presets[currentPreset].inductance;
  bestC1 = presets[currentPreset].capacitor1;
  bestC2 = presets[currentPreset].capacitor2;

  Serial.println(F("\n=== AUTO-TUNE START ==="));
  Serial.print(F("Preset: "));
  Serial.println(presets[currentPreset].name);

  unsigned long startTime = millis();
  uint8_t startL  = presets[currentPreset].inductance;
  uint8_t startC1 = presets[currentPreset].capacitor1;
  uint8_t startC2 = presets[currentPreset].capacitor2;

  // Pas 1: Cauta inductanta optima (+-4 pasi fata de preset)
  for (int dL = -4; dL <= 4; dL++) {
    if (millis() - startTime > AUTOTUNE_TIMEOUT) break;
    uint8_t testL = startL + dL;
    setRelays(testL, startC1, startC2);
    delay(30);
    readSWR();
    showAutoTuneProgress(testL, startC1, startC2);
    if (currentSWR < minSWR && currentSWR > 0) {
      minSWR = currentSWR;
      bestL = testL;
    }
    if (minSWR <= 1.1) break;
  }

  // Pas 2: Cauta C1 optim
  for (int dC1 = -4; dC1 <= 4; dC1++) {
    if (millis() - startTime > AUTOTUNE_TIMEOUT) break;
    uint8_t testC1 = startC1 + dC1;
    setRelays(bestL, testC1, startC2);
    delay(30);
    readSWR();
    showAutoTuneProgress(bestL, testC1, startC2);
    if (currentSWR < minSWR && currentSWR > 0) {
      minSWR = currentSWR;
      bestC1 = testC1;
    }
    if (minSWR <= 1.1) break;
  }

  // Pas 3: Cauta C2 optim
  for (int dC2 = -4; dC2 <= 4; dC2++) {
    if (millis() - startTime > AUTOTUNE_TIMEOUT) break;
    uint8_t testC2 = startC2 + dC2;
    setRelays(bestL, bestC1, testC2);
    delay(30);
    readSWR();
    showAutoTuneProgress(bestL, bestC1, testC2);
    if (currentSWR < minSWR && currentSWR > 0) {
      minSWR = currentSWR;
      bestC2 = testC2;
    }
    if (minSWR <= 1.1) break;
  }

  // Aplica cel mai bun rezultat
  setRelays(bestL, bestC1, bestC2);
  tunerActive = true;
  autoTuning  = false;

  Serial.println(F("=== AUTO-TUNE DONE ==="));
  Serial.print(F("Best SWR: ")); Serial.println(minSWR, 2);
  Serial.print(F("L=0x"));  Serial.print(bestL,  HEX);
  Serial.print(F(" C1=0x")); Serial.print(bestC1, HEX);
  Serial.print(F(" C2=0x")); Serial.println(bestC2, HEX);

  char swrStr[10];
  dtostrf(minSWR, 4, 2, swrStr);
  char msg1[20];
  sprintf(msg1, "SWR: %s", swrStr);
  showMessage("TUNE DONE", msg1, 2000);

  updateDisplay();
}

void showAutoTuneProgress(uint8_t l, uint8_t c1, uint8_t c2) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.drawStr(15, 12, "AUTO-TUNE...");

  char swrStr[16];
  if (currentSWR >= 10.0) strcpy(swrStr, ">10");
  else dtostrf(currentSWR, 5, 2, swrStr);

  u8g2.setFont(u8g2_font_ncenB18_tr);
  u8g2.drawStr(20, 35, swrStr);

  char bestStr[20];
  dtostrf(minSWR, 4, 2, bestStr);
  char line[24];
  sprintf(line, "Best: %s", bestStr);
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(25, 50, line);

  u8g2.sendBuffer();
}

// ============================================================
// DISPLAY
// ============================================================

void updateDisplay() {
  u8g2.clearBuffer();

  switch (currentMode) {
    case MODE_PRESET:    displayPresetMode();                     break;
    case MODE_MANUAL_L:  displayManualMode("L (P1-P8)",  manualL); break;
    case MODE_MANUAL_C1: displayManualMode("C1(P9-16)", manualC1); break;
    case MODE_MANUAL_C2: displayManualMode("C2(P17-24)",manualC2); break;
    case MODE_SWR:       displaySWRMode();                        break;
    case MODE_TEST:      displayTestMode();                       break;
  }

  displayStatusBar();
  u8g2.sendBuffer();
}

void displayPresetMode() {
  u8g2.setFont(u8g2_font_ncenB24_tr);
  u8g2.drawStr(25, 30, presets[currentPreset].name);

  char freqStr[20];
  dtostrf(presets[currentPreset].frequency, 5, 2, freqStr);
  strcat(freqStr, " MHz");
  u8g2.setFont(u8g2_font_ncenB12_tr);
  u8g2.drawStr(10, 50, freqStr);

  char posStr[10];
  sprintf(posStr, "%d/%d", currentPreset + 1, numPresets);
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(95, 10, posStr);

  // SWR mic in colt stanga sus
  if (fwdRaw > AUTOTUNE_MIN_FWD) {
    char swrStr[10];
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
    u8g2.drawStr(5, 46, "Aplica RF pentru");
    return;
  }

  // Valoare SWR mare centru
  char swrStr[12];
  if (currentSWR >= 10.0) strcpy(swrStr, ">10");
  else dtostrf(currentSWR, 4, 2, swrStr);

  u8g2.setFont(u8g2_font_ncenB18_tr);
  u8g2.drawStr(30, 32, swrStr);

  // Bar grafic SWR (1.0 = gol, 3.0 = plin)
  int barWidth = (int)((currentSWR - 1.0) / 2.0 * 100.0);
  if (barWidth > 100) barWidth = 100;
  if (barWidth < 0)   barWidth = 0;
  u8g2.drawFrame(14, 36, 102, 8);
  if (barWidth > 0) u8g2.drawBox(14, 36, barWidth, 8);

  // FWD/REV raw
  char fwrStr[24];
  sprintf(fwrStr, "F:%d R:%d", fwdRaw, revRaw);
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(10, 53, fwrStr);

  // Status SWR
  if (currentSWR <= SWR_GOOD_THRESHOLD) {
    u8g2.drawStr(82, 10, "GOOD");
  } else if (currentSWR <= SWR_ALERT_THRESHOLD) {
    u8g2.drawStr(90, 10, "OK");
  } else {
    u8g2.drawStr(78, 10, "HIGH!");
  }

  // Hint encoder
  u8g2.drawStr(5, 63, "[enc]=AUTOTUNE");
}

void displayManualMode(const char* label, uint8_t value) {
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 10, "MANUAL");
  u8g2.drawStr(0, 20, label);

  char hexStr[10];
  sprintf(hexStr, "0x%02X", value);
  u8g2.setFont(u8g2_font_ncenB18_tr);
  u8g2.drawStr(25, 38, hexStr);

  char binStr[12];
  sprintf(binStr, "%d%d%d%d %d%d%d%d",
    (value>>7)&1,(value>>6)&1,(value>>5)&1,(value>>4)&1,
    (value>>3)&1,(value>>2)&1,(value>>1)&1, value&1);
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(20, 50, binStr);

  // SWR mic in colt dreapta
  if (fwdRaw > AUTOTUNE_MIN_FWD) {
    char swrStr[10];
    dtostrf(currentSWR, 4, 1, swrStr);
    u8g2.drawStr(85, 10, swrStr);
  }
}

void displayTestMode() {
  u8g2.setFont(u8g2_font_ncenB18_tr);
  u8g2.drawStr(25, 25, "TEST");
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(5, 38, "24 relays test");
  u8g2.drawStr(10, 50, "Press encoder");
}

void displayStatusBar() {
  u8g2.drawHLine(0, 54, 128);
  u8g2.setFont(u8g2_font_6x10_tr);

  // ON/OFF
  u8g2.drawStr(2, 63, tunerActive ? "ON" : "OFF");

  // SWR in status bar
  if (fwdRaw > AUTOTUNE_MIN_FWD) {
    char swrStr[12];
    dtostrf(currentSWR, 3, 1, swrStr);
    char line[12];
    sprintf(line, "S%s", swrStr);

    if (swrAlert) {
      // Inversat pentru alerta
      u8g2.setDrawColor(1);
      u8g2.drawBox(28, 55, 42, 9);
      u8g2.setDrawColor(0);
      u8g2.drawStr(30, 63, line);
      u8g2.setDrawColor(1);
    } else {
      u8g2.drawStr(30, 63, line);
    }
  }

  u8g2.drawStr(80, 63, "[M]");
  u8g2.drawStr(105, 63, "[X]");
}

void showSplashScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB18_tr);
  u8g2.drawStr(10, 22, "ATU-450");
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.drawStr(5, 40, "24 Relay Tuner");
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(5, 53, "8L+16C+SWR  v3.0");
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
  switch (currentMode) {
    case MODE_PRESET:
      if (encoderPos > lastEncoderPos) {
        currentPreset++;
        if (currentPreset >= numPresets) currentPreset = 0;
      } else {
        currentPreset--;
        if (currentPreset < 0) currentPreset = numPresets - 1;
      }
      break;
    case MODE_MANUAL_L:
      manualL += (encoderPos > lastEncoderPos) ? 1 : -1;
      break;
    case MODE_MANUAL_C1:
      manualC1 += (encoderPos > lastEncoderPos) ? 1 : -1;
      break;
    case MODE_MANUAL_C2:
      manualC2 += (encoderPos > lastEncoderPos) ? 1 : -1;
      break;
    case MODE_SWR:
      // Encoder in modul SWR nu face nimic - doar afisaj live
      break;
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
      autoTune();  // Encoder press in SWR mode = AUTOTUNE
      break;
    case MODE_TEST:
      testAllRelays();
      break;
  }
}

void changeMode() {
  currentMode = (Mode)((currentMode + 1) % 6);

  if (currentMode >= MODE_MANUAL_L && currentMode <= MODE_MANUAL_C2) {
    if (tunerActive) {
      manualL  = presets[currentPreset].inductance;
      manualC1 = presets[currentPreset].capacitor1;
      manualC2 = presets[currentPreset].capacitor2;
    }
  }

  updateDisplay();
  Serial.print(F("Mode: "));
  Serial.println(currentMode);
}

void quickAction() {
  if (currentMode == MODE_TEST) {
    testAllRelays();
  } else if (currentMode == MODE_SWR) {
    autoTune();
  } else {
    setRelays(0x00, 0x00, 0x00);
    tunerActive = false;
    showMessage("ALL", "OFF", 800);
    updateDisplay();
    Serial.println(F("All OFF"));
  }
}

// ============================================================
// RELEE
// ============================================================

void setRelays(uint8_t inductance, uint8_t cap1, uint8_t cap2) {
  digitalWrite(LE_TU_PIN, LOW);
  shiftOut(SDATA_PIN, SCLK_PIN, MSBFIRST, cap2);
  shiftOut(SDATA_PIN, SCLK_PIN, MSBFIRST, cap1);
  shiftOut(SDATA_PIN, SCLK_PIN, MSBFIRST, inductance);
  digitalWrite(LE_TU_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(LE_TU_PIN, LOW);
}

void activatePreset() {
  setRelays(presets[currentPreset].inductance,
            presets[currentPreset].capacitor1,
            presets[currentPreset].capacitor2);
  tunerActive = true;

  char msg[16];
  sprintf(msg, "%s TUNED", presets[currentPreset].name);
  showMessage("PRESET", msg, 800);
  updateDisplay();

  Serial.print(F("Active: "));
  Serial.print(presets[currentPreset].name);
  Serial.print(F(" L=0x"));  Serial.print(presets[currentPreset].inductance, HEX);
  Serial.print(F(" C1=0x")); Serial.print(presets[currentPreset].capacitor1, HEX);
  Serial.print(F(" C2=0x")); Serial.println(presets[currentPreset].capacitor2, HEX);
}

void applyManualSettings() {
  setRelays(manualL, manualC1, manualC2);
  tunerActive = true;

  showMessage("MANUAL", "APPLIED!", 800);
  updateDisplay();

  Serial.print(F("Manual: L=0x")); Serial.print(manualL,  HEX);
  Serial.print(F(" C1=0x"));       Serial.print(manualC1, HEX);
  Serial.print(F(" C2=0x"));       Serial.println(manualC2, HEX);
}

// ============================================================
// TEST
// ============================================================

void testAllRelays() {
  Serial.println(F("\n=== Testing 24 Relays ==="));

  for (int i = 0; i < 8; i++) {
    uint8_t pattern = (1 << i);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr(15, 12, "INDUCTANCE");
    char label[10]; sprintf(label, "P%d", i + 1);
    u8g2.setFont(u8g2_font_ncenB18_tr);
    u8g2.drawStr(40, 35, label);
    char binStr[12];
    sprintf(binStr, "%d%d%d%d %d%d%d%d",
      (pattern>>7)&1,(pattern>>6)&1,(pattern>>5)&1,(pattern>>4)&1,
      (pattern>>3)&1,(pattern>>2)&1,(pattern>>1)&1, pattern&1);
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(15, 50, binStr);
    u8g2.sendBuffer();
    setRelays(pattern, 0x00, 0x00);
    Serial.print(F("  P")); Serial.println(i + 1);
    delay(400);
  }

  for (int i = 0; i < 8; i++) {
    uint8_t pattern = (1 << i);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr(15, 12, "CAPACITOR 1");
    char label[10]; sprintf(label, "P%d", i + 9);
    u8g2.setFont(u8g2_font_ncenB18_tr);
    u8g2.drawStr(35, 35, label);
    char binStr[12];
    sprintf(binStr, "%d%d%d%d %d%d%d%d",
      (pattern>>7)&1,(pattern>>6)&1,(pattern>>5)&1,(pattern>>4)&1,
      (pattern>>3)&1,(pattern>>2)&1,(pattern>>1)&1, pattern&1);
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(15, 50, binStr);
    u8g2.sendBuffer();
    setRelays(0x00, pattern, 0x00);
    Serial.print(F("  P")); Serial.println(i + 9);
    delay(400);
  }

  for (int i = 0; i < 8; i++) {
    uint8_t pattern = (1 << i);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr(15, 12, "CAPACITOR 2");
    char label[10]; sprintf(label, "P%d", i + 17);
    u8g2.setFont(u8g2_font_ncenB18_tr);
    u8g2.drawStr(35, 35, label);
    char binStr[12];
    sprintf(binStr, "%d%d%d%d %d%d%d%d",
      (pattern>>7)&1,(pattern>>6)&1,(pattern>>5)&1,(pattern>>4)&1,
      (pattern>>3)&1,(pattern>>2)&1,(pattern>>1)&1, pattern&1);
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(15, 50, binStr);
    u8g2.sendBuffer();
    setRelays(0x00, 0x00, pattern);
    Serial.print(F("  P")); Serial.println(i + 17);
    delay(400);
  }

  setRelays(0x00, 0x00, 0x00);
  showMessage("DONE!", "24 relays OK", 2000);
  Serial.println(F("Test complete!"));
  updateDisplay();
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
    tunerActive = false;
    updateDisplay();
    Serial.println(F("All OFF"));
  } else if (cmd == "MODE") {
    changeMode();
  } else if (cmd == "TUNE") {
    activatePreset();
  } else if (cmd == "SWR") {
    readSWR();
    Serial.print(F("SWR: "));    Serial.print(currentSWR, 2);
    Serial.print(F("  FWD: "));  Serial.print(fwdRaw);
    Serial.print(F("  REV: "));  Serial.println(revRaw);
  } else if (cmd == "AUTOTUNE") {
    autoTune();
  } else if (cmd == "INFO") {
    Serial.println(F("\n=== Status ==="));
    Serial.print(F("Mode: "));    Serial.println(currentMode);
    Serial.print(F("Preset: "));  Serial.print(presets[currentPreset].name);
    Serial.print(F(" ("));        Serial.print(presets[currentPreset].frequency);
    Serial.println(F(" MHz)"));
    Serial.print(F("Tuner: "));   Serial.println(tunerActive ? F("ON") : F("OFF"));
    Serial.print(F("SWR: "));     Serial.println(currentSWR, 2);
    Serial.print(F("FWD: "));     Serial.print(fwdRaw);
    Serial.print(F("  REV: "));   Serial.println(revRaw);
    Serial.print(F("Alert>: "));  Serial.println(SWR_ALERT_THRESHOLD);
    Serial.print(F("RAM free: ")); Serial.println(freeMemory());
  } else if (cmd.startsWith("LCC ")) {
    int space1 = cmd.indexOf(' ', 4);
    int space2 = cmd.indexOf(' ', space1 + 1);
    if (space1 > 0 && space2 > 0) {
      uint8_t l  = cmd.substring(4, space1).toInt();
      uint8_t c1 = cmd.substring(space1 + 1, space2).toInt();
      uint8_t c2 = cmd.substring(space2 + 1).toInt();
      setRelays(l, c1, c2);
      tunerActive = true;
      Serial.print(F("L=")); Serial.print(l);
      Serial.print(F(" C1=")); Serial.print(c1);
      Serial.print(F(" C2=")); Serial.println(c2);
    }
  } else {
    Serial.println(F("\n=== Commands ==="));
    Serial.println(F("TEST      - test 24 relays"));
    Serial.println(F("OFF       - all off"));
    Serial.println(F("MODE      - change mode (6 modes)"));
    Serial.println(F("TUNE      - activate preset"));
    Serial.println(F("SWR       - read SWR now"));
    Serial.println(F("AUTOTUNE  - auto-tune SWR minimum"));
    Serial.println(F("INFO      - status"));
    Serial.println(F("LCC L C1 C2 - direct control (0-255)"));
    Serial.println(F("\nExample: LCC 15 240 128"));
  }
}

int freeMemory() {
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}
