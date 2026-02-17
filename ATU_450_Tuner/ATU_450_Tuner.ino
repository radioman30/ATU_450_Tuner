/*
 * ATU-450 Tuner Controller
 * Display OLED 1.3" cu U8g2 + Rotary Encoder
 * Control BU2152FS - 24 relee (8L + 16C)
 * 
 * P1-P8:   Inductanțe
 * P9-P16:  Condensatoare Set 1
 * P17-P24: Condensatoare Set 2
 */

#include <U8g2lib.h>
#include <Wire.h>

// Display
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// Pini tuner
#define SDATA_PIN  11
#define SCLK_PIN   13
#define LE_TU_PIN  10

// Encoder
#define ENC_CLK    2
#define ENC_DT     3
#define ENC_SW     4

// Butoane
#define BTN_1      5
#define BTN_2      6

// Variabile encoder
volatile int encoderPos = 0;
volatile int lastEncoded = 0;

// Structura pentru presets - acum cu 3 bytes
struct Preset {
  const char* name;
  uint8_t inductance;    // P1-P8
  uint8_t capacitor1;    // P9-P16
  uint8_t capacitor2;    // P17-P24
  float frequency;
};

// Presets - TREBUIE CALIBRATE!
// Valorile sunt exemple - ajustează-le pentru antena ta
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
  MODE_MANUAL_L,   // Ajustare inductanțe
  MODE_MANUAL_C1,  // Ajustare condensatoare set 1
  MODE_MANUAL_C2,  // Ajustare condensatoare set 2
  MODE_TEST
};

Mode currentMode = MODE_PRESET;
uint8_t manualL = 0;
uint8_t manualC1 = 0;
uint8_t manualC2 = 0;
bool tunerActive = false;

void setup() {
  // Pini tuner
  pinMode(SDATA_PIN, OUTPUT);
  pinMode(SCLK_PIN, OUTPUT);
  pinMode(LE_TU_PIN, OUTPUT);
  digitalWrite(SCLK_PIN, LOW);
  digitalWrite(LE_TU_PIN, LOW);
  
  // Encoder și butoane
  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);
  pinMode(BTN_1, INPUT_PULLUP);
  pinMode(BTN_2, INPUT_PULLUP);
  
  // Interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_CLK), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DT), updateEncoder, CHANGE);
  
  // Serial
  Serial.begin(115200);
  Serial.println(F("ATU-450 Controller v2.0"));
  Serial.println(F("24 Relays: 8L + 16C"));
  
  // Display
  u8g2.begin();
  Serial.println(F("Display OK"));
  
  // Splash
  showSplashScreen();
  delay(2000);
  
  // Reset toate relee
  setRelays(0x00, 0x00, 0x00);
  
  updateDisplay();
  
  Serial.println(F("Ready!"));
  Serial.println(F("Commands: TEST, OFF, MODE, TUNE, INFO"));
}

void loop() {
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
  
  // Buton 2 - OFF
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

void updateEncoder() {
  int MSB = digitalRead(ENC_CLK);
  int LSB = digitalRead(ENC_DT);
  
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;
  
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderPos++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderPos--;
    
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
    case MODE_TEST:
      testAllRelays();
      break;
  }
}

void changeMode() {
  currentMode = (Mode)((currentMode + 1) % 5);
  
  if (currentMode >= MODE_MANUAL_L && currentMode <= MODE_MANUAL_C2) {
    if (tunerActive) {
      manualL = presets[currentPreset].inductance;
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
  } else {
    setRelays(0x00, 0x00, 0x00);
    tunerActive = false;
    
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB24_tr);
    u8g2.drawStr(25, 40, "OFF");
    u8g2.sendBuffer();
    delay(1000);
    
    updateDisplay();
    Serial.println(F("All OFF"));
  }
}

void activatePreset() {
  setRelays(presets[currentPreset].inductance, 
            presets[currentPreset].capacitor1,
            presets[currentPreset].capacitor2);
  tunerActive = true;
  
  u8g2.setDrawColor(1);
  u8g2.drawBox(0, 50, 128, 14);
  u8g2.setDrawColor(0);
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.drawStr(30, 62, "TUNED!");
  u8g2.sendBuffer();
  delay(1000);
  u8g2.setDrawColor(1);
  
  updateDisplay();
  
  Serial.print(F("Active: "));
  Serial.print(presets[currentPreset].name);
  Serial.print(F(" L=0x"));
  Serial.print(presets[currentPreset].inductance, HEX);
  Serial.print(F(" C1=0x"));
  Serial.print(presets[currentPreset].capacitor1, HEX);
  Serial.print(F(" C2=0x"));
  Serial.println(presets[currentPreset].capacitor2, HEX);
}

void applyManualSettings() {
  setRelays(manualL, manualC1, manualC2);
  tunerActive = true;
  
  u8g2.setDrawColor(1);
  u8g2.drawBox(0, 50, 128, 14);
  u8g2.setDrawColor(0);
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.drawStr(20, 62, "APPLIED!");
  u8g2.sendBuffer();
  delay(1000);
  u8g2.setDrawColor(1);
  
  updateDisplay();
  
  Serial.print(F("Manual: L=0x"));
  Serial.print(manualL, HEX);
  Serial.print(F(" C1=0x"));
  Serial.print(manualC1, HEX);
  Serial.print(F(" C2=0x"));
  Serial.println(manualC2, HEX);
}

void showSplashScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB18_tr);
  u8g2.drawStr(10, 25, "ATU-450");
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.drawStr(5, 45, "24 Relay Tuner");
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(20, 60, "8L + 16C v2.0");
  u8g2.sendBuffer();
}

void updateDisplay() {
  u8g2.clearBuffer();
  
  switch (currentMode) {
    case MODE_PRESET:
      displayPresetMode();
      break;
    case MODE_MANUAL_L:
      displayManualMode("L (P1-P8)", manualL);
      break;
    case MODE_MANUAL_C1:
      displayManualMode("C1(P9-P16)", manualC1);
      break;
    case MODE_MANUAL_C2:
      displayManualMode("C2(P17-24)", manualC2);
      break;
    case MODE_TEST:
      displayTestMode();
      break;
  }
  
  displayStatusBar();
  u8g2.sendBuffer();
}

void displayPresetMode() {
  // Bandă
  u8g2.setFont(u8g2_font_ncenB24_tr);
  u8g2.drawStr(25, 30, presets[currentPreset].name);
  
  // Frecvență
  char freqStr[20];
  dtostrf(presets[currentPreset].frequency, 5, 2, freqStr);
  strcat(freqStr, " MHz");
  u8g2.setFont(u8g2_font_ncenB12_tr);
  u8g2.drawStr(10, 50, freqStr);
  
  // Poziție
  char posStr[10];
  sprintf(posStr, "%d/%d", currentPreset + 1, numPresets);
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(95, 10, posStr);
}

void displayManualMode(const char* label, uint8_t value) {
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(0, 10, "MANUAL");
  u8g2.drawStr(0, 20, label);
  
  // HEX
  char hexStr[10];
  sprintf(hexStr, "0x%02X", value);
  u8g2.setFont(u8g2_font_ncenB18_tr);
  u8g2.drawStr(25, 38, hexStr);
  
  // Binar
  char binStr[12];
  sprintf(binStr, "%d%d%d%d %d%d%d%d",
    (value>>7)&1, (value>>6)&1, (value>>5)&1, (value>>4)&1,
    (value>>3)&1, (value>>2)&1, (value>>1)&1, value&1);
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(20, 50, binStr);
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
  u8g2.drawStr(2, 63, tunerActive ? "ON" : "OFF");
  u8g2.drawStr(70, 63, "[M]");
  u8g2.drawStr(95, 63, "[X]");
}

void testAllRelays() {
  Serial.println(F("\n=== Testing 24 Relays ==="));
  
  // Test inductanțe P1-P8
  Serial.println(F("Inductances (P1-P8):"));
  for (int i = 0; i < 8; i++) {
    uint8_t pattern = (1 << i);
    
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr(15, 12, "INDUCTANCE");
    
    char label[10];
    sprintf(label, "P%d", i + 1);
    u8g2.setFont(u8g2_font_ncenB18_tr);
    u8g2.drawStr(40, 35, label);
    
    char binStr[12];
    sprintf(binStr, "%d%d%d%d %d%d%d%d",
      (pattern>>7)&1, (pattern>>6)&1, (pattern>>5)&1, (pattern>>4)&1,
      (pattern>>3)&1, (pattern>>2)&1, (pattern>>1)&1, pattern&1);
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(15, 50, binStr);
    u8g2.sendBuffer();
    
    setRelays(pattern, 0x00, 0x00);
    Serial.print(F("  P"));
    Serial.println(i + 1);
    delay(400);
  }
  
  // Test condensatoare set 1 P9-P16
  Serial.println(F("Capacitors 1 (P9-P16):"));
  for (int i = 0; i < 8; i++) {
    uint8_t pattern = (1 << i);
    
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr(15, 12, "CAPACITOR 1");
    
    char label[10];
    sprintf(label, "P%d", i + 9);
    u8g2.setFont(u8g2_font_ncenB18_tr);
    u8g2.drawStr(35, 35, label);
    
    char binStr[12];
    sprintf(binStr, "%d%d%d%d %d%d%d%d",
      (pattern>>7)&1, (pattern>>6)&1, (pattern>>5)&1, (pattern>>4)&1,
      (pattern>>3)&1, (pattern>>2)&1, (pattern>>1)&1, pattern&1);
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(15, 50, binStr);
    u8g2.sendBuffer();
    
    setRelays(0x00, pattern, 0x00);
    Serial.print(F("  P"));
    Serial.println(i + 9);
    delay(400);
  }
  
  // Test condensatoare set 2 P17-P24
  Serial.println(F("Capacitors 2 (P17-P24):"));
  for (int i = 0; i < 8; i++) {
    uint8_t pattern = (1 << i);
    
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB10_tr);
    u8g2.drawStr(15, 12, "CAPACITOR 2");
    
    char label[10];
    sprintf(label, "P%d", i + 17);
    u8g2.setFont(u8g2_font_ncenB18_tr);
    u8g2.drawStr(35, 35, label);
    
    char binStr[12];
    sprintf(binStr, "%d%d%d%d %d%d%d%d",
      (pattern>>7)&1, (pattern>>6)&1, (pattern>>5)&1, (pattern>>4)&1,
      (pattern>>3)&1, (pattern>>2)&1, (pattern>>1)&1, pattern&1);
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(15, 50, binStr);
    u8g2.sendBuffer();
    
    setRelays(0x00, 0x00, pattern);
    Serial.print(F("  P"));
    Serial.println(i + 17);
    delay(400);
  }
  
  // Toate OFF
  setRelays(0x00, 0x00, 0x00);
  
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB18_tr);
  u8g2.drawStr(15, 30, "DONE!");
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(10, 50, "24 relays OK");
  u8g2.sendBuffer();
  delay(2000);
  
  Serial.println(F("Test complete - 24 relays!"));
  updateDisplay();
}

/**
 * Control BU2152FS - trimite 3 bytes (24 biți)
 * @param inductance - P1-P8
 * @param cap1 - P9-P16
 * @param cap2 - P17-P24
 */
void setRelays(uint8_t inductance, uint8_t cap1, uint8_t cap2) {
  digitalWrite(LE_TU_PIN, LOW);
  
  // Trimite cei 3 bytes în ordine
  // ATENȚIE: Ordinea poate varia - testează!
  shiftOut(SDATA_PIN, SCLK_PIN, MSBFIRST, cap2);        // P17-P24
  shiftOut(SDATA_PIN, SCLK_PIN, MSBFIRST, cap1);        // P9-P16
  shiftOut(SDATA_PIN, SCLK_PIN, MSBFIRST, inductance);  // P1-P8
  
  // Activează latch
  digitalWrite(LE_TU_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(LE_TU_PIN, LOW);
}

void handleSerialCommand() {
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toUpperCase();
  
  if (cmd == "TEST") {
    testAllRelays();
  }
  else if (cmd == "OFF") {
    setRelays(0x00, 0x00, 0x00);
    tunerActive = false;
    updateDisplay();
    Serial.println(F("All OFF"));
  }
  else if (cmd == "MODE") {
    changeMode();
  }
  else if (cmd == "TUNE") {
    activatePreset();
  }
  else if (cmd == "INFO") {
    Serial.println(F("\n=== Status ==="));
    Serial.print(F("Mode: "));
    Serial.println(currentMode);
    Serial.print(F("Preset: "));
    Serial.print(presets[currentPreset].name);
    Serial.print(F(" ("));
    Serial.print(presets[currentPreset].frequency);
    Serial.println(F(" MHz)"));
    Serial.print(F("Tuner: "));
    Serial.println(tunerActive ? F("ON") : F("OFF"));
    Serial.print(F("RAM free: "));
    Serial.println(freeMemory());
  }
  else if (cmd.startsWith("LCC ")) {
    // Format: LCC 255 128 64
    int space1 = cmd.indexOf(' ', 4);
    int space2 = cmd.indexOf(' ', space1 + 1);
    if (space1 > 0 && space2 > 0) {
      uint8_t l = cmd.substring(4, space1).toInt();
      uint8_t c1 = cmd.substring(space1 + 1, space2).toInt();
      uint8_t c2 = cmd.substring(space2 + 1).toInt();
      setRelays(l, c1, c2);
      tunerActive = true;
      Serial.print(F("L="));
      Serial.print(l);
      Serial.print(F(" C1="));
      Serial.print(c1);
      Serial.print(F(" C2="));
      Serial.println(c2);
    }
  }
  else {
    Serial.println(F("\n=== Commands ==="));
    Serial.println(F("TEST - test 24 relays"));
    Serial.println(F("OFF - all off"));
    Serial.println(F("MODE - change mode (5 modes)"));
    Serial.println(F("TUNE - activate preset"));
    Serial.println(F("INFO - status"));
    Serial.println(F("LCC <L> <C1> <C2> - direct (0-255)"));
    Serial.println(F("\nExample: LCC 15 240 128"));
  }
}

int freeMemory() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
