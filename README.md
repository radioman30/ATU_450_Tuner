# ATU-450 Tuner Controller

Controler pentru tunerul automat de antenă **ATU-450** (din Yaesu FT-450D), pe Arduino MEGA 2560.

- Display OLED 1.3" SH1106 (I2C), encoder rotativ, 2 butoane
- BU2152FS → 24 relee: 8 inductanță (L) + 8 C1 + 8 C2
- Punte SWR pe A0 (FWD) / A1 (REV)
- EEPROM: ultimele setări + 20 de memorii (bandă + L + C1 + C2)
- Auto-tune: hill-climbing pe 3 axe (L, C1, C2), pornind din presetul benzii
- Comenzi seriale: `TEST, OFF, MODE, TUNE, SWR, AUTOTUNE, INFO, SAVE n, LOAD n, MEM, LCC L C1 C2, PRESET n, RESET`

Sketch: [`ATU_450_Tuner/ATU_450_Tuner.ino`](ATU_450_Tuner/ATU_450_Tuner.ino).
Pinout și schema punții SWR în [`docs/`](docs/). Schema tunerului (TUNER-UNIT BR015310B,
relee RL6001–RL6024) e în manualul de service Yaesu FT-450 — nu e inclusă aici (copyright Yaesu).

Versiunea curentă (v4 fix) are ordinea releelor confirmată din schema ATU-450:
P1–P8 = inductanță, P9–P16 = C2, P17–P24 = C1.
Istoricul git conține versiunile anterioare (Tuner_radio → v3 → v4.0).
