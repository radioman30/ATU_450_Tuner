# ATU-450 Tuner Controller - Pinout

## Arduino (Nano/Uno)

### 🔌 Tuner - BU2152FS (Shift Register)
| Pin Arduino | Funcție     | Descriere                        |
|-------------|-------------|----------------------------------|
| D11         | SDATA       | Date seriale către BU2152FS      |
| D13         | SCLK        | Clock serial                     |
| D10         | LE_TU       | Latch Enable - activează releele |

### 🖥️ Display OLED 1.3" SH1106 (I2C)
| Pin Arduino | Funcție | Descriere         |
|-------------|---------|-------------------|
| A4 (SDA)    | SDA     | Date I2C          |
| A5 (SCL)    | SCL     | Clock I2C         |
| 5V          | VCC     | Alimentare 5V     |
| GND         | GND     | Masă              |

### 🎛️ Rotary Encoder
| Pin Arduino | Funcție | Descriere                        |
|-------------|---------|----------------------------------|
| D2          | CLK     | Canal A (Interrupt 0)            |
| D3          | DT      | Canal B (Interrupt 1)            |
| D4          | SW      | Buton encoder (apăsare)          |
| 5V          | +       | Alimentare                       |
| GND         | GND     | Masă                             |

### 🔘 Butoane externe
| Pin Arduino | Funcție | Descriere                        |
|-------------|---------|----------------------------------|
| D5          | BTN_1   | Schimbare mod (MODE)             |
| D6          | BTN_2   | Acțiune rapidă (OFF/TEST)        |

> Butoanele sunt active LOW cu INPUT_PULLUP intern activat.
> Conectează un capăt la pinul Arduino, celălalt la GND.

---

## Relee BU2152FS - 24 canale

### Mapare relee
| Canale    | Funcție            | Byte transmis |
|-----------|--------------------|---------------|
| P1 - P8   | Inductanțe (L)     | Byte 3 (ultimul) |
| P9 - P16  | Condensatoare C1   | Byte 2           |
| P17 - P24 | Condensatoare C2   | Byte 1 (primul)  |

### Ordine transmisie SPI (MSBFIRST)
```
shiftOut → cap2 (P17-P24) → cap1 (P9-P16) → inductance (P1-P8)
```
> ⚠️ Ordinea poate varia în funcție de cablaj - testează cu comanda TEST!

---

## Moduri de operare
| Mod          | Descriere                          |
|--------------|------------------------------------|
| MODE_PRESET  | Selectare preset bandă (0)         |
| MODE_MANUAL_L| Ajustare manuală inductanțe (1)    |
| MODE_MANUAL_C1| Ajustare manuală condensatoare 1 (2)|
| MODE_MANUAL_C2| Ajustare manuală condensatoare 2 (3)|
| MODE_TEST    | Test secvențial 24 relee (4)       |

---

## Comenzi Serial (115200 baud)
| Comandă         | Funcție                              |
|-----------------|--------------------------------------|
| `TEST`          | Test secvențial toate cele 24 relee  |
| `OFF`           | Dezactivează toate releele           |
| `MODE`          | Schimbă modul de operare             |
| `TUNE`          | Activează presetul curent            |
| `INFO`          | Afișează status curent               |
| `LCC L C1 C2`   | Control direct (0-255 per canal)     |

**Exemplu:** `LCC 15 240 128`

---

## Benzi presetate
| Preset | Frecvență  | L (P1-P8)    | C1 (P9-P16)  | C2 (P17-P24) |
|--------|------------|--------------|--------------|--------------|
| 160m   | 1.900 MHz  | 0b11111111   | 0b11110000   | 0b00000000   |
| 80m    | 3.750 MHz  | 0b01111111   | 0b11111000   | 0b00000000   |
| 60m    | 5.350 MHz  | 0b00111111   | 0b01111100   | 0b00001000   |
| 40m    | 7.150 MHz  | 0b00011111   | 0b00111110   | 0b00011000   |
| 30m    | 10.100 MHz | 0b00001111   | 0b00011111   | 0b00111000   |
| 20m    | 14.200 MHz | 0b00000111   | 0b00001111   | 0b01111000   |
| 17m    | 18.100 MHz | 0b00000011   | 0b00000111   | 0b11110000   |
| 15m    | 21.200 MHz | 0b00000001   | 0b00000011   | 0b11111000   |
| 12m    | 24.900 MHz | 0b00000000   | 0b00000001   | 0b11111100   |
| 10m    | 28.500 MHz | 0b00000000   | 0b00000000   | 0b11111110   |
| 6m     | 50.000 MHz | 0b00000000   | 0b00000000   | 0b01111111   |

> ⚠️ Valorile L/C sunt exemple - **trebuie calibrate** pentru antena ta!
