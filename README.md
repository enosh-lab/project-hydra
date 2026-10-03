# HYDRA — Water & Gas Monitor

A beginner-friendly ESP8266 project that measures tank water level and estimates methane concentration, shows readings on a small OLED, sounds an alert, and serves a browser dashboard over Wi-Fi.

> **Safety:** This is an educational prototype. The MQ-4 reading is an estimate and this project is not a certified gas detector, alarm, or safety system. Never use it to protect people or property. Keep electronics away from water, use suitable low-voltage supplies, and get experienced help with mains or gas equipment.

## What it does

- Reads tank distance with an HC-SR04 ultrasonic sensor and converts it to a percentage.
- Reads an MQ-4 analog output and estimates methane concentration.
- Shows water and gas readings on a 128×64 SH1106 OLED.
- Sounds a buzzer when configured thresholds are crossed.
- Hosts a dashboard at the ESP8266's local network address, with a simulation fallback when opened without the hardware.
- Offers a standalone dashboard at [`dashboard/hydra.html`](dashboard/hydra.html).

## Parts

- ESP8266 NodeMCU development board (the pin labels below assume a NodeMCU-style board)
- HC-SR04 ultrasonic distance sensor
- MQ-4 gas sensor module
- 128×64 SH1106 I²C OLED module (address `0x3C` in this sketch)
- 3.3 V compatible buzzer module
- Resistors for a voltage divider on the HC-SR04 Echo signal; a separate divider may be needed for MQ-4 analog output
- Breadboard, jumper wires, and appropriate regulated power supplies

## Pin map

| Module signal | NodeMCU label | ESP8266 GPIO / pin | Sketch mode or use |
| --- | --- | ---: | --- |
| HC-SR04 TRIG | D6 | GPIO12 | `OUTPUT` |
| HC-SR04 ECHO | D7 | GPIO13 | `INPUT` |
| Buzzer signal | D5 | GPIO14 | `OUTPUT` |
| MQ-4 analog out (AO) | A0 | ADC input | `analogRead()` |
| OLED SDA | D2 | GPIO4 | I²C data (`Wire`) |
| OLED SCL | D1 | GPIO5 | I²C clock (`Wire`) |
| OLED VCC / GND | Board-specific | — | Power / ground |

**Do not wire by GPIO number alone:** use the printed NodeMCU labels and verify your exact board pinout. ESP8266 board variants differ.

### What `pinMode()` means here

`pinMode(pin, OUTPUT)` configures a pin so the ESP8266 can drive a signal out. HYDRA uses this for the ultrasonic trigger and buzzer. `pinMode(pin, INPUT)` configures a pin to read a digital signal; HYDRA uses it for the ultrasonic echo. The MQ-4 uses the analog input and is read with `analogRead()`. OLED pins are controlled by the I²C library after `Wire.begin()`.

The sketch sets these modes in `setup()`:

```cpp
pinMode(TRIG_PIN, OUTPUT);
pinMode(ECHO_PIN, INPUT);
pinMode(BUZZER_PIN, OUTPUT);
pinMode(MQ4_AO_PIN, INPUT);
```

`INPUT_PULLUP` is another common mode for a switch connected between a GPIO pin and ground, but it is not used by this wiring. `INPUT_PULLDOWN` and `OUTPUT_OPEN_DRAIN` support depends on the specific board/core; you do not need them for this project.

## Wiring safely

1. Disconnect USB and all power before wiring. Connect every module ground to ESP8266 ground (common ground).
2. Power the HC-SR04 and MQ-4 only from a supply appropriate for their module. MQ-4 heaters can draw substantial current; do not assume a GPIO pin can power a sensor.
3. The HC-SR04 Echo output is typically 5 V. ESP8266 GPIO inputs are not 5 V tolerant. Use a resistor divider or a level shifter before GPIO13 (D7). Never connect Echo directly to the ESP8266.
4. Check the maximum voltage allowed on your exact board's A0 input. NodeMCU boards often add an onboard divider, while a bare ESP8266 ADC has a lower input range. If the MQ-4 AO may exceed that limit, add an appropriately calculated divider before connecting A0.
5. Connect OLED SDA to D2 and SCL to D1 on a NodeMCU-style ESP8266. The sketch expects OLED address `0x3C`.
6. Connect buzzer signal to D5 only if the buzzer module is safe to drive from 3.3 V logic. Use a transistor driver for a buzzer that draws more current than a GPIO can supply.
7. Inspect the wiring twice before powering the board. Keep water physically separated from the electronics.

## Build from scratch

1. Install the Arduino IDE and add ESP8266 board support using the official ESP8266 Arduino Core instructions.
2. In Boards Manager, install the ESP8266 platform. Select your NodeMCU/ESP8266 board and the correct USB port.
3. In Library Manager, install **Adafruit GFX Library** and **Adafruit SH110X**. The other listed headers are included with the ESP8266 core or Arduino framework.
4. Copy `firmware/secrets.example.h` to `firmware/secrets.h`. Replace the two example values with your Wi-Fi network name and password. `secrets.h` is ignored by Git; do not add it to a public commit.
5. Open `firmware/water.ino` in Arduino IDE. The `secrets.h` file must remain beside the sketch.
6. Confirm your wiring and voltage levels, connect the board by USB, select the board and port, then click **Upload**.
7. Open Serial Monitor at **115200 baud**. After it joins Wi-Fi, use the printed local IP address in a browser. The device also attempts `http://hydra.local` on networks that support mDNS.
8. If Wi-Fi does not connect, verify the values in your local `secrets.h`, network range, and serial output. The ESP8266 generally requires a 2.4 GHz Wi-Fi network.

## Dashboard modes

- **Hardware mode:** Visit the URL printed by the sketch after Wi-Fi connects. The ESP serves the dashboard and provides `/level` and `/gas` readings.
- **Demo mode:** Open `dashboard/hydra.html` in a browser. It can show simulated dashboard data without a device. Simulated values are not sensor measurements.
- The dashboard's optional weather panel is disabled in this public starter copy. Do not put private API keys in a public HTML file: browser code is visible to anyone who opens it.

## Adjusting thresholds

The main settings are near the top of `firmware/water.ino`: `TANK_EMPTY_CM`, `TANK_FULL_CM`, `ALERT_PERCENT`, `GAS_ALERT_PPM`, and related release values. Calibrate the empty/full distances for your tank. MQ-4 values depend on sensor warm-up, calibration, supply, airflow, and the particular module; do not treat the displayed ppm as a safety measurement.

## Troubleshooting

- **OLED blank:** Check SDA/SCL, power, ground, SH1106 compatibility, and address `0x3C`.
- **Water level stays unchanged:** Check the Echo divider, sensor aim, common ground, and TRIG/ECHO labels. The sketch holds the last valid reading if no echo arrives.
- **Gas reading is implausible:** Allow the MQ-4 to warm up as specified by its manufacturer, confirm the board's A0 range, and verify the load-resistor assumptions in the sketch.
- **Dashboard says simulation:** The device endpoint could not be reached. Use the ESP's printed IP address on the same network, or treat the standalone file as a demo.
- **Buzzer never sounds / always sounds:** Check buzzer polarity and module type, then review threshold and release values.

## Repository layout

```text
firmware/water.ino          ESP8266 firmware
firmware/secrets.example.h  Safe Wi-Fi configuration template
firmware/secrets.h          Your local Wi-Fi settings (ignored by Git)
dashboard/hydra.html        Standalone dashboard and demo mode
docs/WIRING.md              Focused wiring and pin-mode reference
```

## Credits and limitations

This repository organizes the supplied HYDRA prototype into a safer beginner project. Sensor accuracy, electrical compatibility, dashboard behavior, and alarm performance must be verified on the actual hardware before use. Contributions and issue reports are welcome.

