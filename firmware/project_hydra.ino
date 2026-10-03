/*
  ===========================================================================
   PROJECT HYDRA - Smart Sewer / Manhole Monitoring Node
  ===========================================================================
   What it does
     * Measures how full a tank / manhole is with an ultrasonic sensor.
     * Measures methane (CH4) gas with an MQ-4 sensor.
     * Shows both readings on a small OLED screen.
     * Sounds a buzzer when the water level or the gas level is dangerous.
     * Hosts a live web dashboard you can open from any phone or laptop.

   IMPORTANT - which board?
     This sketch is for an ESP8266 board (NodeMCU or Wemos D1 mini).
     It will NOT work on an Arduino Uno: the Uno has no Wi-Fi.

   Quick start (5 steps)
     1. Arduino IDE > File > Preferences > "Additional boards manager URLs",
        add:  http://arduino.esp8266.com/stable/package_esp8266com_index.json
     2. Tools > Board > Boards Manager > install "esp8266".
        Then choose Tools > Board > "NodeMCU 1.0 (ESP-12E Module)".
     3. Sketch > Include Library > Manage Libraries, install:
          - "Adafruit GFX Library"
          - "Adafruit SH110X"
     4. Scroll down to "USER SETTINGS" and type in your own Wi-Fi name and
        password (the values there are only DEMO placeholders).
     5. Upload, open Serial Monitor at 115200 baud, and read the address
        it prints (or just visit  http://hydra.local ).

   If the node cannot join your Wi-Fi it creates its own hotspot instead
   (see AP_SSID below). Join that hotspot and open http://192.168.4.1 .

   Wiring (NodeMCU labels / GPIO numbers)
     HC-SR04 TRIG ..... D6 (GPIO12)
     HC-SR04 ECHO ..... D7 (GPIO13)  <- the ECHO pin outputs 5 V. Put a
                                        1k + 2k voltage divider on it (or use
                                        a 3.3 V-safe sensor) to protect the
                                        ESP8266.
     Buzzer (+) ....... D5 (GPIO14)  (buzzer (-) to GND)
     MQ-4 AO .......... A0           <- A0 only tolerates 3.3 V. If your MQ-4
                                        board outputs up to 5 V, use a divider.
     OLED SDA ......... D2 (GPIO4)
     OLED SCL ......... D1 (GPIO5)
     All sensors: VCC and GND to the board's 3V3/5V (per sensor) and GND.

   Safety note
     This is a learning / prototype project. Do NOT rely on it as the only
     safety system where lives are at risk (confined-space gas detection
     must use certified equipment).
  ===========================================================================
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>        // Wi-Fi support
#include <ESP8266WebServer.h>   // tiny web server for the dashboard
#include <ESP8266mDNS.h>        // lets you use http://hydra.local
#include <Wire.h>               // I2C (used by the OLED)
#include <Adafruit_GFX.h>       // drawing helpers for the OLED
#include <Adafruit_SH110X.h>
#include "secrets.h"
// driver for the SH1106 OLED

// ===========================================================================
//  USER SETTINGS  -  the only part most people need to edit
// ===========================================================================

// --- Wi-Fi your node should join ------------------------------------------
// Wi-Fi credentials are loaded from your local secrets.h file.
// (2.4 GHz networks only - the ESP8266 cannot use 5 GHz Wi-Fi.)
// Keep secrets.h private; it is excluded from Git.
// --- Fallback hotspot -------------------------------------------------------
// Used automatically when the Wi-Fi above is missing or cannot be joined.
// Password must be at least 8 characters. Change it before real use.
// Address shown as http://<name>.local when you are on your own Wi-Fi.
#define MDNS_NAME "hydra"

// --- Tank / manhole size ----------------------------------------------------
// Distances are measured from the sensor face down to the water surface.
#define TANK_EMPTY_CM 30   // reading (cm) when the tank is EMPTY (sensor to bottom)
#define TANK_FULL_CM  2    // reading (cm) when the tank is FULL  (sensor nearly touching)

// --- Water alarm ------------------------------------------------------------
#define ALERT_PERCENT 80   // buzzer turns ON at/above this level (%)
#define ALERT_RELEASE 75   // ...and only turns OFF once back below this (hysteresis,
                           //    so the buzzer does not chatter near the limit)

// --- Methane alarm ----------------------------------------------------------
#define GAS_ALERT_PPM    150   // buzzer ON at/above this methane level (ppm)
#define GAS_RELEASE_PPM  120   // ...and OFF once back below this (hysteresis)
#define GAS_WARN_PPM     100   // "ELEV" (elevated) warning band shown on the OLED

// ===========================================================================
//  HARDWARE PIN SETTINGS  -  change only if you wired things differently
// ===========================================================================
#define TRIG_PIN    12   // D6  ultrasonic trigger
#define ECHO_PIN    13   // D7  ultrasonic echo
#define BUZZER_PIN  14   // D5  buzzer

// ---- MQ-4 methane / natural-gas sensor ------------------------------------
// AO (analog out) goes to A0. The MQ-4 AO can swing up to ~5 V while a NodeMCU
// A0 tolerates ~3.3 V (a bare ESP8266 chip only 1.0 V), so use a divider on AO
// if your module outputs more than the board can take.
#define MQ4_AO_PIN       A0       // analog output pin
#define MQ4_RL_KOHM      10.0     // load resistor on the module (kOhm)
#define MQ4_CLEAN_AIR    4.4      // Rs/Ro ratio in clean air (from the MQ-4 datasheet)
#define MQ4_CURVE_A      1012.7   // ppm = A * (Rs/Ro)^B  - fitted to the CH4 curve
#define MQ4_CURVE_B      -2.786   //        (advanced: leave as is unless you re-fit the curve)

// ---- I2C OLED (SH1106, 128x64 pixels) -------------------------------------
// SDA -> D2/GPIO4, SCL -> D1/GPIO5 (the ESP8266 defaults, set by Wire.begin()).
#define OLED_W        128
#define OLED_H        64
#define OLED_ADDR     0x3C   // most modules use 0x3C; a few use 0x3D
#define OLED_INTERVAL 300    // redraw the screen every 300 ms
Adafruit_SH1106G display(OLED_W, OLED_H, &Wire, -1);

// ---- Timing (milliseconds) --------------------------------------------------
#define SERIAL_BAUD       115200
#define SENSOR_INTERVAL   100    // read the water sensor every 100 ms
#define GAS_INTERVAL      500    // read the gas sensor every 500 ms
#define BUZZER_RAPID      100    // buzzer beeps on/off every 100 ms while alarming
#define WIFI_CHECK_MS     5000   // check the Wi-Fi link every 5 s
#define WIFI_CONNECT_TRIES 30    // 30 tries x 0.5 s = wait up to 15 s at boot

// ===========================================================================
//  PROGRAM STATE  (internal variables - no need to edit)
// ===========================================================================
ESP8266WebServer server(80);   // web server on the standard HTTP port

int   waterLevel = 0;          // tank fill level, 0-100 %
int   gasPPM     = 0;          // estimated methane, in ppm
float mq4Ro      = 10.0;       // MQ-4 resistance in clean air (set by calibration)

bool alarmActive = false;      // true while the WATER alarm is latched on
bool gasAlarm    = false;      // true while the METHANE alarm is latched on
bool buzzerState = false;      // current on/off state of the beeping buzzer
bool oledOk      = false;      // true if the OLED was found at boot
bool apMode      = false;      // true if we are running our own hotspot
bool mdnsOk      = false;      // true if http://hydra.local is available

// "Last time we did X" timers used to schedule work without blocking.
unsigned long lastSensorRead   = 0;
unsigned long lastGasRead      = 0;
unsigned long lastBuzzerToggle = 0;
unsigned long lastWifiCheck    = 0;
unsigned long lastOled         = 0;

// ---- Function declarations (so the order of the code below does not matter) ----
float readDistanceCM();
float readDistanceMedian();
int   distanceToPercent(float dist);
float mq4Rs();
float mq4Calibrate();
int   readGasPPM();
bool  connectToWiFi();
void  startAccessPoint();
void  oledMessage(const char* l1, const char* l2 = "", const char* l3 = "");
void  handleRoot();
void  handleLevel();
void  handleGas();
void  handleNotFound();
void  drawOLED();
void  drawDroplet(int x, int y);
void  drawGasIcon(int x, int y);
void  drawWarn(int x, int y);

// ===========================================================================
//  SENSOR FUNCTIONS
// ===========================================================================

// Fires ONE ultrasonic ping and returns the distance in cm,
// or -1 if no echo came back (timeout / nothing in range).
float readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);          // a 10 microsecond pulse starts the ping
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long d = pulseIn(ECHO_PIN, HIGH, 30000);   // echo time in microseconds (30 ms max)
  if (d == 0) return -1;
  return d * 0.034 / 2.0;                // speed of sound 0.034 cm/us, there and back
}

// Takes up to 5 pings and returns the MIDDLE value (the median).
// HC-SR04 sensors throw out the occasional wild reading; the median ignores
// those, so the level display and buzzer do not jitter.
float readDistanceMedian() {
  const int N = 5;
  float s[N];
  int valid = 0;
  for (int i = 0; i < N; i++) {
    float d = readDistanceCM();
    if (d >= 0) s[valid++] = d;          // keep only pings that got an echo
    delay(8);                            // let trailing echoes settle between pings
  }
  if (valid == 0) return -1;             // every ping failed
  for (int i = 1; i < valid; i++) {      // tiny insertion sort (list is only 5 long)
    float key = s[i];
    int j = i - 1;
    while (j >= 0 && s[j] > key) { s[j + 1] = s[j]; j--; }
    s[j + 1] = key;
  }
  return s[valid / 2];
}

// Converts a distance (cm) into a fill percentage (0-100).
// Close to the sensor = full tank, far away = empty tank.
// Returns -1 when the distance is invalid so the caller can ignore it.
int distanceToPercent(float dist) {
  if (dist < 0) return -1;
  int pct = map((int)dist, TANK_EMPTY_CM, TANK_FULL_CM, 0, 100);
  return constrain(pct, 0, 100);
}

// Works out the MQ-4 sensor's current resistance (Rs) from the analog reading.
// We use the FRACTION of full scale, so the result does not depend on the
// supply voltage.
float mq4Rs() {
  int adc = analogRead(MQ4_AO_PIN);      // 0-1023
  float frac = adc / 1023.0;
  if (frac < 0.001) frac = 0.001;        // avoid dividing by zero when the pin reads 0
  return MQ4_RL_KOHM * (1.0 - frac) / frac;
}

// Measures the sensor in (assumed) CLEAN AIR and works out its baseline
// resistance Ro. Runs once at startup - so power the device on in fresh air!
// For best accuracy an MQ-4 needs a long burn-in (ideally ~24 h on power);
// this quick version gives a working baseline for a demo.
float mq4Calibrate() {
  float rs = 0;
  for (int i = 0; i < 50; i++) { rs += mq4Rs(); delay(20); }   // average 50 readings
  rs /= 50.0;
  return rs / MQ4_CLEAN_AIR;
}

// Converts the sensor reading into an estimated methane concentration (ppm)
// using the datasheet's log-log curve. Treat it as an estimate, not a
// calibrated measurement.
int readGasPPM() {
  float ratio = mq4Rs() / mq4Ro;
  float ppm = MQ4_CURVE_A * pow(ratio, MQ4_CURVE_B);
  if (ppm < 0) ppm = 0;
  if (ppm > 50000) ppm = 50000;          // cap silly values
  return (int)ppm;
}

// ===========================================================================
//  OLED DISPLAY
// ===========================================================================

// Shows up to three short lines of text (used for boot / Wi-Fi status messages).
// Each line fits about 21 characters at this text size.
void oledMessage(const char* l1, const char* l2, const char* l3) {
  if (!oledOk) return;
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);  display.println(l1);
  display.setCursor(0, 16); display.println(l2);
  display.setCursor(0, 28); display.println(l3);
  display.display();
}

// Small icons drawn with simple shapes - no image files needed.
void drawDroplet(int x, int y) {          // water drop symbol, about 10x12 px
  display.fillCircle(x + 5, y + 8, 4, SH110X_WHITE);
  display.fillTriangle(x + 5, y, x + 1, y + 8, x + 9, y + 8, SH110X_WHITE);
  display.fillCircle(x + 5, y + 8, 1, SH110X_BLACK);   // little highlight
}
void drawGasIcon(int x, int y) {          // methane "cloud", about 14x9 px
  display.fillCircle(x + 3,  y + 6, 3, SH110X_WHITE);
  display.fillCircle(x + 7,  y + 5, 4, SH110X_WHITE);
  display.fillCircle(x + 11, y + 6, 3, SH110X_WHITE);
  display.fillRect(x + 3, y + 6, 9, 3, SH110X_WHITE);
}
void drawWarn(int x, int y) {             // warning triangle with a "!"
  display.drawTriangle(x + 5, y, x, y + 9, x + 10, y + 9, SH110X_WHITE);
  display.drawLine(x + 5, y + 3, x + 5, y + 6, SH110X_WHITE);
  display.drawPixel(x + 5, y + 8, SH110X_WHITE);
}

// Draws the main screen:
//   top row    - name, warning sign (if alarming), Wi-Fi / hotspot status
//   middle row - water level % with a progress bar
//   bottom row - methane ppm with a SAFE / ELEV / DANGER label
void drawOLED() {
  if (!oledOk) return;
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);

  // Title + network state ("WiFi" = joined your router, "AP" = own hotspot, "----" = offline)
  display.setTextSize(1);
  display.setCursor(0, 0);   display.print("HYDRA");
  display.setCursor(92, 0);
  if (WiFi.status() == WL_CONNECTED) display.print("WiFi");
  else if (apMode)                   display.print("AP");
  else                               display.print("----");
  if (alarmActive || gasAlarm) drawWarn(78, 0);
  display.drawLine(0, 10, 127, 10, SH110X_WHITE);

  // Water row
  drawDroplet(2, 15);
  display.setTextSize(1);  display.setCursor(16, 15); display.print("WATER");
  display.setTextSize(2);  display.setCursor(66, 13); display.print(waterLevel); display.print("%");
  int bx = 2, by = 32, bw = 124, bh = 6;                       // progress bar outline
  display.drawRoundRect(bx, by, bw, bh, 2, SH110X_WHITE);
  int fw = (int)((bw - 2) * (waterLevel / 100.0));             // filled width
  if (fw > 0) display.fillRoundRect(bx + 1, by + 1, fw, bh - 2, 1, SH110X_WHITE);

  display.drawLine(0, 41, 127, 41, SH110X_WHITE);

  // Gas row
  drawGasIcon(2, 46);
  display.setTextSize(1);  display.setCursor(20, 45); display.print("CH4");
  const char* gs = gasPPM >= GAS_ALERT_PPM ? "DANGER" : gasPPM >= GAS_WARN_PPM ? "ELEV" : "SAFE";
  display.setCursor(20, 55); display.print(gs);
  display.setTextSize(2);  display.setCursor(56, 46); display.print(gasPPM);
  display.setTextSize(1);
  display.setCursor(display.getCursorX() + 2, 52); display.print("ppm");

  display.display();                     // push everything to the screen
}

// ===========================================================================
//  WEB DASHBOARD  (the page your phone / laptop shows)
// ===========================================================================
// The whole dashboard is stored below as one block of HTML/CSS/JavaScript
// inside the sketch, so no SD card or extra files are needed.
//
// Notes for anyone customising it:
//   * The page asks this node for /level and /gas every ~0.8 s.
//   * The map, fonts and weather forecast load from the internet in the
//     VIEWER'S browser. In hotspot mode (no internet) those parts are
//     skipped, but the live water/gas numbers still work.
//   * Weather: to enable the rain-aware alert threshold, paste your free
//     OpenWeatherMap key into  weatherAPIKey  below (search for it).
//     Never publish a real key in a public repository.
//   * The other 11 "nodes" on the dashboard are SIMULATED demo data. Only
//     "Node 1" is fed by this real hardware.
const char PAGE[] PROGMEM = R"HYDRA(
<!doctype html>
<html lang="en" data-theme="light">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width,initial-scale=1" />
    <title>Project HYDRA | Sanitation Node</title>
    <link
      rel="icon"
      type="image/svg+xml"
      href="data:image/svg+xml,%3Csvg%20xmlns%3D%27http%3A//www.w3.org/2000/svg%27%20viewBox%3D%270%200%2064%2064%27%3E%3Crect%20width%3D%2764%27%20height%3D%2764%27%20rx%3D%2716%27%20fill%3D%27%230ea5e9%27/%3E%3Cg%20transform%3D%27translate%288%2C8%29%20scale%282%29%27%20fill%3D%27none%27%20stroke%3D%27%23fff%27%20stroke-width%3D%271.7%27%20stroke-linecap%3D%27round%27%20stroke-linejoin%3D%27round%27%3E%3Cpath%20d%3D%27M12%202.5c0%200%206.6%206.8%206.6%2011.1a6.6%206.6%200%200%201-13.2%200C5.4%209.3%2012%202.5%2012%202.5Z%27%20fill%3D%27%23fff%27%20fill-opacity%3D%270.16%27/%3E%3C/g%3E%3C/svg%3E"
    />
    <link rel="preconnect" href="https://fonts.googleapis.com" />
    <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin />
    <link
      href="https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700;800&display=swap"
      rel="stylesheet"
    />
    <link
      rel="stylesheet"
      href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"
    />
    <style>
      :root {
        --bg: #f8fafc;
        --panel: #ffffff;
        --line: #e2e8f0;
        --text: #0f172a;
        --muted: #64748b;
        --dim: #94a3b8;
        --green: #10b981;
        --amber: #f59e0b;
        --red: #ef4444;
        --cyan: #0ea5e9;
        --blue: #3b82f6;
        --shadow: 0 4px 20px rgba(0, 0, 0, 0.04);
        --sewage: rgba(94, 105, 84, 0.85);
        --sewage-light: rgba(126, 138, 114, 0.5);
        --vessel-bg: #f1f5f9;
      }
      [data-theme="dark"] {
        --bg: #050607;
        --panel: #0b0e10;
        --line: #1c2428;
        --text: #f4f7f7;
        --muted: #829099;
        --dim: #4e5b62;
        --shadow: 0 8px 30px rgba(0, 0, 0, 0.4);
        --vessel-bg: #101417;
      }
      * {
        box-sizing: border-box;
      }
      html,
      body {
        min-height: 100%;
        margin: 0;
      }
      body {
        background: var(--bg);
        color: var(--text);
        font-family: "Inter", sans-serif;
        -webkit-font-smoothing: antialiased;
        text-rendering: optimizeLegibility;
        overflow-x: hidden;
        overflow-y: auto;
        transition:
          background 0.3s ease,
          color 0.3s ease;
      }
      button {
        font: inherit;
        border: none;
        outline: none;
      }
      .app {
        display: flex;
        min-height: 100vh;
      }
      .side {
        position: fixed;
        inset: 0 auto 0 0;
        width: 240px;
        background: var(--panel);
        border-right: 1px solid var(--line);
        padding: 24px 20px;
        display: flex;
        flex-direction: column;
        gap: 20px;
        z-index: 3;
        transition:
          background 0.3s,
          border-color 0.3s;
      }
      .brand {
        display: flex;
        align-items: center;
        gap: 12px;
        padding: 4px 4px 14px;
      }
      .mark {
        width: 44px;
        height: 44px;
        border-radius: 13px;
        display: grid;
        place-items: center;
        background: #3b82f6;
        color: white;
        font-weight: 800;
        font-size: 16px;
        letter-spacing: 0.04em;
        box-shadow: 0 6px 16px rgba(59, 130, 246, 0.3);
        flex-shrink: 0;
      }
      .brand h1 {
        font-size: 18px;
        margin: 0;
        letter-spacing: 0.05em;
        color: var(--text);
      }
      .brand p {
        font-size: 11px;
        margin: 3px 0 0;
        color: var(--muted);
      }
      .nav {
        display: flex;
        flex-direction: column;
        gap: 4px;
      }
      .nav a {
        text-decoration: none;
        color: var(--muted);
        display: flex;
        align-items: center;
        gap: 12px;
        min-height: 40px;
        padding: 0 10px;
        border-radius: 10px;
        font-size: 14px;
        font-weight: 500;
        transition: 0.2s;
      }
      .nav a span:first-child {
        width: 36px;
        height: 36px;
        border-radius: 9px;
        display: grid;
        place-items: center;
        flex-shrink: 0;
        color: var(--muted);
        background: transparent;
      }
      .nav a.active,
      .nav a:hover {
        color: var(--text);
        background: var(--line);
      }
      .nav a.active span:first-child {
        color: var(--text);
      }
      .main {
        margin-left: 240px;
        width: calc(100% - 240px);
        min-height: 100vh;
      }
      .top {
        position: sticky;
        top: 0;
        z-index: 2;
        display: flex;
        align-items: center;
        justify-content: space-between;
        gap: 16px;
        padding: 24px 32px 16px;
        background: var(--bg);
        opacity: 0.95;
        backdrop-filter: blur(12px);
        transition: background 0.3s;
      }
      .title h2 {
        margin: 0;
        font-size: 26px;
        letter-spacing: -0.02em;
        font-weight: 700;
      }
      .title p {
        margin: 6px 0 0;
        color: var(--muted);
        font-size: 14px;
      }
      .actions {
        display: flex;
        gap: 10px;
        align-items: center;
        flex-wrap: wrap;
        justify-content: flex-end;
      }
      .pill,
      .status {
        border: 1px solid var(--line);
        background: var(--panel);
        color: var(--text);
        border-radius: 999px;
        padding: 8px 14px;
        font-size: 12px;
        font-weight: 500;
        display: flex;
        gap: 8px;
        align-items: center;
        white-space: nowrap;
        box-shadow: 0 2px 8px rgba(0, 0, 0, 0.02);
        transition:
          background 0.3s,
          border-color 0.3s,
          color 0.3s;
      }
      .status i {
        width: 8px;
        height: 8px;
        border-radius: 50%;
        background: var(--green);
      }
      .status.sim i {
        background: var(--amber);
      }
      .btn {
        cursor: pointer;
      }
      .btn:hover {
        background: var(--line);
      }
      .content {
        padding: 0 32px 32px;
      }
      .content.page {
        padding-top: 28px;
      }
      .page[hidden] {
        display: none;
      }
      .page-head {
        margin-bottom: 24px;
      }
      .page-head h3 {
        margin: 0 0 6px;
        font-size: 20px;
        font-weight: 700;
        letter-spacing: -0.01em;
      }
      .page-head p {
        margin: 0;
        color: var(--muted);
        font-size: 14px;
      }
      .metrics {
        display: grid;
        grid-template-columns: repeat(5, minmax(140px, 1fr));
        gap: 16px;
        margin-bottom: 24px;
      }
      .metric {
        background: var(--panel);
        border: 1px solid var(--line);
        border-radius: 16px;
        padding: 20px;
        min-height: 112px;
        box-shadow: var(--shadow);
        transition:
          background 0.3s,
          border-color 0.3s;
      }
      .metric small {
        display: block;
        color: var(--muted);
        font-size: 13px;
        font-weight: 600;
        margin-bottom: 12px;
      }
      .metric strong {
        font-size: 32px;
        font-weight: 800;
        line-height: 1;
        letter-spacing: -0.03em;
      }
      .metric p {
        margin: 8px 0 0;
        color: var(--dim);
        font-size: 12px;
        font-weight: 500;
      }
      .grid {
        display: grid;
        grid-template-columns: 1.2fr 0.8fr;
        gap: 20px;
        margin-bottom: 24px;
        align-items: stretch;
      }
      .chart-card,
      .zone-card {
        background: var(--panel);
        border: 1px solid var(--line);
        border-radius: 16px;
        padding: 20px;
        box-shadow: var(--shadow);
        transition:
          background 0.3s,
          border-color 0.3s;
      }
      .chart-card {
        display: flex;
        flex-direction: column;
      }
      .card-head {
        display: flex;
        align-items: flex-start;
        justify-content: space-between;
        gap: 12px;
        margin-bottom: 16px;
      }
      .card-head h3 {
        margin: 0;
        font-size: 16px;
        font-weight: 600;
        color: var(--text);
      }
      .card-head p {
        margin: 4px 0 0;
        color: var(--muted);
        font-size: 13px;
      }
      .legend {
        display: flex;
        gap: 12px;
        color: var(--muted);
        font-size: 12px;
        flex-wrap: wrap;
      }
      .legend span {
        display: flex;
        gap: 6px;
        align-items: center;
      }
      .legend i {
        width: 8px;
        height: 8px;
        border-radius: 50%;
        display: block;
      }
      .canvas-wrap {
        flex: 1;
        min-height: 200px;
        position: relative;
        border-radius: 12px;
        overflow: hidden;
        background: var(--bg);
        border: 1px solid var(--line);
        cursor: crosshair;
      }
      canvas {
        width: 100%;
        height: 100%;
        display: block;
      }
      .zone-list {
        display: grid;
        gap: 12px;
        max-height: 390px;
        overflow-y: auto;
        padding-right: 8px;
        scrollbar-width: thin;
        scrollbar-color: var(--dim) transparent;
      }
      .zone-list::-webkit-scrollbar {
        width: 8px;
      }
      .zone-list::-webkit-scrollbar-track {
        background: transparent;
      }
      .zone-list::-webkit-scrollbar-thumb {
        background: var(--dim);
        border-radius: 99px;
        border: 2px solid transparent;
        background-clip: padding-box;
      }
      .zone-list::-webkit-scrollbar-thumb:hover {
        background: var(--muted);
        background-clip: padding-box;
        border: 2px solid transparent;
      }
      .zone-mini {
        color: inherit;
        text-decoration: none;
        display: grid;
        grid-template-columns: 1fr auto;
        gap: 10px;
        padding: 14px;
        border: 1px solid var(--line);
        border-radius: 12px;
        background: var(--bg);
        cursor: pointer;
        transition: 0.2s ease;
      }
      .zone-mini:hover,
      .zone-mini.active {
        border-color: var(--dim);
        box-shadow: 0 4px 12px rgba(0, 0, 0, 0.05);
      }
      .zone-mini h4 {
        margin: 0;
        font-size: 14px;
        font-weight: 600;
      }
      .zone-mini p {
        margin: 4px 0 0;
        color: var(--muted);
        font-size: 12px;
      }
      .mini-pct {
        font-weight: 700;
        font-size: 20px;
        line-height: 1;
      }
      .mini-bar {
        grid-column: 1 / -1;
        height: 6px;
        border-radius: 99px;
        background: var(--line);
        overflow: hidden;
        margin-top: 4px;
      }
      .mini-bar div {
        height: 100%;
        border-radius: 99px;
        transition:
          width 0.4s,
          background 0.4s;
      }
      /* ── tank cards (compact row layout) ── */
      .cards {
        display: grid;
        grid-template-columns: repeat(2, minmax(0, 1fr));
        gap: 12px;
      }
      .cards--3col {
        grid-template-columns: repeat(3, minmax(0, 1fr));
        gap: 16px;
      }
      .tank-card {
        background: var(--panel);
        border: 1px solid var(--line);
        border-radius: 16px;
        padding: 20px 22px;
        box-shadow: var(--shadow);
        transition:
          background 0.3s,
          border-color 0.3s;
        display: flex;
        flex-direction: column;
        gap: 14px;
      }
      /* top row: name + tag */
      .tank-top {
        display: flex;
        align-items: center;
        justify-content: space-between;
        gap: 12px;
      }
      .tank-id {
        display: flex;
        align-items: center;
        gap: 10px;
        min-width: 0;
      }
      .icon {
        display: none;
      } /* removed — adds no info */
      .tank-id h3 {
        margin: 0;
        font-size: 14px;
        font-weight: 700;
        white-space: nowrap;
        overflow: hidden;
        text-overflow: ellipsis;
      }
      .tank-id p {
        margin: 2px 0 0;
        color: var(--muted);
        font-size: 12px;
      }
      .tag {
        font-size: 11px;
        font-weight: 600;
        border-radius: 999px;
        padding: 4px 10px;
        border: 1px solid transparent;
        white-space: nowrap;
      }
      .tag.ok {
        color: var(--green);
        background: rgba(16, 185, 129, 0.08);
        border-color: rgba(16, 185, 129, 0.2);
      }
      .tag.warn {
        color: var(--amber);
        background: rgba(245, 158, 11, 0.08);
        border-color: rgba(245, 158, 11, 0.2);
      }
      .tag.crit {
        color: var(--red);
        background: rgba(239, 68, 68, 0.08);
        border-color: rgba(239, 68, 68, 0.2);
      }
      /* fill bar row */
      .tank-fill-row {
        display: flex;
        align-items: center;
        gap: 10px;
      }
      .tank-fill-pct {
        font-size: 20px;
        font-weight: 800;
        letter-spacing: -0.03em;
        min-width: 48px;
        line-height: 1;
      }
      .tank-fill-bar-wrap {
        flex: 1;
        height: 8px;
        border-radius: 99px;
        background: var(--line);
        overflow: hidden;
      }
      .tank-fill-bar {
        height: 100%;
        border-radius: 99px;
        transition:
          width 0.7s cubic-bezier(0.2, 0.8, 0.2, 1),
          background 0.4s;
      }
      /* stats row */
      .tank-stats {
        display: flex;
        gap: 0;
        border-top: 1px solid var(--line);
        padding-top: 10px;
      }
      .tank-stat {
        flex: 1;
      }
      .tank-stat span {
        display: block;
        color: var(--muted);
        font-size: 11px;
        font-weight: 500;
        margin-bottom: 3px;
      }
      .tank-stat b {
        font-size: 14px;
        font-weight: 700;
        color: var(--text);
      }
      .tank-stat + .tank-stat {
        border-left: 1px solid var(--line);
        padding-left: 14px;
        margin-left: 14px;
      }
      /* keep kv/label/vessel/telemetry/wave defs but they won't be used */
      .kv {
        display: none;
      }
      .vessel {
        display: none;
      }
      .telemetry {
        display: none;
      }
      .label {
        display: none;
      }
      .tank-body {
        display: none;
      }
      .toast {
        position: fixed;
        right: 24px;
        bottom: 24px;
        z-index: 4;
        background: var(--text);
        border: 1px solid var(--text);
        color: var(--bg);
        border-radius: 12px;
        padding: 14px 18px;
        font-size: 13px;
        font-weight: 500;
        box-shadow: 0 12px 32px rgba(0, 0, 0, 0.15);
        opacity: 0;
        transform: translateY(15px);
        pointer-events: none;
        transition: 0.3s cubic-bezier(0.2, 0.8, 0.2, 1);
      }
      .toast.show {
        opacity: 1;
        transform: translateY(0);
      }
      #map {
        height: 500px;
        border-radius: 16px;
        border: 1px solid var(--line);
        z-index: 1;
      }
      .map-legend {
        background: var(--panel);
        color: var(--text);
        border: 1px solid var(--line);
        border-radius: 10px;
        padding: 8px 12px;
        font-size: 12px;
        font-weight: 500;
        display: flex;
        flex-direction: column;
        gap: 5px;
        box-shadow: var(--shadow);
      }
      .map-legend span {
        display: flex;
        align-items: center;
        gap: 7px;
      }
      .map-legend i {
        width: 10px;
        height: 10px;
        border-radius: 50%;
        display: block;
      }
      .leaflet-popup-content-wrapper {
        border-radius: 10px;
      }
      .leaflet-popup-content {
        font-family: "Inter", sans-serif;
        font-size: 13px;
        line-height: 1.5;
      }
      .tank-card,
      .metric {
        transition:
          transform 0.18s ease,
          box-shadow 0.18s ease,
          background 0.3s,
          border-color 0.3s;
      }
      .tank-card:hover {
        transform: translateY(-2px);
        box-shadow: 0 8px 24px rgba(0, 0, 0, 0.08);
      }
      /* ── node detail page ── */
      .detail-head {
        display: flex;
        align-items: flex-start;
        justify-content: space-between;
        gap: 16px;
        margin-bottom: 24px;
      }
      .detail-head h3 {
        margin: 0 0 6px;
        font-size: 22px;
        font-weight: 700;
        letter-spacing: -0.01em;
      }
      .detail-head p {
        margin: 0;
        color: var(--muted);
        font-size: 14px;
      }
      #dBack {
        cursor: pointer;
      }
      .metrics--4 {
        grid-template-columns: repeat(4, minmax(150px, 1fr));
      }
      .detail-vessel {
        display: flex;
        flex-direction: column;
        align-items: center;
      }
      .cyl {
        position: relative;
        width: 210px;
        height: 300px;
        margin: 8px 0 4px;
        border-radius: 26px;
        overflow: hidden;
        background: var(--vessel-bg);
        border: 1px solid var(--line);
        box-shadow: inset 0 3px 10px rgba(0, 0, 0, 0.08);
      }
      .cyl-fill {
        position: absolute;
        left: 0;
        right: 0;
        bottom: 0;
        height: 50%;
        background: linear-gradient(180deg, #5eead4 0%, #0ea5e9 100%);
        transition:
          height 0.8s cubic-bezier(0.2, 0.8, 0.2, 1),
          background 0.5s ease;
        box-shadow: 0 -6px 20px rgba(14, 165, 233, 0.25);
      }
      .cyl-fill::before {
        content: "";
        position: absolute;
        top: -9px;
        left: -10%;
        width: 120%;
        height: 18px;
        border-radius: 50%;
        background: rgba(255, 255, 255, 0.35);
        animation: cylbob 4.5s ease-in-out infinite;
      }
      .cyl-fill::after {
        content: "";
        position: absolute;
        top: 0;
        left: 0;
        right: 0;
        height: 40%;
        background: linear-gradient(
          180deg,
          rgba(255, 255, 255, 0.22),
          rgba(255, 255, 255, 0)
        );
      }
      @keyframes cylbob {
        0%,
        100% {
          transform: translateX(-4%);
        }
        50% {
          transform: translateX(4%);
        }
      }
      .cyl-pct {
        position: absolute;
        inset: 0;
        display: grid;
        place-items: center;
        font-size: 46px;
        font-weight: 800;
        letter-spacing: -0.03em;
        color: #ffffff;
        text-shadow: 0 2px 8px rgba(0, 0, 0, 0.28);
      }
      .detail-stats {
        width: 100%;
        margin-top: 18px;
      }
      /* network summary row */
      .summary-grid {
        display: grid;
        grid-template-columns: 1fr 1fr;
        gap: 20px;
        margin-bottom: 24px;
        align-items: stretch;
      }
      .dist-bar {
        display: flex;
        height: 16px;
        border-radius: 99px;
        overflow: hidden;
        background: var(--line);
      }
      .dist-bar span {
        display: block;
        height: 100%;
        transition: width 0.5s ease;
      }
      .dist-legend {
        display: flex;
        gap: 20px;
        margin-top: 16px;
        flex-wrap: wrap;
      }
      .dist-legend > span {
        display: flex;
        align-items: center;
        gap: 8px;
        font-size: 13px;
        color: var(--muted);
      }
      .dist-legend i {
        width: 10px;
        height: 10px;
        border-radius: 50%;
        display: block;
      }
      .dist-legend b {
        color: var(--text);
        font-weight: 700;
      }
      .flow-stats {
        display: grid;
        grid-template-columns: repeat(3, 1fr);
        gap: 12px;
      }
      .flow-stat {
        padding: 4px 0;
      }
      .flow-stat span {
        display: block;
        color: var(--muted);
        font-size: 12px;
        font-weight: 600;
        margin-bottom: 6px;
      }
      .flow-stat b {
        font-size: 26px;
        font-weight: 800;
        letter-spacing: -0.02em;
        line-height: 1;
      }
      .flow-stat small {
        display: block;
        margin-top: 4px;
        color: var(--dim);
        font-size: 11px;
        font-weight: 500;
      }
      .flow-stat + .flow-stat {
        border-left: 1px solid var(--line);
        padding-left: 16px;
      }
      @media (max-width: 1000px) {
        .summary-grid {
          grid-template-columns: 1fr;
        }
      }
      /* skeleton loading */
      @keyframes shimmer {
        0%,
        100% {
          opacity: 0.4;
        }
        50% {
          opacity: 0.9;
        }
      }
      .skeleton-val {
        animation: shimmer 1.4s ease-in-out infinite;
      }
      .skeleton-line {
        animation: shimmer 1.4s ease-in-out 0.2s infinite;
      }

      /* metric hierarchy */
      .metric--primary {
        border-color: var(--sewage);
        background: linear-gradient(
          135deg,
          var(--panel) 0%,
          rgba(94, 105, 84, 0.06) 100%
        );
      }
      .metric--primary strong {
        font-size: 42px;
      }

      /* header action group */
      .action-group {
        display: flex;
        gap: 6px;
      }
      .action-group .btn {
        padding: 8px 10px;
      }
      .export-btn {
        padding: 8px 14px !important;
      }

      /* weather pill */
      .weather-icon {
        font-style: normal;
      }

      /* net flow in card */
      .net-flow b {
        font-size: 15px;
      }

      .metric-sub {
        margin: 8px 0 0;
        color: var(--dim);
        font-size: 12px;
        font-weight: 500;
      }

      @media (max-width: 1200px) {
        .cards--3col {
          grid-template-columns: repeat(2, 1fr);
        }
        .grid {
          grid-template-columns: 1fr;
        }
      }
      @media (max-width: 1120px) {
        .actions {
          flex-direction: column;
          align-items: stretch;
          width: 100%;
          margin-top: 10px;
        }
        .top {
          flex-direction: column;
          align-items: flex-start;
        }
      }
      @media (max-width: 840px) {
        body {
          overflow: auto;
        }
        .app {
          display: block;
        }
        .side {
          position: relative;
          width: auto;
          inset: auto;
          flex-direction: row;
          align-items: center;
          overflow: auto;
          padding: 16px 20px;
          border-right: none;
          border-bottom: 1px solid var(--line);
        }
        .brand {
          padding: 0;
          min-width: 150px;
        }
        .nav {
          flex-direction: row;
          gap: 12px;
        }
        .nav a {
          min-width: 44px;
          padding: 0 10px;
        }
        .nav a span + span {
          display: none;
        }
        .main {
          margin-left: 0;
          width: auto;
          min-height: auto;
        }
        .top {
          position: relative;
          align-items: flex-start;
          flex-direction: column;
          padding: 20px;
        }
        .actions {
          flex-direction: row;
          align-items: center;
          width: auto;
        }
        .content {
          padding: 0 20px 24px;
        }
        .metrics {
          grid-template-columns: repeat(2, minmax(0, 1fr));
        }
        .cards {
          grid-template-columns: 1fr;
        }
      }
      @media (max-width: 520px) {
        .metrics {
          grid-template-columns: 1fr;
        }
        .tank-top {
          align-items: flex-start;
          flex-direction: column;
        }
        .canvas-wrap {
          height: 220px;
        }
        .tank-stats {
          flex-wrap: wrap;
          gap: 8px;
        }
        .tank-stat + .tank-stat {
          border-left: none;
          padding-left: 0;
          margin-left: 0;
          border-top: 1px solid var(--line);
          padding-top: 8px;
        }
      }
    </style>
  </head>
  <body>
    <div class="app">
      <aside class="side">
        <div class="brand">
          <div class="mark">
            <svg
              width="26"
              height="26"
              viewBox="0 0 24 24"
              fill="none"
              stroke="#ffffff"
              stroke-width="1.7"
              stroke-linecap="round"
              stroke-linejoin="round"
            >
              <path
                d="M12 2.5c0 0 6.6 6.8 6.6 11.1a6.6 6.6 0 0 1-13.2 0C5.4 9.3 12 2.5 12 2.5Z"
                fill="#ffffff"
                fill-opacity="0.16"
              />
            </svg>
          </div>
          <div>
            <h1>HYDRA</h1>
            <p>Urban Sanitation</p>
          </div>
        </div>
        <nav class="nav">
          <a class="active" href="#overview" data-page="overview">
            <span
              ><svg
                width="20"
                height="20"
                viewBox="0 0 24 24"
                fill="none"
                stroke="currentColor"
                stroke-width="2"
                stroke-linecap="round"
                stroke-linejoin="round"
              >
                <rect x="3" y="3" width="7" height="7" rx="1" />
                <rect x="14" y="3" width="7" height="7" rx="1" />
                <rect x="3" y="14" width="7" height="7" rx="1" />
                <rect x="14" y="14" width="7" height="7" rx="1" /></svg
            ></span>
            <span>Overview</span>
          </a>
          <a href="#nodes" data-page="nodes">
            <span
              ><svg
                width="20"
                height="20"
                viewBox="0 0 24 24"
                fill="none"
                stroke="currentColor"
                stroke-width="2"
                stroke-linecap="round"
                stroke-linejoin="round"
              >
                <ellipse cx="12" cy="5" rx="3" ry="2" />
                <ellipse cx="4" cy="19" rx="3" ry="2" />
                <ellipse cx="20" cy="19" rx="3" ry="2" />
                <path d="M12 7v4m0 0-6 6m6-6 6 6" /></svg
            ></span>
            <span>Nodes</span>
          </a>
          <a href="#zones" data-page="zones">
            <span
              ><svg
                width="20"
                height="20"
                viewBox="0 0 24 24"
                fill="none"
                stroke="currentColor"
                stroke-width="2"
                stroke-linecap="round"
                stroke-linejoin="round"
              >
                <path
                  d="M12 2C8.686 2 6 4.686 6 8c0 4.5 6 12 6 12s6-7.5 6-12c0-3.314-2.686-6-6-6z"
                />
                <circle cx="12" cy="8" r="2" /></svg
            ></span>
            <span>Map</span>
          </a>
        </nav>
      </aside>

      <main class="main">
        <header class="top">
          <div class="title">
            <h2>System Dashboard</h2>
            <p>
              Water-level and gas monitoring dashboard · simulated node network
            </p>
          </div>
          <div class="actions">
            <div class="pill weather-pill" id="weatherBadge">
              <svg
                width="16"
                height="16"
                viewBox="0 0 24 24"
                fill="none"
                stroke="currentColor"
                stroke-width="2"
                stroke-linecap="round"
                stroke-linejoin="round"
                class="weather-svg"
              >
                <path d="M20 17.58A5 5 0 0 0 18 8h-1.26A8 8 0 1 0 4 16.25" />
                <line x1="8" y1="19" x2="8" y2="21" />
                <line x1="8" y1="13" x2="8" y2="15" />
                <line x1="16" y1="19" x2="16" y2="21" />
                <line x1="16" y1="13" x2="16" y2="15" />
                <line x1="12" y1="21" x2="12" y2="23" />
                <line x1="12" y1="15" x2="12" y2="17" />
              </svg>
              <span id="weatherText">Loading...</span>
            </div>
            <div class="pill threshold-pill" id="thresholdBadge">
              Threshold: 80%
            </div>
            <div class="action-group">
              <button
                class="pill btn"
                id="btnTheme"
                title="Toggle theme"
                aria-label="Toggle light/dark theme"
              >
                <svg
                  width="16"
                  height="16"
                  viewBox="0 0 24 24"
                  fill="none"
                  stroke="currentColor"
                  stroke-width="2"
                  stroke-linecap="round"
                  stroke-linejoin="round"
                >
                  <circle cx="12" cy="12" r="5" />
                  <path
                    d="M12 1v2M12 21v2M4.22 4.22l1.42 1.42M18.36 18.36l1.42 1.42M1 12h2M21 12h2M4.22 19.78l1.42-1.42M18.36 5.64l1.42-1.42"
                  />
                </svg>
              </button>
              <button
                class="pill btn"
                id="btnRefresh"
                title="Refresh data"
                aria-label="Refresh sensor data"
              >
                <svg
                  width="16"
                  height="16"
                  viewBox="0 0 24 24"
                  fill="none"
                  stroke="currentColor"
                  stroke-width="2"
                  stroke-linecap="round"
                  stroke-linejoin="round"
                >
                  <path
                    d="M3 12a9 9 0 0 1 9-9 9.75 9.75 0 0 1 6.74 2.74L21 8"
                  />
                  <path d="M21 3v5h-5" />
                  <path
                    d="M21 12a9 9 0 0 1-9 9 9.75 9.75 0 0 1-6.74-2.74L3 16"
                  />
                  <path d="M8 16H3v5" />
                </svg>
              </button>
              <button class="pill btn export-btn" id="btnExport">
                Export CSV
              </button>
            </div>
            <div class="status" id="modeBadge">
              <i></i><span>Connecting</span>
            </div>
          </div>
        </header>

        <section class="content page" id="overviewPage">
          <div class="metrics">
            <div class="metric metric--primary">
              <small>Node 1 Node</small
              ><strong id="mLive" class="skeleton-val">--%</strong>
              <p id="mLiveSub" class="metric-sub skeleton-line">
                Connecting to sensor…
              </p>
            </div>
            <div class="metric">
              <small>Surface Gap</small
              ><strong id="mDistance" class="skeleton-val">-- cm</strong>
              <p class="metric-sub">Ultrasonic distance</p>
            </div>
            <div class="metric">
              <small>Critical Manholes</small
              ><strong id="mCritical" class="skeleton-val">–</strong>
              <p id="mCriticalSub" class="metric-sub skeleton-line">
                Checking…
              </p>
            </div>
            <div class="metric">
              <small>Methane · Node 1</small
              ><strong id="mGas" class="skeleton-val">-- ppm</strong>
              <p id="mGasSub" class="metric-sub skeleton-line">MQ-4 sensor</p>
            </div>
            <div class="metric">
              <small>Network Load</small
              ><strong id="mAverage" class="skeleton-val">--%</strong>
              <p class="metric-sub" id="mAvgSub">network average</p>
            </div>
          </div>

          <div class="grid">
            <section class="chart-card">
              <div class="card-head">
                <div>
                  <h3>Real-time Distance</h3>
                  <p>Node 1 incoming readings. Hover for precise values.</p>
                </div>
                <div class="legend">
                  <span><i style="background: var(--cyan)"></i>Distance cm</span
                  ><span
                    ><i style="background: var(--red)"></i>Backup limit</span
                  >
                </div>
              </div>
              <div class="canvas-wrap"><canvas id="trendCanvas"></canvas></div>
            </section>

            <aside class="zone-card">
              <div class="card-head">
                <div>
                  <h3>Node Index</h3>
                  <p>Tap a node to inspect.</p>
                </div>
              </div>
              <div class="zone-list" id="zoneList"></div>
            </aside>
          </div>

          <section class="summary-grid">
            <div class="chart-card">
              <div class="card-head">
                <div>
                  <h3>Status Distribution</h3>
                  <p>Live breakdown across all nodes.</p>
                </div>
              </div>
              <div class="dist-bar" id="distBar"></div>
              <div class="dist-legend" id="distLegend"></div>
            </div>
            <div class="chart-card">
              <div class="card-head">
                <div>
                  <h3>Network Flow</h3>
                  <p>Aggregate throughput, all nodes.</p>
                </div>
              </div>
              <div class="flow-stats" id="flowStats"></div>
            </div>
          </section>
        </section>

        <section class="content page" id="nodesPage" hidden>
          <div class="page-head">
            <h3>All Nodes</h3>
            <p id="nodesSub">Live readings from the connected device; other nodes are simulated.</p>
          </div>
          <section class="cards cards--3col" id="tankCards"></section>
        </section>

        <section class="content page" id="zonesPage" hidden>
          <div class="page-head">
            <h3>Geographic Map</h3>
            <p>Illustrative positions for simulated demo nodes.</p>
          </div>
          <div id="map"></div>
        </section>

        <section class="content page" id="detailPage" hidden>
          <div class="detail-head">
            <div>
              <h3 id="dTitle">Node</h3>
              <p id="dMeta">—</p>
            </div>
            <button class="pill btn" id="dBack">&#8592; Back to Nodes</button>
          </div>
          <div class="metrics metrics--4">
            <div class="metric metric--primary">
              <small>Current Volume (L)</small><strong id="dVol">—</strong>
              <p class="metric-sub" id="dStatus">—</p>
            </div>
            <div class="metric">
              <small>Flow Capacity (L)</small><strong id="dCap">—</strong>
              <p class="metric-sub">Design capacity</p>
            </div>
            <div class="metric">
              <small>Methane (CH&#8324;)</small><strong id="dGas">—</strong>
              <p class="metric-sub" id="dGasSub">—</p>
            </div>
            <div class="metric">
              <small>Net Flow</small><strong id="dNet">—</strong>
              <p class="metric-sub">Inflow − Outflow</p>
            </div>
          </div>
          <div class="grid">
            <section class="chart-card">
              <div class="card-head">
                <div>
                  <h3>Real-time Distance</h3>
                  <p id="dChartSub">Incoming readings.</p>
                </div>
                <div class="legend">
                  <span><i style="background: var(--cyan)"></i>Distance cm</span
                  ><span
                    ><i style="background: var(--red)"></i>Backup limit</span
                  >
                </div>
              </div>
              <div class="canvas-wrap"><canvas id="detailCanvas"></canvas></div>
            </section>
            <aside class="zone-card detail-vessel">
              <div class="card-head" style="width: 100%">
                <div>
                  <h3>Sewage Level</h3>
                  <p>Live fill</p>
                </div>
              </div>
              <div class="cyl" id="dCyl">
                <div class="cyl-fill" id="dCylFill"></div>
                <div class="cyl-pct" id="dCylPct">—</div>
              </div>
              <div class="tank-stats detail-stats">
                <div class="tank-stat">
                  <span>Surface Gap</span><b id="dGap">—</b>
                </div>
                <div class="tank-stat">
                  <span>Inflow</span><b id="dIn">—</b>
                </div>
                <div class="tank-stat">
                  <span>Outflow</span><b id="dOut">—</b>
                </div>
              </div>
            </aside>
          </div>
          <section class="chart-card" style="margin-top: 20px">
            <div class="card-head">
              <div>
                <h3>Methane Trend</h3>
                <p id="dGasChartSub">Live CH₄ readings.</p>
              </div>
              <div class="legend">
                <span><i style="background: var(--amber)"></i>CH₄ ppm</span
                ><span><i style="background: var(--red)"></i>Danger limit</span>
              </div>
            </div>
            <div class="canvas-wrap" style="flex: none; height: 200px">
              <canvas id="detailGasCanvas"></canvas>
            </div>
          </section>
        </section>
      </main>
    </div>
    <div class="toast" id="toast">
      Sensor endpoint unavailable.<br />Simulation mode is now active.
    </div>

    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    <script>
      (function () {
        var EMPTY_CM = 30,
          FULL_CM = 2;
        var ALERT_PERCENT = 80; // Dynamically updated by weather
        var POLL_MS = 800,
          TIMEOUT_MS = 850,
          TREND_MS = 60000;
        var simulation = false,
          focused = "krpuram",
          lastToast = 0;
        var lastTick = Date.now();
        var series = [];

        // Weather API Setup (Replace with your actual OpenWeatherMap API Key)
        var weatherAPIKey = "YOUR_OPENWEATHER_API_KEY_HERE";
        // Demo Region Coordinates
        var blrLat = 12.9716,
          blrLon = 77.5946;

        var zones = [
          {
            id: "demo-node-01",
            name: "Demo Node 01",
            meta: "Ward 85 - East BLR",
            capacity: 500000,
            level: 0,
            temp: 22,
            pressure: 45,
            fill: 1200,
            drain: 980,
            gas: 420,
            live: true,
            lat: 0.058,
            lng: 0.058,
          },
          {
            id: "demo-node-02",
            name: "Demo Node 02",
            meta: "ITPL Main Line",
            capacity: 300000,
            level: 52,
            temp: 24,
            pressure: 40,
            fill: 950,
            drain: 750,
            gas: 900,
            lat: 0.066,
            lng: 0.066,
          },
          {
            id: "demo-node-03",
            name: "Demo Node 03",
            meta: "South Grid Basin",
            capacity: 250000,
            level: 61,
            temp: 25,
            pressure: 30,
            fill: 800,
            drain: 600,
            gas: 1400,
            lat: 0.074,
            lng: 0.074,
          },
          {
            id: "demo-node-04",
            name: "Demo Node 04",
            meta: "North Ring Buffer",
            capacity: 120000,
            level: 79,
            temp: 23,
            pressure: 25,
            fill: 600,
            drain: 400,
            gas: 2600,
            lat: 0.082,
            lng: 0.082,
          },
          {
            id: "demo-node-05",
            name: "Demo Node 05",
            meta: "Industrial Supply",
            capacity: 220000,
            level: 37,
            temp: 26,
            pressure: 34,
            fill: 720,
            drain: 510,
            gas: 650,
            lat: 0.09,
            lng: 0.09,
          },
          {
            id: "demo-node-06",
            name: "Demo Node 06",
            meta: "Aviation Rd Reserve",
            capacity: 185000,
            level: 84,
            temp: 22,
            pressure: 42,
            fill: 680,
            drain: 880,
            gas: 3100,
            lat: 0.098,
            lng: 0.098,
          },
          {
            id: "demo-node-07",
            name: "Demo Node 07",
            meta: "ORR East Junction",
            capacity: 270000,
            level: 45,
            temp: 24,
            pressure: 38,
            fill: 840,
            drain: 690,
            gas: 780,
            lat: 0.106,
            lng: 0.106,
          },
          {
            id: "demo-node-08",
            name: "Demo Node 08",
            meta: "80ft Rd Catchment",
            capacity: 310000,
            level: 72,
            temp: 25,
            pressure: 33,
            fill: 910,
            drain: 640,
            gas: 1900,
            lat: 0.114,
            lng: 0.114,
          },
          {
            id: "demo-node-09",
            name: "Demo Node 09",
            meta: "CMH Rd Line",
            capacity: 240000,
            level: 58,
            temp: 24,
            pressure: 36,
            fill: 780,
            drain: 610,
            gas: 1100,
            lat: 0.122,
            lng: 0.122,
          },
          {
            id: "demo-node-10",
            name: "Demo Node 10",
            meta: "Sector 7 Grid",
            capacity: 200000,
            level: 88,
            temp: 26,
            pressure: 28,
            fill: 700,
            drain: 900,
            gas: 5200,
            lat: 0.13,
            lng: 0.13,
          },
          {
            id: "demo-node-11",
            name: "Demo Node 11",
            meta: "Sampige Rd Reserve",
            capacity: 160000,
            level: 33,
            temp: 23,
            pressure: 41,
            fill: 560,
            drain: 470,
            gas: 540,
            lat: 0.138,
            lng: 0.138,
          },
          {
            id: "demo-node-12",
            name: "Demo Node 12",
            meta: "Ring Rd South",
            capacity: 230000,
            level: 67,
            temp: 25,
            pressure: 31,
            fill: 820,
            drain: 660,
            gas: 1600,
            lat: 0.146,
            lng: 0.146,
          },
        ];
        // Methane thresholds (ppm) are per-node:
        //  • Node 1 — the real MQ-4 sensor — alarms at a tight 150 ppm.
        //  • Every other (simulated) node uses a 1000 ppm limit.
        function gasDangerFor(z) {
          return z.id === "krpuram" ? 150 : 1000;
        }
        function gasWarnFor(z) {
          return z.id === "krpuram" ? 100 : 700;
        }
        function gasColor(z) {
          var g = z.gas,
            d = gasDangerFor(z),
            w = gasWarnFor(z);
          return g >= d ? "#ef4444" : g >= w ? "#f59e0b" : "#10b981";
        }
        function gasState(z) {
          var g = z.gas,
            d = gasDangerFor(z),
            w = gasWarnFor(z);
          return g >= d ? "Danger" : g >= w ? "Elevated" : "Safe";
        }
        // Methane: Node 1 (index 0) uses the real MQ-4 — 0 while in simulation.
        // Every other node has no physical sensor, so it shows a realistic value that
        // drifts only slightly (regardless of simulation or live mode).
        zones[0].gas = 0;
        for (var gi = 1; gi < zones.length; gi++) {
          zones[gi].baseGas = zones[gi].gas;
        }
        // Combined severity: a node is critical on high water OR dangerous methane.
        function isCritical(z) {
          return z.level >= ALERT_PERCENT || z.gas >= gasDangerFor(z);
        }
        function nodeSeverity(z) {
          if (z.level >= ALERT_PERCENT || z.gas >= gasDangerFor(z))
            return "crit";
          if (z.level >= ALERT_PERCENT - 20 || z.gas >= gasWarnFor(z))
            return "warn";
          return "ok";
        }
        function nodeStatusText(z) {
          if (z.gas >= gasDangerFor(z) && z.level < ALERT_PERCENT)
            return "Gas Alert";
          return state(z.level);
        }

        function byId(id) {
          return document.getElementById(id);
        }
        function clamp(v, min, max) {
          return Math.max(min, Math.min(max, v));
        }
        function round(v) {
          return Math.round(v);
        }
        function fmt(n) {
          return String(Math.round(n)).replace(/\B(?=(\d{3})+(?!\d))/g, ",");
        }
        function distanceFromLevel(p) {
          return EMPTY_CM - (clamp(p, 0, 100) / 100) * (EMPTY_CM - FULL_CM);
        }
        function state(p) {
          return p >= ALERT_PERCENT
            ? "Critical Backup"
            : p >= ALERT_PERCENT - 20
              ? "High Volume"
              : "Normal Flow";
        }
        function stateClass(p) {
          return p >= ALERT_PERCENT
            ? "crit"
            : p >= ALERT_PERCENT - 20
              ? "warn"
              : "ok";
        }
        function color(p) {
          return p >= ALERT_PERCENT
            ? "#ef4444"
            : p >= ALERT_PERCENT - 20
              ? "#f59e0b"
              : "#10b981";
        }

        var SVG_RAIN =
          '<svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M20 17.58A5 5 0 0 0 18 8h-1.26A8 8 0 1 0 4 16.25"/><line x1="8" y1="19" x2="8" y2="21"/><line x1="8" y1="13" x2="8" y2="15"/><line x1="16" y1="19" x2="16" y2="21"/><line x1="16" y1="13" x2="16" y2="15"/><line x1="12" y1="21" x2="12" y2="23"/><line x1="12" y1="15" x2="12" y2="17"/></svg>';
        var SVG_SUN =
          '<svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><circle cx="12" cy="12" r="5"/><line x1="12" y1="1" x2="12" y2="3"/><line x1="12" y1="21" x2="12" y2="23"/><line x1="4.22" y1="4.22" x2="5.64" y2="5.64"/><line x1="18.36" y1="18.36" x2="19.78" y2="19.78"/><line x1="1" y1="12" x2="3" y2="12"/><line x1="21" y1="12" x2="23" y2="12"/><line x1="4.22" y1="19.78" x2="5.64" y2="18.36"/><line x1="18.36" y1="5.64" x2="19.78" y2="4.22"/></svg>';
        var SVG_WARN =
          '<svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M10.29 3.86L1.82 18a2 2 0 0 0 1.71 3h16.94a2 2 0 0 0 1.71-3L13.71 3.86a2 2 0 0 0-3.42 0z"/><line x1="12" y1="9" x2="12" y2="13"/><line x1="12" y1="17" x2="12.01" y2="17"/></svg>';
        function setWeatherBadge(icon, text) {
          byId("weatherBadge").innerHTML = icon + "<span>" + text + "</span>";
        }

        function fetchWeather() {
          if (weatherAPIKey === "YOUR_OPENWEATHER_API_KEY_HERE") {
            setWeatherBadge(SVG_WARN, "No API Key");
            return;
          }
          var url =
            "https://api.openweathermap.org/data/2.5/forecast?lat=" +
            blrLat +
            "&lon=" +
            blrLon +
            "&appid=" +
            weatherAPIKey;

          fetch(url)
            .then(function (r) {
              if (!r.ok) throw new Error("HTTP " + r.status);
              return r.json();
            })
            .then(function (data) {
              if (!data || !data.list) throw new Error("Bad API data");

              var rainIncoming = false;
              for (var i = 0; i < 4; i++) {
                var condition = data.list[i].weather[0].main.toLowerCase();
                if (
                  condition.includes("rain") ||
                  condition.includes("thunderstorm") ||
                  condition.includes("drizzle")
                ) {
                  rainIncoming = true;
                  break;
                }
              }

              if (rainIncoming) {
                ALERT_PERCENT = 60;
                setWeatherBadge(SVG_RAIN, "Rain expected (12h)");
                byId("thresholdBadge").innerHTML = "Threshold: 60% — Monsoon";
                byId("thresholdBadge").style.borderColor = "var(--amber)";
                byId("thresholdBadge").style.color = "var(--amber)";
              } else {
                ALERT_PERCENT = 80;
                setWeatherBadge(SVG_SUN, "Clear (12h)");
                byId("thresholdBadge").innerHTML = "Threshold: 80%";
                byId("thresholdBadge").style.borderColor = "var(--line)";
                byId("thresholdBadge").style.color = "var(--text)";
              }
              renderAll(false);
            })
            .catch(function (e) {
              console.log("Weather fetch failed:", e.message);
              if (e.message.includes("401")) {
                setWeatherBadge(SVG_WARN, "Invalid API key");
              } else if (e.message.includes("Failed to fetch")) {
                setWeatherBadge(SVG_WARN, "Offline");
              } else {
                setWeatherBadge(SVG_WARN, "Weather unavailable");
              }
            });
        }
        function setMode(isSim) {
          simulation = isSim;
          var badge = byId("modeBadge");
          badge.className = "status" + (simulation ? " sim" : "");
          badge.querySelector("span").textContent = simulation
            ? "Simulation Mode"
            : "Live Sensor";
        }

        function showToast() {
          var now = Date.now();
          if (now - lastToast < 7000) return;
          lastToast = now;
          var t = byId("toast");
          t.classList.add("show");
          setTimeout(function () {
            t.classList.remove("show");
          }, 3600);
        }

        function simulateStep() {
          var now = Date.now();
          var dt = Math.max(0.2, (now - lastTick) / 1000);
          lastTick = now;
          for (var i = 1; i < zones.length; i++) {
            var z = zones[i];
            var drift =
              Math.sin(now / 8000 + i) * 0.18 +
              Math.sin(now / 13000 + i * 2) * 0.08;
            var noise = (Math.random() - 0.5) * 0.28;
            var bias = z.level > 88 ? -0.18 : z.level < 18 ? 0.18 : 0;
            z.level = clamp(z.level + (drift + noise + bias) * dt, 6, 96);
            z.fill = clamp(z.fill + (Math.random() - 0.5) * 6, 350, 1300);
            z.drain = clamp(z.drain + (Math.random() - 0.5) * 6, 260, 1050);
            // gentle, realistic methane drift around each node's baseline (±~5%)
            z.gas = clamp(
              z.baseGas +
                Math.sin(now / 15000 + i) * z.baseGas * 0.05 +
                (Math.random() - 0.5) * 8,
              100,
              8000,
            );
          }
        }

        function pushReading() {
          var now = Date.now();
          zones.forEach(function (z) {
            if (!z.hist) z.hist = [];
            z.hist.push({ t: now, d: distanceFromLevel(z.level) });
            while (z.hist.length && now - z.hist[0].t > TREND_MS + 5000)
              z.hist.shift();
            if (!z.ghist) z.ghist = [];
            z.ghist.push({ t: now, v: z.gas });
            while (z.ghist.length && now - z.ghist[0].t > TREND_MS + 5000)
              z.ghist.shift();
          });
        }
        function zoneById(id) {
          for (var i = 0; i < zones.length; i++)
            if (zones[i].id === id) return zones[i];
          return null;
        }

        function renderMetrics() {
          var live = zones[0],
            sum = 0,
            crit = [];
          for (var i = 0; i < zones.length; i++) {
            sum += zones[i].level;
            if (isCritical(zones[i])) crit.push(zones[i].name);
          }
          // remove skeleton once we have real data
          var skels = document.querySelectorAll(".skeleton-val,.skeleton-line");
          for (var j = 0; j < skels.length; j++)
            skels[j].classList.remove("skeleton-val", "skeleton-line");
          byId("mLive").textContent = round(live.level) + "%";
          byId("mLive").style.color = color(live.level);
          byId("mLiveSub").textContent = simulation
            ? "Simulation active"
            : state(live.level);
          byId("mDistance").textContent =
            distanceFromLevel(live.level).toFixed(1) + " cm";
          byId("mDistance").style.color = color(live.level);
          byId("mCritical").textContent = crit.length;
          byId("mCritical").style.color = crit.length ? "#ef4444" : "#10b981";
          byId("mCriticalSub").textContent = crit.length
            ? crit.join(", ")
            : "All nodes within limits";
          byId("mAverage").textContent = round(sum / zones.length) + "%";
          byId("mAvgSub").textContent = zones.length + "-node average";
          byId("nodesSub").textContent =
            "Live telemetry across all " +
            zones.length +
            " monitored chambers.";
          var lg = round(zones[0].gas);
          byId("mGas").textContent = fmt(lg) + " ppm";
          byId("mGas").style.color = gasColor(zones[0]);
          byId("mGasSub").textContent =
            (simulation ? "Simulated · " : "MQ-4 · ") + gasState(zones[0]);
        }

        function renderZoneList() {
          var html = "";
          zones.forEach(function (z) {
            var p = round(z.level),
              c = color(p);
            html +=
              '<a class="zone-mini ' +
              (z.id === focused ? "active" : "") +
              '" href="#area-' +
              z.id +
              '" data-zone="' +
              z.id +
              '">' +
              "<div><h4>" +
              z.name +
              "</h4><p>" +
              z.meta +
              "</p></div>" +
              '<div class="mini-pct" style="color:' +
              c +
              '">' +
              p +
              "%</div>" +
              '<div class="mini-bar"><div style="width:' +
              p +
              "%;background:" +
              c +
              '"></div></div>' +
              "</a>";
          });
          byId("zoneList").innerHTML = html;
          var nodes = document.querySelectorAll(".zone-mini");
          for (var i = 0; i < nodes.length; i++) {
            nodes[i].onclick = function (e) {
              e.preventDefault();
              openDetail(this.getAttribute("data-zone"));
            };
          }
        }

        function renderCards() {
          var ordered = zones.slice();
          ordered.sort(function (a, b) {
            if (a.id === focused) return -1;
            if (b.id === focused) return 1;
            return b.level - a.level;
          });
          var html = "";
          ordered.forEach(function (z) {
            var p = round(z.level),
              dist = distanceFromLevel(z.level),
              st = nodeStatusText(z),
              sc = nodeSeverity(z),
              c = color(p);
            var netFlow = round(z.fill - z.drain);
            var netColor = netFlow > 0 ? "var(--amber)" : "var(--green)";
            var netStr = (netFlow > 0 ? "+" : "") + fmt(netFlow) + " L/m";
            html +=
              '<article class="tank-card" id="area-' +
              z.id +
              '" data-node="' +
              z.id +
              '" style="cursor:pointer">' +
              '<div class="tank-top"><div class="tank-id"><div><h3>' +
              z.name +
              "</h3><p>" +
              z.meta +
              '</p></div></div><div class="tag ' +
              sc +
              '">' +
              st +
              "</div></div>" +
              '<div class="tank-fill-row">' +
              '<div class="tank-fill-pct" style="color:' +
              c +
              '">' +
              p +
              "%</div>" +
              '<div class="tank-fill-bar-wrap"><div class="tank-fill-bar" style="width:' +
              p +
              "%;background:" +
              c +
              '"></div></div>' +
              "</div>" +
              '<div class="tank-stats">' +
              '<div class="tank-stat"><span>Gap</span><b>' +
              dist.toFixed(1) +
              " cm</b></div>" +
              '<div class="tank-stat"><span>Inflow</span><b>' +
              round(z.fill) +
              " L/m</b></div>" +
              '<div class="tank-stat"><span>Outflow</span><b>' +
              round(z.drain) +
              " L/m</b></div>" +
              '<div class="tank-stat"><span>Net</span><b style="color:' +
              netColor +
              '">' +
              netStr +
              "</b></div>" +
              '<div class="tank-stat"><span>CH&#8324;</span><b style="color:' +
              gasColor(z) +
              '">' +
              fmt(z.gas) +
              " ppm</b></div>" +
              "</div>" +
              "</article>";
          });
          byId("tankCards").innerHTML = html;
          var cards = document.querySelectorAll("#tankCards .tank-card");
          for (var i = 0; i < cards.length; i++) {
            cards[i].onclick = function () {
              openDetail(this.getAttribute("data-node"));
            };
          }
        }

        function renderSummary() {
          var ok = 0,
            warn = 0,
            crit = 0,
            inflow = 0,
            outflow = 0,
            n = zones.length;
          zones.forEach(function (z) {
            var sev = nodeSeverity(z);
            if (sev === "crit") crit++;
            else if (sev === "warn") warn++;
            else ok++;
            inflow += z.fill;
            outflow += z.drain;
          });
          byId("distBar").innerHTML =
            '<span style="width:' +
            (ok / n) * 100 +
            '%;background:#10b981"></span>' +
            '<span style="width:' +
            (warn / n) * 100 +
            '%;background:#f59e0b"></span>' +
            '<span style="width:' +
            (crit / n) * 100 +
            '%;background:#ef4444"></span>';
          byId("distLegend").innerHTML =
            '<span><i style="background:#10b981"></i>Normal <b>' +
            ok +
            "</b></span>" +
            '<span><i style="background:#f59e0b"></i>High <b>' +
            warn +
            "</b></span>" +
            '<span><i style="background:#ef4444"></i>Critical <b>' +
            crit +
            "</b></span>";
          var net = round(inflow - outflow);
          var netColor = net > 0 ? "var(--amber)" : "var(--green)";
          byId("flowStats").innerHTML =
            '<div class="flow-stat"><span>Total Inflow</span><b>' +
            fmt(inflow) +
            "</b><small>L/min</small></div>" +
            '<div class="flow-stat"><span>Total Outflow</span><b>' +
            fmt(outflow) +
            "</b><small>L/min</small></div>" +
            '<div class="flow-stat"><span>Net Flow</span><b style="color:' +
            netColor +
            '">' +
            (net > 0 ? "+" : "") +
            fmt(net) +
            "</b><small>L/min network</small></div>";
        }

        function renderAll(addPoint) {
          if (addPoint) pushReading();
          renderMetrics();
          renderZoneList();
          renderCards();
          renderSummary();
          updateMap();
          if (!byId("detailPage").hidden) renderDetail();
        }

        var map,
          markers = {};
        function popupHtml(z) {
          var p = round(z.level),
            c = color(p);
          return (
            "<b>" +
            z.name +
            '</b><br><span style="color:#64748b">' +
            z.meta +
            "</span>" +
            '<br>System Load: <b style="color:' +
            c +
            '">' +
            p +
            "%</b>" +
            "<br>Status: " +
            state(z.level)
          );
        }
        function initMap() {
          if (map) return;
          map = L.map("map").setView([12.9716, 77.5946], 11);
          L.tileLayer(
            "https://{s}.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}{r}.png",
            {
              maxZoom: 19,
              attribution: "&copy; OpenStreetMap contributors &copy; CARTO",
            },
          ).addTo(map);

          zones.forEach(function (z) {
            var m = L.circleMarker([z.lat, z.lng], {
              radius: 9,
              color: "#ffffff",
              weight: 2,
              fillColor: color(round(z.level)),
              fillOpacity: 0.9,
            }).addTo(map);
            m.bindPopup(popupHtml(z));
            m.on("click", function () {
              openDetail(z.id);
            });
            markers[z.id] = m;
          });

          var legend = L.control({ position: "bottomright" });
          legend.onAdd = function () {
            var div = L.DomUtil.create("div", "map-legend");
            div.innerHTML =
              '<span><i style="background:#10b981"></i>Normal</span>' +
              '<span><i style="background:#f59e0b"></i>High</span>' +
              '<span><i style="background:#ef4444"></i>Critical</span>';
            return div;
          };
          legend.addTo(map);
        }
        function updateMap() {
          if (!map) return;
          zones.forEach(function (z) {
            var m = markers[z.id];
            if (!m) return;
            m.setStyle({ fillColor: color(round(z.level)) });
            m.setPopupContent(popupHtml(z));
          });
        }

        function showPage(page) {
          byId("overviewPage").hidden = page !== "overview";
          byId("nodesPage").hidden = page !== "nodes";
          byId("zonesPage").hidden = page !== "zones";
          byId("detailPage").hidden = page !== "detail";
          var navFor = page === "detail" ? "nodes" : page;
          var links = document.querySelectorAll(".nav a");
          for (var i = 0; i < links.length; i++) {
            links[i].className =
              links[i].getAttribute("data-page") === navFor ? "active" : "";
          }
          if (page === "zones") {
            initMap();
            setTimeout(function () {
              map.invalidateSize();
            }, 100);
          }
        }

        function fillGradient(sev) {
          if (sev === "crit")
            return "linear-gradient(180deg,#fb7185 0%,#e11d48 100%)";
          if (sev === "warn")
            return "linear-gradient(180deg,#fcd34d 0%,#f59e0b 100%)";
          return "linear-gradient(180deg,#5eead4 0%,#0ea5e9 100%)";
        }

        function renderDetail() {
          var z = zoneById(focused);
          if (!z) return;
          var p = round(z.level),
            sev = nodeSeverity(z),
            c = color(p);
          byId("dTitle").textContent = z.name;
          byId("dMeta").textContent = z.meta;
          var vol = round((z.capacity * z.level) / 100);
          byId("dVol").textContent = fmt(vol);
          byId("dCap").textContent = fmt(z.capacity);
          byId("dGas").textContent = fmt(round(z.gas)) + " ppm";
          byId("dGas").style.color = gasColor(z);
          byId("dGasSub").textContent =
            (z.id === "krpuram"
              ? simulation
                ? "Simulated · "
                : "MQ-4 · "
              : "Modeled · ") + gasState(z);
          var net = round(z.fill - z.drain);
          byId("dNet").textContent = (net > 0 ? "+" : "") + fmt(net) + " L/m";
          byId("dNet").style.color = net > 0 ? "var(--amber)" : "var(--green)";
          byId("dStatus").textContent = nodeStatusText(z);
          byId("dStatus").style.color = c;
          byId("dGap").textContent =
            distanceFromLevel(z.level).toFixed(1) + " cm";
          byId("dIn").textContent = round(z.fill) + " L/m";
          byId("dOut").textContent = round(z.drain) + " L/m";
          byId("dChartSub").textContent =
            z.name + " incoming readings. Hover for precise values.";
          var fill = byId("dCylFill");
          fill.style.height = p + "%";
          fill.style.background = fillGradient(sev);
          byId("dCylPct").textContent = p + "%";
        }

        function openDetail(id) {
          focused = id;
          detailCvs = byId("detailCanvas");
          if (!hovDetail) hovDetail = attachHover(detailCvs);
          detailGasCvs = byId("detailGasCanvas");
          if (!hovGas) hovGas = attachHover(detailGasCvs);
          showPage("detail");
          if (location.hash !== "#node-" + id) location.hash = "node-" + id;
          renderDetail();
          window.scrollTo({ top: 0, behavior: "smooth" });
        }

        function syncRoute() {
          var h = location.hash.replace("#", "");
          if (h.indexOf("node-") === 0) {
            focused = h.slice(5);
            detailCvs = byId("detailCanvas");
            if (!hovDetail) hovDetail = attachHover(detailCvs);
            detailGasCvs = byId("detailGasCanvas");
            if (!hovGas) hovGas = attachHover(detailGasCvs);
            showPage("detail");
            renderDetail();
            return;
          }
          showPage(
            h === "zones" ? "zones" : h === "nodes" ? "nodes" : "overview",
          );
        }

        function fetchWithTimeout(url, ms) {
          if (window.AbortController) {
            var controller = new AbortController();
            var timer = setTimeout(function () {
              controller.abort();
            }, ms);
            return fetch(url, {
              cache: "no-store",
              signal: controller.signal,
            }).then(function (r) {
              clearTimeout(timer);
              if (!r.ok) throw new Error("HTTP " + r.status);
              return r.text();
            });
          }
          return Promise.race([
            fetch(url, { cache: "no-store" }).then(function (r) {
              if (!r.ok) throw new Error("HTTP " + r.status);
              return r.text();
            }),
            new Promise(function (_, reject) {
              setTimeout(function () {
                reject(new Error("timeout"));
              }, ms);
            }),
          ]);
        }

        function fetchLevel() {
          fetchWithTimeout("/level", TIMEOUT_MS)
            .then(function (text) {
              var n = parseInt(text, 10);
              if (isNaN(n)) throw new Error("invalid level");
              zones[0].level = clamp(n, 0, 100);
              setMode(false);
              simulateStep();
              renderAll(true);
            })
            .catch(function () {
              if (!simulation) showToast();
              setMode(true);
              simulateStep();
              renderAll(true);
            });
        }

        function fetchGas() {
          fetchWithTimeout("/gas", TIMEOUT_MS)
            .then(function (text) {
              var g = parseInt(text, 10);
              if (isNaN(g)) throw new Error("invalid gas");
              zones[0].gas = clamp(g, 0, 50000);
              renderAll(false);
            })
            .catch(function () {
              // No sensor connected / simulation mode → methane reads 0.
              zones[0].gas = 0;
              renderAll(false);
            });
        }

        function attachHover(cvs) {
          var hov = { mouseX: 0, hovering: false };
          cvs.addEventListener("mousemove", function (e) {
            var rect = cvs.getBoundingClientRect();
            hov.mouseX = e.clientX - rect.left;
            hov.hovering = true;
          });
          cvs.addEventListener("mouseleave", function () {
            hov.hovering = false;
          });
          return hov;
        }

        function drawChart(cvs, series, hov) {
          var ctx = cvs.getContext("2d");
          var rect = cvs.getBoundingClientRect();
          var ratio = window.devicePixelRatio || 1;
          var w = Math.max(320, Math.floor(rect.width)),
            h = Math.max(180, Math.floor(rect.height));
          if (cvs.width !== w * ratio || cvs.height !== h * ratio) {
            cvs.width = w * ratio;
            cvs.height = h * ratio;
            ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
          }
          var mouseX = hov.mouseX,
            hovering = hov.hovering;
          var isDark =
            document.documentElement.getAttribute("data-theme") === "dark";
          ctx.clearRect(0, 0, w, h);
          var padL = 44,
            padR = 16,
            padT = 18,
            padB = 30;
          var now = Date.now();
          var minD = FULL_CM,
            maxD = EMPTY_CM;

          ctx.fillStyle = isDark ? "#0b0e10" : "#ffffff";
          ctx.fillRect(0, 0, w, h);
          ctx.strokeStyle = isDark ? "#1c2428" : "#e2e8f0";
          ctx.lineWidth = 1;
          ctx.fillStyle = isDark ? "#64748b" : "#94a3b8";
          ctx.font = "11px Inter, sans-serif";

          for (var gy = 0; gy <= 4; gy++) {
            var y = padT + (gy * (h - padT - padB)) / 4;
            ctx.globalAlpha = gy === 4 ? 1 : 0.75;
            ctx.beginPath();
            ctx.moveTo(padL, y);
            ctx.lineTo(w - padR, y);
            ctx.stroke();
            var val = maxD - (gy * (maxD - minD)) / 4;
            ctx.fillText(val.toFixed(0) + " cm", 8, y + 4);
          }
          ctx.globalAlpha = 1;
          var alertD = distanceFromLevel(ALERT_PERCENT);
          var ay = padT + ((maxD - alertD) / (maxD - minD)) * (h - padT - padB);

          ctx.fillStyle = "rgba(239, 68, 68, 0.08)";
          ctx.fillRect(padL, padT, w - padL - padR, Math.max(0, ay - padT));
          ctx.strokeStyle = "rgba(239, 68, 68, 0.6)";
          ctx.setLineDash([6, 5]);
          ctx.beginPath();
          ctx.moveTo(padL, ay);
          ctx.lineTo(w - padR, ay);
          ctx.stroke();
          ctx.setLineDash([]);

          var closestPoint = null,
            minPxDist = Infinity,
            cx = 0,
            cy = 0;

          if (series.length > 1) {
            ctx.beginPath();
            for (var i = 0; i < series.length; i++) {
              var point = series[i];
              var x =
                w - padR - ((now - point.t) / TREND_MS) * (w - padL - padR);
              var y =
                padT + ((point.d - minD) / (maxD - minD)) * (h - padT - padB);
              if (i === 0) ctx.moveTo(x, y);
              else ctx.lineTo(x, y);

              if (hovering && x >= padL && x <= w - padR) {
                var dx = Math.abs(x - mouseX);
                if (dx < minPxDist) {
                  minPxDist = dx;
                  closestPoint = point;
                  cx = x;
                  cy = y;
                }
              }
            }
            var g = ctx.createLinearGradient(padL, 0, w - padR, 0);
            g.addColorStop(0, "rgba(14, 165, 233, 0.2)");
            g.addColorStop(1, "#0ea5e9");
            ctx.strokeStyle = g;
            ctx.lineWidth = 3;
            ctx.lineCap = "round";
            ctx.lineJoin = "round";
            ctx.stroke();

            var last = series[series.length - 1];
            var lx = w - padR - ((now - last.t) / TREND_MS) * (w - padL - padR);
            var ly =
              padT + ((last.d - minD) / (maxD - minD)) * (h - padT - padB);
            ctx.fillStyle = "#0ea5e9";
            ctx.beginPath();
            ctx.arc(lx, ly, 4, 0, Math.PI * 2);
            ctx.fill();
          }

          if (hovering && closestPoint && minPxDist < 30) {
            ctx.strokeStyle = isDark ? "#4e5b62" : "#cbd5e1";
            ctx.lineWidth = 1;
            ctx.setLineDash([4, 4]);
            ctx.beginPath();
            ctx.moveTo(cx, padT);
            ctx.lineTo(cx, h - padB);
            ctx.stroke();
            ctx.setLineDash([]);

            ctx.beginPath();
            ctx.arc(cx, cy, 5, 0, Math.PI * 2);
            ctx.fillStyle = "#0ea5e9";
            ctx.fill();
            ctx.strokeStyle = isDark ? "#000" : "#fff";
            ctx.lineWidth = 2;
            ctx.stroke();

            var timeStr = new Date(closestPoint.t).toLocaleTimeString();
            var txt = closestPoint.d.toFixed(1) + " cm @ " + timeStr;

            var tw = ctx.measureText(txt).width + 16;
            var tx = Math.max(padL, Math.min(cx - tw / 2, w - padR - tw));
            var ty = Math.max(padT + 15, cy - 20);

            ctx.fillStyle = isDark ? "#1c2428" : "#f8fafc";
            ctx.beginPath();
            ctx.roundRect(tx, ty - 14, tw, 22, 4);
            ctx.fill();
            ctx.strokeStyle = isDark ? "#4e5b62" : "#e2e8f0";
            ctx.stroke();

            ctx.fillStyle = isDark ? "#f4f7f7" : "#0f172a";
            ctx.fillText(txt, tx + 8, ty + 2);
          }

          ctx.fillStyle = isDark ? "#64748b" : "#94a3b8";
          ctx.fillText("now", w - padR - 22, h - 10);
          ctx.fillText("-60s", padL, h - 10);
        }

        // Methane trend chart (ppm over time) for the node detail page.
        function drawGasChart(cvs, series, hov, danger) {
          var ctx = cvs.getContext("2d");
          var rect = cvs.getBoundingClientRect();
          var ratio = window.devicePixelRatio || 1;
          var w = Math.max(320, Math.floor(rect.width)),
            h = Math.max(160, Math.floor(rect.height));
          if (cvs.width !== w * ratio || cvs.height !== h * ratio) {
            cvs.width = w * ratio;
            cvs.height = h * ratio;
            ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
          }
          var mouseX = hov.mouseX,
            hovering = hov.hovering;
          var isDark =
            document.documentElement.getAttribute("data-theme") === "dark";
          ctx.clearRect(0, 0, w, h);
          var padL = 52,
            padR = 18,
            padT = 28,
            padB = 30;
          var now = Date.now();
          // auto-scale: headroom above the danger line and the recent peak
          var peak = 0;
          for (var k = 0; k < series.length; k++)
            if (series[k].v > peak) peak = series[k].v;
          var maxV = Math.max(danger * 1.4, peak * 1.15, 200);

          ctx.fillStyle = isDark ? "#0b0e10" : "#ffffff";
          ctx.fillRect(0, 0, w, h);
          ctx.strokeStyle = isDark ? "#1c2428" : "#e2e8f0";
          ctx.lineWidth = 1;
          ctx.fillStyle = isDark ? "#64748b" : "#94a3b8";
          ctx.font = "11px Inter, sans-serif";
          for (var gy = 0; gy <= 4; gy++) {
            var y = padT + (gy * (h - padT - padB)) / 4;
            ctx.globalAlpha = gy === 4 ? 1 : 0.75;
            ctx.beginPath();
            ctx.moveTo(padL, y);
            ctx.lineTo(w - padR, y);
            ctx.stroke();
            var val = maxV - (gy * maxV) / 4;
            ctx.fillText(Math.round(val) + "", 8, y + 4);
          }
          ctx.globalAlpha = 1;

          // danger threshold line
          var dy = padT + (1 - danger / maxV) * (h - padT - padB);
          ctx.strokeStyle = "rgba(239, 68, 68, 0.6)";
          ctx.setLineDash([6, 5]);
          ctx.beginPath();
          ctx.moveTo(padL, dy);
          ctx.lineTo(w - padR, dy);
          ctx.stroke();
          ctx.setLineDash([]);

          var closest = null,
            minPx = Infinity,
            cx = 0,
            cy = 0;
          if (series.length > 1) {
            ctx.beginPath();
            for (var i = 0; i < series.length; i++) {
              var pt = series[i];
              var x =
                w - padR - ((now - pt.t) / TREND_MS) * (w - padL - padR);
              var y = padT + (1 - pt.v / maxV) * (h - padT - padB);
              if (i === 0) ctx.moveTo(x, y);
              else ctx.lineTo(x, y);
              if (hovering && x >= padL && x <= w - padR) {
                var d2 = Math.abs(x - mouseX);
                if (d2 < minPx) {
                  minPx = d2;
                  closest = pt;
                  cx = x;
                  cy = y;
                }
              }
            }
            var g = ctx.createLinearGradient(padL, 0, w - padR, 0);
            g.addColorStop(0, "rgba(245, 158, 11, 0.25)");
            g.addColorStop(1, "#f59e0b");
            ctx.strokeStyle = g;
            ctx.lineWidth = 3;
            ctx.lineCap = "round";
            ctx.lineJoin = "round";
            ctx.stroke();

            var last = series[series.length - 1];
            var lx = w - padR - ((now - last.t) / TREND_MS) * (w - padL - padR);
            var ly = padT + (1 - last.v / maxV) * (h - padT - padB);
            ctx.fillStyle = last.v >= danger ? "#ef4444" : "#f59e0b";
            ctx.beginPath();
            ctx.arc(lx, ly, 4, 0, Math.PI * 2);
            ctx.fill();
          }

          if (hovering && closest && minPx < 30) {
            ctx.strokeStyle = isDark ? "#4e5b62" : "#cbd5e1";
            ctx.lineWidth = 1;
            ctx.setLineDash([4, 4]);
            ctx.beginPath();
            ctx.moveTo(cx, padT);
            ctx.lineTo(cx, h - padB);
            ctx.stroke();
            ctx.setLineDash([]);
            ctx.beginPath();
            ctx.arc(cx, cy, 5, 0, Math.PI * 2);
            ctx.fillStyle = "#f59e0b";
            ctx.fill();
            ctx.strokeStyle = isDark ? "#000" : "#fff";
            ctx.lineWidth = 2;
            ctx.stroke();
            var txt =
              Math.round(closest.v) +
              " ppm @ " +
              new Date(closest.t).toLocaleTimeString();
            var tw = ctx.measureText(txt).width + 16;
            var tx = Math.max(padL, Math.min(cx - tw / 2, w - padR - tw));
            var ty = Math.max(padT + 15, cy - 20);
            ctx.fillStyle = isDark ? "#1c2428" : "#f8fafc";
            ctx.beginPath();
            ctx.roundRect(tx, ty - 14, tw, 22, 4);
            ctx.fill();
            ctx.strokeStyle = isDark ? "#4e5b62" : "#e2e8f0";
            ctx.stroke();
            ctx.fillStyle = isDark ? "#f4f7f7" : "#0f172a";
            ctx.fillText(txt, tx + 8, ty + 2);
          }

          ctx.fillStyle = isDark ? "#64748b" : "#94a3b8";
          ctx.fillText("now", w - padR - 22, h - 10);
          ctx.fillText("-60s", padL, h - 10);
          ctx.fillText("ppm", 10, 14);
        }

        var trendCvs = byId("trendCanvas");
        var hovOverview = attachHover(trendCvs);
        var detailCvs, hovDetail, detailGasCvs, hovGas;
        function animate() {
          if (!byId("overviewPage").hidden)
            drawChart(trendCvs, zones[0].hist || [], hovOverview);
          if (!byId("detailPage").hidden) {
            var z = zoneById(focused);
            if (z && detailCvs) drawChart(detailCvs, z.hist || [], hovDetail);
            if (z && detailGasCvs)
              drawGasChart(detailGasCvs, z.ghist || [], hovGas, gasDangerFor(z));
          }
          requestAnimationFrame(animate);
        }

        byId("dBack").onclick = function () {
          location.hash = "nodes";
        };

        byId("btnTheme").onclick = function () {
          var html = document.documentElement;
          var isLight = html.getAttribute("data-theme") === "light";
          html.setAttribute("data-theme", isLight ? "dark" : "light");
        };

        byId("btnRefresh").onclick = function () {
          var btn = this;
          btn.style.opacity = "0.5";
          fetchLevel();
          fetchGas();
          fetchWeather();
          setTimeout(function () {
            btn.style.opacity = "1";
          }, 500);
        };

        byId("btnExport").onclick = function () {
          var csv =
            "Chamber Name,Level (%),Gap (cm),Status,Methane (ppm),Gas Status\n";
          zones.forEach(function (z) {
            csv +=
              z.name +
              "," +
              round(z.level) +
              "," +
              distanceFromLevel(z.level).toFixed(1) +
              "," +
              state(z.level) +
              "," +
              round(z.gas) +
              "," +
              gasState(z) +
              "\n";
          });
          var blob = new Blob([csv], { type: "text/csv" });
          var a = document.createElement("a");
          a.href = URL.createObjectURL(blob);
          a.download = "hydra_sanitation_log.csv";
          document.body.appendChild(a);
          a.click();
          document.body.removeChild(a);
        };

        // Initialization
        zones[0].level = 0;
        setMode(false);
        for (var i = 0; i < 14; i++) {
          simulateStep();
          pushReading();
        }
        renderAll(false);
        syncRoute();
        window.addEventListener("hashchange", syncRoute);

        fetchWeather();
        setInterval(fetchWeather, 900000); // Check weather every 15 minutes

        fetchLevel();
        setInterval(fetchLevel, POLL_MS);
        fetchGas();
        setInterval(fetchGas, 1500);
        requestAnimationFrame(animate);
      })();
    </script>
  </body>
</html>

)HYDRA";

// ===========================================================================
//  WEB SERVER HANDLERS
// ===========================================================================
void handleRoot()  { server.send_P(200, "text/html", PAGE); }             // the dashboard page
void handleLevel() { server.send(200, "text/plain", String(waterLevel)); } // water level, e.g. "42"
void handleGas()   { server.send(200, "text/plain", String(gasPPM)); }     // methane ppm, e.g. "87"
void handleNotFound() { server.send(404, "text/plain", "Not found. Try /  /level  or  /gas"); }

// ===========================================================================
//  WI-FI HELPERS
// ===========================================================================

// Tries to join the Wi-Fi network from USER SETTINGS.
// Returns true if connected, false if it failed or still has demo values.
bool connectToWiFi() {
  // Friendly check: stop early if the demo placeholder was never changed.
  if (strcmp(WIFI_SSID, "DEMO_WIFI_NAME") == 0) {
    Serial.println("Wi-Fi name is still the DEMO value - edit WIFI_SSID / WIFI_PASSWORD.");
    oledMessage("HYDRA", "Set your WiFi in", "the code (demo)");
    delay(2500);
    return false;
  }

  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(WIFI_SSID);
  oledMessage("HYDRA", "Connecting WiFi...", WIFI_SSID);

  WiFi.mode(WIFI_STA);                   // act as a normal Wi-Fi client
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  for (int i = 0; i < WIFI_CONNECT_TRIES; i++) {
    if (WiFi.status() == WL_CONNECTED) break;
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  return WiFi.status() == WL_CONNECTED;
}

// Starts the node's own Wi-Fi hotspot so the dashboard is still reachable
// when there is no router (or the password was wrong).
void startAccessPoint() {
  apMode = true;
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(AP_SSID, AP_PASSWORD);
  if (ok) {
    Serial.print("Hotspot started. Join Wi-Fi \"");
    Serial.print(AP_SSID);
    Serial.print("\" then open http://");
    Serial.println(WiFi.softAPIP());     // normally 192.168.4.1
    String ip = WiFi.softAPIP().toString();
    oledMessage("HYDRA hotspot", AP_SSID, ip.c_str());
  } else {
    Serial.println("Could not start the hotspot (is AP_PASSWORD at least 8 characters?).");
    oledMessage("HYDRA", "No network!", "Check settings");
  }
  delay(3000);                           // leave the message readable for a moment
}

// ===========================================================================
//  SETUP  -  runs once when the board powers on or resets
// ===========================================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  // 1) Set up the pins.
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(MQ4_AO_PIN, INPUT);
  digitalWrite(BUZZER_PIN, LOW);         // buzzer off at start

  // 2) Start the OLED. If it is missing the rest still works (Serial + web).
  Wire.begin();                          // SDA = D2/GPIO4, SCL = D1/GPIO5
  if (display.begin(OLED_ADDR, true)) {
    oledOk = true;
    oledMessage("HYDRA", "booting...");
  } else {
    Serial.println("OLED (SH1106) not found at 0x3C - check wiring/address.");
  }

  // 3) Calibrate the gas sensor against the air around it.
  Serial.println("\nProject HYDRA starting...");
  Serial.println("Warming up MQ-4 & calibrating baseline (keep it in fresh air)...");
  oledMessage("HYDRA", "Calibrating gas", "sensor...");
  delay(2000);                           // short warm-up (a long burn-in is better)
  mq4Ro = mq4Calibrate();
  Serial.print("MQ-4 Ro = "); Serial.println(mq4Ro);

  // 4) Get on the network: your Wi-Fi first, own hotspot as a backup.
  if (connectToWiFi()) {
    String ip = WiFi.localIP().toString();
    Serial.println("Connected!");
    Serial.print("Open: http://"); Serial.println(ip);
    oledMessage("HYDRA online", ip.c_str(), "hydra.local");
    delay(3000);                         // leave the address readable for a moment

    if (MDNS.begin(MDNS_NAME)) {         // friendly name, works on most phones/laptops
      MDNS.addService("http", "tcp", 80);
      mdnsOk = true;
      Serial.println("mDNS ready: http://" MDNS_NAME ".local");
    }
  } else {
    Serial.println("Wi-Fi not available - starting own hotspot instead.");
    startAccessPoint();
  }

  // 5) Start the web server (same pages in both Wi-Fi and hotspot mode).
  server.on("/", handleRoot);
  server.on("/level", handleLevel);
  server.on("/gas", handleGas);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HYDRA online.");
}

// ===========================================================================
//  LOOP  -  runs forever. Uses millis() timers instead of long delay()s so
//  the sensors, buzzer, screen and web server all keep responding.
// ===========================================================================
void loop() {
  unsigned long now = millis();

  // --- Wi-Fi watchdog: reconnect automatically if the router link drops ---
  if (!apMode && now - lastWifiCheck >= WIFI_CHECK_MS) {
    lastWifiCheck = now;
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Wi-Fi lost - reconnecting...");
      WiFi.reconnect();
    }
  }

  // --- Read the water level ---
  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;
    float dist = readDistanceMedian();
    int level = distanceToPercent(dist);
    if (level >= 0) {                    // ignore failed readings, keep the last good one
      waterLevel = level;
      Serial.print("Water level: ");
      Serial.print(level);
      Serial.println("%");
    }
  }

  // --- Read the methane level ---
  if (now - lastGasRead >= GAS_INTERVAL) {
    lastGasRead = now;
    gasPPM = readGasPPM();
    Serial.print("CH4: "); Serial.print(gasPPM); Serial.println(" ppm");
  }

  // --- Decide whether the alarms are on. ---
  // "Hysteresis": the alarm switches ON at the alert level but only switches
  // OFF once the reading drops clearly below it, so it cannot flicker.
  if (!alarmActive && waterLevel >= ALERT_PERCENT) alarmActive = true;
  else if (alarmActive && waterLevel < ALERT_RELEASE) alarmActive = false;

  if (!gasAlarm && gasPPM >= GAS_ALERT_PPM) gasAlarm = true;
  else if (gasAlarm && gasPPM < GAS_RELEASE_PPM) gasAlarm = false;

  // --- Buzzer: rapid beeping while EITHER alarm is active, silent otherwise ---
  if (alarmActive || gasAlarm) {
    if (now - lastBuzzerToggle >= BUZZER_RAPID) {
      lastBuzzerToggle = now;
      buzzerState = !buzzerState;
      digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
    }
  } else {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerState = false;
  }

  // --- Refresh the OLED ---
  if (now - lastOled >= OLED_INTERVAL) {
    lastOled = now;
    drawOLED();
  }

  // --- Keep network services alive (must be called often) ---
  if (mdnsOk) MDNS.update();
  server.handleClient();
}





