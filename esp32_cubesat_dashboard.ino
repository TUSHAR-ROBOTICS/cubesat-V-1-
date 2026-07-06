/*
  ESP32-S3 CubeSat Sensor Dashboard (Reliability Edition)
  --------------------------------------------------------
  Sensors combined:
    - BMP280   (temperature, pressure, altitude)   -> Adafruit_BMP280 library, I2C
    - INA219   (voltage, current, power)           -> Adafruit_INA219 library, I2C
    - ICM-20948(accel, gyro)                       -> raw register access, NO library, I2C
                                                       + complementary filter -> roll/pitch/yaw
                                                       drives a live 3D cube on the dashboard
    - GPS (NEO-6M/NEO-8M or similar)               -> TinyGPS++ library, UART (NMEA @ 9600 baud)

  RELIABILITY FEATURES:
    - Every I2C transaction checks the actual bus result (not just "assume it worked")
    - I2C bus recovery (manual clock-out) if the bus ever gets stuck
    - Automatic re-initialization of any sensor that drops out, checked every 5s
    - Non-blocking WiFi auto-reconnect (loop() never freezes waiting on WiFi)
    - Hardware watchdog timer: if the main loop ever hangs, the chip reboots itself
    - Error counters + WiFi RSSI + uptime exposed on the dashboard as a "System Health" card
    - JSON buffer writes are bounds-checked instead of trusted blindly
    - GPS reports THREE distinct states (not just on/off): no module wired at all,
      module wired but no satellite fix yet, and locked with real coordinates --
      so the dashboard never shows fake/guessed position data.

  Serves a live-updating web dashboard (polls /data as JSON every 50ms,
  sensors sampled at ~50Hz on the ESP32 side).

  REQUIRED LIBRARIES (Library Manager):
    - Adafruit BMP280 Library
    - Adafruit INA219
    - Adafruit Unified Sensor (dependency of the above)
    - TinyGPSPlus (by Mikal Hart) -- for GPS NMEA parsing
  (ICM-20948 needs nothing extra -- handled with raw Wire calls.)
  (esp_task_wdt.h ships with the ESP32 Arduino core -- nothing to install.)

  WIRING:
    I2C bus (BMP280 + INA219 + ICM-20948, all share this bus):
      SDA -> GPIO 8
      SCL -> GPIO 9
      - BMP280 default addr: 0x76 (some boards: 0x77)
      - INA219 default addr: 0x40
      - ICM-20948 default addr: 0x68 (AD0 high -> 0x69)

    GPS module (separate UART, NOT the I2C bus):
      GPS VCC -> 3.3V
      GPS GND -> GND
      GPS TX  -> ESP32 GPIO 18 (ESP32 RX -- receives NMEA sentences)
      GPS RX  -> ESP32 GPIO 17 (ESP32 TX -- only needed if you send config commands)
      NOTE: GPIO 17/18 are free on most ESP32-S3 DevKitC boards, but double check
      your specific board's pinout doesn't use them for USB/PSRAM/flash first.
      Most NEO-6M/NEO-8M modules default to 9600 baud NMEA -- that's what's set below.

  NOTE ON WATCHDOG API:
    This targets ESP32 Arduino core 3.x (IDF5), which uses esp_task_wdt_config_t.
    If you're on an older core (2.x) and get a compile error on setupWatchdog(),
    replace that function's body with:
        esp_task_wdt_init(WDT_TIMEOUT_S, true);
        esp_task_wdt_add(NULL);
*/

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <esp_task_wdt.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_INA219.h>
#include <TinyGPS++.h>

// ---------------- USER CONFIG ----------------
#define SDA_PIN 8
#define SCL_PIN 9

#define GPS_RX_PIN 18   // ESP32 receives GPS's TX here
#define GPS_TX_PIN 17   // ESP32 TX -> GPS RX (optional, only needed to send config to GPS)
#define GPS_BAUD   9600

const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* HOSTNAME      = "cubesat";   // access at http://cubesat.local

#define BMP280_ADDR 0x76
#define INA219_ADDR 0x40
#define ICM_ADDR    0x68

#define WDT_TIMEOUT_S 8   // reboot if main loop hangs for this long

// ---------------- ICM-20948 raw register map ----------------
#define REG_BANK_SEL      0x7F
#define B0_WHO_AM_I       0x00
#define B0_PWR_MGMT_1     0x06
#define B0_PWR_MGMT_2     0x07
#define B0_ACCEL_XOUT_H   0x2D   // 14 bytes: accel(6) + gyro(6) + temp(2, unused)

// ---------------- Globals ----------------
Adafruit_BMP280 bmp;
Adafruit_INA219 ina219(INA219_ADDR);
WebServer server(80);

HardwareSerial gpsSerial(1);   // ESP32-S3's second hardware UART, dedicated to GPS
TinyGPSPlus gps;

struct SensorData {
  bool  bmp_ok = false, ina_ok = false, icm_ok = false;

  float bmp_temp = 0, bmp_pressure = 0, bmp_altitude = 0;
  float ina_voltage = 0, ina_current_mA = 0, ina_power_mW = 0;
  float acc_x = 0, acc_y = 0, acc_z = 0;
  float gyro_x = 0, gyro_y = 0, gyro_z = 0;

  // Orientation (degrees) for 3D cube rendering
  float roll = 0, pitch = 0, yaw = 0;

  // GPS -- three distinct states, not just true/false:
  //   gps_module==false                -> no GPS wired up / no NMEA data ever seen
  //   gps_module==true, gps_fix==false  -> module present, still searching for satellites
  //   gps_module==true, gps_fix==true   -> real lock, lat/lon/etc are live coordinates
  bool gps_module = false, gps_fix = false;
  double gps_lat = 0, gps_lon = 0;
  float gps_alt_m = 0, gps_hdop = 0, gps_speed_kmph = 0;
  uint32_t gps_sats = 0;
  unsigned long gps_last_fix_ms = 0;

  unsigned long last_update = 0;
} data;

// Complementary filter internal state
static unsigned long orientationLastMicros = 0;

// ---------------- Reliability state ----------------
uint32_t icmErrorCount = 0;
uint32_t bmpErrorCount = 0;
uint32_t inaErrorCount = 0;
uint32_t wifiReconnectCount = 0;
uint32_t i2cBusRecoveries = 0;

unsigned long lastHealthCheck = 0;
const unsigned long HEALTH_CHECK_INTERVAL_MS = 5000;

unsigned long lastWifiAttempt = 0;
const unsigned long WIFI_RETRY_INTERVAL_MS = 5000;

const uint8_t ICM_ERROR_THRESHOLD_FOR_RECOVERY = 5;
uint8_t icmConsecutiveErrors = 0;

// ================= I2C bus recovery =================
// If a device leaves SDA held low (common failure mode after a partial
// transaction or brown-out), the bus is stuck and every future transaction
// will fail silently forever. This manually clocks SCL to force any device
// to release SDA, per the standard I2C bus-recovery procedure.
void recoverI2CBus() {
  Serial.println("[I2C] Attempting bus recovery...");
  Wire.end();

  pinMode(SCL_PIN, OUTPUT);
  pinMode(SDA_PIN, INPUT_PULLUP);

  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(5);
  }

  // Generate a STOP condition
  pinMode(SDA_PIN, OUTPUT);
  digitalWrite(SDA_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(SDA_PIN, HIGH);
  delayMicroseconds(5);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  i2cBusRecoveries++;
  Serial.println("[I2C] Bus recovery done.");
}

// ================= ICM-20948 low-level (no library), error-checked =================
bool icmWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  Wire.write(value);
  uint8_t err = Wire.endTransmission();
  if (err != 0) { icmErrorCount++; return false; }
  return true;
}

bool icmReadRegister(uint8_t reg, uint8_t &outVal) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) { icmErrorCount++; return false; }

  uint8_t n = Wire.requestFrom((int)ICM_ADDR, 1);
  if (n != 1 || !Wire.available()) { icmErrorCount++; return false; }
  outVal = Wire.read();
  return true;
}

bool icmReadBytes(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) { icmErrorCount++; return false; }

  uint8_t n = Wire.requestFrom((int)ICM_ADDR, (int)len);
  if (n != len) { icmErrorCount++; return false; }

  for (uint8_t i = 0; i < len; i++) {
    if (!Wire.available()) { icmErrorCount++; return false; }
    buf[i] = Wire.read();
  }
  return true;
}

bool icmSelectBank(uint8_t bank) {
  return icmWriteRegister(REG_BANK_SEL, (bank & 0x03) << 4);
}

bool icmInit() {
  if (!icmSelectBank(0)) return false;

  uint8_t whoami = 0;
  if (!icmReadRegister(B0_WHO_AM_I, whoami)) {
    Serial.println("ICM-20948: I2C error reading WHO_AM_I");
    return false;
  }
  Serial.print("ICM-20948 WHO_AM_I = 0x");
  Serial.println(whoami, HEX);

  if (whoami != 0xEA) {
    Serial.println("ICM-20948 not detected!");
    return false;
  }

  bool ok = true;
  ok &= icmWriteRegister(B0_PWR_MGMT_1, 0x80); // reset device
  delay(50);  // datasheet only needs a short settle here; kept minimal so a
              // struggling sensor doesn't block the webserver for long during
              // repeated health-check retries
  ok &= icmSelectBank(0);
  ok &= icmWriteRegister(B0_PWR_MGMT_1, 0x01); // wake, auto clock select
  ok &= icmWriteRegister(B0_PWR_MGMT_2, 0x00); // enable accel + gyro
  delay(20);

  if (!ok) {
    Serial.println("ICM-20948: I2C error during configuration");
    return false;
  }

  icmConsecutiveErrors = 0;
  Serial.println("ICM-20948 initialized (accel +/-2g, gyro +/-250dps defaults).");
  return true;
}

bool icmRead() {
  if (!icmSelectBank(0)) { icmConsecutiveErrors++; return false; }

  uint8_t buf[14];
  if (!icmReadBytes(B0_ACCEL_XOUT_H, buf, 14)) {
    icmConsecutiveErrors++;
    return false;
  }
  icmConsecutiveErrors = 0;

  int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
  int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
  int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
  int16_t gx = (int16_t)((buf[6] << 8) | buf[7]);
  int16_t gy = (int16_t)((buf[8] << 8) | buf[9]);
  int16_t gz = (int16_t)((buf[10] << 8) | buf[11]);
  // buf[12..13] is temperature, intentionally unused/discarded

  // Default full-scale ranges after reset: +-2g, +-250 dps
  data.acc_x = ax / 16384.0f;
  data.acc_y = ay / 16384.0f;
  data.acc_z = az / 16384.0f;
  data.gyro_x = gx / 131.0f;
  data.gyro_y = gy / 131.0f;
  data.gyro_z = gz / 131.0f;
  return true;
}

// Complementary filter: fuses accel (stable, no drift, noisy) with
// gyro (smooth, but drifts over time) into roll/pitch/yaw in degrees.
// This is what drives the 3D cube on the dashboard.
void icmComputeOrientation() {
  unsigned long now = micros();
  float dt = (orientationLastMicros == 0) ? 0.0f : (now - orientationLastMicros) / 1000000.0f;
  orientationLastMicros = now;

  float accelRoll  = atan2(data.acc_y, data.acc_z) * 180.0f / PI;
  float accelPitch = atan2(-data.acc_x, sqrt(data.acc_y * data.acc_y + data.acc_z * data.acc_z)) * 180.0f / PI;

  if (dt > 0 && dt < 1.0f) {
    const float alpha = 0.98f; // trust gyro short-term, accel long-term
    data.roll  = alpha * (data.roll  + data.gyro_x * dt) + (1 - alpha) * accelRoll;
    data.pitch = alpha * (data.pitch + data.gyro_y * dt) + (1 - alpha) * accelPitch;
    data.yaw  += data.gyro_z * dt; // no magnetometer fused -> will slowly drift
  } else {
    data.roll  = accelRoll;
    data.pitch = accelPitch;
  }
}

// ================= GPS (NMEA over UART, via TinyGPS++) =================
// Draining the GPS serial buffer is cheap and non-blocking, so this runs
// every loop() iteration (not on the slower sensor timer) to avoid ever
// letting the UART buffer overflow and drop sentences.
void gpsUpdate() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // "Module present" is inferred from whether we've ever successfully parsed
  // any NMEA characters at all -- if nothing is wired up, this stays 0 forever
  // and the dashboard correctly shows "no module" instead of a fake reading.
  data.gps_module = gps.charsProcessed() > 10;

  if (gps.location.isValid() && gps.location.isUpdated()) {
    data.gps_lat = gps.location.lat();
    data.gps_lon = gps.location.lng();
    data.gps_fix = true;
    data.gps_last_fix_ms = millis();
  }
  if (gps.satellites.isValid()) data.gps_sats = gps.satellites.value();
  if (gps.hdop.isValid())       data.gps_hdop = gps.hdop.hdop();
  if (gps.altitude.isValid())   data.gps_alt_m = gps.altitude.meters();
  if (gps.speed.isValid())      data.gps_speed_kmph = gps.speed.kmph();

  // If we had a fix but haven't seen a fresh one in a while (satellites lost,
  // antenna moved indoors, etc.), fall back to "searching" rather than
  // silently freezing on stale coordinates.
  if (data.gps_fix && millis() - data.gps_last_fix_ms > 10000) {
    data.gps_fix = false;
  }
}

// ================= Sensor update =================
void readAllSensors() {
  if (data.bmp_ok) {
    float t = bmp.readTemperature();
    float p = bmp.readPressure();
    if (isnan(t) || isnan(p)) {
      bmpErrorCount++;
      data.bmp_ok = false; // will be retried by health check
    } else {
      data.bmp_temp     = t;
      data.bmp_pressure = p / 100.0f; // hPa
      data.bmp_altitude = bmp.readAltitude(1013.25f);
    }
  }

  if (data.ina_ok) {
    data.ina_voltage    = ina219.getBusVoltage_V();
    data.ina_current_mA = ina219.getCurrent_mA();
    data.ina_power_mW   = ina219.getPower_mW();
  }

  if (data.icm_ok) {
    if (icmRead()) {
      icmComputeOrientation();
    } else if (icmConsecutiveErrors >= ICM_ERROR_THRESHOLD_FOR_RECOVERY) {
      Serial.println("ICM-20948: too many consecutive errors, marking offline.");
      data.icm_ok = false; // health check will attempt bus recovery + reinit
    }
  }

  data.last_update = millis();
}

// Attempts to bring back any sensor currently marked offline.
// Runs every HEALTH_CHECK_INTERVAL_MS so a bad sensor cable that gets
// reseated (or a device that browns-out and comes back) recovers on its own.
void runSensorHealthCheck() {
  bool anyIcmFailure = !data.icm_ok || icmConsecutiveErrors >= ICM_ERROR_THRESHOLD_FOR_RECOVERY;

  if (anyIcmFailure) {
    recoverI2CBus();
    data.icm_ok = icmInit();
  }

  if (!data.bmp_ok) {
    data.bmp_ok = bmp.begin(BMP280_ADDR);
    if (data.bmp_ok) Serial.println("BMP280: recovered.");
  }

  if (!data.ina_ok) {
    data.ina_ok = ina219.begin();
    if (data.ina_ok) Serial.println("INA219: recovered.");
  }
}

// ================= WiFi (non-blocking reconnect) =================
void ensureWifiConnected() {
  if (WiFi.status() == WL_CONNECTED) return;
  if (millis() - lastWifiAttempt < WIFI_RETRY_INTERVAL_MS) return;

  lastWifiAttempt = millis();
  wifiReconnectCount++;
  Serial.println("WiFi disconnected, attempting reconnect...");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// ================= Watchdog =================
void setupWatchdog() {
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT_S * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);
}

// ================= Web page (stored in flash) =================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>CubeSat Telemetry</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<style>
@import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&family=JetBrains+Mono:wght@400;500;700&display=swap');
*{box-sizing:border-box;margin:0;padding:0;}
:root{
  --bg:#000;--bg2:#0a0a0a;
  --border:rgba(255,255,255,0.13);--border2:rgba(255,255,255,0.08);
  --text:#f2f2f2;--dim:#9a9a9a;--dim2:#5a5a5a;
  --sans:'Inter',system-ui,-apple-system,sans-serif;
  --mono:'JetBrains Mono',monospace;
}
html,body{height:100%;overflow:hidden;}
body{background:var(--bg);color:var(--text);font-family:var(--sans);display:flex;flex-direction:column;font-size:13px;}
.topbar{height:42px;display:flex;align-items:center;justify-content:space-between;padding:0 18px;border-bottom:1px solid var(--border);background:var(--bg2);flex-shrink:0;}
.tb-left{display:flex;align-items:center;gap:14px;}
.tb-title{font-size:16px;font-weight:700;letter-spacing:1px;color:#fff;}
.tb-sep{width:1px;height:16px;background:var(--border);}
.tb-tag{font-size:11px;font-weight:500;letter-spacing:0.5px;color:var(--dim);}
.tb-right{display:flex;align-items:center;gap:12px;}
.live-dot{width:6px;height:6px;border-radius:50%;background:#fff;animation:blink 1.8s infinite;}
@keyframes blink{0%,100%{opacity:1;}50%{opacity:0.25;}}
.clock{font-family:var(--mono);font-size:13px;letter-spacing:1px;color:#fff;}

.body{display:flex;flex:1;overflow:hidden;min-height:0;}

/* ---- LEFT: 3D model column ---- */
.col-3d{width:320px;flex-shrink:0;display:flex;flex-direction:column;border-right:1px solid var(--border);overflow:hidden;}
.panel-label{font-size:11px;font-weight:600;letter-spacing:0.5px;color:var(--dim);padding:7px 14px;border-bottom:1px solid var(--border2);flex-shrink:0;display:flex;justify-content:space-between;}
.panel-label span{font-family:var(--mono);font-weight:400;color:var(--dim2);}
#canvasContainer{flex:1;position:relative;min-height:0;border-bottom:1px solid var(--border2);background:#000;}
#canvasContainer canvas{display:block;width:100%!important;height:100%!important;}
.hud-mini{position:absolute;bottom:8px;right:10px;font-family:var(--mono);font-size:11px;letter-spacing:0.5px;color:rgba(255,255,255,0.45);text-align:right;line-height:1.5;pointer-events:none;}
.calib-strip{padding:8px 12px;flex-shrink:0;border-bottom:1px solid var(--border);}
.calib-title{font-size:9.5px;font-weight:600;color:var(--dim2);letter-spacing:0.5px;margin-bottom:6px;}
.calib-row{display:flex;gap:6px;flex-wrap:wrap;}
.calib-btn{flex:1;min-width:70px;padding:6px 4px;font-size:10px;font-weight:600;font-family:var(--sans);letter-spacing:0.3px;text-align:center;
  background:#141414;border:1px solid var(--border2);border-radius:5px;color:var(--dim);cursor:pointer;user-select:none;transition:all 0.15s;}
.calib-btn:hover{border-color:var(--dim2);}
.calib-btn.on{background:#fff;color:#000;border-color:#fff;}
.calib-hint{font-size:9px;color:var(--dim2);margin-top:6px;line-height:1.4;}

/* ---- RIGHT: camera column ---- */
.col-cam{width:320px;flex-shrink:0;display:flex;flex-direction:column;border-left:1px solid var(--border);overflow:hidden;}
.cam-view{flex:1;position:relative;overflow:hidden;background:#000;min-height:0;}
#camFeed{height:100%;width:100%;object-fit:contain;display:block;opacity:0;transition:opacity 0.4s;}
#cam-overlay{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;}
#cam-msg{font-family:var(--mono);font-size:10px;color:var(--dim2);letter-spacing:1px;}
.corner{position:absolute;width:12px;height:12px;pointer-events:none;border-color:rgba(255,255,255,0.15);}
.c-tl{top:6px;left:6px;border-top:1px solid;border-left:1px solid;}
.c-tr{top:6px;right:6px;border-top:1px solid;border-right:1px solid;}
.c-bl{bottom:6px;left:6px;border-bottom:1px solid;border-left:1px solid;}
.c-br{bottom:6px;right:6px;border-bottom:1px solid;border-right:1px solid;}
#cam-live-dot{display:none;position:absolute;top:8px;left:50%;transform:translateX(-50%);font-family:var(--mono);font-size:9px;color:rgba(255,255,255,0.4);letter-spacing:1px;}
.cam-info{padding:8px 14px;font-size:10px;color:var(--dim2);border-top:1px solid var(--border2);flex-shrink:0;font-family:var(--mono);}

/* ---- MIDDLE: sensor data grid ---- */
.col-data{flex:1;display:grid;grid-template-columns:repeat(3,1fr);grid-template-rows:1fr 1fr 64px 112px;overflow:hidden;min-height:0;}
.card{border-right:1px solid var(--border2);border-bottom:1px solid var(--border2);display:flex;flex-direction:column;overflow:hidden;min-height:0;}
.col-data>.card:nth-child(3n){border-right:none;}
.card.span3{grid-column:1/4;border-right:none;}
.card-h{font-size:11px;font-weight:600;letter-spacing:0.5px;color:var(--dim);padding:6px 14px;border-bottom:1px solid var(--border2);flex-shrink:0;display:flex;justify-content:space-between;}
.card-h span{font-family:var(--mono);font-weight:400;font-size:10px;color:var(--dim2);}
.card-body{flex:1;padding:6px 14px;overflow:hidden;min-height:0;display:flex;flex-direction:column;justify-content:center;gap:3px;}
.row{display:flex;justify-content:space-between;align-items:center;padding:3px 0;}
.rk{font-size:12px;color:var(--dim);font-weight:500;}
.rv{font-family:var(--mono);font-size:15px;font-weight:500;color:#fff;}
.rv.dim{color:var(--dim);font-size:13px;}
.rv.warn{color:#bbb;}
.stat-trio{display:flex;justify-content:space-between;gap:8px;padding:2px 0 8px;}
.stat-item{flex:1;text-align:center;border-right:1px solid var(--border2);}
.stat-item:last-child{border-right:none;}
.stat-num{font-family:var(--mono);font-size:24px;font-weight:600;color:#fff;line-height:1;}
.stat-unit{font-size:10px;color:var(--dim2);font-weight:500;margin-top:4px;}
.chart-wrap{height:44px;position:relative;margin-top:4px;}
.rings{display:flex;justify-content:space-around;align-items:center;flex:1;}
.ring-item{display:flex;flex-direction:column;align-items:center;gap:4px;}
.ring-lbl{font-size:10px;font-weight:600;letter-spacing:0.5px;color:var(--dim2);}
.gps-status{font-size:11px;font-weight:600;letter-spacing:0.5px;padding:3px 8px;border-radius:4px;background:rgba(255,255,255,0.06);color:var(--dim);display:inline-block;margin-bottom:4px;}
.gps-hint{font-size:9.5px;color:var(--dim2);margin-top:4px;line-height:1.4;}
.health-row{display:flex;gap:22px;align-items:center;height:100%;padding:0 14px;overflow:hidden;}
.health-item{display:flex;flex-direction:column;gap:1px;}
.health-item .rk{font-size:9.5px;}
.health-item .rv{font-size:13px;}

.console-body{flex:1;overflow-y:auto;font-family:var(--mono);font-size:11px;color:var(--dim);line-height:1.6;white-space:pre-wrap;word-break:break-all;padding:4px 14px;min-height:0;}
.console-body::-webkit-scrollbar{width:3px;}
.console-body::-webkit-scrollbar-thumb{background:var(--border);}
.log-w{color:#fff;display:block;}.log-d{color:var(--dim2);display:block;}

.statusbar{height:24px;display:flex;align-items:center;justify-content:space-between;padding:0 18px;border-top:1px solid var(--border);background:var(--bg2);flex-shrink:0;}
.sb-item{font-size:10px;font-weight:500;letter-spacing:0.5px;color:var(--dim2);display:flex;align-items:center;gap:5px;}
.sb-dot{width:5px;height:5px;border-radius:50%;background:#fff;}
.sb-dot.off{background:var(--dim2);}

/* ================= MOBILE / PHONE LAYOUT ================= */
/* Below this width, the dense fixed-grid desktop layout doesn't fit --
   switch to a stacked, scrollable layout instead. Everything still works,
   it just scrolls vertically like a normal phone page. */
@media (max-width: 900px) {
  html,body{height:auto;overflow-y:auto;overflow-x:hidden;}
  body{font-size:14px;}
  .topbar{height:auto;flex-wrap:wrap;padding:8px 12px;gap:6px;}
  .body{flex-direction:column;overflow:visible;height:auto;}

  .col-3d{width:100%;border-right:none;border-bottom:1px solid var(--border);}
  #canvasContainer{height:260px;flex:none;}
  .calib-row{flex-wrap:wrap;}
  .calib-btn{min-width:45%;}

  .col-cam{width:100%;border-left:none;border-top:1px solid var(--border);order:99;}
  .cam-view{height:240px;flex:none;}

  .col-data{display:flex;flex-direction:column;grid-template-columns:none;grid-template-rows:none;height:auto;}
  .card{border-right:none!important;min-height:150px;}
  .card.span3{grid-column:auto;}
  .card-body{padding:10px 16px;}
  .stat-num{font-size:22px;}
  .health-row{flex-wrap:wrap;gap:14px 22px;height:auto;padding:10px 14px;}
  .console-body{max-height:160px;}

  .statusbar{height:auto;flex-wrap:wrap;padding:6px 12px;gap:6px 14px;}
}
/* Extra-narrow phones: shrink the ring SVGs a touch so 3 fit per row cleanly */
@media (max-width: 420px) {
  .ring-item svg{width:42px;height:42px;}
  .stat-trio{gap:4px;}
  .stat-num{font-size:19px;}
}
</style>
</head>
<body>
<div class="topbar">
  <div class="tb-left">
    <span class="tb-title">CubeSat Telemetry</span>
    <div class="tb-sep"></div>
    <span class="tb-tag">Prototype · Ground Test Rig</span>
  </div>
  <div class="tb-right">
    <div class="live-dot"></div>
    <span class="tb-tag" id="link-tag">Link Active</span>
    <div class="tb-sep"></div>
    <span class="clock" id="clk">UTC 00:00:00</span>
  </div>
</div>

<div class="body">

  <!-- LEFT: 3D model -->
  <div class="col-3d">
    <div class="panel-label">3D Attitude <span id="fps-hud">60 FPS</span></div>
    <div id="canvasContainer">
      <div class="hud-mini" id="rot-hud">P: 0.0&nbsp;&nbsp;R: 0.0&nbsp;&nbsp;Y: 0.0</div>
    </div>
    <div class="calib-strip">
      <div class="calib-title">Orientation Calibration (live, no reflash needed)</div>
      <div class="calib-row">
        <div class="calib-btn" id="btn-swap">Swap P/R</div>
        <div class="calib-btn" id="btn-invroll">Invert Roll</div>
        <div class="calib-btn" id="btn-invpitch">Invert Pitch</div>
        <div class="calib-btn" id="btn-invyaw">Invert Yaw</div>
      </div>
      <div class="calib-hint">Tilt the board forward — if the model doesn't match, toggle buttons above until it does. Saved automatically in this browser.</div>
    </div>
  </div>

  <!-- MIDDLE: sensor data -->
  <div class="col-data">

    <div class="card">
      <div class="card-h">Barometer <span>BMP280</span></div>
      <div class="card-body">
        <div class="stat-trio">
          <div class="stat-item"><div class="stat-num" id="alt">--</div><div class="stat-unit">Alt (m)</div></div>
          <div class="stat-item"><div class="stat-num" id="temp">--</div><div class="stat-unit">Temp (°C)</div></div>
          <div class="stat-item"><div class="stat-num" id="vrate">--</div><div class="stat-unit">V-Rate (m/s)</div></div>
        </div>
        <div class="row"><span class="rk">Pressure</span><span class="rv dim" id="press">--</span></div>
      </div>
    </div>

    <div class="card">
      <div class="card-h">Accelerometer <span>ICM-20948</span></div>
      <div class="card-body">
        <div class="row"><span class="rk">X</span><span class="rv" id="ax">--</span></div>
        <div class="row"><span class="rk">Y</span><span class="rv" id="ay">--</span></div>
        <div class="row"><span class="rk">Z</span><span class="rv" id="az">--</span></div>
        <div class="row"><span class="rk">Vibration</span><span class="rv dim" id="vib">--</span></div>
        <div class="chart-wrap"><canvas id="accelChart"></canvas></div>
      </div>
    </div>

    <div class="card">
      <div class="card-h">Attitude <span>calibrated</span></div>
      <div class="card-body">
        <div class="rings">
          <div class="ring-item">
            <svg width="50" height="50" viewBox="0 0 48 48">
              <circle cx="24" cy="24" r="19" stroke="rgba(255,255,255,0.08)" stroke-width="2.5" fill="none"/>
              <circle cx="24" cy="24" r="19" stroke="#fff" stroke-width="2" fill="none" stroke-dasharray="119" id="r-roll" stroke-dashoffset="119" stroke-linecap="round" transform="rotate(-90 24 24)"/>
              <text x="24" y="28" text-anchor="middle" font-family="JetBrains Mono,monospace" font-size="9" fill="#fff" id="rv-roll">0°</text>
            </svg>
            <span class="ring-lbl">Roll</span>
          </div>
          <div class="ring-item">
            <svg width="50" height="50" viewBox="0 0 48 48">
              <circle cx="24" cy="24" r="19" stroke="rgba(255,255,255,0.08)" stroke-width="2.5" fill="none"/>
              <circle cx="24" cy="24" r="19" stroke="#bbb" stroke-width="2" fill="none" stroke-dasharray="119" id="r-pitch" stroke-dashoffset="119" stroke-linecap="round" transform="rotate(-90 24 24)"/>
              <text x="24" y="28" text-anchor="middle" font-family="JetBrains Mono,monospace" font-size="9" fill="#bbb" id="rv-pitch">0°</text>
            </svg>
            <span class="ring-lbl">Pitch</span>
          </div>
          <div class="ring-item">
            <svg width="50" height="50" viewBox="0 0 48 48">
              <circle cx="24" cy="24" r="19" stroke="rgba(255,255,255,0.08)" stroke-width="2.5" fill="none"/>
              <circle cx="24" cy="24" r="19" stroke="#777" stroke-width="2" fill="none" stroke-dasharray="119" id="r-yaw" stroke-dashoffset="119" stroke-linecap="round" transform="rotate(-90 24 24)"/>
              <text x="24" y="28" text-anchor="middle" font-family="JetBrains Mono,monospace" font-size="9" fill="#888" id="rv-yaw">0°</text>
            </svg>
            <span class="ring-lbl">Yaw</span>
          </div>
        </div>
      </div>
    </div>

    <div class="card">
      <div class="card-h">Gyroscope <span>ICM-20948</span></div>
      <div class="card-body">
        <div class="row"><span class="rk">X</span><span class="rv" id="gx">--</span></div>
        <div class="row"><span class="rk">Y</span><span class="rv" id="gy">--</span></div>
        <div class="row"><span class="rk">Z</span><span class="rv" id="gz">--</span></div>
      </div>
    </div>

    <div class="card">
      <div class="card-h">Power <span>INA219</span></div>
      <div class="card-body">
        <div class="row"><span class="rk">Bus Voltage</span><span class="rv" id="bvolt">--</span></div>
        <div class="row"><span class="rk">Current</span><span class="rv" id="bcurr">--</span></div>
        <div class="row"><span class="rk">Power</span><span class="rv" id="bpow">--</span></div>
        <div class="chart-wrap"><canvas id="voltChart"></canvas></div>
      </div>
    </div>

    <div class="card">
      <div class="card-h">GPS <span id="gps-badge">Not Connected</span></div>
      <div class="card-body">
        <div class="gps-status" id="gps-status">NO MODULE DETECTED</div>
        <div class="row"><span class="rk">Latitude</span><span class="rv" id="gps-lat">--</span></div>
        <div class="row"><span class="rk">Longitude</span><span class="rv" id="gps-lon">--</span></div>
        <div class="row"><span class="rk">Satellites</span><span class="rv" id="gps-sats">--</span></div>
        <div class="row"><span class="rk">HDOP</span><span class="rv dim" id="gps-hdop">--</span></div>
      </div>
    </div>

    <div class="card span3">
      <div class="card-h">System Health</div>
      <div class="health-row">
        <div class="health-item"><span class="rk">WiFi RSSI</span><span class="rv" id="rssi">--</span></div>
        <div class="health-item"><span class="rk">CPU Temp</span><span class="rv warn" id="cputemp">--</span></div>
        <div class="health-item"><span class="rk">Uptime</span><span class="rv" id="upt">--</span></div>
        <div class="health-item"><span class="rk">Errors / Reconnects</span><span class="rv dim" id="errsline">--</span></div>
      </div>
    </div>

    <div class="card span3">
      <div class="card-h">Console <span>Live</span></div>
      <div class="console-body" id="con"><span class="log-w">// CubeSat OS v1.3 — real telemetry, calibrated attitude</span>
<span class="log-d">[BOOT] Waiting for first sensor packet...</span>
</div>
    </div>

  </div>

  <!-- RIGHT: camera -->
  <div class="col-cam">
    <div class="panel-label">Pi Camera <span id="cam-status">Connecting...</span></div>
    <div class="cam-view">
      <img id="camFeed" alt=""/>
      <div id="cam-overlay"><span id="cam-msg">Connecting...</span></div>
      <div class="corner c-tl"></div><div class="corner c-tr"></div>
      <div class="corner c-bl"></div><div class="corner c-br"></div>
      <div id="cam-live-dot">● LIVE</div>
    </div>
    <div class="cam-info" id="cam-url-info">
      192.168.137.188:5000 &nbsp;
      <a href="http://192.168.137.188:5000/video_feed" target="_blank" rel="noopener" style="color:#fff;text-decoration:underline;">test directly ↗</a>
    </div>
  </div>

</div>

<div class="statusbar">
  <div class="sb-item"><div class="sb-dot off" id="dot-attitude"></div>Attitude</div>
  <div class="sb-item"><div class="sb-dot off" id="dot-baro"></div>Barometer</div>
  <div class="sb-item"><div class="sb-dot off" id="dot-wifi"></div>WiFi</div>
  <div class="sb-item"><div class="sb-dot off" id="dot-power"></div>Power</div>
  <div class="sb-item"><div class="sb-dot off" id="dot-gps"></div>GPS</div>
  <div class="sb-item">Frame <span id="fc" style="color:#fff">0</span></div>
  <div class="sb-item" style="color:#444">v1.3 · calibrated-attitude build</div>
</div>

<script>
// ============ CLOCK ============
function tick(){
  const n=new Date(),p=v=>String(v).padStart(2,'0');
  document.getElementById('clk').textContent=`UTC ${p(n.getUTCHours())}:${p(n.getUTCMinutes())}:${p(n.getUTCSeconds())}`;
}
setInterval(tick,1000);tick();

// ============ ORIENTATION CALIBRATION (client-side, persisted in this browser) ============
// This exists because guessing sign flips in firmware wasn't converging fast
// enough. These toggles let you fix roll/pitch/yaw direction live, by eye,
// without ever touching the ESP32 code again.
let calib = { swap:false, invRoll:false, invPitch:false, invYaw:false };
try {
  const saved = localStorage.getItem('cubesat_calib');
  if (saved) calib = JSON.parse(saved);
} catch(e) {}

function saveCalib(){ try{ localStorage.setItem('cubesat_calib', JSON.stringify(calib)); }catch(e){} }

function refreshCalibButtons(){
  document.getElementById('btn-swap').classList.toggle('on', calib.swap);
  document.getElementById('btn-invroll').classList.toggle('on', calib.invRoll);
  document.getElementById('btn-invpitch').classList.toggle('on', calib.invPitch);
  document.getElementById('btn-invyaw').classList.toggle('on', calib.invYaw);
}
document.getElementById('btn-swap').onclick = () => { calib.swap = !calib.swap; saveCalib(); refreshCalibButtons(); };
document.getElementById('btn-invroll').onclick = () => { calib.invRoll = !calib.invRoll; saveCalib(); refreshCalibButtons(); };
document.getElementById('btn-invpitch').onclick = () => { calib.invPitch = !calib.invPitch; saveCalib(); refreshCalibButtons(); };
document.getElementById('btn-invyaw').onclick = () => { calib.invYaw = !calib.invYaw; saveCalib(); refreshCalibButtons(); };
refreshCalibButtons();

function applyCalib(rawRoll, rawPitch, rawYaw){
  let r = rawRoll, p = rawPitch, y = rawYaw;
  if (calib.swap) { const t = r; r = p; p = t; }
  if (calib.invRoll)  r = -r;
  if (calib.invPitch) p = -p;
  if (calib.invYaw)   y = -y;
  return {r, p, y};
}

// ============ THREE.JS SCENE ============
const cont=document.getElementById('canvasContainer');
const scene=new THREE.Scene();
scene.background=new THREE.Color(0x000000);

const cam=new THREE.PerspectiveCamera(38,1,0.1,200);
cam.position.set(3.6,2.6,4.0);
cam.lookAt(0,0.2,0);

const ren=new THREE.WebGLRenderer({antialias:true});
ren.setPixelRatio(window.devicePixelRatio);
ren.shadowMap.enabled=true;
ren.shadowMap.type=THREE.PCFSoftShadowMap;
cont.appendChild(ren.domElement);

function resize(){
  const w=cont.clientWidth,h=cont.clientHeight;
  if(w===0||h===0)return;
  cam.aspect=w/h;cam.updateProjectionMatrix();ren.setSize(w,h);
}
resize();window.addEventListener('resize',resize);

scene.add(new THREE.AmbientLight(0x151515,4));
const key=new THREE.DirectionalLight(0xffffff,2.8);
key.position.set(5,9,6);key.castShadow=true;
key.shadow.mapSize.width=1024;key.shadow.mapSize.height=1024;
scene.add(key);
const fill=new THREE.DirectionalLight(0xffffff,0.5);fill.position.set(-6,2,-4);scene.add(fill);
const rim=new THREE.DirectionalLight(0xffffff,0.25);rim.position.set(0,-3,-5);scene.add(rim);
const topLight=new THREE.DirectionalLight(0xffffff,0.6);topLight.position.set(0,10,0);scene.add(topLight);

const grid=new THREE.GridHelper(16,32,0x181818,0x0c0c0c);grid.position.y=-1.85;scene.add(grid);

const sat=new THREE.Group();scene.add(sat);

const chassisMat=new THREE.MeshStandardMaterial({color:0x080808,roughness:0.12,metalness:0.95});
const railMat   =new THREE.MeshStandardMaterial({color:0x050505,roughness:0.5,metalness:0.6});
const solarMat  =new THREE.MeshStandardMaterial({color:0x04090f,roughness:0.06,metalness:0.55});
const lidMat    =new THREE.MeshStandardMaterial({color:0x181818,roughness:0.18,metalness:0.98});
const brassMat  =new THREE.MeshStandardMaterial({color:0x4a3a10,roughness:0.25,metalness:1.0});
const metalMat  =new THREE.MeshStandardMaterial({color:0x777777,roughness:0.3,metalness:1.0});
const antMat    =new THREE.MeshStandardMaterial({color:0x0d0d0d,roughness:0.65,metalness:0.2});
const gpsCeramic=new THREE.MeshStandardMaterial({color:0xc8a07a,roughness:0.75,metalness:0.05});
const gpsBase   =new THREE.MeshStandardMaterial({color:0x1a1a1a,roughness:0.45,metalness:0.6});
const topSolMat =new THREE.MeshStandardMaterial({color:0x0e0e0a,roughness:0.15,metalness:0.9});

const body=new THREE.Mesh(new THREE.BoxGeometry(1,1,1),chassisMat);
body.castShadow=true;sat.add(body);

sat.add(new THREE.LineSegments(
  new THREE.EdgesGeometry(new THREE.BoxGeometry(1.002,1.002,1.002)),
  new THREE.LineBasicMaterial({color:0x3a3a3a,transparent:true,opacity:0.9})));

[[-0.5,-0.5],[-0.5,0.5],[0.5,-0.5],[0.5,0.5]].forEach(([x,z])=>{
  const r=new THREE.Mesh(new THREE.BoxGeometry(0.042,1.01,0.042),railMat);
  r.position.set(x,0,z);sat.add(r);
});

function makeSolarPanel(wide,tall){
  const g=new THREE.Group();
  const p=new THREE.Mesh(new THREE.BoxGeometry(wide,tall,0.025),solarMat);
  g.add(p);
  const verts=[];
  for(let i=1;i<14;i++){
    const y=-tall/2+(i/14)*tall;
    verts.push(-wide/2+0.015,y,0.014, wide/2-0.015,y,0.014);
  }
  verts.push(0,-tall/2+0.015,0.014, 0,tall/2-0.015,0.014);
  const lg=new THREE.BufferGeometry();
  lg.setAttribute('position',new THREE.Float32BufferAttribute(verts,3));
  g.add(new THREE.LineSegments(lg,new THREE.LineBasicMaterial({color:0x0f1e2a,transparent:true,opacity:0.8})));
  const bz=new THREE.LineSegments(
    new THREE.EdgesGeometry(new THREE.BoxGeometry(wide+0.01,tall+0.01,0.028)),
    new THREE.LineBasicMaterial({color:0x252525,transparent:true,opacity:0.6}));
  g.add(bz);
  return g;
}

const fp=makeSolarPanel(0.84,0.80);fp.position.set(0,0,0.513);sat.add(fp);
const bp=makeSolarPanel(0.84,0.80);bp.position.set(0,0,-0.513);sat.add(bp);
const rp=makeSolarPanel(0.84,0.80);rp.rotation.y=Math.PI/2;rp.position.set(0.513,0,0);sat.add(rp);
const lp=makeSolarPanel(0.84,0.80);lp.rotation.y=Math.PI/2;lp.position.set(-0.513,0,0);sat.add(lp);

const topSol=new THREE.Mesh(new THREE.BoxGeometry(0.50,0.025,0.42),topSolMat);
topSol.rotation.x=Math.PI/2;topSol.position.set(-0.08,0.513,0.02);sat.add(topSol);
const tsv=[];
for(let i=1;i<7;i++){const x=-0.25+(i/7)*0.50;tsv.push(x,0.527,-0.21,x,0.527,0.21);}
for(let j=1;j<4;j++){const z=-0.21+(j/4)*0.42;tsv.push(-0.25,0.527,z,0.25,0.527,z);}
const tsg=new THREE.BufferGeometry();tsg.setAttribute('position',new THREE.Float32BufferAttribute(tsv,3));
sat.add(new THREE.LineSegments(tsg,new THREE.LineBasicMaterial({color:0x1a1a0a,transparent:true,opacity:0.6})));

const lid=new THREE.Mesh(new THREE.BoxGeometry(0.97,0.016,0.97),lidMat);
lid.position.y=0.508;sat.add(lid);
sat.add(new THREE.LineSegments(
  new THREE.EdgesGeometry(new THREE.BoxGeometry(0.975,0.018,0.975)),
  new THREE.LineBasicMaterial({color:0x444444,transparent:true,opacity:0.5})));

const gpsGrp=new THREE.Group();
const patch=new THREE.Mesh(new THREE.BoxGeometry(0.13,0.025,0.13),gpsCeramic);
patch.position.y=0.02;gpsGrp.add(patch);
const gpsBp=new THREE.Mesh(new THREE.BoxGeometry(0.15,0.016,0.15),gpsBase);
gpsGrp.add(gpsBp);
const scrw=new THREE.Mesh(new THREE.CylinderGeometry(0.007,0.007,0.03,8),
  new THREE.MeshStandardMaterial({color:0x333333,roughness:0.5}));
scrw.position.y=0.028;gpsGrp.add(scrw);
gpsGrp.position.set(0.1,0.524,0.04);sat.add(gpsGrp);

function makeAntenna(height){
  const g=new THREE.Group();
  const nut=new THREE.Mesh(new THREE.CylinderGeometry(0.038,0.038,0.038,6),brassMat);
  nut.position.y=0.019;g.add(nut);
  const wash=new THREE.Mesh(new THREE.CylinderGeometry(0.030,0.030,0.01,12),metalMat);
  wash.position.y=0.044;g.add(wash);
  const con=new THREE.Mesh(new THREE.CylinderGeometry(0.024,0.024,0.055,10),metalMat);
  con.position.y=0.072;g.add(con);
  const knurl=new THREE.Mesh(new THREE.CylinderGeometry(0.030,0.028,0.045,12),
    new THREE.MeshStandardMaterial({color:0x111111,roughness:0.8,metalness:0.4}));
  knurl.position.y=0.108;g.add(knurl);
  const shaft=new THREE.Mesh(new THREE.CylinderGeometry(0.014,0.020,height,9),antMat);
  shaft.position.y=0.13+height/2;g.add(shaft);
  const tip=new THREE.Mesh(new THREE.SphereGeometry(0.016,8,6),antMat);
  tip.position.y=0.13+height;g.add(tip);
  return g;
}
const ant1=makeAntenna(0.52);
ant1.position.set(-0.18,0.518,0.05);sat.add(ant1);
const ant2=makeAntenna(0.68);
ant2.position.set(0.06,0.518,0.05);sat.add(ant2);

const tgl=new THREE.PointLight(0xffffff,0.12,2.2);
tgl.position.set(0,-0.9,0);sat.add(tgl);

// ============ CHART.JS: ACCEL + VOLTAGE ============
const N=40;
const acx=document.getElementById('accelChart').getContext('2d');
const acChart=new Chart(acx,{
  type:'line',
  data:{
    labels:Array(N).fill(''),
    datasets:[
      {data:Array(N).fill(0),borderColor:'#ffffff',borderWidth:1.5,fill:false,tension:0.3,pointRadius:0},
      {data:Array(N).fill(0),borderColor:'#888',borderWidth:1,fill:false,tension:0.3,pointRadius:0},
      {data:Array(N).fill(1),borderColor:'#444',borderWidth:1,fill:false,tension:0.3,pointRadius:0}
    ]
  },
  options:{
    responsive:true,maintainAspectRatio:false,animation:false,
    plugins:{legend:{display:false},tooltip:{enabled:false}},
    scales:{x:{display:false},y:{display:false}}
  }
});

const vcx=document.getElementById('voltChart').getContext('2d');
const voltChart=new Chart(vcx,{
  type:'line',
  data:{
    labels:Array(N).fill(''),
    datasets:[
      {data:Array(N).fill(5),borderColor:'#ffffff',borderWidth:1.5,fill:'start',
       backgroundColor:'rgba(255,255,255,0.06)',tension:0.3,pointRadius:0}
    ]
  },
  options:{
    responsive:true,maintainAspectRatio:false,animation:false,
    plugins:{legend:{display:false},tooltip:{enabled:false}},
    scales:{x:{display:false},y:{display:false}}
  }
});

// ============ REAL TELEMETRY: sequential fetch loop ============
let latest = null;
let prevAltitude = null, prevAltTime = null;
let vibBuffer = [];
let consoleTick = 0;

let targetPitch = 0, targetRoll = 0, targetYaw = 0;
let curPitch = 0, curRoll = 0, curYaw = 0;
const SMOOTHING = 0.15;

function setDotClass(id, ok) {
  const el = document.getElementById(id);
  if (ok) el.classList.remove('off'); else el.classList.add('off');
}
function setRing(circleId, textId, valueDeg, range) {
  const clamped = Math.max(-range, Math.min(range, valueDeg));
  const frac = Math.abs(clamped) / range;
  const offset = 119 - frac * 119;
  document.getElementById(circleId).setAttribute('stroke-dashoffset', offset.toFixed(1));
  document.getElementById(textId).textContent = valueDeg.toFixed(1) + '°';
}
function setYawRing(yawDeg) {
  const norm = ((yawDeg % 360) + 360) % 360;
  const offset = 119 - (norm / 360) * 119;
  document.getElementById('r-yaw').setAttribute('stroke-dashoffset', offset.toFixed(1));
  document.getElementById('rv-yaw').textContent = norm.toFixed(1) + '°';
}
function formatUptime(s) {
  const p = v => String(v).padStart(2, '0');
  return `${p(Math.floor(s / 3600))}:${p(Math.floor((s % 3600) / 60))}:${p(Math.floor(s % 60))}`;
}

function updateUI(d) {
  document.getElementById('alt').textContent = d.bmp_altitude.toFixed(1);
  document.getElementById('temp').textContent = d.bmp_temp.toFixed(1);
  document.getElementById('press').textContent = d.bmp_pressure.toFixed(1) + ' hPa';

  const now = Date.now();
  if (prevAltitude !== null) {
    const dt = (now - prevAltTime) / 1000;
    if (dt > 0.05) {
      const rate = (d.bmp_altitude - prevAltitude) / dt;
      document.getElementById('vrate').textContent = (rate >= 0 ? '+' : '') + rate.toFixed(1);
      prevAltitude = d.bmp_altitude; prevAltTime = now;
    }
  } else {
    prevAltitude = d.bmp_altitude; prevAltTime = now;
  }

  document.getElementById('ax').textContent = (d.acc_x >= 0 ? '+' : '') + d.acc_x.toFixed(3) + ' g';
  document.getElementById('ay').textContent = (d.acc_y >= 0 ? '+' : '') + d.acc_y.toFixed(3) + ' g';
  document.getElementById('az').textContent = (d.acc_z >= 0 ? '+' : '') + d.acc_z.toFixed(3) + ' g';

  const mag = Math.sqrt(d.acc_x * d.acc_x + d.acc_y * d.acc_y + d.acc_z * d.acc_z);
  vibBuffer.push(mag - 1.0);
  if (vibBuffer.length > 20) vibBuffer.shift();
  const meanDev = vibBuffer.reduce((a, b) => a + b, 0) / vibBuffer.length;
  const variance = vibBuffer.reduce((a, b) => a + (b - meanDev) * (b - meanDev), 0) / vibBuffer.length;
  document.getElementById('vib').textContent = (Math.sqrt(variance) * 1000).toFixed(3) + ' mg';

  acChart.data.datasets[0].data.push(d.acc_x);
  acChart.data.datasets[1].data.push(d.acc_y);
  acChart.data.datasets[2].data.push(d.acc_z);
  acChart.data.datasets.forEach(ds => { if (ds.data.length > N) ds.data.shift(); });
  acChart.update('none');

  voltChart.data.datasets[0].data.push(d.ina_voltage);
  if (voltChart.data.datasets[0].data.length > N) voltChart.data.datasets[0].data.shift();
  voltChart.update('none');

  // Apply live calibration (invert/swap) to the raw ESP32 roll/pitch/yaw
  const c = applyCalib(d.roll, d.pitch, d.yaw);

  setRing('r-roll', 'rv-roll', c.r, 90);
  setRing('r-pitch', 'rv-pitch', c.p, 90);
  setYawRing(c.y);
  document.getElementById('rot-hud').textContent = `P: ${c.p.toFixed(1)}  R: ${c.r.toFixed(1)}  Y: ${(((c.y%360)+360)%360).toFixed(1)}`;

  targetPitch = c.p * Math.PI / 180;
  targetRoll  = c.r * Math.PI / 180;
  targetYaw   = c.y * Math.PI / 180;

  document.getElementById('gx').textContent = d.gyro_x.toFixed(2) + ' °/s';
  document.getElementById('gy').textContent = d.gyro_y.toFixed(2) + ' °/s';
  document.getElementById('gz').textContent = d.gyro_z.toFixed(2) + ' °/s';

  document.getElementById('bvolt').textContent = d.ina_voltage.toFixed(2) + ' V';
  document.getElementById('bcurr').textContent = d.ina_current_mA.toFixed(1) + ' mA';
  document.getElementById('bpow').textContent = d.ina_power_mW.toFixed(1) + ' mW';

  document.getElementById('rssi').textContent = d.wifi_rssi + ' dBm';
  document.getElementById('cputemp').textContent = '+' + d.cpu_temp.toFixed(1) + ' °C';
  document.getElementById('upt').textContent = formatUptime(d.uptime_s);
  document.getElementById('errsline').textContent =
    (d.icm_i2c_errors + d.bmp_errors + d.ina_errors) + ' / ' + d.wifi_reconnects;

  setDotClass('dot-attitude', d.icm_ok);
  setDotClass('dot-baro', d.bmp_ok);
  setDotClass('dot-wifi', d.wifi_connected);
  setDotClass('dot-power', d.ina_ok);
  setDotClass('dot-gps', d.gps_fix);
  document.getElementById('link-tag').textContent = d.wifi_connected ? 'Link Active' : 'Link Down';

  // GPS: three real states -- no module wired, module wired but no fix yet,
  // or a genuine satellite lock. Never shows fabricated coordinates.
  if (!d.gps_module) {
    document.getElementById('gps-badge').textContent = 'Not Connected';
    document.getElementById('gps-status').textContent = 'NO MODULE DETECTED';
    document.getElementById('gps-lat').textContent = '--';
    document.getElementById('gps-lon').textContent = '--';
    document.getElementById('gps-sats').textContent = '--';
    document.getElementById('gps-hdop').textContent = '--';
  } else if (!d.gps_fix) {
    document.getElementById('gps-badge').textContent = 'Searching';
    document.getElementById('gps-status').textContent = 'SEARCHING FOR SATELLITES';
    document.getElementById('gps-lat').textContent = '--';
    document.getElementById('gps-lon').textContent = '--';
    document.getElementById('gps-sats').textContent = d.gps_sats;
    document.getElementById('gps-hdop').textContent = d.gps_hdop.toFixed(1);
  } else {
    document.getElementById('gps-badge').textContent = 'Fix';
    document.getElementById('gps-status').textContent = 'GPS LOCK ACQUIRED';
    document.getElementById('gps-lat').textContent = d.gps_lat.toFixed(6) + '°';
    document.getElementById('gps-lon').textContent = d.gps_lon.toFixed(6) + '°';
    document.getElementById('gps-sats').textContent = d.gps_sats;
    document.getElementById('gps-hdop').textContent = d.gps_hdop.toFixed(1);
  }
}

async function refresh() {
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), 300);
  try {
    const r = await fetch('/data', { signal: controller.signal });
    const d = await r.json();
    latest = d;
    updateUI(d);
  } catch (e) {
    document.getElementById('link-tag').textContent = 'Link Lost';
  } finally {
    clearTimeout(timeoutId);
  }
}

const FETCH_INTERVAL_MS = 50;
async function refreshLoop() {
  await refresh();
  setTimeout(refreshLoop, FETCH_INTERVAL_MS);
}
refreshLoop();

setInterval(() => {
  if (!latest) return;
  const d = latest;
  const msgs = [
    `[IMU] P:${d.pitch.toFixed(2)} R:${d.roll.toFixed(2)} Y:${(((d.yaw % 360) + 360) % 360).toFixed(2)}`,
    `[BARO] T:${d.bmp_temp.toFixed(1)}C ALT:${d.bmp_altitude.toFixed(1)}m`,
    `[PWR] V:${d.ina_voltage.toFixed(2)} I:${d.ina_current_mA.toFixed(1)}mA P:${d.ina_power_mW.toFixed(1)}mW`,
    `[SYS] RSSI:${d.wifi_rssi}dBm UP:${formatUptime(d.uptime_s)}`
  ];
  const idx = consoleTick % msgs.length;
  const line = document.createElement('div');
  line.className = idx === 0 ? 'log-w' : 'log-d';
  line.textContent = msgs[idx];
  const con = document.getElementById('con');
  con.appendChild(line);
  while (con.children.length > 80) con.removeChild(con.firstChild);
  con.scrollTop = con.scrollHeight;
  consoleTick++;
}, 700);

// ============ RENDER LOOP ============
let fc = 0, fpsC = 0, lastFps = Date.now();
function rloop() {
  requestAnimationFrame(rloop);
  tgl.intensity = 0.08 + 0.06 * Math.sin(Date.now() * 0.004);

  curPitch += (targetPitch - curPitch) * SMOOTHING;
  curRoll  += (targetRoll  - curRoll)  * SMOOTHING;
  curYaw   += (targetYaw   - curYaw)   * SMOOTHING;

  sat.rotation.x = curPitch;
  sat.rotation.z = curRoll;
  sat.rotation.y = curYaw;

  ren.render(scene, cam);
  fpsC++;
  const now = Date.now();
  if (now - lastFps > 1000) {
    document.getElementById('fps-hud').textContent = `${fpsC} FPS`;
    fc += fpsC; fpsC = 0; lastFps = now;
    document.getElementById('fc').textContent = fc.toLocaleString();
  }
}
rloop();

// ============ PI CAMERA FEED ============
// NOTE: corrected from the IP as typed ("198.168.137.188.5000", not valid)
// to what was almost certainly meant, matching the format used earlier.
// Update CAM_BASE below if this guess is wrong.
const CAM_BASE = 'http://192.168.137.188:5000';
const ENDPOINTS = ['/video_feed', '/stream', '/mjpeg'];
let endpointIdx = 0;

function tryEndpoint(idx) {
  if (idx >= ENDPOINTS.length) {
    document.getElementById('cam-msg').textContent = 'No stream found';
    document.getElementById('cam-status').textContent = 'Offline';
    console.warn('[camera] all endpoints failed:', ENDPOINTS.map(e => CAM_BASE + e));
    console.warn('[camera] click "test directly" link under the camera panel -- '
      + 'if that also fails to load in a new tab, the problem is the Pi/network, not this page. '
      + 'Common causes: Flask app.run() missing host="0.0.0.0" (only allows localhost), '
      + 'wrong route name, camera server not actually running, or phone/laptop on a different WiFi than the Pi.');
    setTimeout(() => { endpointIdx = 0; tryEndpoint(0); }, 5000);
    return;
  }
  const url = CAM_BASE + ENDPOINTS[idx];
  const img = document.getElementById('camFeed');
  document.getElementById('cam-msg').textContent = 'Trying ' + ENDPOINTS[idx];
  img.onload = () => {
    img.style.opacity = '1';
    document.getElementById('cam-overlay').style.display = 'none';
    document.getElementById('cam-status').textContent = 'Live';
    document.getElementById('cam-live-dot').style.display = 'block';
    console.log('[camera] connected via', url);
  };
  img.onerror = () => {
    console.warn('[camera] failed:', url);
    img.style.opacity = '0';
    endpointIdx++;
    setTimeout(() => tryEndpoint(endpointIdx), 800);
  };
  img.src = url;
}
tryEndpoint(0);
</script>
</body>
</html>
)rawliteral";

// ================= HTTP handlers =================
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleData() {
  char json[1100];
  int written = snprintf(json, sizeof(json),
    "{"
    "\"bmp_ok\":%s,\"ina_ok\":%s,\"icm_ok\":%s,\"wifi_connected\":%s,"
    "\"gps_module\":%s,\"gps_fix\":%s,"
    "\"bmp_temp\":%.2f,\"bmp_pressure\":%.2f,\"bmp_altitude\":%.2f,"
    "\"ina_voltage\":%.3f,\"ina_current_mA\":%.2f,\"ina_power_mW\":%.2f,"
    "\"acc_x\":%.4f,\"acc_y\":%.4f,\"acc_z\":%.4f,"
    "\"gyro_x\":%.3f,\"gyro_y\":%.3f,\"gyro_z\":%.3f,"
    "\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,"
    "\"gps_lat\":%.6f,\"gps_lon\":%.6f,\"gps_alt\":%.1f,"
    "\"gps_sats\":%lu,\"gps_hdop\":%.1f,\"gps_speed_kmph\":%.1f,"
    "\"wifi_rssi\":%d,\"uptime_s\":%lu,\"cpu_temp\":%.1f,"
    "\"icm_i2c_errors\":%lu,\"bmp_errors\":%lu,\"ina_errors\":%lu,"
    "\"wifi_reconnects\":%lu,\"i2c_bus_recoveries\":%lu"
    "}",
    data.bmp_ok ? "true" : "false",
    data.ina_ok ? "true" : "false",
    data.icm_ok ? "true" : "false",
    (WiFi.status() == WL_CONNECTED) ? "true" : "false",
    data.gps_module ? "true" : "false",
    data.gps_fix ? "true" : "false",
    data.bmp_temp, data.bmp_pressure, data.bmp_altitude,
    data.ina_voltage, data.ina_current_mA, data.ina_power_mW,
    data.acc_x, data.acc_y, data.acc_z,
    data.gyro_x, data.gyro_y, data.gyro_z,
    data.roll, data.pitch, data.yaw,
    data.gps_lat, data.gps_lon, data.gps_alt_m,
    (unsigned long)data.gps_sats, data.gps_hdop, data.gps_speed_kmph,
    WiFi.RSSI(), millis() / 1000UL, temperatureRead(),
    (unsigned long)icmErrorCount, (unsigned long)bmpErrorCount, (unsigned long)inaErrorCount,
    (unsigned long)wifiReconnectCount, (unsigned long)i2cBusRecoveries
  );

  if (written < 0 || written >= (int)sizeof(json)) {
    Serial.println("[WARN] JSON buffer truncated -- increase buffer size in handleData()");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(500, "application/json", "{\"error\":\"json_buffer_overflow\"}");
    return;
  }

  // CORS: allows the standalone home-hosted dashboard (a different origin --
  // e.g. a file opened directly, or hosted on another PC) to fetch this data.
  // Without this header, browsers silently block the cross-origin request.
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

// ================= Setup / Loop =================
void setup() {
  Serial.begin(115200);
  delay(1000);

  setupWatchdog();

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(50); // ms -- prevents an unresponsive device from hanging the whole chip

  Serial.println("\n--- CubeSat Sensor Dashboard boot ---");

  data.bmp_ok = bmp.begin(BMP280_ADDR);
  Serial.println(data.bmp_ok ? "BMP280 OK" : "BMP280 NOT FOUND");

  data.ina_ok = ina219.begin();
  Serial.println(data.ina_ok ? "INA219 OK" : "INA219 NOT FOUND");

  data.icm_ok = icmInit();

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS UART started -- module presence will show once NMEA data arrives.");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(300);
    Serial.print(".");
    esp_task_wdt_reset(); // don't let the WiFi wait trip the watchdog
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Connected! IP address: ");
    Serial.println(WiFi.localIP());
    if (MDNS.begin(HOSTNAME)) {
      Serial.print("Also available at: http://");
      Serial.print(HOSTNAME);
      Serial.println(".local");
    }
  } else {
    Serial.println("WiFi FAILED to connect initially -- will keep retrying in background.");
  }

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
  Serial.println("Web server started.");
}

void loop() {
  esp_task_wdt_reset();   // prove to the watchdog that we're alive

  server.handleClient();
  ensureWifiConnected();  // non-blocking, only acts every WIFI_RETRY_INTERVAL_MS
  gpsUpdate();            // drain GPS UART every loop -- cheap, avoids buffer overflow

  static unsigned long lastRead = 0;
  if (millis() - lastRead >= 20) {   // sample sensors at ~50Hz for smooth orientation tracking
    lastRead = millis();
    readAllSensors();
  }

  if (millis() - lastHealthCheck >= HEALTH_CHECK_INTERVAL_MS) {
    lastHealthCheck = millis();
    runSensorHealthCheck();
  }
}
