/* Dispositivo 4G by 4Spark
 * V2.8 (ICM rimosso → D7S: SI/PGA/EQ con anti-stallo a 3 s)
 * 2025
 */

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <math.h>
#include <Wire.h>
#include <SPI.h>

unsigned int time_of_system_update;
static unsigned int default_time_of_system_update = 0;
unsigned long lastEpochTime = 0;
unsigned long lastMillis = 0;
String latitude;
String longitude;

// === File locali (restano i tuoi) ===
#include "sd_card.h"
#include "dateTime.h"

// === Libreria webserver UNIFICATA (tutto in un solo .h) ===
#include "webServer.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

// --- Sensirion SCD30 & SEN55 ---
#include <SensirionI2cScd30.h>
#include <SensirionI2CSen5x.h>

#define WIRE Wire

// ======= D7S (Grove) =======
#include <D7S.h>
// Logica anti-stallo (come nel tuo esempio)
static const uint32_t D7S_STALE_TIME_MS       = 40000;
static const uint32_t D7S_EQ_SEND_INTERVAL_MS = 5000;
static const float    D7S_EPS_SI              = 0.002f;  // m/s
static const float    D7S_EPS_PGA             = 0.020f;  // m/s^2
// Stato anti-stallo
static bool     d7s_eq_app         = false;
static uint32_t d7s_last_change_ms = 0;
static uint32_t d7s_last_send_ms   = 0;
static float    d7s_last_si        = 0.0f;
static float    d7s_last_pga       = 0.0f;
static float    d7s_si_out         = 0.0f;
static float    d7s_pga_out        = 0.0f;
static int      d7s_eq_out         = 0;

static void collectAndSendSamples(bool earthquakeTriggered);
static void updateD7SState();

// Flags / buffer tempo
int flag = 0;
char currentDate[50];
char currentTime[50];
int tempTick = 0;

// ======= Sensori =======
// SCD30
SensirionI2cScd30 scd30;
static char errorMessage[128];
static int16_t error;

// SEN55
SensirionI2CSen5x sen5x;

// BME680
Adafruit_BME680 BME680;
bool bme680_ok = false;

/* =========================
   BATTERIA
   ========================= */
#define BAT_ADC_PIN 35
#define ADC_MAX 4095.0f
#define ADC_REF_VOLTAGE 3.3f
#define DIVIDER_RATIO 2.0f
#define CALIB_K 1.00f
#define VBAT_EMPTY 3.00f
#define VBAT_FULL  4.20f

static float readBatteryVoltage() {
  const int N = 8;
  uint32_t acc = 0;
  for (int i = 0; i < N; ++i) { acc += analogRead(BAT_ADC_PIN); delayMicroseconds(300); }
  float raw = (float)acc / (float)N;
  float v_adc = (raw / ADC_MAX) * ADC_REF_VOLTAGE;
  float v_bat = v_adc * DIVIDER_RATIO;
  return v_bat * CALIB_K;
}

static int voltageToPercent(float v) {
  if (v <= VBAT_EMPTY) return 0;
  if (v >= VBAT_FULL)  return 100;
  return (int)roundf((v - VBAT_EMPTY) * 100.0f / (VBAT_FULL - VBAT_EMPTY));
}

// ====== Utility I2C ======
static bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

/* ============================================================
   GNSS (SIMCom A7608E-H) — TinyGSM + TinyGPSPlus
   ============================================================ */
#define TINY_GSM_MODEM_SIM7600
#define TINY_GSM_RX_BUFFER 1024
#include <TinyGsmClient.h>
#include <TinyGPSPlus.h>

#define UART_BAUD    115200
#define MODEM_DTR_PIN 25
#define MODEM_TXD_PIN 26
#define MODEM_RXD_PIN 27
#define MODEM_PWR_PIN 4
#define BOARD_POWER_ON 12
#define MODEM_RI_PIN 33
#define MODEM_RST_PIN 5
#define SerialAT Serial1

TinyGsm modem(SerialAT);
TinyGPSPlus gps;

static bool gnssReady = false;
static unsigned long lastGNSSAttemptMs = 0;

static void gnssFeed() { while (SerialAT.available()) gps.encode(SerialAT.read()); }
static bool ensureModemAlive() {
  int retry = 0;
  while (!modem.testAT(1000)) {
    if (retry == 0) Serial.print(F("Modem starting"));
    Serial.print(".");
    if (++retry > 10) { digitalWrite(MODEM_PWR_PIN, LOW); delay(100); digitalWrite(MODEM_PWR_PIN, HIGH); delay(1000); digitalWrite(MODEM_PWR_PIN, LOW); retry = 0; }
  }
  if (retry > 0) Serial.println();
  return true;
}
static bool gnssPowerOnSequence() {
  modem.sendAT("+CGDRT=5,1");   modem.waitResponse(10000L);
  modem.sendAT("+CGSETV=5,0");  modem.waitResponse(10000L);
  modem.sendAT("+CVAUXS=1");    modem.waitResponse(10000L);
  modem.sendAT("+CGNSSPWR=1");
  int8_t r = modem.waitResponse(10000UL, "+CGNSSPWR: READY!");
  if (r != 1) r = modem.waitResponse(5000UL);
  if (r != 1) { Serial.println(F("ERRORE: GNSS non si accende.")); return false; }
  modem.sendAT("+CGNSSIPR=9600");             modem.waitResponse(1000L);
  modem.sendAT("+CGNSSMODE=3");               modem.waitResponse(1000L);
  modem.sendAT("+CGNSSNMEA=1,1,1,1,1,1,0,0"); modem.waitResponse(1000L);
  modem.sendAT("+CGPSNMEARATE=1");            modem.waitResponse(1000L);
  modem.sendAT("+CGNSSTST=1");                modem.waitResponse(1000L);
  modem.sendAT("+CGNSSPORTSWITCH=0,1");       modem.waitResponse(1000L);
  Serial.println(F("GPS Ready!")); return true;
}
static void tryStartGNSS() {
  unsigned long now = millis();
  if (now - lastGNSSAttemptMs < 5000UL) return;
  lastGNSSAttemptMs = now;
  if (!ensureModemAlive()) return;
  gnssReady = gnssPowerOnSequence();
}
static bool gpsFixValid() { return gps.location.isValid() && gps.location.age() < 5000; }
static void updateLatLonStrings() {
  if (gpsFixValid()) { latitude = String(gps.location.lat(), 6); longitude = String(gps.location.lng(), 6); }
}

static void collectAndSendSamples(bool earthquakeTriggered) {
  Serial.println(earthquakeTriggered
                   ? F("=== Raccolta dati (trigger terremoto) ===")
                   : F("=== Raccolta dati (timer) ==="));

  // WiFi & NTP
  if (WiFi.status() != WL_CONNECTED) { WiFi.begin(); delay(4000); }
  if (WiFi.status() == WL_CONNECTED && !flag) { timeClient.begin(); flag = 1; }

  // Piccola attesa non bloccante con GNSS feed
  for (int i = 0; i < 20; ++i) { delay(100); gnssFeed(); }
  if (!gnssReady || (!gps.satellites.isValid() && !gps.location.isValid())) tryStartGNSS();

  // --- SCD30 ---
  float co2Concentration = NAN, SCD30temperature = NAN, SCD30humidity = NAN;
  uint16_t dataReady = 0;
  if (scd30.getDataReady(dataReady) == 0 && dataReady) {
    error = scd30.blockingReadMeasurementData(co2Concentration, SCD30temperature, SCD30humidity);
    if (error != NO_ERROR) { Serial.print("SCD30 read error: "); errorToString(error, errorMessage, sizeof errorMessage); Serial.println(errorMessage); }
    else {
      Serial.printf("SCD30 CO2=%.2f ppm (T=%.2fC RH=%.2f%%)\n", co2Concentration, SCD30temperature, SCD30humidity);
    }
  } else Serial.println("SCD30 data not ready.");

  // --- SEN55 ---
  float pm1p0=NAN, pm2p5=NAN, pm4p0=NAN, pm10p0=NAN;
  float senT=NAN, senRH=NAN, vocIndex=NAN, noxIndex=NAN;
  for (int i=0;i<15;++i){ delay(80); gnssFeed(); }
  uint16_t sErr = sen5x.readMeasuredValues(pm1p0, pm2p5, pm4p0, pm10p0, senRH, senT, vocIndex, noxIndex);
  if (sErr) { Serial.print("SEN55 read err=0x"); Serial.println(sErr, HEX); }
  else {
    Serial.printf("SEN55 PM1=%.2f PM2.5=%.2f PM4=%.2f PM10=%.2f VOC=%.2f NOx=%.2f (T=%.2fC RH=%.2f%%)\n",
                  pm1p0, pm2p5, pm4p0, pm10p0, vocIndex, noxIndex, senT, senRH);
  }
  uint32_t sen55_status_word=0; uint8_t sen55_status_b=0;
  uint16_t sStatErr = sen5x.readDeviceStatus(sen55_status_word);
  if (!sStatErr) {
    sen55_status_b = sen55StatusByte(sen55_status_word);
    Serial.printf("SEN55 status 0x%02X | FAN=%u SPEED=%u LASER=%u RHT=%u GAS=%u CLEAN=%u\n",
      sen55_status_b,
      (sen55_status_b>>0)&1, (sen55_status_b>>1)&1, (sen55_status_b>>2)&1,
      (sen55_status_b>>3)&1, (sen55_status_b>>4)&1, (sen55_status_b>>5)&1);
  } else { Serial.print("SEN55 status read err=0x"); Serial.println(sStatErr, HEX); }

  // --- BME680 ---
  static int BME680temp=0, BME680humidity=0, BME680pressure=0, BME680gas=0;
  if (bme680_ok) {
    if (!BME680.performReading()) Serial.println("BME680 reading failure.");
    else {
      BME680temp     = (int)round(BME680.temperature    * 100.0);
      BME680humidity = (int)round(BME680.humidity       * 100.0);
      BME680pressure = (int)round(BME680.pressure       * 100.0);
      BME680gas      = (int)round(BME680.gas_resistance * 100.0);
      Serial.printf("BME680 T=%.2fC RH=%.2f%% P=%.2fPa Gas=%.2fΩ\n",
                    BME680temp/100.0, BME680humidity/100.0, BME680pressure/100.0, BME680gas/100.0);
    }
  }

  int d7s_si_x100  = lroundf(d7s_si_out  * 100.0f);
  int d7s_pga_x100 = lroundf(d7s_pga_out * 100.0f);
  String d7sSiStr  = String(d7s_si_x100);
  String d7sPgaStr = String(d7s_pga_x100);
  Serial.printf("D7S SI=%.3f m/s (x100=%d)  PGA=%.3f m/s^2 (x100=%d)  EQ=%d\n",
                d7s_si_out, d7s_si_x100, d7s_pga_out, d7s_pga_x100, d7s_eq_out);

  // --- Batteria ---
  float vbat = readBatteryVoltage();
  int batPct = voltageToPercent(vbat);
  int batV_x100 = (int)lroundf(vbat * 100.0f);
  Serial.printf("BAT: %.3f V  ~ %d%%\n", vbat, batPct);

  // --- Data/ora ---
  unsigned long currentEpochTime;
  if (WiFi.status() == WL_CONNECTED) {
    if (timeClient.update()) { currentEpochTime = timeClient.getEpochTime(); lastEpochTime = currentEpochTime; lastMillis = millis(); }
    else                      { currentEpochTime = lastEpochTime + ((millis() - lastMillis) / 1000); }
  } else {
    currentEpochTime = lastEpochTime + ((millis() - lastMillis) / 1000);
  }
  struct tm *ptm = gmtime((time_t *)&currentEpochTime);
  sprintf(currentDate, "%4d/%02d/%02d", ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday);
  sprintf(currentTime, "%02d:%02d:%02d", ptm->tm_hour, ptm->tm_min, ptm->tm_sec);

  // --- GNSS: aggiorna lat/lon ---
  updateLatLonStrings();

  String macStr = String(WiFi.macAddress());

  // ========= ORDINAMENTO COLONNE: Sheet == SD =========
  // data, ora, mac_address,
  // co2_ppm,
  // pm1, pm2_5, pm4, pm10, voc_index, nox_index,
  // sen55_fan_err, sen55_speed_warn, sen55_laser_err, sen55_rht_err, sen55_gas_err, sen55_cleaning,
  // bme_temp_c_x100, bme_rh_x100, bme_pressure_pa_x100, bme_gas_ohm_x100,
  // d7s_eq_bit, d7s_si_mps_x100, d7s_pga_mps2_x100,
  // bat_mV_x100, bat_pct,
  // latitude, longitude

  // ===== SD =====
  writeDataOnSD(
    String(currentDate), String(currentTime), macStr,
    String(co2Concentration, 2),
    String(pm1p0), String(pm2p5), String(pm4p0), String(pm10p0),
    String(vocIndex), String(noxIndex),
    String((sen55_status_b>>0)&1), String((sen55_status_b>>1)&1), String((sen55_status_b>>2)&1),
    String((sen55_status_b>>3)&1), String((sen55_status_b>>4)&1), String((sen55_status_b>>5)&1),
    String(BME680temp), String(BME680humidity), String(BME680pressure), String(BME680gas),
    String(d7s_eq_out), d7sSiStr, d7sPgaStr,
    String(batV_x100), String(batPct),
    String(latitude), String(longitude)
  );

  // ===== Google Sheet =====
  if (WiFi.status() == WL_CONNECTED) {
    sendDataToGoogle(
      String(currentDate), String(currentTime), macStr,
      String(co2Concentration, 2),
      String(pm1p0), String(pm2p5), String(pm4p0), String(pm10p0),
      String(vocIndex), String(noxIndex),
      String((sen55_status_b>>0)&1), String((sen55_status_b>>1)&1), String((sen55_status_b>>2)&1),
      String((sen55_status_b>>3)&1), String((sen55_status_b>>4)&1), String((sen55_status_b>>5)&1),
      String(BME680temp), String(BME680humidity), String(BME680pressure), String(BME680gas),
      String(d7s_eq_out), d7sSiStr, d7sPgaStr,
      String(batV_x100), String(batPct),
      String(latitude), String(longitude)
    );
  }
}

static void updateD7SState() {
  float d7s_si_raw  = D7S.getInstantaneusSI();    // [m/s]
  float d7s_pga_raw = D7S.getInstantaneusPGA();   // [m/s^2]
  bool  d7s_eq_raw  = D7S.isEarthquakeOccuring(); // bit “terremoto” del sensore
  uint32_t nowMs    = millis();

  bool d7s_changed = (fabsf(d7s_si_raw  - d7s_last_si)  > D7S_EPS_SI) ||
                     (fabsf(d7s_pga_raw - d7s_last_pga) > D7S_EPS_PGA);

  bool prev_eq_app = d7s_eq_app;

  if (!d7s_eq_app && d7s_eq_raw && d7s_changed) {
    d7s_eq_app = true;
    d7s_last_change_ms = nowMs;
  }

  if (d7s_eq_app) {
    if (d7s_changed) d7s_last_change_ms = nowMs;
    if ((nowMs - d7s_last_change_ms) >= D7S_STALE_TIME_MS) {
      d7s_eq_app = false;
      D7S.resetEvents();
    }
  }

  d7s_last_si  = d7s_si_raw;
  d7s_last_pga = d7s_pga_raw;

  if (d7s_eq_app) {
    d7s_si_out  = d7s_si_raw;
    d7s_pga_out = d7s_pga_raw;
    d7s_eq_out  = 1;
  } else {
    d7s_si_out  = 0.0f;
    d7s_pga_out = 0.0f;
    d7s_eq_out  = 0;
  }

  if (!prev_eq_app && d7s_eq_app) {
    Serial.println(F("[D7S] Terremoto rilevato: invio immediato dati."));
    collectAndSendSamples(true);
    d7s_last_send_ms = millis();
    time_of_system_update = 0;
    tempTick = 0;
    Serial.println(F("[D7S] Invio periodico sospeso durante l'evento."));
  }

  if (d7s_eq_app && prev_eq_app) {
    uint32_t nowSendMs = millis();
    if ((nowSendMs - d7s_last_send_ms) >= D7S_EQ_SEND_INTERVAL_MS) {
      Serial.println(F("[D7S] Evento in corso: invio periodico dei dati sismici."));
      collectAndSendSamples(true);
      d7s_last_send_ms = millis();
    }
    if (time_of_system_update != 0) {
      time_of_system_update = 0;
      tempTick = 0;
    }
  }

  if (prev_eq_app && !d7s_eq_app) {
    d7s_last_send_ms = 0;
    Serial.println(F("[D7S] Evento sismico concluso: ripristino invio periodico."));
    time_of_system_update = default_time_of_system_update;
    if (time_of_system_update > 0) {
      tempTick = millis() + time_of_system_update;
      Serial.printf("Prossima lettura tra %lu secondi\n", time_of_system_update / 1000);
    } else {
      tempTick = 0;
      Serial.println(F("[D7S] Periodo predefinito nullo: invio periodico disabilitato."));
    }
  }
}

/* ===================== STATUS BYTE (SEN55) ===================== */
static inline uint8_t sen55StatusByte(uint32_t status) {
  uint8_t b = 0;
  if (status & (1UL << 4))  b |= (1 << 0);
  if (status & (1UL << 21)) b |= (1 << 1);
  if (status & (1UL << 5))  b |= (1 << 2);
  if (status & (1UL << 6))  b |= (1 << 3);
  if (status & (1UL << 7))  b |= (1 << 4);
  if (status & (1UL << 19)) b |= (1 << 5);
  return b;
}
/* Clear Device Status (0xD210) */
static bool sen55ClearDeviceStatusRaw() {
  const uint8_t i2cAddr = 0x69;
  Wire.beginTransmission(i2cAddr);
  Wire.write(0xD2); Wire.write(0x10);
  uint8_t err = Wire.endTransmission();
  delay(20);
  if (err) { Serial.print("clear-status I2C err="); Serial.println(err); return false; }
  Serial.println("Device Status cleared");
  return true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // SD
  delay(800);
  initializeSD();
  delay(200);

  // Webserver (carica config + eventuale cache GNSS)
  startWebserver();
  default_time_of_system_update = time_of_system_update;

  tempTick = 0;

  // I2C
  Wire.begin(21, 22);
  Wire.setClock(100000);
  delay(100);

  // ===== D7S =====
  Serial.println(F("Initializing D7S..."));
  D7S.begin();
  uint32_t t0 = millis();
  while (!D7S.isReady()) {
    if (millis() - t0 > 5000) { Serial.println("D7S not ready (timeout)"); break; }
    delay(200);
  }
  // opzionale: orientamento/assi fissi se necessario
  // D7S.setAxis((d7s_axis_settings)2); // 0=YZ,1=XZ,2=XY — usa se coerente con il montaggio
  D7S.resetEvents(); // pulizia iniziale

  // SCD30
  Serial.println(F("Initializing SCD30..."));
  scd30.begin(Wire, SCD30_I2C_ADDR_61);
  scd30.stopPeriodicMeasurement();
  scd30.softReset();
  delay(2000);
  error = scd30.startPeriodicMeasurement(0);
  if (error != NO_ERROR) { Serial.print("SCD30 start error: "); errorToString(error, errorMessage, sizeof errorMessage); Serial.println(errorMessage); }
  else Serial.println("SCD30: OK");

  // SEN55
  Serial.println(F("Initializing SEN55..."));
  sen5x.begin(Wire);
  uint16_t sErr = sen5x.deviceReset();
  if (sErr) { Serial.print("SEN55 deviceReset err=0x"); Serial.println(sErr, HEX); }
  delay(50);
  sErr = sen5x.startMeasurement();
  if (sErr) { Serial.print("SEN55 startMeasurement err=0x"); Serial.println(sErr, HEX); }
  else Serial.println("SEN55: OK");
  sen55ClearDeviceStatusRaw();

  // BME680
  Serial.println(F("Initializing BME680..."));
  if (!BME680.begin(0x77)) {
    Serial.println("BME680 NOT found!");
    bme680_ok = false;
  } else {
    bme680_ok = true;
    BME680.setTemperatureOversampling(BME680_OS_8X);
    BME680.setHumidityOversampling(BME680_OS_2X);
    BME680.setPressureOversampling(BME680_OS_4X);
    BME680.setIIRFilterSize(BME680_FILTER_SIZE_3);
    BME680.setGasHeater(320, 150);
    Serial.println("BME680: OK");
  }

  // ADC batt
#if defined(ESP32)
  analogReadResolution(12);
  #ifdef analogSetPinAttenuation
  analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);
  #endif
#endif

  // GNSS
  pinMode(BOARD_POWER_ON, OUTPUT); digitalWrite(BOARD_POWER_ON, HIGH);
  pinMode(MODEM_DTR_PIN,  OUTPUT); digitalWrite(MODEM_DTR_PIN, LOW);
  pinMode(MODEM_RST_PIN,  OUTPUT); digitalWrite(MODEM_RST_PIN, HIGH);
  pinMode(MODEM_PWR_PIN,  OUTPUT);
  SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RXD_PIN, MODEM_TXD_PIN);
  digitalWrite(MODEM_PWR_PIN, LOW);  delay(100);
  digitalWrite(MODEM_PWR_PIN, HIGH); delay(1000);
  digitalWrite(MODEM_PWR_PIN, LOW);
  ensureModemAlive();
  gnssReady = gnssPowerOnSequence();
  if (!gnssReady) Serial.println(F("[GNSS] Avvio differito; tenterò nel loop"));
}

void loop() {
  gnssFeed();
  if (captiveRun) server.updateDNS();

  updateD7SState();

  if (time_of_system_update > 0 && millis() >= (unsigned long)tempTick) {
    tempTick = millis() + time_of_system_update;
    Serial.printf("Prossima lettura tra %lu secondi\n", time_of_system_update / 1000);
    collectAndSendSamples(false);
  }

  gnssFeed();
}

