#pragma once
#include <Arduino.h>
#include "FS.h"
#include "SD_MMC.h"
#include <WiFi.h>

/* ======================================================
   SD Card (SD_MMC, forzata 1-bit) - CSV logging
   Allineata a sendDataToGoogle(...) e a D7S (EQ / SI x100 / PGA x100).
   Separatore CSV: '; ' (punto e virgola + spazio)
   ====================================================== */

namespace sd4g {

// ---------------- Stato SD_MMC ----------------
static char FILE_NAME[32] = "";
static bool SD_MMC_OK = false;   // flag globale usato anche dal web server

// Path opzionale per file di config (non usato qui ma esposto)
static constexpr const char* CONFIG_FILE = "/config.txt";

// ---------- Helper FS ----------
inline void writeFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Writing file: %s\r\n", path);
  File file = fs.open(path, FILE_WRITE);
  if (!file) { Serial.println("Failed to open file for writing"); return; }
  if (file.print(message)) Serial.println("File written");
  else                     Serial.println("Write failed");
  file.close();
}

inline void readFile(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\r\n", path);
  File file = fs.open(path, FILE_READ);
  if (!file) { Serial.println("Failed to open file for reading"); return; }
  Serial.print("Read from file: ");
  while (file.available()) Serial.write(file.read());
  file.close();
}

inline void appendFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("\r\nAppending to file: %s\r\n", path);
  File file = fs.open(path, FILE_APPEND);
  if (!file) { Serial.println("Failed to open file for appending"); return; }
  if (file.print(message)) Serial.println("Message appended");
  else                     Serial.println("Append failed");
  file.close();
}

// ---------- Intestazione CSV ----------
inline const char* csvHeader() {
  return
    "Data; Ora; MAC Address; "
    "SCD30 CO2 ppm; "
    "SEN55 PM1.0 ug/m3; SEN55 PM2.5 ug/m3; SEN55 PM4.0 ug/m3; SEN55 PM10 ug/m3; "
    "SEN55 VOC Index; SEN55 NOx Index; "
    "SEN55 Fan Error; SEN55 Speed Warn; SEN55 Laser Error; SEN55 RHT Error; SEN55 Gas Error; SEN55 Cleaning Active; "
    "BME680 Temperatura °C x100; BME680 Umidità % x100; BME680 Pressione Pa x100; BME680 Gas Ohm x100; "
    "D7S EQ Flag; D7S SI m/s x100; D7S PGA m/s^2 x100; "
    "Batteria Tensione V x100; Batteria Percentuale %; "
    "Latitudine; Longitudine\r\n";
}

// ---------- Nome file CSV da MAC ----------
inline void makeCsvFileNameFromMac(char out[32]) {
  uint8_t mac[6]; WiFi.macAddress(mac);
  snprintf(out, 32, "/%02X%02X%02X%02X%02X%02X.csv",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// ---------- Init SD_MMC: SOLO 1-bit, con cleanup pin ----------
inline bool initializeSD() {
  Serial.println("\nInitializing SD_MMC (FORCED 1-bit, safe) ...");

  // Sgancia eventuali mount precedenti
  SD_MMC.end();
  delay(50);

  // Porta le linee dello slot SD in pull-up (D0..D3, CLK, CMD)
  const int sd_pins[] = {2, 4, 12, 13, 14, 15};
  for (int i = 0; i < 6; ++i) pinMode(sd_pins[i], INPUT_PULLUP);
  delay(250);

  // Mount 1-bit
  SD_MMC_OK = SD_MMC.begin("/sdcard", /*mode1bit=*/true, /*format_if_mount_failed=*/false);
  if (!SD_MMC_OK) {
    Serial.println("❌ SD_MMC 1-bit mount FAILED (card/slot non pronto).");
    return false;
  }

  Serial.println("✅ SD_MMC 1-bit mount OK");
  uint8_t type = SD_MMC.cardType();
  uint64_t sizeMB = SD_MMC.cardSize() / (1024ULL * 1024ULL);
  Serial.printf("Card type: %u   size: %llu MB\n", type, sizeMB);

  // Nome file basato su MAC
  makeCsvFileNameFromMac(FILE_NAME);
  Serial.print("Nome file SD: "); Serial.println(FILE_NAME);

  // Crea file se non esiste o se è vuoto -> scrivi intestazione
  bool needHeader = false;
  {
    File file = SD_MMC.open(FILE_NAME, FILE_READ);
    if (!file) {
      Serial.println("File doesn't exist. Will create with header...");
      needHeader = true;
    } else {
      if (file.size() == 0) {
        Serial.println("File is empty. Will write header...");
        needHeader = true;
      }
      file.close();
    }
  }
  if (needHeader) writeFile(SD_MMC, FILE_NAME, csvHeader());
  else            Serial.println("File already exists (header not changed).");

  Serial.println("SD_MMC initialization done.");
  return true;
}

inline bool isMounted() { return SD_MMC_OK; }
inline const char* csvFileName() { return FILE_NAME; }

// ---------- Scrittura riga dati ----------
// Ordine: intestazione, SCD30, SEN55, BME680, D7S (EQ/SI*100/PGA*100), Batteria, GNSS
inline void writeDataOnSD(
  // Intestazione
  const String& currentDate, const String& currentTime, const String& macAddress,
  // SCD30
  const String& co2_ppm,
  // SEN55
  const String& pm1p0, const String& pm2p5, const String& pm4p0, const String& pm10p0,
  const String& vocIndex, const String& noxIndex,
  const String& sen55_fan_err, const String& sen55_speed_warn, const String& sen55_laser_err,
  const String& sen55_rht_err, const String& sen55_gas_err, const String& sen55_cleaning,
  // BME680
  const String& bmeT_x100, const String& bmeRH_x100, const String& bmeP_x100, const String& bmeGas_x100,
  // D7S
  const String& d7s_eq_flag, const String& d7s_SI_ms_x100, const String& d7s_PGA_ms2_x100,
  // Batteria
  const String& bat_v_x100, const String& bat_pct,
  // GNSS
  const String& latitude, const String& longitude
) {
  if (!SD_MMC_OK) {
    Serial.println("⚠️  SD non montata: salto scrittura riga.");
    return;
  }

  String data; data.reserve(512);
  auto add = [&](const String& v, bool last=false) {
    data += v;
    data += last ? "\r\n" : "; ";
  };

  add(currentDate);
  add(currentTime);
  add(macAddress);

  // SCD30
  add(co2_ppm);

  // SEN55
  add(pm1p0); add(pm2p5); add(pm4p0); add(pm10p0);
  add(vocIndex); add(noxIndex);
  add(sen55_fan_err); add(sen55_speed_warn); add(sen55_laser_err);
  add(sen55_rht_err); add(sen55_gas_err);   add(sen55_cleaning);

  // BME680
  add(bmeT_x100); add(bmeRH_x100); add(bmeP_x100); add(bmeGas_x100);

  // D7S
  add(d7s_eq_flag);
  add(d7s_SI_ms_x100);
  add(d7s_PGA_ms2_x100);

  // Batteria
  add(bat_v_x100);
  add(bat_pct);

  // GNSS
  add(latitude);
  add(longitude, /*last=*/true);

  appendFile(SD_MMC, FILE_NAME, data.c_str());
  Serial.print("[SD] "); Serial.print(FILE_NAME); Serial.print(" << ");
  Serial.print(data);
}

} // namespace sd4g

// ===== Alias globali di compatibilità (se vuoi evitare di toccare il .ino) =====
inline bool initializeSD() { return sd4g::initializeSD(); }
inline bool isMounted()    { return sd4g::isMounted(); }
inline const char* csvFileName() { return sd4g::csvFileName(); }
inline void writeDataOnSD(
  const String& currentDate, const String& currentTime, const String& macAddress,
  const String& co2_ppm,
  const String& pm1p0, const String& pm2p5, const String& pm4p0, const String& pm10p0,
  const String& vocIndex, const String& noxIndex,
  const String& sen55_fan_err, const String& sen55_speed_warn, const String& sen55_laser_err,
  const String& sen55_rht_err, const String& sen55_gas_err, const String& sen55_cleaning,
  const String& bmeT_x100, const String& bmeRH_x100, const String& bmeP_x100, const String& bmeGas_x100,
  const String& d7s_eq_flag, const String& d7s_SI_ms_x100, const String& d7s_PGA_ms2_x100,
  const String& bat_v_x100, const String& bat_pct,
  const String& latitude, const String& longitude
) {
  sd4g::writeDataOnSD(
    currentDate, currentTime, macAddress,
    co2_ppm,
    pm1p0, pm2p5, pm4p0, pm10p0,
    vocIndex, noxIndex,
    sen55_fan_err, sen55_speed_warn, sen55_laser_err,
    sen55_rht_err, sen55_gas_err, sen55_cleaning,
    bmeT_x100, bmeRH_x100, bmeP_x100, bmeGas_x100,
    d7s_eq_flag, d7s_SI_ms_x100, d7s_PGA_ms2_x100,
    bat_v_x100, bat_pct,
    latitude, longitude
  );
}
