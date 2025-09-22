#pragma once
/*
 * WebServer unificato (AsyncFsWebServer + LittleFS + SD_MMC)
 * - Opzioni persistenti (/setup): DEVICE_NAME, LATITUDE, LONGITUDE, TIME_OF_SYSTEM_UPDATE, SHEET_ID
 * - Persistenza GNSS separata su file LittleFS (/gps_last.json)
 * - Endpoint: /setup, /sdexplorer, /download, /gps, /loadOptions
 * - Integrazione Google Apps Script (sendDataToGoogle) — campi coerenti con SD/CSV e D7S
 * - HTML/CSS/JS inline (nessun file esterno)
 */

#include <AsyncFsWebServer.h>   // https://github.com/cotestatnt/async-esp-fs-webserver
#include <FS.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <SD_MMC.h>
#include <ArduinoJson.h>

// ===== Etichette opzioni /setup =====
#define DEVICE_NAME            "Device name"
#define LATITUDE               "Latitude"
#define LONGITUDE              "Longitude"
#define TIME_OF_SYSTEM_UPDATE  "Time of system update [milliseconds]"
#define SHEET_ID               "Google Spreadsheet script ID"

// ===== Variabili condivise con lo sketch principale =====
extern String latitude;
extern String longitude;
extern unsigned int time_of_system_update;

// ===== Config rete / Google =====
static String wifi_ssid_str;             // SSID (MAC)
static const char* wifi_password = "";   // AP open
const char* host = "script.google.com";
const int httpsPort = 443;

WiFiClientSecure clientGoogle;
String sheet_id = "AKfycbyqKwPpPM4TzQcAVEE5LVQFsrokuzYiaxEG-NeYGMCwwE8XVYPRoee9aPMCzs50Yc-lDA";

String deviceName;
bool captiveRun = false;

// =================== Filesystem LittleFS ===================
#define FILESYSTEM LittleFS
AsyncFsWebServer server(80, FILESYSTEM);

inline bool startFilesystem() {
  if (FILESYSTEM.begin()) {
    server.printFileList(FILESYSTEM, "/", 2);
    return true;
  } else {
    Serial.println("ERROR on mounting filesystem. It will be reformatted!");
    FILESYSTEM.format();
    ESP.restart();
  }
  return false;
}

inline void getFsInfo(fsInfo_t* fsInfo) {
  fsInfo->fsName = "LittleFS";
  fsInfo->totalBytes = LittleFS.totalBytes();
  fsInfo->usedBytes  = LittleFS.usedBytes();
}

// ====================== Opzioni (LittleFS) ====================
inline bool loadOptions() {
  if (FILESYSTEM.exists(server.getConfiFileName())) {
    server.getOptionValue(DEVICE_NAME, deviceName);
    server.getOptionValue(LATITUDE, latitude);
    server.getOptionValue(LONGITUDE, longitude);
    server.getOptionValue(TIME_OF_SYSTEM_UPDATE, time_of_system_update);
    server.getOptionValue(SHEET_ID, sheet_id);

    Serial.println("\nValori correnti:");
    Serial.printf("%s: %s\r\n", DEVICE_NAME, deviceName.c_str());
    Serial.printf("%s: %u\r\n", TIME_OF_SYSTEM_UPDATE, time_of_system_update);
    Serial.printf("%s: %s\r\n", LATITUDE, latitude.c_str());
    Serial.printf("%s: %s\r\n", LONGITUDE, longitude.c_str());
    Serial.printf("%s: %s\r\n", SHEET_ID, sheet_id.c_str());
    return true;
  } else {
    Serial.println(F("Config file not exist"));
  }
  return false;
}

inline void saveOptions() {
  Serial.println(F("Application options saved."));
  // server.saveOptionValues(); // abilita se necessario
}

// ===== Persistenza GNSS separata (indipendente dalla UI /setup) =====
static const char* GPS_CACHE_PATH = "/gps_last.json";

// ArduinoJson v7: usare JsonDocument senza capacity nel costruttore
inline bool saveGpsToFS(const String& lat, const String& lon) {
  if (!LittleFS.begin()) return false;
  File f = LittleFS.open(GPS_CACHE_PATH, FILE_WRITE);
  if (!f) return false;
  JsonDocument doc;
  doc["lat"] = lat;
  doc["lon"] = lon;
  bool ok = (serializeJson(doc, f) > 0);
  f.close();
  return ok;
}

inline bool loadGpsFromFS(String& lat, String& lon) {
  if (!LittleFS.begin()) return false;
  if (!LittleFS.exists(GPS_CACHE_PATH)) return false;
  File f = LittleFS.open(GPS_CACHE_PATH, FILE_READ);
  if (!f) return false;
  JsonDocument doc;
  DeserializationError e = deserializeJson(doc, f);
  f.close();
  if (e) return false;
  lat = doc["lat"].as<String>();
  lon = doc["lon"].as<String>();
  return (lat.length() && lon.length());
}

// ===== HTML/JS inline (semplici) =====
static const char* SAVE_BTN_HTML = R"HTML(
<div style="margin:16px 0">
  <button id="saveBtn" style="padding:10px 16px;border-radius:8px;border:0;background:#1b73e8;color:#fff;cursor:pointer">
    Salva configurazione
  </button>
  <span id="saveStatus" style="margin-left:10px;color:#0a0"></span>
</div>
)HTML";

static const char* SIMPLE_JS = R"JS(
document.addEventListener('DOMContentLoaded',()=>{
  const btn = document.getElementById('saveBtn');
  const status = document.getElementById('saveStatus');
  if(btn){
    btn.addEventListener('click', async ()=>{
      status.textContent = 'Salvataggio...';
      try{
        const r = await fetch('/loadOptions');
        if(r.ok){
          status.textContent = 'Salvato!';
          setTimeout(()=>status.textContent='',1500);
        }else{
          status.textContent = 'Errore nel salvataggio';
          status.style.color = '#a00';
        }
      }catch(e){
        status.textContent = 'Errore di rete';
        status.style.color = '#a00';
      }
    });
  }
});
)JS";

// ======================= HTTP Handlers =========================
inline void handleLoadOptions(AsyncWebServerRequest* request) {
  loadOptions();
  request->send(200, "text/plain", "Options loaded");
  Serial.println("Application option loaded after web request");
}

inline void handleSDDownload(AsyncWebServerRequest *request) {
  if (SD_MMC.cardType() == CARD_NONE) {
    request->send(503, "text/plain", "SD non montata o non disponibile");
    return;
  }
  if (!request->hasParam("file")) {
    request->send(400, "text/plain", "Parametro 'file' mancante");
    return;
  }
  String filePath = request->getParam("file")->value();
  if (!filePath.startsWith("/")) filePath = "/" + filePath;

  File file = SD_MMC.open(filePath.c_str(), FILE_READ);
  if (!file || file.isDirectory()) {
    request->send(404, "text/plain", "File non trovato o è una cartella");
    return;
  }
  file.close();
  request->send(SD_MMC, filePath, "application/octet-stream", /*download=*/true);
}

inline String listSDDir(const char * dirname = "/", uint8_t levels = 1) {
  if (SD_MMC.cardType() == CARD_NONE) {
    return "<p>SD non montata o non disponibile</p>";
  }

  String html = "<ul>";
  File root = SD_MMC.open(dirname);
  if (!root || !root.isDirectory()) {
    return "<p>Directory non valida</p>";
  }

  File file = root.openNextFile();
  while (file) {
    if (!file.isDirectory()) {
      String name = file.name();
      if (!name.startsWith("/")) name = "/" + name;
      html += "<li><a href=\"/download?file=" + name + "\">" + name + "</a></li>";
    }
    file = root.openNextFile();
  }
  html += "</ul>";
  return html;
}

// ============================ Avvio WebServer ============================
inline void startWebserver() {
  // LittleFS
  if (startFilesystem()) {
    if (loadOptions()) Serial.println(F("Application option loaded"));
    else               Serial.println(F("Application options NOT loaded!"));
  }

  // Cache GNSS
  {
    String latFS, lonFS;
    if (loadGpsFromFS(latFS, lonFS)) {
      if (!latitude.length())  latitude  = latFS;
      if (!longitude.length()) longitude = lonFS;
      Serial.printf("[GPS cache] Caricate da FS: lat=%s lon=%s\n",
                    latitude.c_str(), longitude.c_str());
    }
  }

  // SSID AP = MAC
  wifi_ssid_str = WiFi.macAddress();

  // WiFi o AP + Captive
  IPAddress myIP = server.startWiFi(15000);
  if (!myIP) {
    Serial.println("\n\nNo WiFi connection, start AP and Captive Portal\n");
    server.startCaptivePortal(wifi_ssid_str.c_str(), /*pwd*/"", "/setup");
    myIP = WiFi.softAPIP();
    captiveRun = true;
  }

  // === /setup page ===
  server.addOptionBox("Device");
  server.addOption(DEVICE_NAME, deviceName);
  server.addOption(LATITUDE, latitude);
  server.addOption(LONGITUDE, longitude);
  server.addOption(TIME_OF_SYSTEM_UPDATE, time_of_system_update);

  server.addOptionBox("Google Sheet");
  server.addOption(SHEET_ID, sheet_id.c_str());

  server.addHTML(SAVE_BTN_HTML, "buttons", false);
  server.addJavascript(SIMPLE_JS, "js", false);

  // === /sdexplorer page ===
  server.on("/sdexplorer", HTTP_GET, [](AsyncWebServerRequest *request) {
    String html = "<html><head><title>SD Explorer</title></head><body>";
    html += "<h2>File sulla SD Card</h2>";
    html += listSDDir("/");
    html += "</body></html>";
    request->send(200, "text/html", html);
  });

  // === /download ===
  server.on("/download", HTTP_GET, handleSDDownload);

  // === /loadOptions ===
  server.on("/loadOptions", HTTP_GET, handleLoadOptions);

  // === /gps ===
  server.on("/gps", HTTP_GET, [](AsyncWebServerRequest* request){
    String latFS, lonFS;
    bool ok = loadGpsFromFS(latFS, lonFS);
    String out = "{\"ok\":";
    out += ok ? "true" : "false";
    out += ",\"lat\":\"" + (ok ? latFS : "") + "\",\"lon\":\"" + (ok ? lonFS : "") + "\"}";
    request->send(200, "application/json", out);
  });

  // Editor web LittleFS + info filesystem
  server.enableFsCodeEditor();
  server.setFsInfoCallback(getFsInfo);

  // Start
  server.init();
  Serial.print(F("ESP Web Server started on IP Address: "));
  Serial.println(myIP);
  Serial.println(F(
    "Open /setup page to configure optional parameters.\n"
    "Open /edit page to view, edit or upload example or your custom webserver source files.\n"
    "Open /sdexplorer to browse and download files from SD card.\n"
    "Open /gps to view cached GNSS coordinates (if any)."));
}

// ======================= Google Sheets (HTTPS) ===========================
// x-www-form-urlencoded (spazio -> '+', altri byte -> %HH)
inline String urlEncode(const String& s) {
  String out; out.reserve(s.length() * 3);
  const char *hex = "0123456789ABCDEF";
  for (size_t i = 0; i < s.length(); ++i) {
    uint8_t c = (uint8_t)s[i];
    if ( (c >= 'A' && c <= 'Z') ||
         (c >= 'a' && c <= 'z') ||
         (c >= '0' && c <= '9') ||
         c == '-' || c == '_' || c == '.' || c == '~' ) {
      out += (char)c;
    } else if (c == ' ') {
      out += '+';
    } else {
      out += '%';
      out += hex[c >> 4];
      out += hex[c & 0x0F];
    }
  }
  return out;
}

inline void qsAdd(String &qs, const char* key, const String& val) {
  if (!qs.isEmpty()) qs += '&';
  qs += key; qs += '=';
  qs += urlEncode(val);
}

// FIRMWARE 2.8 — ordine campi coerente con SD/CSV e D7S (eq_flag, SI*100, PGA*100)
inline void sendDataToGoogle(
  // intestazione
  const String& currentDate,
  const String& currentTime,
  const String& macAddress,
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
  const String& latitude_in, const String& longitude_in
) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[sendDataToGoogle] WiFi non connesso: abort.");
    return;
  }

  String qs; qs.reserve(1024);

  // intestazione
  qsAdd(qs, "currentDate",  currentDate);
  qsAdd(qs, "currentTime",  currentTime);
  qsAdd(qs, "macAddress",   macAddress);

  // SCD30
  qsAdd(qs, "co2_ppm",      co2_ppm);

  // SEN55
  qsAdd(qs, "pm1p0",            pm1p0);
  qsAdd(qs, "pm2p5",            pm2p5);
  qsAdd(qs, "pm4p0",            pm4p0);
  qsAdd(qs, "pm10p0",           pm10p0);
  qsAdd(qs, "vocIndex",         vocIndex);
  qsAdd(qs, "noxIndex",         noxIndex);
  qsAdd(qs, "sen55_fan_err",    sen55_fan_err);
  qsAdd(qs, "sen55_speed_warn", sen55_speed_warn);
  qsAdd(qs, "sen55_laser_err",  sen55_laser_err);
  qsAdd(qs, "sen55_rht_err",    sen55_rht_err);
  qsAdd(qs, "sen55_gas_err",    sen55_gas_err);
  qsAdd(qs, "sen55_cleaning",   sen55_cleaning);

  // BME680
  qsAdd(qs, "bmeT_x100",    bmeT_x100);
  qsAdd(qs, "bmeRH_x100",   bmeRH_x100);
  qsAdd(qs, "bmeP_x100",    bmeP_x100);
  qsAdd(qs, "bmeGas_x100",  bmeGas_x100);

  // D7S
  qsAdd(qs, "d7s_eq_flag",      d7s_eq_flag);
  qsAdd(qs, "d7s_SI_ms_x100",   d7s_SI_ms_x100);
  qsAdd(qs, "d7s_PGA_ms2_x100", d7s_PGA_ms2_x100);

  // Batteria
  qsAdd(qs, "bat_v_x100",   bat_v_x100);
  qsAdd(qs, "bat_pct",      bat_pct);

  // GNSS
  qsAdd(qs, "latitude",     latitude_in);
  qsAdd(qs, "longitude",    longitude_in);

  const String base = "https://script.google.com/macros/s/";
  const String url  = base + sheet_id + "/exec";
  Serial.printf("[sendDataToGoogle] QS length: %u bytes\n", (unsigned)qs.length());

  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(12000);

  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setUserAgent("4Spark-4G/2.8 (ESP32)");
  http.setTimeout(15000);

  const size_t GET_LIMIT = 1800; // margine di sicurezza
  int httpCode = -1;

  if (qs.length() <= GET_LIMIT) {
    String full = url + "?" + qs;
    Serial.println("[sendDataToGoogle] GET:");
    Serial.println(full);
    if (!http.begin(client, full)) {
      Serial.println("[sendDataToGoogle] Errore http.begin() (GET)");
      return;
    }
    httpCode = http.GET();
  } else {
    Serial.println("[sendDataToGoogle] POST (QS troppo lunga per GET)");
    Serial.println(url);
    if (!http.begin(client, url)) {
      Serial.println("[sendDataToGoogle] Errore http.begin() (POST)");
      return;
    }
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    httpCode = http.POST(qs);
  }

  if (httpCode > 0) {
    Serial.printf("[sendDataToGoogle] HTTP %d\n", httpCode);
    String response = http.getString();
    Serial.println("[sendDataToGoogle] Risposta:");
    Serial.println(response);
  } else {
    Serial.printf("[sendDataToGoogle] Errore HTTP: %s\n", http.errorToString(httpCode).c_str());
  }
  http.end();
}
