#pragma once

#include <Update.h>
#include <AsyncFsWebServer.h> 
extern AsyncFsWebServer server;

//////////////////////////////////////
// Function for managing upload OTA
//////////////////////////////////////
void handleOTAupload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  if (!index) {
    Serial.printf("Update Start: %s\n", filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { // start OTA
      Update.printError(Serial);
    }
  }

  if (Update.write(data, len) != len) {
    Update.printError(Serial);
  }

  if (final) {
    if (Update.end(true)) { // true in order to force the closing
      Serial.printf("Update Success: %u bytes\n", index + len);
      Serial.println("Rebooting...");
      delay(1000);
      ESP.restart();
    } else {
      Update.printError(Serial);
    }
  }
}

//////////////////////////////////////
// Setup OTA of the /setup page
//////////////////////////////////////
void setupOTAwebBlock() {
  // Blocco HTML per OTA
  static const char ota_block[] PROGMEM = R"rawliteral(
    <div class='card'>
      <h3>Firmware Update</h3>
      <form method='POST' action='/update' enctype='multipart/form-data'>
        <input type='file' name='update'>
        <input type='submit' value='Upload Firmware'>
      </form>
    </div>
  )rawliteral";

  // add the OTA form on the /setup page
  server.addHTML(ota_block, "ota", /*overwrite=*/false);

  // Managing for /update POST
  server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request){
    bool success = !Update.hasError();
    request->send(200, "text/plain", success ? "Update Success! Rebooting..." : "Update Failed!");
  }, handleOTAupload);
}
