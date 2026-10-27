#pragma once
#include <Arduino.h>

// Hook point for the Wi-Fi CSI presence module (to be added later over OTA by
// another agent). main_c3.cpp already calls all of these; replace the empty
// bodies in csi.cpp, do not touch the call sites.
//   setup()      once, after WiFi is up, OTA is initialised and BLE has started
//   loop()       every main-loop pass (keep it short)
//   pause(true)  an OTA upload starts / pause(false) it failed: stop heavy work
//   subscribe()  MQTT (re)connect: call sub(topic) for each topic needed
//   onMessage()  MQTT message; return true when the topic was yours
//   publish      MQTT publish helper installed by main (nullptr until connected)
namespace csi {
typedef bool (*PublishFn)(const char* topic, const char* payload, bool retain);
void setup(PublishFn publish, const char* nodeName);
void loop();
void pause(bool paused);
void subscribe(void (*sub)(const char* topic));
bool onMessage(const char* topic, const uint8_t* payload, unsigned int length);
}  // namespace csi
