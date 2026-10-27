#include "csi.h"

#include "csi_presence.h"

// Adapter between the hook points in main_c3.cpp (csi.h) and the shared CSI module
// (room-presence/csi/csi_presence.*, same file as on lightbar2mqtt / LightController).
namespace csi {
void setup(PublishFn publish, const char* nodeName) { csipresence::setup(nodeName, publish); }
void loop() { csipresence::loop(); }
void pause(bool paused) { csipresence::setPaused(paused); }
void subscribe(void (*sub)(const char*)) { csipresence::subscribe(sub); }
bool onMessage(const char* topic, const uint8_t* payload, unsigned int length) {
  return csipresence::onMessage(topic, payload, length);
}
}  // namespace csi
