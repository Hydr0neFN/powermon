#include "csi.h"

// Empty on purpose: see csi.h. Planned: esp_wifi_set_csi, features published to
// presence/<node>/csi.
namespace csi {
void setup(PublishFn, const char*) {}
void loop() {}
void pause(bool) {}
void subscribe(void (*)(const char*)) {}
bool onMessage(const char*, const uint8_t*, unsigned int) { return false; }
}  // namespace csi
