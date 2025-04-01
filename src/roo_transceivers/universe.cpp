#include "roo_transceivers/universe.h"

namespace roo_transceivers {}  // namespace roo_transceivers

bool operator==(const roo_transceivers_Descriptor& a,
                const roo_transceivers_Descriptor& b) {
  if (a.sensors_count != b.sensors_count) return false;
  if (a.actuators_count != b.actuators_count) return false;
  for (size_t i = 0; i < a.sensors_count; ++i) {
    if (strcmp(a.sensors[i].id, b.sensors[i].id) != 0) return false;
    if (a.sensors[i].quantity != b.sensors[i].quantity) return false;
  }
  for (size_t i = 0; i < a.actuators_count; ++i) {
    if (strcmp(a.actuators[i].id, b.actuators[i].id) != 0) return false;
    if (a.actuators[i].quantity != b.actuators[i].quantity) return false;
  }
  return true;
}
