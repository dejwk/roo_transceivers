#include "roo_transceivers/descriptor.h"

namespace roo_transceivers {

bool operator==(const roo_transceivers::Descriptor& a,
                const roo_transceivers::Descriptor& b) {
  if (a.sensors_size() != b.sensors_size()) return false;
  if (a.actuators_size() != b.actuators_size()) return false;
  for (size_t i = 0; i < a.sensors_size(); ++i) {
    if (a.sensors(i).id().size() != b.sensors(i).id().size()) return false;
    if (memcmp(a.sensors(i).id().data(), b.sensors(i).id().data(),
               a.sensors(i).id().size()) != 0)
      return false;
    if (a.sensors(i).quantity() != b.sensors(i).quantity()) return false;
  }
  for (size_t i = 0; i < a.actuators_size(); ++i) {
    if (a.actuators(i).id().size() != b.actuators(i).id().size()) return false;
    if (memcmp(a.actuators(i).id().data(), b.actuators(i).id().data(),
               a.actuators(i).id().size()) != 0)
      return false;
    if (a.actuators(i).quantity() != b.actuators(i).quantity()) return false;
  }
  return true;
}

}  // namespace roo_transceivers
