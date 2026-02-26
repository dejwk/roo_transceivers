#pragma once

#include "roo_transceivers/id.h"
#include "roo_transceivers/measurement.h"

namespace roo_transceivers {

/// Listener for universe-level change notifications.
class EventListener {
 public:
  virtual ~EventListener() = default;

  /// Called when the set of devices changes.
  virtual void devicesChanged() {}
  /// Called when new readings are available.
  virtual void newReadingsAvailable() {}
};

}  // namespace roo_transceivers