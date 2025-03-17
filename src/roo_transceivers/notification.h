#pragma once

#include "roo_transceivers/id.h"
#include "roo_transceivers/measurement.h"

namespace roo_transceivers {

class EventListener {
 public:
  virtual ~EventListener() = default;

  virtual void devicesChanged() {}
  virtual void newReadingsAvailable() {}
};

}  // namespace roo_transceivers