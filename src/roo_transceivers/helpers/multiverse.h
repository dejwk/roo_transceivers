#pragma once

#include "roo_transceivers/universe.h"

namespace roo_transceivers {

// Multiverse is a transceiver universe that combines multiple other universes
// into one.
class Multiverse : public Universe, public EventListener {
 public:
  Multiverse(std::vector<Universe*> universes)
      : universes_(std::move(universes)) {
    for (auto universe : universes_) {
      universe->addEventListener(this);
    }
  }

  size_t deviceCount() const override {
    size_t count = 0;
    for (const auto& universe : universes_) {
      count += universe->deviceCount();
    }
    return count;
  }

  bool forEachDevice(
      std::function<bool(const DeviceLocator&)> callback) const override {
    for (const auto& universe : universes_) {
      if (!universe->forEachDevice(callback)) return false;
    }
    return true;
  }

  bool getDeviceDescriptor(
      const DeviceLocator& locator,
      roo_transceivers_Descriptor& descriptor) const override {
    for (const auto& universe : universes_) {
      if (universe->getDeviceDescriptor(locator, descriptor)) return true;
    }
    return false;
  }

  Measurement read(const SensorLocator& locator) const override {
    for (const auto& universe : universes_) {
      Measurement m = universe->read(locator);
      if (m.quantity() != roo_transceivers_Quantity_kUnspecifiedQuantity) {
        return m;
      }
    }
    return Measurement();
  }

  bool write(const ActuatorLocator& locator, float value) override {
    for (const auto& universe : universes_) {
      if (universe->write(locator, value)) return true;
    }
    return false;
  }

  void requestUpdate() override {
    for (const auto& universe : universes_) {
      universe->requestUpdate();
    }
  }

  virtual void addEventListener(EventListener* listener) {
    listeners_.insert(listener);
  }

  virtual void removeEventListener(EventListener* listener) {
    listeners_.erase(listener);
  }

  void devicesChanged() override {
    for (auto listener : listeners_) {
      listener->devicesChanged();
    }
  }

  void newReadingsAvailable() override {
    for (auto listener : listeners_) {
      listener->newReadingsAvailable();
    }
  }

 private:
  std::vector<Universe*> universes_;
  roo_collections::FlatSmallHashSet<EventListener*> listeners_;
};

}