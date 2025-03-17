#pragma once

#include "roo_io/text/string_printf.h"
#include "roo_onewire.h"
#include "roo_quantity/temperature.h"
#include "roo_transceivers/id.h"
#include "roo_transceivers/universe.h"

namespace roo_transceivers {

namespace {

static const DeviceSchema kOneWireSchema = DeviceSchema("1-Wire");

}

class OneWireUniverse : public SimpleSensorUniverse {
 public:
  OneWireUniverse(roo_onewire::OneWire& onewire)
      : onewire_(onewire), listener_(*this) {
    onewire_.thermometers().addEventListener(&listener_);
  }

  int deviceCount() const override { return onewire_.thermometers().count(); }

  void forEachDevice(std::function<bool(const DeviceLocator&)> callback) const {
    char code[17];
    for (const auto& t : onewire_.thermometers()) {
      t.rom_code().toCharArray(code);
      code[16] = 0;
      DeviceLocator locator(kOneWireSchema, code);
      if (!callback(locator)) return;
    }
  }

  Measurement readSensor(const DeviceLocator& locator) const override {
    if (locator.schema() != kOneWireSchema) {
      return Measurement();
    }
    roo_onewire::RomCode rom_code =
        roo_onewire::RomCode::FromString(locator.device_id().c_str());
    const roo_onewire::Thermometer* t =
        onewire_.thermometers().thermometerByRomCode(rom_code);
    return Measurement(roo_transceivers_Quantity_kTemperature,
                       roo_time::Uptime::Now(),
                       t == nullptr ? nanf("") : t->temperature().degCelcius());
  }

  roo_transceivers_Quantity getSensorQuantity(
      DeviceLocator device_locator) const override {
    return roo_transceivers_Quantity_kTemperature;
  }

  void requestUpdate() override { onewire_.update(); }

  void addEventListener(EventListener* listener) override {
    auto result = event_listeners_.insert(listener);
    CHECK(result.second) << "Event listener " << listener
                         << " was registered already.";
  }

  void removeEventListener(EventListener* listener) override {
    event_listeners_.erase(listener);
  }

 private:
  void notifySensorsChanged() const {
    for (auto& listener : event_listeners_) {
      listener->devicesChanged();
    }
  }

  void notifyNewReadingsAvailable() const {
    for (auto& listener : event_listeners_) {
      listener->newReadingsAvailable();
    }
  }

  class Listener : public roo_onewire::Thermometers::EventListener {
   public:
    Listener(OneWireUniverse& onewire) : onewire_(onewire) {}

    void discoveryCompleted() const override {
      onewire_.notifySensorsChanged();
    }

    void conversionCompleted() const override {
      onewire_.notifyNewReadingsAvailable();
    }

   private:
    OneWireUniverse& onewire_;
  };

  roo_collections::FlatSmallHashSet<EventListener*> event_listeners_;
  roo_onewire::OneWire& onewire_;
  Listener listener_;
};

}  // namespace roo_transceivers