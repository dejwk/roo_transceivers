#pragma once

#include <cmath>

#include "roo_time.h"
#include "roo_transceivers.pb.h"

namespace roo_transceivers {

/// Measurement of a quantity at a specific time.
class Measurement {
 public:
  Measurement()
      : quantity_(roo_transceivers_Quantity_kUnspecifiedQuantity),
        time_micros_(0),
        value_(nanf("")) {}

  /// Returns true if this is the initial/empty measurement.
  bool isInitial() const { return (isnanf(value_) && time_micros_ == 0); }

  /// Returns true if quantity and value are defined.
  bool isDefined() const {
    return (quantity_ != roo_transceivers_Quantity_kUnspecifiedQuantity &&
            !isnanf(value_));
  }

  Measurement(roo_transceivers_Quantity quantity, roo_time::Uptime time,
              float value = nanf(""))
      : quantity_(quantity),
        time_micros_(time.inMicros() < (1LL << 52) ? time.inMicros()
                                                   : ((1LL << 52) - 1)),
        value_(value) {}

  /// Returns the quantity type.
  roo_transceivers_Quantity quantity() const { return quantity_; }

  /// Returns the measurement timestamp.
  roo_time::Uptime time() const {
    return roo_time::Uptime::Start() + roo_time::Micros(time_micros_);
  }

  /// Returns the measurement value.
  float value() const { return value_; }

  /// Returns true if value is unknown (NaN).
  bool isUnknown() const { return isnanf(value_); }

 private:
  struct {
    roo_transceivers_Quantity quantity_ : 12;
    uint64_t time_micros_ : 52;
  };
  float value_;
};

}  // namespace roo_transceivers
