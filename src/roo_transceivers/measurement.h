#pragma once

#include <cmath>

#include "roo_time.h"
#include "roo_transceivers.pb.h"

namespace roo_transceivers {

class Measurement {
 public:
  Measurement()
      : quantity_(roo_transceivers_Quantity_kUnspecifiedQuantity),
        time_micros_(0),
        value_(nanf("")) {}

  bool isInitial() const {
    return (isnanf(value_) && time_micros_ == 0);
  }

  bool isDefined() const {
    return (quantity_ != roo_transceivers_Quantity_kUnspecifiedQuantity &&
            !isnanf(value_));
  }

  Measurement(roo_transceivers_Quantity quantity, roo_time::Uptime time,
              float value = nanf(""))
      : quantity_(quantity),
        time_micros_(time.inMicros() < (1LL << 52) ? time.inMicros()
                                                   : (1LL << 52 - 1)),
        value_(value) {}

  roo_transceivers_Quantity quantity() const { return quantity_; }

  roo_time::Uptime time() const {
    return roo_time::Uptime::Start() + roo_time::Micros(time_micros_);
  }

  float value() const { return value_; }

  bool isUnknown() const { return isnanf(value_); }

 private:
  struct {
    roo_transceivers_Quantity quantity_ : 12;
    uint64_t time_micros_ : 52;
  };
  float value_;
};

}  // namespace roo_transceivers
