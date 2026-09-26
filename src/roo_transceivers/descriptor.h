#pragma once

#include "roo_transceivers.pb.h"

namespace roo_transceivers {

/// Equality comparison for transceiver descriptors.
bool operator==(const roo_transceivers::Descriptor& a,
                const roo_transceivers::Descriptor& b);

}  // namespace roo_transceivers
