#pragma once

#include "roo_transceivers.pb.h"

/// Equality comparison for transceiver descriptors.
bool operator==(const roo_transceivers_Descriptor& a,
                const roo_transceivers_Descriptor& b);
