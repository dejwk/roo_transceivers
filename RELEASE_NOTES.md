# roo_transceivers 1.2.0

- Updated Roo dependencies in Bazel and PlatformIO: `roo_collections` 1.4.7, `roo_logging` 1.5.10, `roo_prefs` 1.3.2, `roo_threads` 1.2.8, and `roo_time` 2.0.0.
- Updated Bazel dependencies to `rules_cc` 0.2.25, `googletest` 1.18.0.bcr.1, and `nanopb` 0.4.9.1.bcr.3.
- Updated test tooling and shared CI workflow to `roo_testing` 2.1.2.
- Added consolidated release notes for previous versions.

---

# [roo_transceivers 1.1.7](https://github.com/dejwk/roo_transceivers/releases/tag/1.1.7)

Published 2026-08-30.

This release adds a comprehensive programming guide, improves protocol-message initialization safety, and updates the Roo dependency set.

### Highlights

- Added a [[programming guide](https://github.com/dejwk/roo_transceivers/blob/1.1.7/doc/programming_guide.md)](https://github.com/dejwk/roo_transceivers/blob/1.1.7/doc/programming_guide.md) covering universes, locators, descriptors, measurements, remote access, and persistent bindings.
- Fixed initialization of generated nanopb protocol messages to ensure all fields are reliably zero-initialized.
- Moved `roo_logging::Stream` operators for device, sensor, and actuator locators into the `roo_transceivers` namespace for proper lookup.
- Updated minimum Roo library dependencies:
  - `roo_collections` 1.4.6
  - `roo_logging` 1.5.8
  - `roo_prefs` 1.3.1
  - `roo_threads` 1.2.7
  - `roo_time` 1.4.7
- Updated host-test tooling to `roo_testing` 2.1.0, including ESP32 host-emulation and CI improvements.
- Made remote-server test assertions insensitive to nondeterministic message ordering.

**Full Changelog:** https://github.com/dejwk/roo_transceivers/compare/1.1.6...1.1.7

---

# [roo_transceivers 1.1.6](https://github.com/dejwk/roo_transceivers/releases/tag/1.1.6)

Published 2026-02-26.

Fixed the CI, by patching nanopb bazel configuration.

---

# [roo_transceivers 1.1.5](https://github.com/dejwk/roo_transceivers/releases/tag/1.1.5)

Published 2026-02-26.

* Doxygen documentation.
* Updated dependencies. Builds without warnings now.
* Fixed test build.

**Full Changelog**: https://github.com/dejwk/roo_transceivers/compare/1.1.4...1.1.5

---

# [roo_transceivers 1.1.4](https://github.com/dejwk/roo_transceivers/releases/tag/1.1.4)

Published 2026-01-06.

Updated dependencies and fixed unit tests.

**Full Changelog**: https://github.com/dejwk/roo_transceivers/compare/1.1.3...1.1.4

---

# [roo_transceivers 1.1.3](https://github.com/dejwk/roo_transceivers/releases/tag/1.1.3)

Published 2025-11-12.

Minor tweaks.

**Full Changelog**: https://github.com/dejwk/roo_transceivers/compare/1.1.2...1.1.3

---

# [roo_transceivers 1.1.2](https://github.com/dejwk/roo_transceivers/releases/tag/1.1.2)

Published 2025-10-31.

* Added fakes
* Updated dependencies
* Added CI, .gitignore.

**Full Changelog**: https://github.com/dejwk/roo_transceivers/compare/1.1.1...1.1.2

---

# [roo_transceivers 1.1.1](https://github.com/dejwk/roo_transceivers/releases/tag/1.1.1)

Published 2025-10-19.

Fixing a minor compilation issue.

---

# [roo_transceivers 1.1.0](https://github.com/dejwk/roo_transceivers/releases/tag/1.1.0)

Published 2025-10-19.

Bazel module for testing.

---

# [roo_transceivers 1.0.2](https://github.com/dejwk/roo_transceivers/releases/tag/1.0.2)

Published 2025-03-24.

Fixed dependency spec on nanodb for PlatformIO Registry.

---

# [roo_transceivers 1.0.0](https://github.com/dejwk/roo_transceivers/releases/tag/1.0.0)

Published 2025-03-23.

Initial release.

---

