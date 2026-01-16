# rosrustext_rosrs API

This project uses two complementary documentation layers:

- **Rustdoc**: the authoritative API surface (signatures + usage + semantics at the item level).
- **Markdown specs/parity**: the authoritative behavioral contract and parity matrices.

## What to read first

1. Canonical behavior (transport-agnostic):
   - `docs/spec/lifecycle.md`
   - `docs/spec/parameters.md`
2. Public API usage (Rustdoc, item-level semantics):
   - `rosrustext_rosrs::lifecycle` (feature `ros2`)
   - `rosrustext_rosrs::parameters` (feature `ros2`)
3. Adapter parity matrices (rosrs / `rclrs`):
   - `docs/adapters/ros2rust/lifecycle/parity.md`
   - `docs/adapters/ros2rust/parameters/parity.md`

## Rustdoc links

- docs.rs (replace `<VERSION>`):
  - `https://docs.rs/rosrustext_rosrs/<VERSION>/rosrustext_rosrs/`
  - `https://docs.rs/rosrustext_rosrs/<VERSION>/rosrustext_rosrs/lifecycle/`
  - `https://docs.rs/rosrustext_rosrs/<VERSION>/rosrustext_rosrs/parameters/`
- Local build:
  - `cargo doc -p rosrustext_rosrs --features ros2 --open`

## Specs and parity (GitHub)

- [Lifecycle spec](https://github.com/convyares-FCSL/rosrustext/blob/main/docs/spec/lifecycle.md)
- [Parameters spec](https://github.com/convyares-FCSL/rosrustext/blob/main/docs/spec/parameters.md)
- [Lifecycle parity notes](https://github.com/convyares-FCSL/rosrustext/blob/main/parity.md)
- [rosrs lifecycle parity matrix](https://github.com/convyares-FCSL/rosrustext/blob/main/docs/adapters/ros2rust/lifecycle/parity.md)
- [rosrs parameters parity matrix](https://github.com/convyares-FCSL/rosrustext/blob/main/docs/adapters/ros2rust/parameters/parity.md)

## Known blockers (v0.2.x)

- No user-defined set-time validation callback hook in the rosrs adapter (blocked by `rclrs`):
  - [Parameters spec: Validation and rejection (set-time)](https://github.com/convyares-FCSL/rosrustext/blob/main/docs/spec/parameters.md#validation-and-rejection-set-time)
  - [rosrs parameters parity: Not yet implemented](https://github.com/convyares-FCSL/rosrustext/blob/main/docs/adapters/ros2rust/parameters/parity.md#not-yet-implemented-rosrs-parameters)
- Parameter deletion is not supported (setting `NOT_SET` is rejected):
  - [Parameters spec: Deletion semantics (optional)](https://github.com/convyares-FCSL/rosrustext/blob/main/docs/spec/parameters.md#deletion-semantics-optional)
  - [rosrs parameters parity: Not yet implemented](https://github.com/convyares-FCSL/rosrustext/blob/main/docs/adapters/ros2rust/parameters/parity.md#not-yet-implemented-rosrs-parameters)
- No typed convenience accessors in the adapter API (store exposes `Value`):
  - [rosrs parameters parity: Not yet implemented](https://github.com/convyares-FCSL/rosrustext/blob/main/docs/adapters/ros2rust/parameters/parity.md#not-yet-implemented-rosrs-parameters)
