---
description: How to add new ROS 2 messages and update the Rust bundle
---

# Adding and Updating ROS 2 Messages

Since `rosrustext_msgs` is a standalone crate published to crates.io, it does not automatically rebuild when you change `.msg` files. You must manually regenerate and update the bundled code.

## 1. Add Message Definitions
Add your `.msg` or `.srv` files to the `interfaces/` directory.

- **Location**: `interfaces/rosrustext_interfaces/msg/` (or `srv/`)
- **Example**: `MyNewMessage.msg`

## 2. Generate Rust Code
Use `colcon` to generate the Rust bindings.

```bash
# Ensure you are in the workspace root
colcon build --packages-select rosrustext_interfaces
```

## 3. Copy Generated Code
Copy the generated Rust sources into the `rosrustext_msgs` crate.

```bash
# Example for `rosrustext_interfaces`
cp -r install/rosrustext_interfaces/share/rosrustext_interfaces/rust/src/* crates/rosrustext_msgs/src/rosrustext_interfaces/
```

> [!NOTE]
> You may need to recursively find other generated files if you added new packages.

## 4. Refactor Names (If Required)
The generated code might try to reference `rosrustext_interfaces::msg::...`. You usually need to update these references to use `crate::...` or `rosrustext_msgs::...`.
- Search for `rosrustext_interfaces::` in the copied files.
- Replace with `crate::rosrustext_interfaces::`.

## 5. Expose in `lib.rs`
If you added a new *package* (not just a message in an existing package), update `crates/rosrustext_msgs/src/lib.rs` to export it.

```rust
// crates/rosrustext_msgs/src/lib.rs
pub mod my_new_package;
```

## 6. Publish Changes
1. Bump version in `crates/rosrustext_msgs/Cargo.toml`.
2. `cargo publish -p rosrustext_msgs`.
