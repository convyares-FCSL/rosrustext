# rosrustext Implementation Plan

## Purpose
rosrustext is a Rust library that extends ROS 2 capability/feature parity across transports without becoming a new client library. It must remain:
- Focused: parity extensions, not a rewrite of ROS.
- Clean: normal development is pure Cargo.
- Testable: integration tests happen in a separate ROS/colcon dev workspace.

## Core Principle: Separate Concerns
We maintain two worlds:

### A) Library Repo (clean Cargo)
- Pure Rust crates under `crates/`
- No ROS build artifacts tracked
- No generated message crates tracked
- Builds with `cargo build` without sourcing ROS

### B) Dev Workspace (ROS/colcon + message generation)
- A separate colcon workspace used to:
  - build `ros2_rust` tooling (`rosidl_generator_rs`, etc.)
  - generate Rust crates for message packages (std_msgs + custom msgs)
  - build and run example binaries against the library
- All ROS build outputs stay here (install/build/log/.cargo)

This is the “no drift / no bloat” firewall.

---

## Target Repository Structure (Library)
```

rosrustext/
crates/
rosrustext_core/
rosrustext_roslibrust/
rosrustext_ros2_rust/          (optional feature-gated rclrs transport)
rosrustext_lifecycle_proxy/    (optional bin/tool crate)
docs/
implementation_plan.md
methodology.md
tools/
mk_vscode_workspace.sh
.gitignore
Cargo.toml (workspace)

```

### .gitignore (library)
Library repo must ignore:
- `**/target/`
- `**/.cargo/`
- `build/ install/ log/` (if they ever appear by accident)
- any generated ROS artifacts

---

## Dev Workspace Structure (separate folder; not committed to library)
Recommended location: sibling folder to the library.

```

rosrustext_dev_ws/
src/
ros2_rust/              (ros2-rust/ros2_rust checkout) <imported repos>/       (from ros2_rust repos file)
my_test_msgs/           (custom interface package for smoke testing)
rosrustext/             (optional git clone OR path dependency to library)
examples/
scratch_stdmsgs/        (Cargo bin; depends on std_msgs via patches)
scratch_my_msgs/        (Cargo bin; depends on my_test_msgs via patches)
build/ install/ log/ .cargo/ (generated)

```

---

## Phases

### Phase 0 — Lock the rules (documentation + repo hygiene)
Deliverables:
- `docs/implementation_plan.md`
- `docs/methodology.md`
- `.gitignore` updated (library)
- VS Code multi-root workspace generator script

Exit criteria:
- The library repo stays clean (no ROS artifacts, no install/build/log)

---

### Phase 1 — Establish dev workspace toolchain (ros2_rust works on Jazzy/WSL)
Deliverables:
- `ros2_rust` builds in dev ws via `colcon` using a Python environment that contains required deps (notably: numpy headers + lark)
- `install/std_msgs/.../rust/Cargo.toml` exists
- `.cargo/config.toml` contains `[patch.crates-io.std_msgs] -> install/.../rust`

Exit criteria:
- A scratch Cargo binary can `cargo build` with `std_msgs = "*"` (resolved via `.cargo/config.toml` patches)

Notes:
- The `.cargo/config.toml` patch mechanism is *workspace-local glue* and must not be treated as a library requirement.

---

### Phase 2 — Custom message generation smoke test (my_test_msgs)
Deliverables:
- `my_test_msgs` interface package with at least one `.msg`
- `colcon build` generates Rust crate:
  - `install/my_test_msgs/share/my_test_msgs/rust/Cargo.toml`

Exit criteria:
- A scratch Cargo binary can `cargo build` with `my_test_msgs = "*"` (resolved via patches)
- No manual copying of generated crates into the library repo

Known pitfall:
- If the active Python interpreter lacks `numpy` headers or `lark`, rosidl steps fail.
  - Solution: ensure the python used by colcon has those modules available.

---

### Phase 3 — Library integration examples (thin, focused)
Deliverables:
- Example binaries in dev ws that link:
  - rosrustext library crates (path dependency)
  - generated message crates (via patches or explicit path deps)
- One example per transport:
  - roslibrust-based example (current)
  - optional rclrs-based example (feature-gated)

Exit criteria:
- Examples compile and run without modifying the library repo with build artifacts
- A “golden path” README exists describing the end-user workflow

---

### Phase 4 — Stabilize user experience
Goal: make the end user do minimal weird setup.

Deliverables:
- A documented “recommended workflow”:
  - ROS-first (colcon overlay) OR Cargo-first (path deps)
- A minimal template repo / cookiecutter-ish guidance (optional)

Exit criteria:
- A new user can:
  1) generate messages
  2) depend on rosrustext
  3) compile & run
  without tribal knowledge

---

## Policy: What does NOT go into rosrustext
- Generated message crates (std_msgs, custom msgs, etc.)
- ros2_rust source checkout
- colcon build products (install/build/log/.cargo)
- Python venvs and their site-packages
- Anything that forces end-users to adopt our dev workspace layout

---

## Definition of “Done” (library-level)
- rosrustext builds with pure cargo
- parity features implemented as small, composable modules
- integration proven in dev workspace with:
  - std_msgs usage
  - custom msg usage
- documented workflows prevent drift