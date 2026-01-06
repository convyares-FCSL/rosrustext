# rosrustext Methodology

## Why this exists
We drifted when we mixed:
- message generation toolchain concerns (rosidl + colcon + python deps)
with
- library design concerns (clean Rust crates, focused parity features)

This methodology prevents repeating that.

---

## Working Style: “One Thing At A Time”
We proceed in small, reviewable increments:
- implement one small change
- build/test
- review and tweak
- only then move on

Redesign/refactor is allowed only at clear checkpoints when it reduces complexity.

---

## Non-Negotiables

### 1) Separation of concerns
- Library repo = pure Cargo.
- Dev workspace = ROS/colcon + message generation + integration testing.
Never merge these worlds.

### 2) Parity extension, not a rewrite
rosrustext adds missing pieces (naming conventions, lifecycle services, transport adapters, etc.)
It does not attempt to replace rclrs/ros2_rust/roslibrust.

### 3) Minimal surface area
Prefer:
- a small number of explicit traits/types
- feature-gated transport modules
- straightforward composition
Avoid:
- deep abstraction layers
- “framework-ifying” ROS

### 4) User experience matters
The end user should not need to understand:
- colcon plugin internals
- cargo patch tables
- generator pipeline quirks
We hide complexity via documentation + templates, not via heavyweight code.

---

## Design Rules

### A) Core crate is transport-agnostic
`rosrustext_core` contains:
- shared traits
- canonical naming rules
- error model
- logging/tracing hooks
- minimal data structures

Transport crates implement those traits.

### B) Transport crates are adapters
Each transport crate:
- translates between transport APIs and `rosrustext_core`
- owns exactly what it must
- avoids inventing new state machines unless required by parity spec

### C) Generated messages are inputs
Message crates (std_msgs, my_test_msgs, etc.) are never committed.
They are generated and consumed by integration examples.

---

## Testing Strategy

### Unit tests (library)
- run via `cargo test`
- no ROS environment required
- validate naming rules, parsing, conversions, error paths

### Integration tests (dev workspace)
- run as examples / binaries
- validate that:
  - message crates resolve correctly
  - services/topics/actions behave as expected
  - lifecycle parity matches conventions

---

## Toolchain Reality: Python is part of ROS build
rosidl generation steps use Python modules.
Common failure modes:
- missing NumPy headers
- missing `lark` (rosidl parser dependency)
- wrong Python interpreter being used by colcon

Rule:
- be explicit about which python colcon uses (and ensure it has the deps).

---

## Decision Log (lightweight)
When we choose something that affects structure/workflow:
- capture it in docs (implementation_plan.md)
- include the reason and the alternative rejected
This prevents re-litigating decisions.

---

## “Stop Conditions” (when to halt and correct course)
Stop immediately if any of these happen:
- adding ros2_rust source or generated crates into the library repo
- introducing a new abstraction that hides ROS semantics without a strong reason
- requiring end users to adopt our dev workspace layout
- growing the core API surface without a parity requirement
