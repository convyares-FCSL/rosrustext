# Parameters Parity – rosrs (rclrs) Adapter

This document tracks ROS 2 **parameters parity** for the rosrs transport adapter
(`rosrustext_rosrs` using `rclrs`).

Canonical reference:

* `docs/spec/parameters.md` (normative)

This file answers:

> "Given the ROS2 parameters spec, what does the rosrs adapter provide?"

---

## Parity definitions (rosrs)

Tool parity (external behavior) means ROS tools and other nodes observe parameter
behavior that matches ROS 2 semantics, regardless of adapter internals.

Acceptance criteria (tool parity):

* `ros2 param list/get/set/describe` works end-to-end.
* Parameter services exist with correct names and types.
* Accepted parameter updates are immediately observable via services.
* Parameter events are emitted for accepted updates.
* Atomic vs non-atomic semantics are respected.

User parity (developer experience) means authoring parameterized nodes in Rust
feels comparable to rclcpp-style parameter usage.

Acceptance criteria (user parity):

* Parameters can be declared with type, default, and descriptor metadata.
* Users can react to parameter changes without polling.
* Validation logic is expressible in user code and testable.
* Invalid updates do not silently corrupt node behavior.
* Observable behavior matches the canonical spec, even if APIs differ.

Status summary: Tool parity baseline is implemented and validated via scripts;
user parity remains **partial** (watcher helper exists, set-time callback hook missing).

---

## Dependency source

* ROS workspace environment provides `rclrs` and ROS message crates via Cargo
  `[patch.crates-io]`.
* `rosrustext_rosrs` is **not publishable** until ROS msg crates are available
  on crates.io.

---

## Not yet implemented (rosrs parameters)

* No user-defined set-time validation callback hook (blocked by `rclrs`).
* Parameter deletion semantics not supported.
* No convenience typed parameter accessors (store access only).

---

## Tool Parity (External behavior)

Tool parity tracks ROS-facing behavior: parameter services, parameter events,
atomicity, and rejection semantics.

### Services (ROS-facing)

| Service                              | ROS Type                                              | Status | Notes                                                                 |
| ------------------------------------ | ----------------------------------------------------- | ------ | --------------------------------------------------------------------- |
| `/<node>/get_parameters`             | `rcl_interfaces/srv/GetParameters`                    | ✅     | Unknown names return `NOT_SET`; ordering preserved.                    |
| `/<node>/get_parameter_types`        | `rcl_interfaces/srv/GetParameterTypes`                | ✅     | Unknown names return `NOT_SET`; ordering preserved.                    |
| `/<node>/list_parameters`            | `rcl_interfaces/srv/ListParameters`                   | ✅     | Prefix + depth semantics respected.                                   |
| `/<node>/describe_parameters`        | `rcl_interfaces/srv/DescribeParameters`               | ✅     | Descriptors returned for declared params; unknowns return defaults.   |
| `/<node>/set_parameters`             | `rcl_interfaces/srv/SetParameters`                    | ✅     | Partial success; rejects undeclared/read-only/type mismatch.          |
| `/<node>/set_parameters_atomically`  | `rcl_interfaces/srv/SetParametersAtomically`          | ✅     | All-or-nothing; rejection leaves state unchanged.                     |

### Topics

| Topic                      | ROS Type                                   | Status | Notes                                                  |
| -------------------------- | ------------------------------------------ | ------ | ------------------------------------------------------ |
| `/<node>/parameter_events` | `rcl_interfaces/msg/ParameterEvent`        | ✅     | Emitted on accepted updates only; QoS = `parameter_events_default`. |

### Behavioral semantics

| Aspect                              | Status | Notes                                                                 |
| ----------------------------------- | ------ | --------------------------------------------------------------------- |
| Declared-only vs allow-undeclared   | ✅     | Default declared-only; explicit allow-undeclared constructor/option.   |
| Unknown parameter get/type          | ✅     | Unknowns return `NOT_SET`.                                            |
| Non-atomic set semantics            | ✅     | Per-parameter validation; partial success allowed.                    |
| Atomic set semantics                | ✅     | Pre-validated in store; all-or-nothing; no events on rejection.       |
| Read-only parameters                | ✅     | Rejected with reason string.                                          |
| Parameter deletion                  | ❌     | `NOT_SET` rejected; deletion unsupported.                             |
| Parameter events on success only    | ✅     | Rejected updates emit no events.                                      |

---

## Execution model (transport-specific)

The rosrs adapter is a **native `rclrs` node** with parameter services handled
synchronously in the service callback. Service callbacks must be bounded and
must not block indefinitely. Parameter validation is performed in the
adapter store (type/read-only/undeclared/dynamic typing), and events are emitted
only after successful updates.

`rclrs` does not expose a user-defined set-time validation hook, so adapters
cannot offer rclcpp-style callbacks for cross-parameter validation.

---

## User Parity (Developer experience)

User parity tracks how natural it is to *write* parameterized nodes in Rust.

| Aspect                                | Status | Notes                                                                 |
| ------------------------------------- | ------ | --------------------------------------------------------------------- |
| Parameter declaration                 | ✅     | `ParameterNode::declare` supports type, default, descriptor.          |
| Parameter get access                  | ⚠️     | Access via `ParameterNode::store` + lock; no typed helpers.           |
| Reacting to parameter changes         | ✅     | `ParameterWatcher` dispatches per-parameter handlers via events.      |
| Set-time validation & rejection       | ⚠️     | Core validation only; no user callback hook.                          |
| Atomic validation (cross-parameter)   | ❌     | No user-defined invariant hook for atomic sets (store is atomic, rules are core-only). |
| Library-provided set-time validation surface | ❌ | No set-time callback surface (blocked by `rclrs`).                    |
| Poll-free dynamic updates             | ✅     | `ParameterWatcher` drives updates without polling.                    |
| rclcpp-style authoring ergonomics     | ⚠️     | Watcher exists; missing set-time validation hooks + typed accessors.  |

---

## Parity Gaps / Required Changes (rosrs)

| Gap | Evidence | Fix | Risk | Test |
| --- | -------- | --- | ---- | ---- |
| No user-defined set-time validation callbacks | `rclrs` lacks `on_set_parameters_callback` surface | Add adapter callback API when upstream exposes hook | Medium | Set-time rejection tests |
| Parameter deletion unsupported | Store rejects `NOT_SET` | Add deletion semantics + events | Medium | Deletion CLI test |
| No typed convenience accessors | `ParameterNode` exposes store only | Add typed get/set helpers | Low | Compile-only example |

---

## Known intentional differences (current behavior)

* No user-defined set-time validation callbacks; validation is limited to core rules.
* Parameter deletion is not supported; `NOT_SET` requests are rejected.
* Allow-undeclared mode must be explicitly enabled; default is declared-only.

---

## Validation (Jazzy)

Scripts:

* `scripts/test/ros2_rust/parameters/test_param_cli.sh`
* `scripts/test/ros2_rust/parameters/test_set_parameters_atomically.sh`
* `scripts/test/ros2_rust/parameters/test_parameter_events.sh`
* `scripts/test/ros2_rust/parameters/test_parameter_watcher.sh`
* `scripts/test/ros2_rust/run_all_tests.sh` (includes the above)

---

## Definition of Done (rosrs parameters)

Tool parity is complete when a Rust node using `rosrustext_rosrs` can be:

* Fully controlled via `ros2 param` CLI
* Observed via `parameter_events`
* Used safely by other ROS 2 nodes expecting canonical parameter behavior

User parity is complete when parameter changes can be:
* validated at set-time,
* rejected atomically,
* reacted to without polling,
* and authored in Rust with ergonomics comparable to rclcpp.
