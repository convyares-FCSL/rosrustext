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

Status summary: Tool parity is largely complete; user parity is **partial** due to
upstream `rclrs` limitations (see gaps).

---

## Dependency source

* ROS workspace environment (e.g., `DEV_WS_ROOT`) provides `rclrs` and ROS
  message crates via Cargo `[patch.crates-io]`.
* `rosrustext_rosrs` is **not publishable** until ROS msg crates are available
  on crates.io.

---

## Tool Parity (External behavior)

Tool parity tracks ROS-facing behavior: parameter services, parameter events,
atomicity, and rejection semantics.

### Services (ROS-facing)

| Service                              | ROS Type                                              | Status | Notes                                                                 |
| ------------------------------------ | ----------------------------------------------------- | ------ | --------------------------------------------------------------------- |
| `/<node>/get_parameters`             | `rcl_interfaces/srv/GetParameters`                    | ✅     | Unknown names return `NOT_SET`. Ordering preserved.                    |
| `/<node>/get_parameter_types`        | `rcl_interfaces/srv/GetParameterTypes`                | ✅     | Unknown names return `NOT_SET`.                                        |
| `/<node>/list_parameters`            | `rcl_interfaces/srv/ListParameters`                   | ✅     | Prefix + depth semantics respected.                                   |
| `/<node>/describe_parameters`        | `rcl_interfaces/srv/DescribeParameters`               | ✅     | Descriptors returned for declared params.                              |
| `/<node>/set_parameters`             | `rcl_interfaces/srv/SetParameters`                    | ⚠️     | Accepted/rejected results returned; no set-time callback hook.         |
| `/<node>/set_parameters_atomically`  | `rcl_interfaces/srv/SetParametersAtomically`          | ⚠️     | Atomicity at service level; validation limitations apply.              |

### Topics

| Topic               | ROS Type                                   | Status | Notes                                                                 |
| ------------------- | ------------------------------------------ | ------ | --------------------------------------------------------------------- |
| `/<node>/parameter_events` | `rcl_interfaces/msg/ParameterEvent` | ✅     | Emitted for accepted updates only; ordering preserved.                |

### Behavioral semantics

| Aspect                              | Status | Notes                                                                 |
| ---------------------------------- | ------ | --------------------------------------------------------------------- |
| Declared-only vs allow-undeclared  | ⚠️     | Behavior depends on node configuration; must be documented by node.   |
| Unknown parameter get/type         | ✅     | Returns `NOT_SET` per ROS spec.                                        |
| Non-atomic set semantics           | ⚠️     | Partial success allowed; validation is post-set only.                 |
| Atomic set semantics               | ⚠️     | All-or-nothing at service boundary; true pre-validation unavailable.  |
| Read-only parameters               | ⚠️     | Descriptor respected; enforcement depends on validation strategy.     |
| Parameter deletion                 | ⚠️     | Not consistently supported; behavior must be documented.              |
| Parameter events on success only   | ✅     | Rejected updates do not emit events.                                   |

---

## Execution model (transport-specific)

The rosrs adapter is a **native `rclrs` node**.

* Parameter services are handled synchronously by `rclrs`.
* `rclrs` **does not currently expose set-time validation callbacks**
  (equivalent to rclcpp `on_set_parameters_callback`).
* As a result, adapters cannot reliably reject invalid values *before*
  they are applied by the service layer.

**Implication:**
Validation may be:
* post-set (detect + ignore or revert), or
* simulated via parameter events.

This is a known limitation and is tracked under user parity gaps.

---

## User Parity (Developer experience)

User parity tracks how natural it is to *write* parameterized nodes in Rust.

| Aspect                                | Status | Notes                                                                 |
| ------------------------------------ | ------ | --------------------------------------------------------------------- |
| Parameter declaration                 | ✅     | Typed declaration with defaults and descriptors supported.            |
| Parameter get access                  | ✅     | Direct access via declared handles (e.g., `MandatoryParameter`).      |
| Reacting to parameter changes         | ⚠️     | Requires `parameter_events` subscription or polling.                  |
| Set-time validation & rejection       | ❌     | No native rclrs callback hook; cannot reject before apply.            |
| Atomic validation (cross-parameter)   | ❌     | Not possible pre-set; only detect after.                              |
| Testable validation logic             | ⚠️     | Possible via pure helpers, but wiring is manual.                      |
| Poll-free dynamic updates             | ⚠️     | Achievable via event-driven watcher (adapter-level helper).           |
| rclcpp-style authoring ergonomics     | ❌     | API parity not yet achievable without upstream changes.               |

---

## Parity Gaps / Required Changes (rosrs)

| Gap | Evidence | Fix | Risk | Test |
| --- | -------- | --- | ---- | ---- |
| No set-time validation callback | `rclrs` lacks on-set hook | Upstream rclrs API or adapter-level FFI | Medium | rclpy-style rejection tests |
| No atomic pre-validation | Same as above | Same as above | Medium | Atomic failure tests |
| Event-driven helper missing | Users resort to polling | Add `ParameterWatcher` helper | Low | Dev_ws param update script |
| Undeclared param mode unclear | Node-defined | Require explicit doc / flag | Low | CLI smoke |

---

## Known intentional differences

* Parameter rejection may occur **after** service acceptance (post-set),
  not at set-time.
* Some invalid values may be ignored or reverted rather than rejected.
* Parameter deletion semantics are adapter-defined unless explicitly supported.
* True rclcpp-style `on_set_parameters_callback` parity is not currently possible
  with `rclrs` 0.6.x.

---

## Validation (Jazzy)

* `ros2 param list /<node>`
* `ros2 param get /<node> <param>`
* `ros2 param set /<node> <param> <value>`
* `ros2 param describe /<node> <param>`
* `ros2 topic echo /<node>/parameter_events --once`
* Atomic set smoke: `scripts/test/ros2_rust/parameters/test_set_parameters_atomically.sh`
* Dynamic update smoke (event-driven): `scripts/test/ros2_rust/parameters/test_parameter_events.sh`
* Full suite: `scripts/test/ros2_rust/run_all_tests.sh`

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
```
