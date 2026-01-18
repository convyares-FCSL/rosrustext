# Parameters Parity – rosrs (rclrs) Adapter

This document tracks ROS 2 **parameters parity** for the rosrs transport adapter
(`rosrustext_rosrs` using `rclrs`).

Canonical reference:

* `docs/spec/parameters.md` (normative)

This file answers:

> **“Given the ROS 2 parameters spec, what does the rosrs adapter provide?”**

---

### Feature requirements

Spec feature flag: `parameters`

Normative meaning:
- Defines canonical ROS2 parameter services + `parameter_events` semantics (atomic/non-atomic, rejection, ordering).
- Transport-agnostic; focuses on observable behavior.

Required (to claim parameters parity in an adapter):
- `core`
- Adapter-specific feature enabling parameter services/events (adapter-defined)

Optional (adapter-defined, must be explicit):
- `allow_undeclared` mode (if supported)
- Deletion semantics (if supported)

Notes:
- If the underlying transport cannot support set-time validation/rejection, the adapter parity doc MUST state limitations.


## Parity definitions (rosrs)

**Tool parity (external behavior)** means ROS tools and other nodes observe parameter
behavior that matches ROS 2 semantics, regardless of adapter internals.

Acceptance criteria (tool parity):

* `ros2 param list/get/set/describe` works end-to-end.
* Parameter services exist with correct names and types.
* Accepted updates are immediately observable via services.
* `parameter_events` is emitted for accepted updates only.
* Atomic vs non-atomic semantics behave correctly (partial vs all-or-nothing).

**User parity (developer experience)** means authoring parameterized nodes in Rust
feels structurally comparable to rclcpp-style parameter usage.

Acceptance criteria (user parity):

* Parameters can be declared with type, default, and descriptor metadata.
* Users can react to parameter changes **without polling**.
* Validation logic is expressible in user code and testable.
* Invalid updates do not silently corrupt node behavior.
* Observable behavior matches the canonical spec, even if APIs differ.

**Status summary:**
Tool parity is **strong** (services + events + atomicity + ordering).
User parity is **partial**: there is event-driven handling, but no true rclcpp-style
set-time user validation hook in `rclrs` for cross-parameter invariants.

---

## Dependency source

* ROS workspace environment provides `rclrs` and ROS message crates (often via Cargo
  `[patch.crates-io]`).
* The adapter targets **canonical ROS interfaces** (`rcl_interfaces`) and is designed
  so end users do **not** need to generate message bindings for parameter tooling parity.
* Publishability to crates.io depends on message crate availability strategy (pre-generated vs workspace patching).

---

## Tool Parity (External behavior)

Tool parity tracks ROS-facing behavior: parameter services, parameter events,
ordering, and atomic/non-atomic semantics.

### Services (ROS-facing)

| Service                             | ROS Type                                     | Status | Notes                                                                                |
| ----------------------------------- | -------------------------------------------- | ------ | ------------------------------------------------------------------------------------ |
| `/<node>/get_parameters`            | `rcl_interfaces/srv/GetParameters`           | ✅      | Unknown names return `NOT_SET`; ordering preserved.                                  |
| `/<node>/get_parameter_types`       | `rcl_interfaces/srv/GetParameterTypes`       | ✅      | Unknown names return `NOT_SET`; ordering preserved.                                  |
| `/<node>/list_parameters`           | `rcl_interfaces/srv/ListParameters`          | ✅      | Prefix + depth semantics respected.                                                  |
| `/<node>/describe_parameters`       | `rcl_interfaces/srv/DescribeParameters`      | ✅      | Declared parameters return descriptors; unknowns return consistent defaults.         |
| `/<node>/set_parameters`            | `rcl_interfaces/srv/SetParameters`           | ✅      | Non-atomic; per-parameter results; partial success allowed; preserves request order. |
| `/<node>/set_parameters_atomically` | `rcl_interfaces/srv/SetParametersAtomically` | ✅      | Atomic; all-or-nothing; on rejection, state unchanged.                               |

### Topic

| Topic                      | ROS Type                            | Status | Notes                                                                    |
| -------------------------- | ----------------------------------- | ------ | ------------------------------------------------------------------------ |
| `/<node>/parameter_events` | `rcl_interfaces/msg/ParameterEvent` | ✅      | Emitted for accepted updates only; ordering preserved per node instance. |

### Behavioral semantics

| Aspect                            | Status | Notes                                                                                                                                            |
| --------------------------------- | ------ | ------------------------------------------------------------------------------------------------------------------------------------------------ |
| Declared-only vs allow-undeclared | ✅/⚠️   | Declared-only is the default. Allow-undeclared is supported only if explicitly enabled and must be documented by the node/adapter configuration. |
| Unknown get/type                  | ✅      | Returns `NOT_SET` per spec.                                                                                                                      |
| Non-atomic set semantics          | ✅      | Independent application, ordered results.                                                                                                        |
| Atomic set semantics              | ✅      | All-or-nothing; no intermediate observable state; no events on rejection.                                                                        |
| Read-only enforcement             | ✅      | Attempts rejected with reason.                                                                                                                   |
| Reason strings on rejection       | ✅      | Failures return `success=false` with a human-readable reason (best-effort).                                                                      |
| Parameter event emission rules    | ✅      | Accepted updates emit exactly one event; rejected updates emit none.                                                                             |
| Deletion semantics                | ❌      | Deletion via `NOT_SET` is rejected; deletion is not supported and must be documented.                                                            |

---

## Execution model (transport-specific)

The rosrs adapter is a **native `rclrs` node**.

* Parameter services are handled synchronously.
* Changes are reflected immediately in the parameter set on acceptance.
* `parameter_events` is published only after successful application.
* Service handlers must be bounded and must not block indefinitely.

---

## User Parity (Developer experience)

User parity tracks how natural it is to *author* parameterized nodes in Rust.

| Aspect                                     | Status | Notes                                                                                               |
| ------------------------------------------ | ------ | --------------------------------------------------------------------------------------------------- |
| Parameter declaration                      | ✅      | Declare type + default + descriptor metadata.                                                       |
| Basic get access                           | ✅/⚠️   | Access is available but may require manual store access; typed helper ergonomics may be incomplete. |
| Reacting to parameter changes (no polling) | ✅      | Event-driven `ParameterWatcher` pattern supported.                                                  |
| Per-parameter validation (core rules)      | ✅      | Type/read-only/declaredness rules enforced.                                                         |
| User-defined set-time validation hook      | ❌      | No rclcpp-equivalent `on_set_parameters_callback` surface in `rclrs` today.                         |
| Cross-parameter atomic invariants          | ❌      | Without a set-time hook, cannot reject atomically based on user invariants before apply.            |
| Testable validation logic                  | ✅/⚠️   | Pure helpers can be tested; wiring into set-time rejection is limited by missing hook.              |
| rclcpp-style authoring ergonomics          | ⚠️     | Strong baseline, but missing hook prevents full parity.                                             |

---

## Key gap (the “rclcpp hook problem”)

Full canonical parity requires:

* **set-time validation callbacks** that can accept/reject before the service applies values
* ability to reject **atomically** based on cross-parameter invariants

`rclrs` does not currently expose a user-level set-time callback surface comparable to rclcpp/rclpy.
This blocks full user-parity for sophisticated parameter validation.

**Policy:** The adapter must not fake parity by “accept then revert” unless explicitly configured and documented as a limitation (it breaks canonical semantics).

---

## Parity Gaps / Required Changes (rosrs)

| Gap                                    | Evidence                            | Fix                                                  | Risk   | Test                              |
| -------------------------------------- | ----------------------------------- | ---------------------------------------------------- | ------ | --------------------------------- |
| No user set-time validation hook       | `rclrs` lacks rclcpp-style callback | Upstream `rclrs` surface or minimal FFI seam         | Medium | Reject-at-set tests (CLI + rclpy) |
| Cross-parameter atomic rejection       | Blocked by above                    | Same                                                 | Medium | Atomic invariant tests            |
| Parameter deletion unsupported         | `NOT_SET` rejected                  | Add deletion support + deleted_parameters events     | Medium | Deletion CLI tests                |
| Typed convenience accessors incomplete | Store-oriented API                  | Add `get<T>()`/`declare<T>()` helpers (non-breaking) | Low    | Compile-only + doc tests          |

---

## Known intentional differences (current behavior)

* No user-defined set-time validation callback (validation is limited to core rules).
* Parameter deletion is not supported; deletion attempts are rejected.
* Allow-undeclared mode is opt-in and must be documented.

---

## Validation (Jazzy)

Scripts:

* `scripts/test/ros2_rust/parameters/test_param_cli.sh`
* `scripts/test/ros2_rust/parameters/test_set_parameters_atomically.sh`
* `scripts/test/ros2_rust/parameters/test_parameter_events.sh`
* `scripts/test/ros2_rust/parameters/test_parameter_watcher.sh`
* `scripts/test/ros2_rust/run_all_tests.sh`

---

## Definition of Done (rosrs parameters)

Tool parity is complete when a Rust node using `rosrustext_rosrs` can be:

* fully controlled via `ros2 param` CLI
* observed via `parameter_events`
* relied upon by other ROS 2 nodes expecting canonical parameter behavior

User parity is complete when parameter updates can be:

* validated at set-time
* rejected atomically (including cross-parameter invariants)
* reacted to without polling
* authored with ergonomics comparable to rclcpp
