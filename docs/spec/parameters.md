# Parameters Parity Spec (ROS2 Canonical)

This document defines the *canonical* ROS2 parameters surface area and semantics.
It is transport-agnostic and focuses on **observable behavior** (CLI/tools + other nodes),
not on any specific client library API.

Related implementation matrices:
- `docs/adapters/roslibrust/parameters/README.md` (placeholder)
- `docs/adapters/ros2rust/parameters/parity.md`

---

## Names & Interfaces

Node namespace:
- Services: `/<node_name>/<service_name>`
- Topic: `/<node_name>/parameter_events`

Required services (standard ROS 2):
- `get_parameters` — `rcl_interfaces/srv/GetParameters`
- `get_parameter_types` — `rcl_interfaces/srv/GetParameterTypes`
- `set_parameters` — `rcl_interfaces/srv/SetParameters`
- `set_parameters_atomically` — `rcl_interfaces/srv/SetParametersAtomically`
- `list_parameters` — `rcl_interfaces/srv/ListParameters`
- `describe_parameters` — `rcl_interfaces/srv/DescribeParameters`

Required topic:
- `parameter_events` — `rcl_interfaces/msg/ParameterEvent`

Optional (commonly supported in rclcpp/rclpy):
- `get_parameter_descriptions` — (not a standard service; descriptions are returned via `describe_parameters`)
- Parameter event QoS tuning (adapter-defined; must be documented)

---

## Required Semantics (Normative)

### Parameter model

A node maintains a parameter set keyed by name. Each parameter has:
- name (string, hierarchical allowed via `.` separators)
- type (one of: NOT_SET, BOOL, INTEGER, DOUBLE, STRING, BYTE_ARRAY, BOOL_ARRAY, INTEGER_ARRAY, DOUBLE_ARRAY, STRING_ARRAY)
- value (typed payload)
- descriptor metadata (read_only, description, constraints, range hints)

**Canonical constraints:**
- Parameter names are case-sensitive.
- Undeclared parameters behavior is implementation-defined but must be consistent and documented:
  - Some nodes reject setting undeclared parameters.
  - Some nodes allow them (if “allow undeclared” is enabled).
- `ros2 param` tooling assumes parameters are discoverable via the services above.

### Declared vs undeclared

Nodes may operate in one of two modes:

1) **Declared-only mode (common default)**
- Setting an undeclared parameter must be rejected via `SetParametersResult.success=false`
  with a reason string.

2) **Allow-undeclared mode**
- Setting an undeclared parameter creates it (type inferred from value).
- Created parameters must appear in `list_parameters`, `describe_parameters`,
  and `get_parameters` thereafter.

A node MUST document which mode it is operating in.

### GetParameters

- `get_parameters` returns values for the requested names.
- For unknown/undeclared names, returned `ParameterValue` MUST have `type=NOT_SET`.
- The result ordering MUST match the request ordering.

### GetParameterTypes

- Returns the canonical type for each requested name.
- For unknown/undeclared names, returned type MUST be `NOT_SET`.
- The result ordering MUST match the request ordering.

### ListParameters

- Returns parameter names matching:
  - `prefixes` filter (hierarchical name matching)
  - `depth` limit
- Returned names MUST be stable and complete with respect to the node’s current parameter set.
- Result MUST include both fully qualified names and prefixes as defined by the ROS 2 interface contract.

### DescribeParameters

- Returns descriptors for each requested name.
- For unknown/undeclared names, a default descriptor may be returned with minimal information,
  but behavior MUST be consistent and documented.
- If a parameter is declared read-only, descriptor MUST set `read_only=true`.

### SetParameters (non-atomic)

- Attempts to set each parameter independently.
- Returns one `SetParametersResult` per requested parameter, preserving request order.
- Success semantics:
  - If a parameter is accepted, the node’s parameter set MUST reflect the new value immediately.
  - If rejected, the parameter value MUST remain unchanged.
- Partial success is allowed:
  - some parameters may succeed while others fail.
- Reason string:
  - On failure, `reason` SHOULD be non-empty and human readable.

### SetParametersAtomically (atomic)

- Applies all provided parameter updates as a single atomic operation.
- Either:
  - all updates succeed, or
  - none are applied.
- If rejected, the node’s parameter set MUST remain unchanged.
- Result includes a single `SetParametersResult`.

**Atomicity requirement:**
Observers must not see an intermediate state where only a subset has been applied.

### Validation and rejection (set-time)

Nodes may validate parameter changes before they are applied.

Normative behavior:
- Invalid parameter values MUST be rejected via `SetParametersResult.success=false`.
- Rejection MUST NOT modify parameter values.
- In atomic mode, any invalid parameter causes the entire batch to be rejected.

Validation may include:
- type mismatch
- range constraints (e.g. min/max)
- invariants between parameters (cross-parameter validation)

### Read-only parameters

If a parameter descriptor has `read_only=true`:
- any attempt to change it MUST be rejected (`success=false`).

### Parameter events

Parameter updates MUST emit `parameter_events` messages.

Normative event semantics:
- An accepted update MUST emit exactly one event message that includes the change.
- A rejected update MUST NOT emit a change event for that parameter.

Event content:
- `ParameterEvent.node` MUST identify the node.
- `new_parameters` must include parameters that were newly created.
- `changed_parameters` must include parameters whose values changed.
- `deleted_parameters` must include parameters that were removed (if supported).

Ordering:
- Events MUST be published in the same order updates are applied.

### Deletion semantics (optional)

ROS 2 parameters support deletion in some client libraries via setting type `NOT_SET`
or dedicated APIs. Observable behavior varies.

If deletion is supported by the node:
- Deleting a parameter MUST remove it from the parameter set.
- `list_parameters` MUST no longer return it.
- A `parameter_events` message MUST include it under `deleted_parameters`.

If deletion is not supported:
- Requests that attempt deletion MUST be rejected and documented.

---

## Tooling Compatibility (Normative)

A node is tooling-compatible if:

- `ros2 param list/get/set/describe` works against it using the standard ROS 2 services.
- `parameter_events` topic exists and updates are observable after successful sets.
- Unknown parameters behave consistently (NOT_SET for get/types).
- Atomic sets either fully apply or do not apply.

---

## Transport caveat (informative)

Some transports or adapters may not support true set-time validation callbacks.
In those cases, a node may:
- accept changes at the service boundary and validate asynchronously,
- then revert values and emit a compensating event.

This is NOT full parity with canonical ROS2 parameter semantics and must be
documented as a limitation in adapter parity docs.

---

## Definition of Done

Parameters parity is complete when:

- `ros2 param` commands work end-to-end against the node.
- Parameter updates are observable via `parameter_events`.
- Non-atomic and atomic set semantics behave correctly (partial vs all-or-nothing).
- Rejections do not modify values and do not emit change events.
- Behavior for undeclared parameters and deletion is consistent and documented.
