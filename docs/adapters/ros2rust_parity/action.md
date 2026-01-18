# Actions Parity – rosrs (rclrs) Adapter

This document tracks **actions parity** for the rosrs transport adapter
(`rosrustext_rosrs` using `rclrs`).

Canonical reference:

* `docs/spec/actions.md` (normative)

This file answers:

> “Given the ROS 2 actions spec, what does the rosrs adapter provide?”

---
### Feature requirements

Spec feature flag: `actions`

Normative meaning:
- Defines canonical ROS2 action endpoints (send_goal/get_result/cancel_goal, status, feedback) and goal state semantics.
- Transport-agnostic; tool compatibility is the baseline.

Required (to claim actions parity in an adapter):
- `core`
- Adapter-specific feature enabling action ROS surface (adapter-defined)

Optional (adapter-defined, must be explicit):
- Lifecycle interaction policy (e.g., "reject goals unless Active") if lifecycle is also enabled.
- Multi-goal concurrency policy (single-goal vs multi-goal) must be documented.

Notes:
- Message type sourcing (generated action types) is outside the spec; the adapter must document how types are obtained.

---
## Parity definitions (rosrs)

**Tool parity**
ROS tools and other nodes observe canonical ROS 2 action behavior.

**User parity**
Authoring action servers/clients in Rust feels structurally comparable to rclcpp.

Status summary: **Not yet implemented**.
This document defines acceptance criteria and gaps.

---

## Planned Tool Parity (External behavior)

Target surface:

| Endpoint                | Status | Notes                        |
| ----------------------- | ------ | ---------------------------- |
| `/<action>/send_goal`   | ❌      | Generated action service     |
| `/<action>/get_result`  | ❌      | Generated action service     |
| `/<action>/cancel_goal` | ❌      | `action_msgs/srv/CancelGoal` |
| `/<action>/feedback`    | ❌      | Feedback topic               |
| `/<action>/status`      | ❌      | `GoalStatusArray`            |

CLI compatibility goals:

* `ros2 action list`
* `ros2 action info`
* `ros2 action send_goal`
* `ros2 action cancel`

---

## Planned User Parity (Developer experience)

| Capability             | Target               |
| ---------------------- | -------------------- |
| Action server creation | Builder-based API    |
| Lifecycle-aware gating | Required             |
| Goal execution model   | Explicit async tasks |
| Cancellation handling  | Explicit + testable  |
| Error propagation      | Typed + observable   |

---

## Known Constraints (rosrs)

* Action services and messages require ROS-generated bindings
  (`action_msgs`, `<pkg>/action/*`).
* Lifecycle gating must be implemented explicitly in user-visible logic.
* Executor ownership and async execution must be deterministic.

---

## Definition of Done (rosrs actions)

Tool parity is complete when a Rust action server:

* Is discoverable via `ros2 action`
* Accepts/rejects goals deterministically
* Publishes valid status/feedback
* Handles cancellation correctly
* Interacts correctly with lifecycle state

User parity is complete when:

* Action servers can be authored without boilerplate
* Lifecycle interaction is explicit and safe
* Failure and cancellation paths are observable and testable
