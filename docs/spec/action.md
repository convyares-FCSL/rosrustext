# Actions Parity Spec (ROS 2 Canonical)

This document defines the *canonical* ROS 2 **actions surface area and semantics**.

It is **transport-agnostic** and focuses on **observable behavior**
(CLI tools + other nodes), not on any specific client library API.

Related implementation matrices:

* `docs/adapters/roslibrust/actions/parity.md`
* `docs/adapters/ros2rust/actions/parity.md`

---

## Names & Interfaces

Given an action name:

```
/<node_namespace>/<action_name>
```

(e.g. `/navigate_to_pose`)

an action server MUST expose the following **standard ROS 2 endpoints**.

### Services (standard)

* `/<action>/send_goal`
  `<pkg>/action/<Action>_SendGoal`

* `/<action>/get_result`
  `<pkg>/action/<Action>_GetResult`

* `/<action>/cancel_goal`
  `action_msgs/srv/CancelGoal`

> Note
> `SendGoal` and `GetResult` services are **generated per action type**
> and embed the action’s Goal/Result messages.
> `CancelGoal` is always `action_msgs/srv/CancelGoal`.

### Topics (standard)

* `/<action>/feedback`
  `<pkg>/action/<Action>_FeedbackMessage`

* `/<action>/status`
  `action_msgs/msg/GoalStatusArray`

**Tool compatibility depends on these names and types being exact.**

---

## Required Semantics (Normative)

### Action model

An action server manages **goals**. Each goal has:

* `goal_id` (UUID)
* goal payload (action-specific)
* acceptance timestamp (implementation detail)
* lifecycle state per `action_msgs/msg/GoalStatus`

Canonical status values:

* `STATUS_UNKNOWN`
* `STATUS_ACCEPTED`
* `STATUS_EXECUTING`
* `STATUS_CANCELING`
* `STATUS_SUCCEEDED`
* `STATUS_CANCELED`
* `STATUS_ABORTED`

A goal is uniquely identified by `goal_id`.

---

### SendGoal

#### Acceptance / rejection

* A `send_goal` request MUST deterministically indicate acceptance.

* If rejected:

  * `accepted = false`
  * the goal MUST NOT appear in status
  * no feedback or result MUST be observable

* If accepted:

  * the goal MUST appear in the status stream within a bounded time
  * the `goal_id` MUST be usable for `get_result` and `cancel_goal`

#### Duplicate `goal_id`

If a `goal_id` is already known:

* Behavior MUST be consistent and documented.
* Recommended canonical behavior: **reject as duplicate**.

#### Execution start

* Transition from ACCEPTED → EXECUTING MAY be delayed.
* While delayed:

  * status MUST show ACCEPTED
* When work begins:

  * status MUST transition to EXECUTING

---

### Feedback

* Feedback MUST only be published for accepted goals.
* Feedback MUST include the corresponding `goal_id`.
* Ordering MUST be monotonic within a goal.
* After a goal reaches a terminal state, feedback SHOULD stop promptly.

---

### GetResult

* `get_result` MUST be available for all accepted goals.

Result semantics:

* If the goal is terminal:

  * response MUST include final result and final status
* If the goal is not terminal:

  * the server MUST either:

    * block until terminal, **or**
    * return immediately with a consistent “not ready” status

The chosen behavior MUST be consistent and documented.

For unknown goals:

* response MUST indicate invalid / unknown goal
* results MUST NOT be fabricated

---

### CancelGoal

Cancel requests may target:

* a specific `goal_id`
* all goals (per `CancelGoal` request semantics)

Normative behavior:

* The response MUST list goals and cancellation acceptance per goal.
* If accepted:

  * goal transitions to CANCELING (if executing)
  * then transitions to CANCELED unless it completes first
* Terminal goals MUST NOT transition again.

Race semantics:

* If completion and cancellation race:

  * either outcome is acceptable
  * resulting state MUST be valid and consistent

---

### Status stream (`/<action>/status`)

* MUST publish `GoalStatusArray`
* MUST include entries for all non-terminal goals
* Transitions MUST be valid:

```
ACCEPTED → EXECUTING → (SUCCEEDED | ABORTED)
ACCEPTED → CANCELED
EXECUTING → CANCELING → CANCELED
```

Retention policy:

* Terminal goals MAY remain in the status stream for a bounded period.
* Retention duration MUST be consistent and documented.
* Immediate disappearance of terminal goals is NOT recommended.

---

### Concurrency semantics

The server MUST document and enforce a consistent policy:

* single-goal vs multi-goal execution
* whether new goals preempt existing ones
* whether concurrent execution is allowed

Tool compatibility requires only that behavior is deterministic and observable.

---

### QoS (tool compatibility)

Endpoints MUST use QoS compatible with standard ROS 2 tools.

* Defaults SHOULD match common rclcpp implementations.
* Any deviation MUST be documented.

---

## Lifecycle Interaction (Normative for Lifecycle-Managed Nodes)

When lifecycle is enabled:

* Goals MUST be rejected unless the node is **Active**
* Rejection MUST occur at `send_goal`
* No ghost goals or delayed rejection is permitted

On lifecycle transitions:

* Deactivate / Cleanup MUST:

  * stop accepting new goals
  * deterministically cancel or abort active goals per documented policy
* Cleanup MUST release all goal-related resources

---

## Tooling Compatibility (Normative)

A node is tooling-compatible if:

* `ros2 action list` discovers the action
* `ros2 action info <action>` reports valid endpoints
* `ros2 action send_goal`:

  * sends a goal
  * observes status progression
  * receives feedback (if produced)
  * obtains a result or terminal status
* `ros2 action cancel` cancels active goals consistently

---

## Transport Caveat (Informative)

Some transports may lack:

* fine-grained cancellation hooks
* explicit execution callbacks
* precise QoS control

Any deviation from canonical semantics MUST be documented in adapter parity docs.

---

## Definition of Done

Actions parity is complete when:

* Standard endpoints exist with correct names and types
* `ros2 action` CLI works end-to-end
* Goal lifecycle transitions are valid and observable
* Rejected goals leave no observable artifacts
* Cancellation and concurrency semantics are deterministic and documented
* Lifecycle interaction behaves predictably
