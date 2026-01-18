# rosrustext Implementation Plan (Parity-Driven, Production-Oriented)

## Purpose

`rosrustext` exists to bring **Rust to production parity with rclcpp/rclpy** in ROS 2 systems.

It is **not**:

* a new ROS client library
* a framework
* a generator-first project

It **is**:

* a semantic parity layer
* a set of transport adapters
* a rigorously specified compatibility project

Parity is judged by **observable ROS behavior under real tooling**, not by API similarity.

---

## Governing Principles

### 1. Parity before convenience

If a feature exists in rclcpp/rclpy and is exercised by ROS tools, it must behave identically in Rust before any extensions are considered.

### 2. One semantic truth

All canonical behavior lives in **`rosrustext_core`**.
Adapters project that truth into transports.

### 3. Transport isolation

Differences between:

* `rclrs` (native RCL)
* `roslibrust` (rosbridge)

**must not leak upward** into specs or examples.

### 4. Clean Cargo boundary

The library repository must:

* build with `cargo` alone
* produce docs on docs.rs
* never contain generated ROS artifacts

ROS work happens elsewhere.

---

## Repository Worlds (Hard Boundary)

### A) Library Repository (authoritative)

**Properties**

* Pure Cargo workspace
* No ROS environment required
* Publishable crates only (except where explicitly documented)

**Contains**

* Semantic specs
* Core state machines
* Transport adapters
* Tooling proxies (if required)

**Never contains**

* generated ROS message crates
* colcon output
* `.cargo/config.toml` patches
* Python environments

---

### B) ROS Dev Workspace (integration only)

**Purpose**

* Message generation
* System tests
* Tool parity validation

**Properties**

* Disposable
* Not committed
* Explicitly documented

This is the **integration firewall**.

---

## Target Repository Structure (Library)

```text
rosrustext/
├─ crates/
│  ├─ rosrustext_core/           # semantic truth
│  ├─ rosrustext_roslibrust/     # rosbridge adapter
│  ├─ rosrustext_rosrs/          # rclrs adapter (ROS workspace only)
│  └─ rosrustext_lifecycle_proxy/  # tooling helper (non-library)
├─ docs/
│  ├─ spec/                     # canonical specs (normative)
│  ├─ adapters/                 # parity matrices (descriptive)
│  └─ overview/                 # project-level docs
├─ scripts/
│  └─ test/                     # CLI + system tests
├─ tools/
├─ Cargo.toml
└─ README.md
```

---

## Spec-First Development Model

Every feature follows the same lifecycle:

1. **Canonical spec** (`docs/spec/<feature>.md`)
2. **Adapter parity doc** (`docs/adapters/<adapter>/<feature>.md`)
3. **Implementation**
4. **Tool-level validation**
5. **Checklist update in TODO**

No implementation without a spec.
No parity claim without tooling tests.

---

## Feature Gates (Explicit)

Each major capability is treated as a **feature domain**, not an implicit assumption.

| Feature       | Scope                                 |
| ------------- | ------------------------------------- |
| `core`        | semantic truth only                   |
| `lifecycle`   | state machine + ROS lifecycle surface |
| `parameters`  | full parameter services + events      |
| `actions`     | goal protocol                         |
| `composition` | component containers                  |
| `executor`    | execution model                       |
| `ecosystem`   | packaging, CI, tooling                |

Adapters must **explicitly document** which features they implement.

---

## Phased Execution Plan (Updated)

### Phase 0 — Spec Lock (mandatory)

Deliverables:

* Canonical specs for:

  * core
  * lifecycle
  * parameters
  * actions
  * composition
  * executor
  * ecosystem
* Feature requirements blocks added to each spec
* TODO updated to include feature coverage checks

Exit criteria:

* Every feature has a spec
* Specs make **no implementation claims**

---

### Phase 1 — Lifecycle (maintenance only)

Lifecycle is **done**.

Only allowed work:

* bug fixes
* doc clarification
* parity regression tests

No redesign unless a spec violation is discovered.

---

### Phase 2 — Parameters (finish user parity)

Focus:

* close remaining user-parity gaps
* document transport limitations explicitly

Deliverables:

* Adapter parity docs updated
* Lessons validated against spec
* Clear limitations where rclrs blocks behavior

Exit criteria:

* `ros2 param` tooling fully validated
* No undocumented semantic differences

---

### Phase 3 — Actions (new core feature)

Deliverables:

* Action core state machine
* Canonical action spec
* One adapter implementation (roslibrust or rosrs)
* CLI-level validation

Exit criteria:

* `ros2 action send_goal/cancel/get_result` works
* Lifecycle interaction documented

---

### Phase 4 — Composition

Deliverables:

* Composition spec
* Container semantics defined
* Adapter decision documented:

  * static vs dynamic loading
  * ABI boundaries

Exit criteria:

* `ros2 component` tooling compatibility (where applicable)
* No hidden assumptions about shared libraries

---

### Phase 5 — Executor model

Deliverables:

* Executor spec
* Adapter parity docs
* Explicit ownership & shutdown semantics

Exit criteria:

* Lifecycle + actions + parameters behave correctly under execution stress
* No hidden background threads

---

### Phase 6 — Ecosystem parity

Deliverables:

* crates.io policy documented
* CI split enforced
* docs.rs builds verified
* Tutorial alignment audit

Exit criteria:

* New users can follow docs without tribal knowledge
* Rust nodes behave indistinguishably from C++ under ROS tooling

---

## Policy: What Never Goes Into rosrustext

* Generated ROS message crates
* ros2_rust source trees
* colcon outputs
* lesson-specific shims
* transport-specific hacks in core

If it’s needed for parity, it goes in **core or adapters**.
If it’s needed for tooling, it goes in **tools**.
If it’s needed for teaching, it goes in **lessons**.

---

## Definition of Done (Global)

rosrustext is production-ready when:

* Specs define canonical ROS behavior
* Adapters match specs under real ROS tools
* Lifecycle, parameters, actions, composition all behave predictably
* Rust nodes can replace C++ nodes without system changes
* The project can be reasoned about by engineers who did not write it

---