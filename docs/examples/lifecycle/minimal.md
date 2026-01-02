# Minimal Lifecycle-Managed Node (Wrapper-Level)

This is a minimal, transport-agnostic example that demonstrates how to:
- implement lifecycle callbacks
- gate a publisher and timer with the activation gate
- connect the node to a transport layer (rosbridge + proxy)

It focuses on the wrapper API so it is easy to adapt to any transport.

---

## Core wiring (transport-agnostic)

```rust
use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;
use std::time::Duration;

use rosrustext_roslibrust::lifecycle::{
    CallbackResult, LifecycleCallbacks, LifecycleNode, ManagedInterval, ManagedPublisher, PublishLike,
};

struct DemoCallbacks;

impl LifecycleCallbacks for DemoCallbacks {
    fn on_configure(&mut self) -> CallbackResult {
        CallbackResult::Success
    }
    fn on_activate(&mut self) -> CallbackResult {
        CallbackResult::Success
    }
    fn on_deactivate(&mut self) -> CallbackResult {
        CallbackResult::Success
    }
    fn on_cleanup(&mut self) -> CallbackResult {
        CallbackResult::Success
    }
    fn on_shutdown(&mut self) -> CallbackResult {
        CallbackResult::Success
    }
    fn on_error(&mut self) -> CallbackResult {
        CallbackResult::Success
    }
}

struct ConsolePublisher;

impl PublishLike<String> for ConsolePublisher {
    type Error = std::io::Error;

    fn publish<'a>(
        &'a self,
        msg: &'a String,
    ) -> Pin<Box<dyn Future<Output = Result<(), Self::Error>> + Send + 'a>> {
        Box::pin(async move {
            println!("publish: {msg}");
            Ok(())
        })
    }
}

#[tokio::main]
async fn main() -> rosrustext_roslibrust::Result<()> {
    let node = LifecycleNode::new("demo_node", Box::new(DemoCallbacks))?;
    let gate = node.activation_gate();

    let publisher = Arc::new(ManagedPublisher::new(gate.clone(), Arc::new(ConsolePublisher)));
    let interval = ManagedInterval::new(gate, Duration::from_millis(250));

    tokio::spawn({
        let publisher = Arc::clone(&publisher);
        async move {
            interval
                .run(move || {
                    let publisher = Arc::clone(&publisher);
                    async move {
                        let _ = publisher.publish(&"tick".to_string()).await;
                    }
                })
                .await
        }
    });

    // Lifecycle transitions are driven by the transport layer (rosbridge + proxy).
    Ok(())
}
```

---

## Transport wiring (rosbridge + proxy)

The lifecycle transport adapters live inside the crate (see
`crates/rosrustext_roslibrust/src/transport/roslibrust/lifecycle.rs`). In
practice, the backend app:

- exposes lifecycle services under `/<node>/_rosrustext/*`
- publishes `/<node>/_rosrustext/transition_event`
- runs behind rosbridge so the proxy can bridge the services to standard ROS 2
  lifecycle endpoints

For a runnable backend, use `scripts/run/roslibrust/lifecycle/run_backend.sh`, which launches
`hyfleet_ring_roslibrust` with the correct environment:

```bash
ROSLIBRUST_BRIDGE_URL=ws://localhost:9090 \
HYFLEET_NODE_NAME=hyfleet_ring_roslibrust \
./scripts/run/roslibrust/lifecycle/run_backend.sh
```

Then start the proxy and interact via standard ROS 2 tools:

```bash
./scripts/run/roslibrust/lifecycle/run_proxy.sh
ros2 lifecycle get /hyfleet_ring_roslibrust
```
