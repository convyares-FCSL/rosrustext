use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc, Mutex,
};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use crate::error::Result;

use rclrs::{IntoPrimitiveOptions, Node, QoSProfile, TimerOptions};

use rosrustext_msgs::bond::msg::Status;

/// Nav2 bond QoS is not optional in practice:
/// Reliable + TransientLocal + KeepLast(1)
fn nav2_bond_qos() -> QoSProfile {
    QoSProfile::default().keep_last(1).reliable().transient_local()
}

/// Lifecycle-owned `/bond` publisher + heartbeat timer (Nav2 compatibility).
///
/// # Semantics
/// - Publishes `bond/msg/Status` on `/bond` using Nav2-required QoS.
/// - When active, publishes periodic heartbeats (`active=true`).
/// - On `inactive -> active`, publishes one `active=true` immediately (Nav2 requirement).
/// - On `active -> inactive`, publishes one `active=false` immediately.
///
/// `BondAgent` is typically owned by [`crate::lifecycle::LifecycleNode`] when the
/// crate feature `bond` is enabled.
///
/// # Errors
/// Constructors return [`crate::Error`] if the publisher or timer cannot be created.
///
/// # Example
/// ```text
/// Normally you enable this via LifecycleNode with the `bond` feature.
/// ```
///
/// # See also
/// - [Lifecycle parity notes](https://github.com/convyares-FCSL/rosrustext/blob/main/parity.md)
/// - [`crate::lifecycle::LifecycleNode`]
pub struct BondAgent {
    node_name: String,
    instance_id: String,

    status_pub: Arc<rclrs::Publisher<Status>>,
    // timer must be kept alive
    _timer: Mutex<Option<rclrs::Timer>>,

    active: Arc<AtomicBool>,
    heartbeat_period: Duration,
    heartbeat_timeout: Duration,
}

impl BondAgent {
    /// Create a bond publisher and install a heartbeat timer.
    ///
    /// # Semantics
    /// - Creates a `/bond` publisher with Nav2-required QoS.
    /// - Installs a repeating timer that publishes heartbeats while active.
    /// - Starts inactive (`set_active(false)`).
    ///
    /// # Errors
    /// Returns an error if `rclrs` fails to create the publisher or timer.
    ///
    /// # Example
    /// ```rust,ignore
    /// let agent = BondAgent::new(node, std::time::Duration::from_secs(1), std::time::Duration::from_secs(4))?;
    /// agent.set_active(true);
    /// ```
    ///
    /// # See also
    /// - [`BondAgent::set_active`]
    pub fn new(node: Arc<Node>, heartbeat_period: Duration, heartbeat_timeout: Duration) -> Result<Self> {
        let status_pub = Arc::new(node.create_publisher::<Status>("/bond".qos(nav2_bond_qos()))?);

        let node_name = node.name().to_string();

        let now = SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default();
        let instance_id = format!("rosrustext_rosrs_{}{}", now.as_secs(), now.subsec_nanos());

        let agent = Self {
            node_name,
            instance_id,
            status_pub,
            _timer: Mutex::new(None),
            active: Arc::new(AtomicBool::new(false)),
            heartbeat_period,
            heartbeat_timeout,
        };

        // heartbeat timer (publishes only while active == true)
        agent.install_timer(Arc::clone(&node), Arc::new(agent.clone_inner()))?;

        Ok(agent)
    }

    /// Helper so we can move self into the timer closure via Arc without making BondAgent itself Clone.
    fn clone_inner(&self) -> BondAgentInner {
        BondAgentInner {
            node_name: self.node_name.clone(),
            instance_id: self.instance_id.clone(),
            status_pub: Arc::clone(&self.status_pub),
            active: Arc::clone(&self.active),
            heartbeat_period: self.heartbeat_period,
            heartbeat_timeout: self.heartbeat_timeout,
        }
    }

    fn install_timer(&self, node: Arc<Node>, inner: Arc<BondAgentInner>) -> Result<()> {
        let period = self.heartbeat_period;

        let timer = node.create_timer_repeating(TimerOptions::new(period), move || {
            if inner.active.load(Ordering::Relaxed) {
                let _ = inner.status_pub.publish(inner.make_status(true));
            }
        })?;

        *self._timer.lock().expect("bond timer mutex poisoned") = Some(timer);
        Ok(())
    }

    /// Called by LifecycleNode whenever state changes.
    /// - Active => start heartbeats
    /// - Not Active => stop heartbeats and publish exactly one inactive on edge
    ///
    /// # Semantics
    /// - When enabling (`false -> true`), publishes one immediate `active=true`.
    /// - When disabling (`true -> false`), publishes one immediate `active=false`.
    /// - While enabled, periodic heartbeats are published by the internal timer.
    ///
    /// # Errors
    /// This method does not return errors; publish failures are ignored.
    ///
    /// # Example
    /// ```rust,ignore
    /// agent.set_active(true);
    /// agent.set_active(false);
    /// ```
    ///
    /// # See also
    /// - [`crate::lifecycle::LifecycleNode::is_active`]
    pub fn set_active(&self, enabled: bool) {
        let prev = self.active.swap(enabled, Ordering::Relaxed);
        if !prev && enabled {
            // inactive -> active edge: publish one active immediately
            let _ = self.status_pub.publish(self.make_status(true));
        }
        if prev && !enabled {
            // active -> inactive edge: publish one inactive immediately
            let _ = self.status_pub.publish(self.make_status(false));
        }
    }

    fn make_status(&self, active: bool) -> Status {
        let mut msg = Status::default();

        // bond/Status.header.stamp exists, but "now" source is fine for parity
        let now = SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default();
        msg.header.stamp.sec = now.as_secs() as i32;
        msg.header.stamp.nanosec = now.subsec_nanos();

        msg.id = self.node_name.clone();
        msg.instance_id = self.instance_id.clone();

        msg.active = active;
        msg.heartbeat_timeout = self.heartbeat_timeout.as_secs_f32();
        msg.heartbeat_period = self.heartbeat_period.as_secs_f32();

        msg
    }
}

/// The data that must be captured by the timer closure.
struct BondAgentInner {
    node_name: String,
    instance_id: String,
    status_pub: Arc<rclrs::Publisher<Status>>,
    active: Arc<AtomicBool>,
    heartbeat_period: Duration,
    heartbeat_timeout: Duration,
}

impl BondAgentInner {
    fn make_status(&self, active: bool) -> Status {
        let mut msg = Status::default();

        let now = SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default();
        msg.header.stamp.sec = now.as_secs() as i32;
        msg.header.stamp.nanosec = now.subsec_nanos();

        msg.id = self.node_name.clone();
        msg.instance_id = self.instance_id.clone();

        msg.active = active;
        msg.heartbeat_timeout = self.heartbeat_timeout.as_secs_f32();
        msg.heartbeat_period = self.heartbeat_period.as_secs_f32();

        // keep clippy happy: use the value so optimizer doesn't get cute
        msg
    }
}

// This is intentionally private: only used to seed the Arc for the timer closure.
impl Clone for BondAgentInner {
    fn clone(&self) -> Self {
        Self {
            node_name: self.node_name.clone(),
            instance_id: self.instance_id.clone(),
            status_pub: Arc::clone(&self.status_pub),
            active: Arc::clone(&self.active),
            heartbeat_period: self.heartbeat_period,
            heartbeat_timeout: self.heartbeat_timeout,
        }
    }
}
