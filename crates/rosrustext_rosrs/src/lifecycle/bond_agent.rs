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

/// Lifecycle-owned bond publisher + heartbeat timer.
/// - publishes /bond Status
/// - active only when lifecycle state is Active
/// - publishes an immediate "inactive" once on active->inactive edge
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
