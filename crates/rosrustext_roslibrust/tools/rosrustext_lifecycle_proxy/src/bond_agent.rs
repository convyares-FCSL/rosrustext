use std::fmt;
use std::future::Future;
use std::pin::Pin;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use rosrustext_core::error::{CoreError, Domain, ErrorKind, Payload};
use rosrustext_core::lifecycle::State as CoreState;

use crate::bond::Status as BondStatus;
use crate::std_msgs::Header as StdHeader;
use crate::utils::log_core_error;

pub trait BondPublisher: Send + Sync + 'static {
    type Error: fmt::Display + Send + Sync + 'static;

    fn publish<'a>(
        &'a self,
        msg: &'a BondStatus,
    ) -> Pin<Box<dyn Future<Output = Result<(), Self::Error>> + Send + 'a>>;
}

pub struct BondAgent<P: BondPublisher> {
    status_pub: Arc<P>,
    active: Arc<AtomicBool>,
    id: String,
    instance_id: String,
    heartbeat_period: Duration,
    heartbeat_timeout: Duration,
}

impl<P: BondPublisher> BondAgent<P> {
    pub fn new(
        status_pub: Arc<P>,
        id: String,
        heartbeat_period: Duration,
        heartbeat_timeout: Duration,
    ) -> Self {
        let instance_id = format!("rosrustext_proxy_{}", now_nanos());
        Self {
            status_pub,
            active: Arc::new(AtomicBool::new(false)),
            id,
            instance_id,
            heartbeat_period,
            heartbeat_timeout,
        }
    }

    pub fn set_active(&self, enable: bool) {
        let was_active = self.active.swap(enable, Ordering::SeqCst);
        if was_active && !enable {
            let status = self.build_status(false);
            let pub_handle = Arc::clone(&self.status_pub);
            tokio::spawn(async move {
                if let Err(err) = pub_handle.publish(&status).await {
                    log_core_error(bond_publish_error(err));
                }
            });
        }
    }

    pub fn is_active(&self) -> bool {
        self.active.load(Ordering::Relaxed)
    }

    pub fn spawn(self: Arc<Self>) {
        let agent = Arc::clone(&self);
        tokio::spawn(async move {
            let mut ticker = tokio::time::interval(agent.heartbeat_period);
            loop {
                ticker.tick().await;
                if !agent.active.load(Ordering::Relaxed) {
                    continue;
                }
                let status = agent.build_status(true);
                if let Err(err) = agent.status_pub.publish(&status).await {
                    log_core_error(bond_publish_error(err));
                }
            }
        });
    }

    pub fn build_status(&self, active: bool) -> BondStatus {
        let header = StdHeader {
            stamp: now_time(),
            ..Default::default()
        };
        BondStatus {
            header,
            id: self.id.clone(),
            instance_id: self.instance_id.clone(),
            active,
            heartbeat_timeout: self.heartbeat_timeout.as_secs_f32(),
            heartbeat_period: self.heartbeat_period.as_secs_f32(),
        }
    }
}

pub fn bond_active_for_state(state: CoreState) -> bool {
    matches!(state, CoreState::Active)
}

pub fn set_bond_active<P: BondPublisher>(agent: &Option<Arc<BondAgent<P>>>, enable: bool) {
    if let Some(agent) = agent {
        agent.set_active(enable);
    }
}

fn bond_publish_error<E: fmt::Display>(err: E) -> CoreError {
    CoreError::error()
        .domain(Domain::Transport)
        .kind(ErrorKind::Transport)
        .msgf(format_args!("bond status publish failed: {err}"))
        .payload(Payload::Context {
            key: "where",
            value: "bond status publish failed".into(),
        })
        .build()
}

fn now_nanos() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|dur| dur.as_nanos() as u64)
        .unwrap_or(0)
}

fn now_time() -> crate::builtin_interfaces::Time {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|dur| crate::builtin_interfaces::Time {
            sec: dur.as_secs() as i32,
            nanosec: dur.subsec_nanos(),
        })
        .unwrap_or_default()
}
