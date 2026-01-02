use std::future::Future;
use std::pin::Pin;
use std::sync::Arc;
use std::time::Duration;

use rosrustext_core::lifecycle::{State as CoreState, Transition};
use rosrustext_lifecycle_proxy::bond::Status as BondStatus;
use rosrustext_lifecycle_proxy::bond_agent::{
    bond_active_for_state, set_bond_active, BondAgent, BondPublisher,
};
use rosrustext_lifecycle_proxy::proxy_state::ProxyLifecycle;
use rosrustext_roslibrust::lifecycle::{ros_state_id, ros_transition_id};
use tokio::sync::mpsc;

struct TestPublisher {
    tx: mpsc::UnboundedSender<bool>,
}

impl BondPublisher for TestPublisher {
    type Error = String;

    fn publish<'a>(
        &'a self,
        msg: &'a BondStatus,
    ) -> Pin<Box<dyn Future<Output = Result<(), Self::Error>> + Send + 'a>> {
        let tx = self.tx.clone();
        let active = msg.active;
        Box::pin(async move {
            let _ = tx.send(active);
            Ok(())
        })
    }
}

#[test]
fn optimistic_state_updates_and_reconciles() {
    let mut proxy = ProxyLifecycle::new(CoreState::Unconfigured);
    let transition_id = ros_transition_id(CoreState::Unconfigured, Transition::Configure).unwrap();

    let plan = proxy
        .begin_change(Transition::Configure, transition_id)
        .expect("begin_change should succeed");

    assert_eq!(proxy.state(), CoreState::Configuring);
    assert_eq!(proxy.reported_state(), CoreState::Inactive);
    assert_eq!(
        plan.expected_goal_state_id,
        Some(ros_state_id(CoreState::Inactive))
    );

    let goal_id = ros_state_id(CoreState::Inactive);
    proxy
        .record_event(transition_id, goal_id, plan.last_stamp_before + 1)
        .unwrap();

    let outcome = proxy.confirm_or_timeout(plan);
    assert!(outcome.ok);
    assert!(!outcome.warned);
    assert_eq!(proxy.state(), CoreState::Inactive);
    assert!(proxy.pending_goal().is_none());
}

#[test]
fn timeout_reverts_state_and_warns() {
    let mut proxy = ProxyLifecycle::new(CoreState::Unconfigured);
    let transition_id = ros_transition_id(CoreState::Unconfigured, Transition::Configure).unwrap();

    let plan = proxy
        .begin_change(Transition::Configure, transition_id)
        .expect("begin_change should succeed");

    let outcome = proxy.confirm_or_timeout(plan);
    assert!(!outcome.ok);
    assert!(outcome.warned);
    assert_eq!(proxy.state(), CoreState::Unconfigured);
    assert!(proxy.pending_goal().is_none());
}

#[test]
fn back_to_back_transitions_confirm_independently() {
    let mut proxy = ProxyLifecycle::new(CoreState::Unconfigured);

    let configure_id = ros_transition_id(CoreState::Unconfigured, Transition::Configure).unwrap();
    let configure_plan = proxy
        .begin_change(Transition::Configure, configure_id)
        .expect("configure begin should succeed");
    let inactive_id = ros_state_id(CoreState::Inactive);
    proxy
        .record_event(
            configure_id,
            inactive_id,
            configure_plan.last_stamp_before + 1,
        )
        .unwrap();
    let configure_outcome = proxy.confirm_or_timeout(configure_plan);
    assert!(configure_outcome.ok);

    let activate_id = ros_transition_id(CoreState::Inactive, Transition::Activate).unwrap();
    let activate_plan = proxy
        .begin_change(Transition::Activate, activate_id)
        .expect("activate begin should succeed");
    let active_id = ros_state_id(CoreState::Active);
    proxy
        .record_event(activate_id, active_id, activate_plan.last_stamp_before + 1)
        .unwrap();
    let activate_outcome = proxy.confirm_or_timeout(activate_plan);
    assert!(activate_outcome.ok);
    assert_eq!(proxy.state(), CoreState::Active);
}

#[test]
fn bond_active_for_state_only_when_active() {
    assert!(bond_active_for_state(CoreState::Active));
    assert!(!bond_active_for_state(CoreState::Inactive));
    assert!(!bond_active_for_state(CoreState::Unconfigured));
    assert!(!bond_active_for_state(CoreState::Finalized));
}

#[tokio::test]
async fn bond_agent_publishes_heartbeat_and_stop() {
    let (tx, mut rx) = mpsc::unbounded_channel();
    let publisher = Arc::new(TestPublisher { tx });
    let agent = Arc::new(BondAgent::new(
        publisher,
        "demo".to_string(),
        Duration::from_millis(15),
        Duration::from_millis(120),
    ));

    agent.clone().spawn();
    agent.set_active(true);

    let active = tokio::time::timeout(Duration::from_millis(80), rx.recv())
        .await
        .expect("active heartbeat should arrive")
        .expect("channel should yield value");
    assert!(active);

    agent.set_active(false);

    let mut saw_inactive = false;
    let deadline = tokio::time::Instant::now() + Duration::from_millis(120);
    while tokio::time::Instant::now() < deadline {
        if let Ok(Some(value)) = tokio::time::timeout(Duration::from_millis(30), rx.recv()).await {
            if !value {
                saw_inactive = true;
                break;
            }
        }
    }
    assert!(saw_inactive, "expected an inactive bond publish");
}

#[tokio::test]
async fn set_bond_active_toggles_agent_state() {
    let (tx, _rx) = mpsc::unbounded_channel();
    let publisher = Arc::new(TestPublisher { tx });
    let agent = Arc::new(BondAgent::new(
        publisher,
        "demo".to_string(),
        Duration::from_millis(50),
        Duration::from_millis(200),
    ));

    let handle = Some(agent.clone());
    set_bond_active(&handle, true);
    assert!(agent.is_active());

    set_bond_active(&handle, false);
    assert!(!agent.is_active());
}
