include!(concat!(env!("OUT_DIR"), "/messages.rs"));

use std::env;
use std::sync::{Arc, Mutex};
use anyhow::{Context, Result};
use roslibrust::rosbridge::ClientHandle;
use tracing::{info, warn};

use rosrustext_core::lifecycle::{
    available_transitions as core_available_transitions, begin as core_begin,
    finish as core_finish, transition_from_ros_id, CallbackResult, State as CoreState,
};

use lifecycle_msgs::{
    ChangeState, ChangeStateRequest, ChangeStateResponse, GetAvailableStates,
    GetAvailableStatesRequest, GetAvailableStatesResponse, GetAvailableTransitions,
    GetAvailableTransitionsRequest, GetAvailableTransitionsResponse, GetState, GetStateRequest,
    GetStateResponse, TransitionEvent as RosTransitionEvent,
};

const DEFAULT_TARGET_NODE: &str = "hyfleet_ring_roslibrust";
const DEFAULT_BRIDGE_URL: &str = "ws://localhost:9090";

struct Config {
    node_name: String,
    target_node: String,
    bridge_url: String,
}

impl Config {
    fn from_args() -> Self {
        let mut node_name: Option<String> = None;
        let mut target_node = DEFAULT_TARGET_NODE.to_string();
        let mut bridge_url = DEFAULT_BRIDGE_URL.to_string();

        let mut args = env::args().skip(1).peekable();
        while let Some(arg) = args.next() {
            match arg.as_str() {
                "-h" | "--help" => {
                    print_usage();
                    std::process::exit(0);
                }
                "--node-name" => {
                    if let Some(value) = args.next() {
                        node_name = Some(value);
                    }
                }
                "--target-node" => {
                    if let Some(value) = args.next() {
                        target_node = value;
                    }
                }
                "--bridge-url" => {
                    if let Some(value) = args.next() {
                        bridge_url = value;
                    }
                }
                _ if arg.starts_with("--node-name=") => {
                    node_name = Some(arg["--node-name=".len()..].to_string());
                }
                _ if arg.starts_with("--target-node=") => {
                    target_node = arg["--target-node=".len()..].to_string();
                }
                _ if arg.starts_with("--bridge-url=") => {
                    bridge_url = arg["--bridge-url=".len()..].to_string();
                }
                _ => {}
            }
        }

        let node_name = node_name.unwrap_or_else(|| target_node.clone());

        Self {
            node_name,
            target_node,
            bridge_url,
        }
    }
}

fn print_usage() {
    println!(
        "rosrustext_lifecycle_proxy --target-node <name> [--node-name <name>] [--bridge-url ws://host:port]"
    );
}

struct Backend {
    ros: ClientHandle,
    change_state: String,
    transition_event: String,
}

impl Backend {
    fn new(ros: ClientHandle, target_node: &str) -> Self {
        let base = format!("/{}/_rosrustext", target_node);
        Self {
            ros,
            change_state: format!("{}/change_state", base),
            transition_event: format!("{}/transition_event", base),
        }
    }

    async fn call_change_state(
        &self,
        req: ChangeStateRequest,
    ) -> roslibrust::Result<ChangeStateResponse> {
        self.ros.call_service::<ChangeState>(&self.change_state, req).await
    }
}

fn ros_state_id(state: CoreState) -> u8 {
    match state {
        CoreState::Unconfigured => 1,
        CoreState::Inactive => 2,
        CoreState::Active => 3,
        CoreState::Finalized => 4,
        CoreState::Configuring => 10,
        CoreState::CleaningUp => 11,
        CoreState::ShuttingDown => 12,
        CoreState::Activating => 13,
        CoreState::Deactivating => 14,
        CoreState::ErrorProcessing => 15,
    }
}

fn ros_state(state: CoreState) -> lifecycle_msgs::State {
    let mut msg = lifecycle_msgs::State::default();
    msg.id = ros_state_id(state);
    msg.label = format!("{:?}", state);
    msg
}

fn transition_label(id: u8) -> &'static str {
    match id {
        1 => "configure",
        2 => "cleanup",
        3 => "activate",
        4 => "deactivate",
        5 | 6 | 7 => "shutdown",
        _ => "unknown",
    }
}

fn ros_transition_id(start: CoreState, transition: rosrustext_core::lifecycle::Transition) -> u8 {
    use rosrustext_core::lifecycle::Transition;
    match transition {
        Transition::Configure => 1,
        Transition::Cleanup => 2,
        Transition::Activate => 3,
        Transition::Deactivate => 4,
        Transition::Shutdown => match start {
            CoreState::Unconfigured => 5,
            CoreState::Inactive => 6,
            CoreState::Active => 7,
            _ => 5,
        },
    }
}

fn goal_state_for_transition(start: CoreState, transition: rosrustext_core::lifecycle::Transition) -> CoreState {
    use rosrustext_core::lifecycle::Transition;
    match (start, transition) {
        (CoreState::Unconfigured, Transition::Configure) => CoreState::Inactive,
        (CoreState::Inactive, Transition::Activate) => CoreState::Active,
        (CoreState::Active, Transition::Deactivate) => CoreState::Inactive,
        (CoreState::Inactive, Transition::Cleanup) => CoreState::Unconfigured,
        (_, Transition::Shutdown) => CoreState::Finalized,
        _ => start,
    }
}

fn ros_transition_description(
    start: CoreState,
    transition: rosrustext_core::lifecycle::Transition,
) -> lifecycle_msgs::TransitionDescription {
    let mut desc = lifecycle_msgs::TransitionDescription::default();
    let mut ros_transition = lifecycle_msgs::Transition::default();
    let ros_id = ros_transition_id(start, transition);
    ros_transition.id = ros_id;
    ros_transition.label = transition_label(ros_id).to_string();
    desc.transition = ros_transition;
    desc.start_state = ros_state(start);
    desc.goal_state = ros_state(goal_state_for_transition(start, transition));
    desc
}

#[tokio::main(flavor = "multi_thread")]
async fn main() -> Result<()> {
    tracing_subscriber::fmt::init();

    let config = Config::from_args();

    let ros_server = ClientHandle::new(&config.bridge_url)
        .await
        .with_context(|| format!("connect to rosbridge at {}", config.bridge_url))?;
    let ros_backend = ClientHandle::new(&config.bridge_url)
        .await
        .with_context(|| format!("connect to rosbridge backend at {}", config.bridge_url))?;

    let backend = Arc::new(Backend::new(ros_backend, &config.target_node));
    let lifecycle_state = Arc::new(Mutex::new(CoreState::Unconfigured));

    let change_state_srv = format!("/{}/change_state", config.target_node);
    let get_state_srv = format!("/{}/get_state", config.target_node);
    let get_available_states_srv = format!("/{}/get_available_states", config.target_node);
    let get_available_transitions_srv = format!("/{}/get_available_transitions", config.target_node);

    info!(
        "proxy started node_name={} target=/{}/ bridge={} backend_ns=/{}/_rosrustext",
        config.node_name, config.target_node, config.bridge_url, config.target_node
    );

    let backend_change = Arc::clone(&backend);
    let state_change = Arc::clone(&lifecycle_state);
    let _change_handle = ros_server
        .advertise_service::<ChangeState, _>(&change_state_srv, move |req: ChangeStateRequest| {
            let transition_id = req.transition.id;
            let mut resp = ChangeStateResponse { success: false };
            if let Some(transition) = transition_from_ros_id(transition_id) {
                let mut state = match state_change.lock() {
                    Ok(guard) => guard,
                    Err(poison) => {
                        warn!("lifecycle state mutex poisoned");
                        poison.into_inner()
                    }
                };
                if let Ok(intermediate) = core_begin(*state, transition) {
                    if let Ok(final_state) =
                        core_finish(intermediate, transition, CallbackResult::Success)
                    {
                        *state = final_state;
                        resp.success = true;
                    }
                }
            }

            let backend = Arc::clone(&backend_change);
            tokio::spawn(async move {
                match backend.call_change_state(req).await {
                    Ok(resp) => {
                        if !resp.success {
                            warn!("backend change_state reported failure");
                        }
                    }
                    Err(err) => {
                        warn!("backend change_state call failed: {err}");
                    }
                }
            });

            Ok(resp)
        })
        .await
        .context("advertise change_state")?;

    let state_get = Arc::clone(&lifecycle_state);
    let _get_state_handle = ros_server
        .advertise_service::<GetState, _>(&get_state_srv, move |req: GetStateRequest| {
            let state = match state_get.lock() {
                Ok(guard) => guard,
                Err(poison) => {
                    warn!("lifecycle state mutex poisoned");
                    poison.into_inner()
                }
            };
            let current_state = ros_state(*state);
            let _ = req;
            Ok(GetStateResponse { current_state })
        })
        .await
        .context("advertise get_state")?;

    let _get_available_states_handle = ros_server
        .advertise_service::<GetAvailableStates, _>(
            &get_available_states_srv,
            move |req: GetAvailableStatesRequest| {
                let _ = req;
                let states = [
                    CoreState::Unconfigured,
                    CoreState::Inactive,
                    CoreState::Active,
                    CoreState::Finalized,
                    CoreState::Configuring,
                    CoreState::CleaningUp,
                    CoreState::Activating,
                    CoreState::Deactivating,
                    CoreState::ShuttingDown,
                    CoreState::ErrorProcessing,
                ];
                let available_states = states.into_iter().map(ros_state).collect();
                Ok(GetAvailableStatesResponse { available_states })
            },
        )
        .await
        .context("advertise get_available_states")?;

    let state_transitions = Arc::clone(&lifecycle_state);
    let _get_available_transitions_handle = ros_server
        .advertise_service::<GetAvailableTransitions, _>(
            &get_available_transitions_srv,
            move |req: GetAvailableTransitionsRequest| {
                let _ = req;
                let state = match state_transitions.lock() {
                    Ok(guard) => guard,
                    Err(poison) => {
                        warn!("lifecycle state mutex poisoned");
                        poison.into_inner()
                    }
                };
                let available_transitions = core_available_transitions(*state)
                    .iter()
                    .copied()
                    .map(|transition| ros_transition_description(*state, transition))
                    .collect();
                Ok(GetAvailableTransitionsResponse {
                    available_transitions,
                })
            },
        )
        .await
        .context("advertise get_available_transitions")?;

    let transition_event_topic = format!("/{}/transition_event", config.target_node);
    let transition_pub = Arc::new(
        ros_server
            .advertise::<RosTransitionEvent>(&transition_event_topic)
            .await?,
    );
    let backend_event_topic = backend.transition_event.clone();
    let backend_event_sub = ros_server.subscribe::<RosTransitionEvent>(&backend_event_topic).await?;

    tokio::spawn(async move {
        loop {
            let msg = backend_event_sub.next().await;
            if let Err(err) = transition_pub.publish(&msg).await {
                warn!("transition_event publish failed: {err}");
            }
        }
    });

    tokio::signal::ctrl_c().await?;
    info!("shutdown");
    Ok(())
}
