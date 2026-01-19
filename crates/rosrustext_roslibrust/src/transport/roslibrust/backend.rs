use std::sync::{Arc, Mutex, OnceLock};
use std::time::{SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result};
use roslibrust::rosbridge::ClientHandle;
use rosrustext_msgs::lifecycle_msgs::{msg, srv};
use tracing::warn;

use crate::lifecycle::{
    available_transitions, goal_state_for_transition, ros_state_id, ros_transition_id, transition_from_ros_id,
    LifecycleNode, State, Transition, TransitionEvent, ALL_STATES,
};
use crate::transport::roslibrust::lifecycle::LifecycleService;
use crate::transport::roslibrust::transition_event::{run_transition_event_publisher, FromTransitionEvent};

const BACKEND_NAMESPACE: &str = "_rosrustext";
const SERVICE_CHANGE_STATE: &str = "change_state";
const SERVICE_GET_STATE: &str = "get_state";
const SERVICE_GET_AVAILABLE_STATES: &str = "get_available_states";
const SERVICE_GET_AVAILABLE_TRANSITIONS: &str = "get_available_transitions";
const TOPIC_TRANSITION_EVENT: &str = "transition_event";

struct BackendRegistration {
    _change_state: roslibrust::rosbridge::ServiceHandle,
    _get_state: roslibrust::rosbridge::ServiceHandle,
    _get_available_states: roslibrust::rosbridge::ServiceHandle,
    _get_available_transitions: roslibrust::rosbridge::ServiceHandle,
}

// Service handles must stay alive for the lifetime of the process.
static REGISTRY: OnceLock<Mutex<Vec<BackendRegistration>>> = OnceLock::new();

impl FromTransitionEvent for msg::TransitionEvent {
    fn from_event(ev: &TransitionEvent) -> Self {
        msg::TransitionEvent {
            timestamp: now_nanos(),
            transition: msg::Transition { id: ev.transition_id, label: transition_label(ev.transition_id) },
            start_state: ros_state(ev.start_state),
            goal_state: ros_state(ev.goal_state),
        }
    }
}

/// Register lifecycle backend endpoints for rosbridge transports.
///
/// This advertises the internal `/_rosrustext/*` services and transition event topic
/// that the lifecycle proxy expects to reach.
pub async fn register_lifecycle_backend_rosbridge(
    client: &ClientHandle, node_name: &str, lifecycle_node: Arc<Mutex<LifecycleNode>>,
) -> Result<()> {
    let node_name = node_name.trim_matches('/');
    if node_name.is_empty() {
        anyhow::bail!("node_name must not be empty");
    }

    let service = Arc::new(LifecycleService::new(Arc::clone(&lifecycle_node)));

    let transition_topic = backend_path(node_name, TOPIC_TRANSITION_EVENT);
    let transition_pub = Arc::new(
        client.advertise::<msg::TransitionEvent>(&transition_topic).await.context("advertise transition_event")?,
    );
    let transition_node = Arc::clone(&lifecycle_node);
    tokio::spawn(async move {
        if let Err(err) = run_transition_event_publisher::<msg::TransitionEvent>(transition_node, transition_pub).await
        {
            warn!("transition event publisher stopped: {err}");
        }
    });

    let change_state_srv = backend_path(node_name, SERVICE_CHANGE_STATE);
    let get_state_srv = backend_path(node_name, SERVICE_GET_STATE);
    let get_available_states_srv = backend_path(node_name, SERVICE_GET_AVAILABLE_STATES);
    let get_available_transitions_srv = backend_path(node_name, SERVICE_GET_AVAILABLE_TRANSITIONS);

    let lifecycle_for_change = Arc::clone(&lifecycle_node);
    let service_for_change = Arc::clone(&service);
    let change_state = client
        .advertise_service::<srv::ChangeState, _>(&change_state_srv, move |req: srv::ChangeState_Request| {
            let transition_id = resolve_transition_id(&lifecycle_for_change, req.transition.id, &req.transition.label);
            let (success, _message) = service_for_change.handle_change_state_transition_id(transition_id);
            Ok(srv::ChangeState_Response { success })
        })
        .await
        .context("advertise change_state")?;

    let lifecycle_for_state = Arc::clone(&lifecycle_node);
    let get_state = client
        .advertise_service::<srv::GetState, _>(&get_state_srv, move |_req: srv::GetState_Request| {
            let state = lifecycle_for_state.lock().map_err(|_| anyhow::anyhow!("lifecycle node poisoned"))?.state();
            Ok(srv::GetState_Response { current_state: ros_state(state) })
        })
        .await
        .context("advertise get_state")?;

    let get_available_states = client
        .advertise_service::<srv::GetAvailableStates, _>(
            &get_available_states_srv,
            move |_req: srv::GetAvailableStates_Request| {
                let states = ALL_STATES.iter().copied().map(ros_state).collect();
                Ok(srv::GetAvailableStates_Response { available_states: states })
            },
        )
        .await
        .context("advertise get_available_states")?;

    let lifecycle_for_transitions = Arc::clone(&lifecycle_node);
    let get_available_transitions = client
        .advertise_service::<srv::GetAvailableTransitions, _>(
            &get_available_transitions_srv,
            move |_req: srv::GetAvailableTransitions_Request| {
                let state =
                    lifecycle_for_transitions.lock().map_err(|_| anyhow::anyhow!("lifecycle node poisoned"))?.state();
                let mut transitions = Vec::new();
                for &transition in available_transitions(state) {
                    if let Some(desc) = ros_transition_description(state, transition) {
                        transitions.push(desc);
                    }
                }
                Ok(srv::GetAvailableTransitions_Response { available_transitions: transitions })
            },
        )
        .await
        .context("advertise get_available_transitions")?;

    store_registration(BackendRegistration {
        _change_state: change_state,
        _get_state: get_state,
        _get_available_states: get_available_states,
        _get_available_transitions: get_available_transitions,
    });

    Ok(())
}

fn store_registration(registration: BackendRegistration) {
    let registry = REGISTRY.get_or_init(|| Mutex::new(Vec::new()));
    let mut guard = registry.lock().expect("backend registry poisoned");
    guard.push(registration);
}

fn backend_path(node_name: &str, name: &str) -> String {
    format!("/{node_name}/{BACKEND_NAMESPACE}/{name}")
}

fn ros_state(state: State) -> msg::State {
    msg::State { id: ros_state_id(state), label: state.label().to_string() }
}

fn ros_transition_description(start: State, transition: Transition) -> Option<msg::TransitionDescription> {
    let ros_id = ros_transition_id(start, transition)?;
    let goal_state = goal_state_for_transition(start, transition).ok()?;
    Some(msg::TransitionDescription {
        transition: msg::Transition { id: ros_id, label: transition.label().to_string() },
        start_state: ros_state(start),
        goal_state: ros_state(goal_state),
    })
}

fn transition_from_label(label: &str) -> Option<Transition> {
    match label.trim().to_ascii_lowercase().as_str() {
        "configure" => Some(Transition::Configure),
        "activate" => Some(Transition::Activate),
        "deactivate" => Some(Transition::Deactivate),
        "cleanup" => Some(Transition::Cleanup),
        "shutdown" => Some(Transition::Shutdown),
        _ => None,
    }
}

fn resolve_transition_id(lifecycle: &Arc<Mutex<LifecycleNode>>, raw_id: u8, label: &str) -> u8 {
    if raw_id != 0 {
        return raw_id;
    }
    if let Some(transition) = transition_from_label(label) {
        if let Ok(guard) = lifecycle.lock() {
            if let Some(id) = ros_transition_id(guard.state(), transition) {
                return id;
            }
        }
    }
    raw_id
}

fn transition_label(id: u8) -> String {
    transition_from_ros_id(id).map(|transition| transition.label().to_string()).unwrap_or_else(|| "unknown".to_string())
}

fn now_nanos() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default().as_nanos() as u64
}
