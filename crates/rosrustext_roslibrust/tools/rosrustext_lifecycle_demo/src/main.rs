use std::env;
use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use roslibrust::rosbridge::ClientHandle;

use rosrustext_lifecycle_proxy::lifecycle_msgs::{
    ChangeState, ChangeStateRequest, ChangeStateResponse, GetAvailableStates, GetAvailableStatesRequest,
    GetAvailableStatesResponse, GetAvailableTransitions, GetAvailableTransitionsRequest,
    GetAvailableTransitionsResponse, GetState, GetStateRequest, GetStateResponse,
    State as RosState, Transition as RosTransition, TransitionDescription, TransitionEvent as RosTransitionEvent,
};
use rosrustext_lifecycle_proxy::std_msgs::String as RosString;
use rosrustext_roslibrust::lifecycle::{
    available_transitions, CallbackResult, LifecycleCallbacks, LifecycleNode, ManagedInterval, ManagedPublisher,
    RosbridgePublisher, TransitionEvent, ALL_STATES, goal_state_for_transition, ros_state_id, ros_transition_id,
    transition_from_ros_id,
};
use rosrustext_roslibrust::transport::roslibrust::lifecycle::LifecycleService;
use rosrustext_roslibrust::Transition;

const BACKEND_NAMESPACE: &str = "_rosrustext";
const SERVICE_CHANGE_STATE: &str = "change_state";
const SERVICE_GET_STATE: &str = "get_state";
const SERVICE_GET_AVAILABLE_STATES: &str = "get_available_states";
const SERVICE_GET_AVAILABLE_TRANSITIONS: &str = "get_available_transitions";
const TOPIC_TRANSITION_EVENT: &str = "transition_event";

const DEFAULT_NODE_NAME: &str = "rosrustext_lifecycle_demo";
const DEFAULT_PUBLISH_TOPIC: &str = "demo_tick";

struct Config {
    bridge_url: String,
    node_name: String,
    publish_topic: String,
    publish_interval_ms: u64,
}

struct DemoCallbacks;

impl LifecycleCallbacks for DemoCallbacks {
    fn on_configure(&mut self) -> CallbackResult {
        println!("[callbacks] on_configure");
        CallbackResult::Success
    }

    fn on_activate(&mut self) -> CallbackResult {
        println!("[callbacks] on_activate");
        CallbackResult::Success
    }

    fn on_deactivate(&mut self) -> CallbackResult {
        println!("[callbacks] on_deactivate");
        CallbackResult::Success
    }

    fn on_cleanup(&mut self) -> CallbackResult {
        println!("[callbacks] on_cleanup");
        CallbackResult::Success
    }

    fn on_shutdown(&mut self) -> CallbackResult {
        println!("[callbacks] on_shutdown");
        CallbackResult::Success
    }

    fn on_error(&mut self) -> CallbackResult {
        println!("[callbacks] on_error");
        CallbackResult::Success
    }
}

fn transition_event_to_ros(ev: &TransitionEvent) -> RosTransitionEvent {
    let transition_label = transition_from_ros_id(ev.transition_id)
        .map(|transition| transition.label().to_string())
        .unwrap_or_else(|| "unknown".to_string());
    RosTransitionEvent {
        timestamp: now_nanos(),
        transition: RosTransition {
            id: ev.transition_id,
            label: transition_label,
        },
        start_state: ros_state(ev.start_state),
        goal_state: ros_state(ev.goal_state),
    }
}

fn usage() -> String {
    let cmd = env::args().next().unwrap_or_else(|| "rosrustext_lifecycle_demo".to_string());
    format!(
        "{cmd} [--bridge-url ws://host:port] [--node-name <name>] [--publish-topic <name>] [--publish-ms <ms>]\n\n"
    )
}

fn parse_args() -> Result<Config, String> {
    let mut bridge_url = env::var("BRIDGE_URL").unwrap_or_else(|_| "ws://localhost:9090".to_string());
    let mut node_name = DEFAULT_NODE_NAME.to_string();
    let mut publish_topic = DEFAULT_PUBLISH_TOPIC.to_string();
    let mut publish_interval_ms = 500u64;

    let mut args = env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--bridge-url" => {
                bridge_url = args.next().ok_or_else(|| "--bridge-url requires a value".to_string())?;
            }
            "--node-name" => {
                node_name = args.next().ok_or_else(|| "--node-name requires a value".to_string())?;
            }
            "--publish-topic" => {
                publish_topic = args.next().ok_or_else(|| "--publish-topic requires a value".to_string())?;
            }
            "--publish-ms" => {
                publish_interval_ms = args
                    .next()
                    .ok_or_else(|| "--publish-ms requires a value".to_string())?
                    .parse::<u64>()
                    .map_err(|_| "--publish-ms must be a number".to_string())?;
            }
            "-h" | "--help" => {
                return Err(usage());
            }
            other => {
                return Err(format!("unknown arg: {other}\n\n{}", usage()));
            }
        }
    }

    Ok(Config {
        bridge_url,
        node_name,
        publish_topic,
        publish_interval_ms,
    })
}

fn backend_path(node: &str, name: &str) -> String {
    format!("/{node}/{BACKEND_NAMESPACE}/{name}")
}

fn ros_state(state: rosrustext_roslibrust::State) -> RosState {
    RosState { id: ros_state_id(state), label: state.label().to_string() }
}

fn ros_transition_description(
    start: rosrustext_roslibrust::State, transition: Transition,
) -> Option<TransitionDescription> {
    let ros_id = ros_transition_id(start, transition)?;
    let goal_state = goal_state_for_transition(start, transition).ok()?;
    Some(TransitionDescription {
        transition: RosTransition { id: ros_id, label: transition.label().to_string() },
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

fn now_nanos() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_else(|_| Duration::from_secs(0))
        .as_nanos() as u64
}

#[tokio::main(flavor = "multi_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let config = match parse_args() {
        Ok(config) => config,
        Err(help) => {
            eprintln!("{help}");
            return Ok(());
        }
    };

    let ros = ClientHandle::new(&config.bridge_url).await?;

    let lifecycle = Arc::new(Mutex::new(LifecycleNode::new(
        config.node_name.clone(),
        Box::new(DemoCallbacks),
    )?));
    let service = Arc::new(LifecycleService::new(Arc::clone(&lifecycle)));

    let gate = {
        let guard = lifecycle.lock().expect("lifecycle node poisoned");
        guard.activation_gate()
    };

    let publish_topic = format!("/{}/{}", config.node_name, config.publish_topic);
    let publish_topic_for_log = publish_topic.clone();
    let pub_handle = Arc::new(ros.advertise::<RosString>(&publish_topic).await?);
    let managed_pub = Arc::new(ManagedPublisher::new(
        gate.clone(),
        Arc::new(RosbridgePublisher(pub_handle)),
    ));

    let counter = Arc::new(AtomicUsize::new(0));
    let interval = ManagedInterval::new(gate, Duration::from_millis(config.publish_interval_ms));
    tokio::spawn({
        let managed_pub = Arc::clone(&managed_pub);
        let counter = Arc::clone(&counter);
        async move {
            interval
                .run(move || {
                    let managed_pub = Arc::clone(&managed_pub);
                    let counter = Arc::clone(&counter);
                    let publish_topic = publish_topic_for_log.clone();
                    async move {
                        let count = counter.fetch_add(1, Ordering::Relaxed) + 1;
                        let msg = RosString { data: format!("tick {count}") };
                        match managed_pub.publish(&msg).await {
                            Ok(true) => println!("[publish] {count} -> {publish_topic}"),
                            Ok(false) => println!("[publish] suppressed (inactive)"),
                            Err(err) => eprintln!("[publish] error: {err}"),
                        }
                    }
                })
                .await;
        }
    });

    let transition_topic = backend_path(&config.node_name, TOPIC_TRANSITION_EVENT);
    let transition_pub = Arc::new(ros.advertise::<RosTransitionEvent>(&transition_topic).await?);
    let mut transition_rx = {
        let guard = lifecycle.lock().expect("lifecycle node poisoned");
        guard.subscribe_transition_events()
    };
    tokio::spawn({
        let transition_pub = Arc::clone(&transition_pub);
        async move {
            loop {
                match transition_rx.recv().await {
                    Ok(ev) => {
                        let msg = transition_event_to_ros(&ev);
                        if let Err(err) = transition_pub.publish(&msg).await {
                            eprintln!("[transition_event] publish error: {err}");
                        }
                    }
                    Err(err) => {
                        eprintln!("[transition_event] recv error: {err}");
                    }
                }
            }
        }
    });

    let change_state_srv = backend_path(&config.node_name, SERVICE_CHANGE_STATE);
    let get_state_srv = backend_path(&config.node_name, SERVICE_GET_STATE);
    let get_available_states_srv = backend_path(&config.node_name, SERVICE_GET_AVAILABLE_STATES);
    let get_available_transitions_srv = backend_path(&config.node_name, SERVICE_GET_AVAILABLE_TRANSITIONS);

    let lifecycle_for_change = Arc::clone(&lifecycle);
    let service_for_change = Arc::clone(&service);
    let _change_state_handle = ros.advertise_service::<ChangeState, _>(&change_state_srv, move |req: ChangeStateRequest| {
        let transition_id = resolve_transition_id(&lifecycle_for_change, req.transition.id, &req.transition.label);
        let (success, message) = service_for_change.handle_change_state_transition_id(transition_id);
        println!("[change_state] id={} label='{}' -> {} ({})", req.transition.id, req.transition.label, success, message);
        Ok(ChangeStateResponse { success })
    })
    .await?;

    let lifecycle_for_state = Arc::clone(&lifecycle);
    let _get_state_handle = ros.advertise_service::<GetState, _>(&get_state_srv, move |_req: GetStateRequest| {
        let state = lifecycle_for_state.lock().expect("lifecycle node poisoned").state();
        Ok(GetStateResponse { current_state: ros_state(state) })
    })
    .await?;

    let _get_available_states_handle =
        ros.advertise_service::<GetAvailableStates, _>(&get_available_states_srv, move |_req: GetAvailableStatesRequest| {
        let states = ALL_STATES.iter().copied().map(ros_state).collect();
        Ok(GetAvailableStatesResponse { available_states: states })
    })
    .await?;

    let lifecycle_for_transitions = Arc::clone(&lifecycle);
    let _get_available_transitions_handle = ros.advertise_service::<GetAvailableTransitions, _>(
        &get_available_transitions_srv,
        move |_req: GetAvailableTransitionsRequest| {
            let state = lifecycle_for_transitions.lock().expect("lifecycle node poisoned").state();
            let mut transitions = Vec::new();
            for &transition in available_transitions(state) {
                if let Some(desc) = ros_transition_description(state, transition) {
                    transitions.push(desc);
                }
            }
            Ok(GetAvailableTransitionsResponse { available_transitions: transitions })
        },
    )
    .await?;

    println!("rosrustext lifecycle demo node running");
    println!("  node: {}", config.node_name);
    println!("  bridge: {}", config.bridge_url);
    println!("  backend namespace: /{}/{}/", config.node_name, BACKEND_NAMESPACE);
    println!("  publish topic: {}", publish_topic);

    tokio::signal::ctrl_c().await?;
    let (ok, message) = service.shutdown_best_effort();
    println!("shutdown: {} ({})", ok, message);
    Ok(())
}
