include!(concat!(env!("OUT_DIR"), "/messages.rs"));

mod utils;

use std::collections::HashMap;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc, Mutex,
};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use roslibrust::rosbridge::{ClientHandle, Publisher};
use tracing::info;

use rosrustext_core::error::{CoreError, Domain, ErrorKind, Payload, Result as CoreResult};
use rosrustext_core::lifecycle::{
    available_transitions as core_available_transitions, begin, goal_state_for_transition,
    State as CoreState, Transition, ALL_STATES,
};
use rosrustext_roslibrust::lifecycle::{
    ros_state_id, ros_transition_id, state_from_ros_id, transition_from_ros_id,
};
use utils::{
    backend_path, frontend_service, lock_state, log_core_error, ros_state, ros_transition_description,
    transport_error, Config, BACKEND_NAMESPACE, SERVICE_CHANGE_STATE, SERVICE_GET_AVAILABLE_STATES,
    SERVICE_GET_AVAILABLE_TRANSITIONS, SERVICE_GET_STATE, SERVICE_GET_TRANSITION_GRAPH, TOPIC_BOND,
    TOPIC_TRANSITION_EVENT,
};

use bond::Status as BondStatus;
use lifecycle_msgs::{
    ChangeState, ChangeStateRequest, ChangeStateResponse, GetAvailableStates,
    GetAvailableStatesRequest, GetAvailableStatesResponse, GetAvailableTransitions,
    GetAvailableTransitionsRequest, GetAvailableTransitionsResponse, GetState, GetStateRequest,
    GetStateResponse, TransitionEvent as RosTransitionEvent,
};
use rosrustext_interfaces::{
    GetTransitionGraph, GetTransitionGraphRequest, GetTransitionGraphResponse,
};
use std_msgs::Header as StdHeader;

struct Backend {
    ros: ClientHandle,
    change_state: String,
    transition_event: String,
}

struct BondAgent {
    status_pub: Arc<Publisher<BondStatus>>,
    active: Arc<AtomicBool>,
    id: String,
    instance_id: String,
    heartbeat_period: Duration,
    heartbeat_timeout: Duration,
}

#[derive(Clone, Copy, Debug)]
struct EventInfo {
    goal_state_id: u8,
    stamp: u64,
}

impl BondAgent {
    fn new(
        status_pub: Arc<Publisher<BondStatus>>,
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

    fn set_active(&self, enable: bool) {
        let was_active = self.active.swap(enable, Ordering::SeqCst);
        if was_active && !enable {
            let status = self.build_status(false);
            let pub_handle = Arc::clone(&self.status_pub);
            tokio::spawn(async move {
                if let Err(err) = pub_handle.publish(&status).await {
                    log_core_error(transport_error("bond status publish failed", err));
                }
            });
        }
    }

    fn spawn(self: Arc<Self>) {
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
                    log_core_error(transport_error("bond status publish failed", err));
                }
            }
        });
    }

    fn build_status(&self, active: bool) -> BondStatus {
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

fn set_bond_active(agent: &Option<Arc<BondAgent>>, enable: bool) {
    if let Some(agent) = agent {
        agent.set_active(enable);
    }
}

impl Backend {
    fn new(ros: ClientHandle, target_node: &str) -> Self {
        Self {
            ros,
            change_state: backend_path(target_node, SERVICE_CHANGE_STATE),
            transition_event: backend_path(target_node, TOPIC_TRANSITION_EVENT),
        }
    }

    async fn call_change_state(
        &self,
        req: ChangeStateRequest,
    ) -> roslibrust::Result<ChangeStateResponse> {
        self.ros.call_service::<ChangeState>(&self.change_state, req).await
    }
}

fn now_nanos() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|dur| dur.as_nanos() as u64)
        .unwrap_or(0)
}

fn now_time() -> builtin_interfaces::Time {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|dur| builtin_interfaces::Time {
            sec: dur.as_secs() as i32,
            nanosec: dur.subsec_nanos(),
        })
        .unwrap_or_default()
}

fn bond_active_for_state(state: CoreState) -> bool {
    matches!(state, CoreState::Active)
}

fn update_state_from_event(
    state: &Arc<Mutex<CoreState>>,
    msg: &RosTransitionEvent,
) -> CoreResult<CoreState> {
    let goal_id = msg.goal_state.id;
    let next_state = state_from_ros_id(goal_id).ok_or_else(|| {
        CoreError::warn()
            .domain(Domain::Lifecycle)
            .kind(ErrorKind::InvalidState)
            .msg("unknown ROS lifecycle state id")
            .payload(Payload::Code(goal_id as u32))
            .build()
    })?;
    let mut guard = lock_state(state, "transition_event")?;
    *guard = next_state;
    Ok(next_state)
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

#[tokio::main(flavor = "multi_thread")]
async fn main() -> CoreResult<()> {
    tracing_subscriber::fmt::init();

    let config = Config::from_args();

    let ros_server = ClientHandle::new(&config.bridge_url)
        .await
        .map_err(|err| transport_error("connect to rosbridge", err))?;
    let ros_backend = ClientHandle::new(&config.bridge_url)
        .await
        .map_err(|err| transport_error("connect to rosbridge backend", err))?;

    let backend = Arc::new(Backend::new(ros_backend, &config.target_node));
    let lifecycle_state = Arc::new(Mutex::new(CoreState::Unconfigured));
    let pending_goal_state = Arc::new(Mutex::new(None));
    let event_map: Arc<Mutex<HashMap<u8, EventInfo>>> = Arc::new(Mutex::new(HashMap::new()));

    let bond_agent = if config.bond_enabled {
        let bond_heartbeat_period = Duration::from_secs_f32(1.0);
        let bond_heartbeat_timeout = Duration::from_secs_f32(4.0);
        let bond_topic = TOPIC_BOND.to_string();
        let bond_pub = Arc::new(
            ros_server
                .advertise::<BondStatus>(&bond_topic)
                .await
                .map_err(|err| transport_error("advertise bond status", err))?,
        );
        let agent = Arc::new(BondAgent::new(
            bond_pub,
            config.target_node.clone(),
            bond_heartbeat_period,
            bond_heartbeat_timeout,
        ));
        agent.clone().spawn();
        Some(agent)
    } else {
        None
    };

    let change_state_srv = frontend_service(&config.target_node, SERVICE_CHANGE_STATE);
    let get_state_srv = frontend_service(&config.target_node, SERVICE_GET_STATE);
    let get_available_states_srv =
        frontend_service(&config.target_node, SERVICE_GET_AVAILABLE_STATES);
    let get_available_transitions_srv =
        frontend_service(&config.target_node, SERVICE_GET_AVAILABLE_TRANSITIONS);
    let get_transition_graph_srv =
        frontend_service(&config.target_node, SERVICE_GET_TRANSITION_GRAPH);

    info!(
        "proxy started node_name={} target=/{}/ bridge={} backend_ns=/{}/{}",
        config.node_name, config.target_node, config.bridge_url, config.target_node, BACKEND_NAMESPACE
    );

    let change_state_timeout = Duration::from_millis(
        std::env::var("ROSRUSTEXT_CHANGE_STATE_TIMEOUT_MS")
            .ok()
            .and_then(|value| value.parse::<u64>().ok())
            .unwrap_or(200),
    );

    let backend_change = Arc::clone(&backend);
    let state_for_change = Arc::clone(&lifecycle_state);
    let pending_goal_for_change = Arc::clone(&pending_goal_state);
    let event_map_change = Arc::clone(&event_map);
    let bond_for_change = bond_agent.clone();
    let _change_handle = ros_server.advertise_service::<ChangeState, _>(
        &change_state_srv,
        move |req: ChangeStateRequest| {
            let mut req = req;
            info!(
                "change_state request id={} label='{}'",
                req.transition.id, req.transition.label
            );
            let mut guard = match lock_state(&state_for_change, "change_state") {
                Ok(guard) => guard,
                Err(err) => {
                    log_core_error(err);
                    info!("change_state response success=false (state lock)");
                    return Ok(ChangeStateResponse { success: false });
                }
            };
            let start_state = match *guard {
                CoreState::Configuring
                | CoreState::CleaningUp
                | CoreState::Activating
                | CoreState::Deactivating
                | CoreState::ShuttingDown
                | CoreState::ErrorProcessing => {
                    if let Ok(goal_guard) = pending_goal_for_change.lock() {
                        goal_guard.unwrap_or(*guard)
                    } else {
                        *guard
                    }
                }
                _ => *guard,
            };
            let transition = match transition_from_ros_id(req.transition.id)
                .or_else(|| transition_from_label(&req.transition.label))
            {
                Some(transition) => {
                    if let Some(ros_id) = ros_transition_id(start_state, transition) {
                        req.transition.id = ros_id;
                    }
                    transition
                }
                None => {
                    let err = CoreError::warn()
                        .domain(Domain::Lifecycle)
                        .kind(ErrorKind::InvalidArgument)
                        .msg("unknown ROS transition id")
                        .payload(Payload::Code(req.transition.id as u32))
                        .build();
                    log_core_error(err);
                    info!("change_state response success=false (unknown transition)");
                    return Ok(ChangeStateResponse { success: false });
                }
            };
            let transition_id = req.transition.id;

            let intermediate_state = match begin(start_state, transition) {
                Ok(state) => state,
                Err(err) => {
                    log_core_error(err);
                    info!("change_state response success=false (invalid transition)");
                    return Ok(ChangeStateResponse { success: false });
                }
            };
            *guard = intermediate_state;
            set_bond_active(&bond_for_change, bond_active_for_state(intermediate_state));
            drop(guard);

            let expected_goal_state = match goal_state_for_transition(start_state, transition) {
                Ok(state) => Some(state),
                Err(_err) => {
                    log_core_error(
                        CoreError::warn()
                            .domain(Domain::Lifecycle)
                            .kind(ErrorKind::InvalidTransition)
                            .msg("change_state requested while local state is stale")
                            .payload(Payload::LifecycleTransition {
                                from_state: start_state.id(),
                                via_transition: transition.id(),
                            })
                            .build(),
                    );
                    None
                }
            };
            let expected_goal_state_id = expected_goal_state.map(ros_state_id);

            if let Some(goal) = expected_goal_state {
                if let Ok(mut guard) = pending_goal_for_change.lock() {
                    *guard = Some(goal);
                }
            }

            let last_stamp_before = match event_map_change.lock() {
                Ok(guard) => guard.get(&transition_id).map(|ev| ev.stamp).unwrap_or(0),
                Err(_) => 0,
            };

            let backend = Arc::clone(&backend_change);
            let bond_for_backend_error = bond_for_change.clone();
            tokio::spawn(async move {
                match backend.call_change_state(req).await {
                    Ok(resp) => {
                        if !resp.success {
                            log_core_error(
                                CoreError::warn()
                                    .domain(Domain::Lifecycle)
                                    .msg("backend change_state reported failure")
                                    .payload(Payload::Code(transition_id as u32))
                                    .build(),
                            );
                        }
                    }
                    Err(err) => {
                        log_core_error(transport_error("backend change_state call failed", err));
                        set_bond_active(&bond_for_backend_error, false);
                    }
                }
            });

            let state_for_timeout = Arc::clone(&state_for_change);
            let pending_goal_for_timeout = Arc::clone(&pending_goal_for_change);
            let event_map = Arc::clone(&event_map_change);
            let bond_for_timeout = bond_for_change.clone();
            tokio::spawn(async move {
                tokio::time::sleep(change_state_timeout).await;
                let snapshot = match event_map.lock() {
                    Ok(guard) => guard.get(&transition_id).copied(),
                    Err(_) => None,
                };
                let ok = match snapshot {
                    Some(ev) => {
                        if ev.stamp == last_stamp_before {
                            false
                        } else {
                            match expected_goal_state_id {
                                Some(expected) => ev.goal_state_id == expected,
                                None => true,
                            }
                        }
                    }
                    None => false,
                };
                if !ok {
                    if let Ok(mut guard) = lock_state(&state_for_timeout, "change_state_timeout") {
                        if *guard == intermediate_state {
                            *guard = start_state;
                        }
                        set_bond_active(&bond_for_timeout, false);
                    }
                    if let Ok(mut guard) = pending_goal_for_timeout.lock() {
                        *guard = None;
                    }
                    log_core_error(
                        CoreError::warn()
                            .domain(Domain::Lifecycle)
                            .kind(ErrorKind::Timeout)
                            .msg("change_state not confirmed by transition_event")
                            .payload(Payload::Code(transition_id as u32))
                            .build(),
                    );
                }
            });

            info!("change_state response success=true");
            Ok(ChangeStateResponse { success: true })
        },
    )
    .await
    .map_err(|err| transport_error("advertise change_state", err))?;


    let state_get = Arc::clone(&lifecycle_state);
    let pending_goal_get = Arc::clone(&pending_goal_state);
    let _get_state_handle = ros_server
        .advertise_service::<GetState, _>(&get_state_srv, move |req: GetStateRequest| {
            if let Ok(guard) = pending_goal_get.lock() {
                if let Some(goal) = *guard {
                    let _ = req;
                    return Ok(GetStateResponse {
                        current_state: ros_state(goal),
                    });
                }
            }
            let state = match lock_state(&state_get, "get_state") {
                Ok(guard) => guard,
                Err(err) => {
                    log_core_error(err.clone());
                    return Err(err.into());
                }
            };
            let current_state = ros_state(*state);
            let _ = req;
            Ok(GetStateResponse { current_state })
        })
        .await
        .map_err(|err| transport_error("advertise get_state", err))?;

    let _get_available_states_handle = ros_server
        .advertise_service::<GetAvailableStates, _>(
            &get_available_states_srv,
            move |req: GetAvailableStatesRequest| {
                let _ = req;
                let available_states = ALL_STATES.into_iter().map(ros_state).collect();
                Ok(GetAvailableStatesResponse { available_states })
            },
        )
        .await
        .map_err(|err| transport_error("advertise get_available_states", err))?;

    let state_transitions = Arc::clone(&lifecycle_state);
    let _get_available_transitions_handle = ros_server
        .advertise_service::<GetAvailableTransitions, _>(
            &get_available_transitions_srv,
            move |req: GetAvailableTransitionsRequest| {
                let _ = req;
                let state = match lock_state(&state_transitions, "get_available_transitions") {
                    Ok(guard) => guard,
                    Err(err) => {
                        log_core_error(err.clone());
                        return Err(err.into());
                    }
                };
                let available_transitions = core_available_transitions(*state)
                    .iter()
                    .copied()
                    .map(|transition| {
                        ros_transition_description(*state, transition)
                            .inspect_err(|err| log_core_error(err.clone()))
                    })
                    .collect::<CoreResult<Vec<_>>>()?;
                Ok(GetAvailableTransitionsResponse {
                    available_transitions,
                })
            },
        )
        .await
        .map_err(|err| transport_error("advertise get_available_transitions", err))?;

    let _get_transition_graph_handle = ros_server
        .advertise_service::<GetTransitionGraph, _>(
            &get_transition_graph_srv,
            move |req: GetTransitionGraphRequest| {
                info!("get_transition_graph request");
                let _ = req;
                let graph = rosrustext_core::lifecycle::transition_graph().map_err(|err| {
                    log_core_error(err.clone());
                    err
                })?;
                let states: Vec<_> = graph.states.into_iter().map(ros_state).collect();
                let transitions = graph
                    .transitions
                    .into_iter()
                    .map(|edge| {
                        ros_transition_description(edge.start, edge.transition)
                            .inspect_err(|err| log_core_error(err.clone()))
                    })
                    .collect::<CoreResult<Vec<_>>>()?;
                info!(
                    "get_transition_graph response states={} transitions={}",
                    states.len(),
                    transitions.len()
                );
                Ok(GetTransitionGraphResponse { states, transitions })
            },
        )
        .await
        .map_err(|err| transport_error("advertise get_transition_graph", err))?;

    let transition_event_topic = frontend_service(&config.target_node, TOPIC_TRANSITION_EVENT);
    let transition_pub = Arc::new(
        ros_server
            .advertise::<RosTransitionEvent>(&transition_event_topic)
            .await
            .map_err(|err| transport_error("advertise transition_event", err))?,
    );
    let backend_event_topic = backend.transition_event.clone();
    let backend_event_sub = ros_server
        .subscribe::<RosTransitionEvent>(&backend_event_topic)
        .await
        .map_err(|err| transport_error("subscribe transition_event", err))?;

    let state_events = Arc::clone(&lifecycle_state);
    let event_map_updates = Arc::clone(&event_map);
    let pending_goal_updates = Arc::clone(&pending_goal_state);
    let bond_updates = bond_agent.clone();
    tokio::spawn(async move {
        loop {
            let msg = backend_event_sub.next().await;
            if let Ok(mut guard) = event_map_updates.lock() {
                guard.insert(
                    msg.transition.id,
                    EventInfo {
                        goal_state_id: msg.goal_state.id,
                        stamp: msg.timestamp,
                    },
                );
            }
            match update_state_from_event(&state_events, &msg) {
                Ok(next_state) => set_bond_active(&bond_updates, bond_active_for_state(next_state)),
                Err(err) => log_core_error(err),
            }
            if let Ok(mut guard) = pending_goal_updates.lock() {
                *guard = None;
            }
            if let Err(err) = transition_pub.publish(&msg).await {
                log_core_error(transport_error("transition_event publish failed", err));
            }
        }
    });

    tokio::signal::ctrl_c()
        .await
        .map_err(|err| {
            CoreError::error()
                .domain(Domain::Other)
                .kind(ErrorKind::Io)
                .msgf(format_args!("ctrl-c handler failed: {err}"))
                .build()
        })?;
    set_bond_active(&bond_agent, false);
    info!("shutdown");
    Ok(())
}
