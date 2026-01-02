use std::sync::{Arc, Mutex};
use std::time::Duration;

use roslibrust::rosbridge::ClientHandle;
use tracing::info;

use rosrustext_core::error::{CoreError, Domain, ErrorKind, Payload, Result as CoreResult};
use rosrustext_core::lifecycle::{
    available_transitions as core_available_transitions, State as CoreState, Transition, ALL_STATES,
};
use rosrustext_lifecycle_proxy::bond_agent::{
    bond_active_for_state, set_bond_active, BondAgent, BondPublisher,
};
use rosrustext_lifecycle_proxy::config::Config;
use rosrustext_lifecycle_proxy::proxy_state::{ProxyLifecycle, TimeoutOutcome};
use rosrustext_lifecycle_proxy::utils::{
    backend_path, frontend_service, log_core_error, ros_state, ros_transition_description,
    transport_error, BACKEND_NAMESPACE, SERVICE_CHANGE_STATE, SERVICE_GET_AVAILABLE_STATES,
    SERVICE_GET_AVAILABLE_TRANSITIONS, SERVICE_GET_STATE, SERVICE_GET_TRANSITION_GRAPH, TOPIC_BOND,
    TOPIC_TRANSITION_EVENT,
};
use rosrustext_lifecycle_proxy::{
    bond::Status as BondStatus,
    lifecycle_msgs::{
        ChangeState, ChangeStateRequest, ChangeStateResponse, GetAvailableStates,
        GetAvailableStatesRequest, GetAvailableStatesResponse, GetAvailableTransitions,
        GetAvailableTransitionsRequest, GetAvailableTransitionsResponse, GetState, GetStateRequest,
        GetStateResponse, TransitionEvent as RosTransitionEvent,
    },
    rosrustext_interfaces::{
        GetTransitionGraph, GetTransitionGraphRequest, GetTransitionGraphResponse,
    },
};
use rosrustext_roslibrust::lifecycle::{ros_transition_id, transition_from_ros_id};

struct Backend {
    ros: ClientHandle,
    change_state: String,
    transition_event: String,
}

struct RosbridgeBondPublisher {
    inner: Arc<roslibrust::rosbridge::Publisher<BondStatus>>,
}

impl BondPublisher for RosbridgeBondPublisher {
    type Error = roslibrust::Error;

    fn publish<'a>(
        &'a self,
        msg: &'a BondStatus,
    ) -> std::pin::Pin<Box<dyn std::future::Future<Output = Result<(), Self::Error>> + Send + 'a>>
    {
        Box::pin(async move { self.inner.publish(msg).await })
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
        self.ros
            .call_service::<ChangeState>(&self.change_state, req)
            .await
    }
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
    let lifecycle_state = Arc::new(Mutex::new(ProxyLifecycle::new(CoreState::Unconfigured)));

    let bond_agent = if config.bond_enabled {
        let bond_heartbeat_period = Duration::from_secs_f32(1.0);
        let bond_heartbeat_timeout = Duration::from_secs_f32(4.0);
        let bond_topic = TOPIC_BOND.to_string();
        let bond_pub = Arc::new(RosbridgeBondPublisher {
            inner: Arc::new(
                ros_server
                    .advertise::<BondStatus>(&bond_topic)
                    .await
                    .map_err(|err| transport_error("advertise bond status", err))?,
            ),
        });
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
        config.node_name,
        config.target_node,
        config.bridge_url,
        config.target_node,
        BACKEND_NAMESPACE
    );

    let change_state_timeout = Duration::from_millis(
        std::env::var("ROSRUSTEXT_CHANGE_STATE_TIMEOUT_MS")
            .ok()
            .and_then(|value| value.parse::<u64>().ok())
            .unwrap_or(200),
    );

    let backend_change = Arc::clone(&backend);
    let state_for_change = Arc::clone(&lifecycle_state);
    let bond_for_change = bond_agent.clone();
    let _change_handle = ros_server
        .advertise_service::<ChangeState, _>(&change_state_srv, move |req: ChangeStateRequest| {
            let mut req = req;
            info!(
                "change_state request id={} label='{}'",
                req.transition.id, req.transition.label
            );
            let mut guard = match state_for_change.lock() {
                Ok(guard) => guard,
                Err(_) => {
                    log_core_error(
                        CoreError::error()
                            .domain(Domain::Lifecycle)
                            .kind(ErrorKind::InvalidState)
                            .msg("lifecycle state mutex poisoned")
                            .payload(Payload::Context {
                                key: "where",
                                value: "change_state".into(),
                            })
                            .build(),
                    );
                    info!("change_state response success=false (state lock)");
                    return Ok(ChangeStateResponse { success: false });
                }
            };
            let start_state = guard.state();
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

            let plan = match guard.begin_change(transition, transition_id) {
                Ok(plan) => plan,
                Err(err) => {
                    log_core_error(err);
                    info!("change_state response success=false (invalid transition)");
                    return Ok(ChangeStateResponse { success: false });
                }
            };
            let intermediate_state = plan.intermediate_state;
            set_bond_active(&bond_for_change, bond_active_for_state(intermediate_state));
            drop(guard);

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
            let bond_for_timeout = bond_for_change.clone();
            tokio::spawn(async move {
                tokio::time::sleep(change_state_timeout).await;
                let outcome = match state_for_timeout.lock() {
                    Ok(mut guard) => guard.confirm_or_timeout(plan),
                    Err(_) => TimeoutOutcome {
                        ok: true,
                        warned: false,
                    },
                };
                if outcome.warned {
                    set_bond_active(&bond_for_timeout, false);
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
        })
        .await
        .map_err(|err| transport_error("advertise change_state", err))?;

    let state_get = Arc::clone(&lifecycle_state);
    let _get_state_handle = ros_server
        .advertise_service::<GetState, _>(&get_state_srv, move |req: GetStateRequest| {
            let state = match state_get.lock() {
                Ok(guard) => guard.reported_state(),
                Err(_) => {
                    let err = CoreError::error()
                        .domain(Domain::Lifecycle)
                        .kind(ErrorKind::InvalidState)
                        .msg("lifecycle state mutex poisoned")
                        .payload(Payload::Context {
                            key: "where",
                            value: "get_state".into(),
                        })
                        .build();
                    log_core_error(err.clone());
                    return Err(err.into());
                }
            };
            let current_state = ros_state(state);
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
                let state = match state_transitions.lock() {
                    Ok(guard) => guard.state(),
                    Err(_) => {
                        let err = CoreError::error()
                            .domain(Domain::Lifecycle)
                            .kind(ErrorKind::InvalidState)
                            .msg("lifecycle state mutex poisoned")
                            .payload(Payload::Context {
                                key: "where",
                                value: "get_available_transitions".into(),
                            })
                            .build();
                        log_core_error(err.clone());
                        return Err(err.into());
                    }
                };
                let available_transitions = core_available_transitions(state)
                    .iter()
                    .copied()
                    .map(|transition| {
                        ros_transition_description(state, transition)
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
                let graph = rosrustext_core::lifecycle::transition_graph()
                    .inspect_err(|err| log_core_error(err.clone()))?;
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
                Ok(GetTransitionGraphResponse {
                    states,
                    transitions,
                })
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
    let bond_updates = bond_agent.clone();
    tokio::spawn(async move {
        loop {
            let msg = backend_event_sub.next().await;
            let next_state = match state_events.lock() {
                Ok(mut guard) => {
                    guard.record_event(msg.transition.id, msg.goal_state.id, msg.timestamp)
                }
                Err(_) => Err(CoreError::warn()
                    .domain(Domain::Lifecycle)
                    .kind(ErrorKind::InvalidState)
                    .msg("lifecycle state mutex poisoned")
                    .payload(Payload::Context {
                        key: "where",
                        value: "transition_event".into(),
                    })
                    .build()),
            };
            match next_state {
                Ok(state) => set_bond_active(&bond_updates, bond_active_for_state(state)),
                Err(err) => log_core_error(err),
            }
            if let Err(err) = transition_pub.publish(&msg).await {
                log_core_error(transport_error("transition_event publish failed", err));
            }
        }
    });

    tokio::signal::ctrl_c().await.map_err(|err| {
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
