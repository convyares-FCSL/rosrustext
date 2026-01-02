use std::sync::{Arc, Mutex, MutexGuard};

use rosrustext_core::error::{CoreError, Domain, ErrorKind, Payload, Result as CoreResult};
use rosrustext_core::lifecycle::{goal_state_for_transition, State as CoreState, Transition};
use rosrustext_roslibrust::lifecycle::{ros_state_id, ros_transition_id};
use tracing::warn;

use crate::lifecycle_msgs;

pub const BACKEND_NAMESPACE: &str = "_rosrustext";
pub const SERVICE_CHANGE_STATE: &str = "change_state";
pub const SERVICE_GET_STATE: &str = "get_state";
pub const SERVICE_GET_AVAILABLE_STATES: &str = "get_available_states";
pub const SERVICE_GET_AVAILABLE_TRANSITIONS: &str = "get_available_transitions";
pub const SERVICE_GET_TRANSITION_GRAPH: &str = "get_transition_graph";
pub const TOPIC_TRANSITION_EVENT: &str = "transition_event";
pub const TOPIC_BOND: &str = "/bond";

pub fn frontend_service(target: &str, service: &str) -> String {
    format!("/{target}/{service}")
}

pub fn backend_path(target: &str, name: &str) -> String {
    format!("/{target}/{BACKEND_NAMESPACE}/{name}")
}

pub fn log_core_error(err: CoreError) {
    match err.severity {
        rosrustext_core::error::Severity::Trace => tracing::trace!("{err}"),
        rosrustext_core::error::Severity::Debug => tracing::debug!("{err}"),
        rosrustext_core::error::Severity::Info => tracing::info!("{err}"),
        rosrustext_core::error::Severity::Warn => warn!("{err}"),
        rosrustext_core::error::Severity::Error | rosrustext_core::error::Severity::Fatal => {
            tracing::error!("{err}")
        }
    }
}

pub fn transport_error(context: &'static str, err: roslibrust::Error) -> CoreError {
    CoreError::error()
        .domain(Domain::Transport)
        .kind(ErrorKind::Transport)
        .msgf(format_args!("{context}: {err}"))
        .payload(Payload::Context {
            key: "where",
            value: context.into(),
        })
        .build()
}

pub fn lock_state<'a>(
    state: &'a Arc<Mutex<CoreState>>,
    where_ctx: &'static str,
) -> CoreResult<MutexGuard<'a, CoreState>> {
    state.lock().map_err(|_| {
        CoreError::error()
            .domain(Domain::Lifecycle)
            .kind(ErrorKind::InvalidState)
            .msg("lifecycle state mutex poisoned")
            .payload(Payload::Context {
                key: "where",
                value: where_ctx.into(),
            })
            .build()
    })
}

pub fn ros_state(state: CoreState) -> lifecycle_msgs::State {
    lifecycle_msgs::State {
        id: ros_state_id(state),
        label: state.label().to_string(),
    }
}

pub fn ros_transition_description(
    start: CoreState,
    transition: Transition,
) -> CoreResult<lifecycle_msgs::TransitionDescription> {
    let ros_id = ros_transition_id(start, transition).ok_or_else(|| {
        CoreError::warn()
            .domain(Domain::Lifecycle)
            .kind(ErrorKind::InvalidTransition)
            .msg("no ROS transition id for state")
            .payload(Payload::LifecycleTransition {
                from_state: start.id(),
                via_transition: transition.id(),
            })
            .build()
    })?;
    let goal_state = goal_state_for_transition(start, transition)?;

    Ok(lifecycle_msgs::TransitionDescription {
        transition: lifecycle_msgs::Transition {
            id: ros_id,
            label: transition.label().to_string(),
        },
        start_state: ros_state(start),
        goal_state: ros_state(goal_state),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use rosrustext_roslibrust::lifecycle::ros_ids;

    #[test]
    fn frontend_and_backend_paths_format_correctly() {
        assert_eq!(
            frontend_service("demo", SERVICE_CHANGE_STATE),
            "/demo/change_state"
        );
        assert_eq!(
            backend_path("demo", SERVICE_CHANGE_STATE),
            "/demo/_rosrustext/change_state"
        );
    }

    #[test]
    fn transition_description_contains_ros_ids() {
        let desc =
            ros_transition_description(CoreState::Unconfigured, Transition::Configure).unwrap();

        assert_eq!(desc.transition.id, ros_ids::TRANSITION_CONFIGURE);
        assert_eq!(desc.start_state.label, "Unconfigured");
        assert_eq!(desc.goal_state.label, "Inactive");
    }
}
