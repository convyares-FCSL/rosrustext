use rosrustext_core::lifecycle::{transition_graph, State as CoreState, Transition, ALL_STATES};
use rosrustext_roslibrust::lifecycle::{
    ros_state_id, ros_transition_id, shutdown_ros_id_for_state, state_from_ros_id,
    transition_from_ros_id,
};

#[test]
fn transition_graph_matches_core_and_ros_mappings() {
    let graph = transition_graph().expect("transition graph should build");

    assert_eq!(graph.states.len(), ALL_STATES.len());
    for state in ALL_STATES {
        assert!(graph.states.contains(&state));
        let ros_id = ros_state_id(state);
        assert_eq!(state_from_ros_id(ros_id), Some(state));
    }

    for edge in &graph.transitions {
        let ros_id = ros_transition_id(edge.start, edge.transition)
            .expect("ROS transition id should exist for graph edge");
        assert_eq!(transition_from_ros_id(ros_id), Some(edge.transition));
        let goal_id = ros_state_id(edge.goal);
        assert_eq!(state_from_ros_id(goal_id), Some(edge.goal));
    }

    for state in [
        CoreState::Unconfigured,
        CoreState::Inactive,
        CoreState::Active,
    ] {
        let shutdown_id = shutdown_ros_id_for_state(state).expect("shutdown id required");
        let mapped = ros_transition_id(state, Transition::Shutdown).unwrap();
        assert_eq!(shutdown_id, mapped);
    }
}
