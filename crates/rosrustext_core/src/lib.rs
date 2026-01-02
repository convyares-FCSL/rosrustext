//! rosrustext_core: ROS-agnostic core utilities for lifecycle + actions + config.
//!
//! Design goals:
//! - Pure, testable logic (no ROS deps).
//! - Explicit types; no macro wizardry.
//! - Small, stable public API surface.

pub mod error;

/// Configuration utilities (ROS-compatible concepts, ROS-agnostic core). - TODO
pub mod config;

/// Logging utilities (ROS-compatible concepts, ROS-agnostic core). - TODO
pub mod logging;

/// Lifecycle state machine + transition hooks (ROS-compatible concepts, ROS-agnostic core). - WIP
pub mod lifecycle;

/// Action protocol core types (goal/feedback/result/cancel) without ROS transport.- TODO
pub mod action;
