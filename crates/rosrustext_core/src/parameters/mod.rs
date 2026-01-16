//! Transport-agnostic parameter store and semantics.

mod store;
mod types;

pub use store::ParameterStore;
pub use types::{DescribedParameter, Descriptor, EventRecord, ListResult, Parameter, SetResult, Type, Value};
