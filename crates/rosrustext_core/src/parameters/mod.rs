//! Transport-agnostic parameter store and semantics.

mod store;
mod types;

pub use store::ParameterStore;
pub use types::{
    Descriptor, DescribedParameter, EventRecord, ListResult, Parameter, SetResult, Type, Value,
};
