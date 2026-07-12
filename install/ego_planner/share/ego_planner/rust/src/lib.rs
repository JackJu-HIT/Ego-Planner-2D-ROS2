#![allow(non_camel_case_types)]
#![allow(clippy::derive_partial_eq_without_eq)]
#![allow(clippy::upper_case_acronyms)]

#[path = "msg.rs"]
mod msg_idiomatic;
pub mod msg {
    pub use super::msg_idiomatic::*;
    pub mod rmw;
}


