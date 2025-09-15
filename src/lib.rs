// test sample codes in README.md with `cargo test --doc`
#![cfg_attr(all(doc, not(docsrs)), doc = include_str!("../README.md"))]

pub mod bindgen;
pub use bindgen::{mjMAXVAL, mjMINVAL};

pub mod types;
pub use types::*;

pub mod functions;
pub use functions::*;

pub mod helper;

mod error;
pub use error::MjError;

mod id;
pub use id::*;
