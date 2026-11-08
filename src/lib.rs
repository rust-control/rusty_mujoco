// test sample codes in README.md with `cargo test --doc`
#![cfg_attr(all(doc, not(docsrs)), doc = include_str!("../README.md"))]

mod bindgen;
mod error;
mod id;
pub mod util;
mod wrapper_functions;
mod wrapper_methods;

pub use bindgen::*;
pub use error::{Result, MjError};
pub use id::*;
pub use wrapper_functions::*;
