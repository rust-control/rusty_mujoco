#![allow(non_snake_case)]

mod helper {
    pub struct Dimention<const N: usize>;

    pub trait FrictionDimention {}
    impl FrictionDimention for Dimention<1> {}
    impl FrictionDimention for Dimention<2> {}
}

pub mod attribute_getters;
pub mod components;
pub mod error_and_memory;
pub mod initialization;
pub mod main_simulation;
pub mod miscellaneous;
pub mod parse_and_compile;
pub mod ray_casting;
pub mod sub_components;
pub mod support;

pub use attribute_getters::*;
pub use components::*;
pub use error_and_memory::*;
pub use initialization::*;
pub use main_simulation::*;
pub use miscellaneous::*;
pub use parse_and_compile::*;
pub use ray_casting::*;
pub use sub_components::*;
pub use support::*;
