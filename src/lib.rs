pub mod bindgen;

macro_rules! wrapper {
    ($(#[$attr:meta])* $Name:ident of $inner:path) => {
        $(#[$attr])*
        pub struct $Name($inner);

        impl From<$inner> for $Name {
            fn from(inner: $inner) -> Self {
                $Name(inner)
            }
        }
        impl Into<$inner> for $Name {
            fn into(self) -> $inner {
                self.0
            }
        }

        impl<'a> From<&'a $inner> for &'a $Name {
            fn from(inner: &'a $inner) -> &'a $Name {
                // SAFETY: between wrapper and its inner type
                unsafe {std::mem::transmute(inner)}
            }
        }
        impl<'a> From<&'a mut $inner> for &'a mut $Name {
            fn from(inner: &'a mut $inner) -> &'a mut $Name {
                // SAFETY: between wrapper and its inner type
                unsafe {std::mem::transmute(inner)}
            }
        }

        impl AsRef<$inner> for $Name {
            fn as_ref(&self) -> &$inner {
                &self.0
            }
        }
        impl AsMut<$inner> for $Name {
            fn as_mut(&mut self) -> &mut $inner {
                &mut self.0
            }
        }
    };
}

pub mod types;
pub use types::*;

pub mod functions;
pub use functions::*;

mod error;
pub use error::MjError;

mod id;
pub use id::{ElementId, VertexId, ObjectId, obj};
