//! Types that wrap raw bindgen-generated structs with safe APIs.

macro_rules! resource_wrapper {
    (
        $T:ident for $Bindgen:path;
        drop = $drop:path;
    ) => {
        pub struct $T(*mut $Bindgen);
        impl Drop for $T {
            fn drop(&mut self) {
                $drop(self);
            }
        }
        impl $T {
            pub(crate) fn from_raw(ptr: *mut $Bindgen) -> Self {
                Self(ptr)
            }
            pub(crate) fn as_ptr(&self) -> *const $Bindgen {
                self.0
            }
            pub(crate) fn as_mut_ptr(&mut self) -> *mut $Bindgen {
                self.0
            }
        }
        impl std::ops::Deref for $T {
            type Target = $Bindgen;
            fn deref(&self) -> &Self::Target {
                unsafe { &*self.0 }
            }
        }
        impl std::ops::DerefMut for $T {
            fn deref_mut(&mut self) -> &mut Self::Target {
                unsafe { &mut *self.0 }
            }
        }
        impl std::fmt::Debug for $T {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                (**self).fmt(f)
            }
        }
    };
}

/// Generate getter and/or setter for fields of a struct for most/simple cases.
/// Write some edge cases by hand if needed.
macro_rules! fields_mapping {
    ($T:path {
        $(boolean_flags {
            $(
                $flag_name:ident $(/ $set_flag_name:ident)? = $flag_description:literal;
            )*
        })?
        $(scalars {
            $(
                $scalar_name:ident $(/ $set_scalar_name:ident)?: $Scalar:ty = $scalar_description:literal;
            )*
        })?
        $(enums {
            $(
                $enum_name:ident $(/ $set_enum_name:ident)?: $Enum:ident = $enum_description:literal;
            )*
        })?
        $(structs {
            $(
                $struct_name:ident $(/ $mut_struct_name:ident)?: $Struct:ty = $struct_description:literal;
            )*
        })?
    }) => {
        #[allow(non_snake_case)]
        impl $T {
            $($(
                #[doc = $flag_description]
                pub fn $flag_name(&self) -> bool {
                    self.$flag_name != 0
                }
                $(
                    #[doc = "set "]
                    #[doc = $flag_description]
                    pub fn $set_flag_name(&mut self, value: bool) -> &mut Self {
                        self.$flag_name = if value { 1 } else { 0 };
                        self
                    }
                )?
            )*)?
            $($(
                #[doc = $scalar_description]
                pub fn $scalar_name(&self) -> $Scalar {
                    self.$scalar_name as _
                }
                $(
                    #[doc = "set "]
                    #[doc = $scalar_description]
                    pub fn $set_scalar_name(&mut self, value: $Scalar) -> &mut Self {
                        self.$scalar_name = value as _;
                        self
                    }
                )?
            )*)?
            $($(
                #[doc = $enum_description]
                pub fn $enum_name(&self) -> $Enum {
                    $Enum(self.$enum_name as _)
                }
                $(
                    #[doc = "set "]
                    #[doc = $enum_description]
                    pub fn $set_enum_name(&mut self, value: $Enum) -> &mut Self {
                        self.$enum_name = value.0 as _;
                        self
                    }
                )?
            )*)?
            $($(
                #[doc = $struct_description]
                pub fn $struct_name(&self) -> &$Struct {
                    &self.$struct_name
                }
                $(
                    #[doc = "mutable "]
                    #[doc = $struct_description]
                    pub fn $mut_struct_name(&mut self) -> &mut $Struct {
                        &mut self.$struct_name
                    }
                )?
            )*)?
        }
    };
}

mod mjdata;
mod mjmodel;
mod mjoption;
mod model_editing;
mod auxiliary;
mod rendering;
mod sim_statistics;
mod user_interface;
mod visualization;

pub use mjdata::*;
pub use mjmodel::*;
pub use mjoption::*;
pub use model_editing::*;
pub use auxiliary::*;
pub use rendering::*;
pub use sim_statistics::*;
pub use user_interface::*;
pub use visualization::*;

pub use crate::bindgen::{
    mjtByte, mjByteVec, mjString, mjStringVec, mjIntVec, mjFloatVec, mjDoubleVec,
};
