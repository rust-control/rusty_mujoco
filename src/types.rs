/// Generates getters and setters for fields of a struct for most/simple cases.
/// Write some edge cases by hand if needed.
macro_rules! derive_fields_mapping {
    ($T:path {
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
                    $Enum(self.$enum_name as u32)
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

mod mjmodel;
mod mjoption;
mod mjdata;
mod auxiliary;

pub use mjmodel::*;
pub use mjoption::*;
pub use mjdata::*;
pub use auxiliary::*;
