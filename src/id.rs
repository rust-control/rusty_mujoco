/// element id, used for flex and mesh objects
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct ElementId(pub(crate) usize);

/// vertex id, used for flex and mesh objects
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct VertexId(pub(crate) usize);

pub struct ObjectId<O: Obj> {
    pub(crate) index: usize,
    _type: std::marker::PhantomData<O>,
}

pub trait Obj: private::Sealed { const TYPE: ObjType; }

mod private { pub trait Sealed {} }

macro_rules! obj_types {
    ($($name:ident as $type_name:ident),* $(,)?) => {
        pub enum ObjType {
            $(
                #[allow(non_camel_case_types)]
                $type_name = crate::bindgen::mjtObj::$name as isize,
            )*
        }

        pub mod obj {
            $(
                pub struct $type_name;

                impl super::private::Sealed for $type_name {}
                impl super::Obj for $type_name {
                    const TYPE: super::ObjType = super::ObjType::$type_name;
                }
            )*

            pub(super) fn name<O: super::Obj>() -> &'static str {
                match O::TYPE {
                    $(super::ObjType::$type_name => stringify!($type_name),)*
                }
            }
        }
    };
}
obj_types! {
    mjOBJ_UNKNOWN as Unknown,
    mjOBJ_BODY as Body,
    mjOBJ_XBODY as XBody,
    mjOBJ_JOINT as Joint,
    mjOBJ_DOF as Dof,
    mjOBJ_GEOM as Geom,
    mjOBJ_SITE as Site,
    mjOBJ_CAMERA as Camera,
    mjOBJ_LIGHT as Light,
    mjOBJ_FLEX as Flex,
    mjOBJ_MESH as Mesh,
    mjOBJ_SKIN as Skin,
    mjOBJ_HFIELD as HField,
    mjOBJ_TEXTURE as Texture,
    mjOBJ_MATERIAL as Material,
    mjOBJ_PAIR as Pair,
    mjOBJ_EXCLUDE as Exclude,
    mjOBJ_EQUALITY as Equality,
    mjOBJ_TENDON as Tendon,
    mjOBJ_ACTUATOR as Actuator,
    mjOBJ_SENSOR as Sensor,
    mjOBJ_NUMERIC as Numeric,
    mjOBJ_TEXT as Text,
    mjOBJ_TUPLE as Tuple,
    mjOBJ_KEY as Key,
    mjOBJ_PLUGIN as Plugin,
    mjOBJ_FRAME as Frame,
    mjOBJ_DEFAULT as Default,
    mjOBJ_MODEL as Model,
}

impl<O: Obj> ObjectId<O> {
    pub(crate) fn new(index: usize) -> Self {
        Self {
            index,
            _type: std::marker::PhantomData,
        }
    }

    pub fn index(&self) -> usize {
        self.index
    }

    /// SAFETY: This function should only be used when you are sure that the index is valid for the object type `O`.
    pub unsafe fn new_unchecked(index: usize) -> Self {
        Self {
            index,
            _type: std::marker::PhantomData,
        }
    }
}

impl<O: Obj> std::fmt::Debug for ObjectId<O> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ObjectId<{}>({})", obj::name::<O>(), self.index)
    }
}
impl<O: Obj> std::fmt::Display for ObjectId<O> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ObjectId<{}>({})", obj::name::<O>(), self.index)
    }
}

impl<O: Obj> Clone for ObjectId<O> {// not require `O: Clone`
    fn clone(&self) -> Self {
        Self {
            index: self.index,
            _type: std::marker::PhantomData,
        }
    }
}
impl<O: Obj> Copy for ObjectId<O> {} // `O: Copy` is not required, as `usize` is always `Copy`

impl<O: Obj> PartialEq for ObjectId<O> {// not require `O: PartialEq`
    fn eq(&self, other: &Self) -> bool {
        self.index == other.index
    }
}
impl<O: Obj> Eq for ObjectId<O> {} // `O: Eq` is not required, as `usize` is always `Eq`

impl<O: Obj> std::cmp::PartialOrd for ObjectId<O> {// not require `O: PartialOrd`
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.index.cmp(&other.index))
    }
}
impl<O: Obj> std::cmp::Ord for ObjectId<O> {// not require `O: Ord`
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.index.cmp(&other.index)
    }
}
