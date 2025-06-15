/* ref:
#[repr(u32)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub enum mjtObj_ {
    mjOBJ_UNKNOWN = 0,
    mjOBJ_BODY = 1,
    mjOBJ_XBODY = 2,
    mjOBJ_JOINT = 3,
    mjOBJ_DOF = 4,
    mjOBJ_GEOM = 5,
    mjOBJ_SITE = 6,
    mjOBJ_CAMERA = 7,
    mjOBJ_LIGHT = 8,
    mjOBJ_FLEX = 9,
    mjOBJ_MESH = 10,
    mjOBJ_SKIN = 11,
    mjOBJ_HFIELD = 12,
    mjOBJ_TEXTURE = 13,
    mjOBJ_MATERIAL = 14,
    mjOBJ_PAIR = 15,
    mjOBJ_EXCLUDE = 16,
    mjOBJ_EQUALITY = 17,
    mjOBJ_TENDON = 18,
    mjOBJ_ACTUATOR = 19,
    mjOBJ_SENSOR = 20,
    mjOBJ_NUMERIC = 21,
    mjOBJ_TEXT = 22,
    mjOBJ_TUPLE = 23,
    mjOBJ_KEY = 24,
    mjOBJ_PLUGIN = 25,
    mjNOBJECT = 26,
    mjOBJ_FRAME = 100,
    mjOBJ_DEFAULT = 101,
    mjOBJ_MODEL = 102,
}
pub use self::mjtObj_ as mjtObj;
*/

pub trait Obj: private::Sealed { const TYPE: crate::bindgen::mjtObj; }

mod private { pub trait Sealed {} }

macro_rules! obj_as_types {
    ($($name:ident as $type_name:ident),* $(,)?) => {
        pub mod obj {
            $(
                pub struct $type_name;

                impl super::private::Sealed for $type_name {}
                impl super::Obj for $type_name {
                    const TYPE: crate::bindgen::mjtObj = crate::bindgen::mjtObj::$name;
                }
            )*

            pub(super) fn name<O: super::Obj>() -> &'static str {
                match O::TYPE {
                    $(crate::bindgen::mjtObj::$name => stringify!($type_name),)*
                    _ => "Unknown",
                }
            }
        }
    };
}
obj_as_types! {
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

pub struct ObjectId<O: Obj> {
    index: usize,
    _type: std::marker::PhantomData<O>,
}

impl<O: Obj> ObjectId<O> {
    pub(crate) fn new(index: usize) -> Self {
        Self {
            index,
            _type: std::marker::PhantomData,
        }
    }
}

// **NOT** implementing `DerefMut` to prevent accidental mutation of the index.
// `ObjectId` is intended to be generated **only by** this crate, not by users.
impl<O: Obj> std::ops::Deref for ObjectId<O> {
    type Target = usize;

    fn deref(&self) -> &Self::Target {
        &self.index
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
