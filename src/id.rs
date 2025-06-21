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
    ($($name:ident as $type_name:ident ($id_bytes:literal)),* $(,)?) => {
        #[derive(Clone, Copy, Debug)]
        pub enum ObjType {
            $(
                #[allow(non_camel_case_types)]
                $type_name = crate::bindgen::mjtObj::$name as isize,
            )*
        }
        impl ObjType {
            /// Rusty-[`mju_type2str`](https://github.com/google-deepmind/mujoco/blob/baf84265b8627e6f868bc92ea6422e4e78dacb9c/src/engine/engine_util_misc.c#L1030)
            pub const fn to_str(&self) -> &'static str {
                match self {
                    $(Self::$type_name => unsafe {std::str::from_utf8_unchecked($id_bytes)},)*
                }
            }
            /// Rusty-[`mju_str2type`](https://github.com/google-deepmind/mujoco/blob/baf84265b8627e6f868bc92ea6422e4e78dacb9c/src/engine/engine_util_misc.c#L1118)
            pub const fn from_str(s: &str) -> Self {
                match s.as_bytes() {
                    $(
                        $id_bytes => Self::$type_name,
                    )*
                    _ => Self::Unknown,
                }
            }
        }

        pub mod obj {
            $(
                pub struct $type_name;

                impl super::private::Sealed for $type_name {}
                impl super::Obj for $type_name {
                    const TYPE: super::ObjType = super::ObjType::$type_name;
                }
            )*

            pub(crate) fn display_name<O: super::Obj>() -> &'static str {
                match O::TYPE {
                    $(super::ObjType::$type_name => stringify!($type_name),)*
                }
            }
        }
    };
}
obj_types! {
    mjOBJ_UNKNOWN as Unknown(b"unknown"),
    mjOBJ_BODY as Body(b"body"),
    mjOBJ_XBODY as XBody(b"xbody"),
    mjOBJ_JOINT as Joint(b"joint"),
    mjOBJ_DOF as Dof(b"dof"),
    mjOBJ_GEOM as Geom(b"geom"),
    mjOBJ_SITE as Site(b"site"),
    mjOBJ_CAMERA as Camera(b"camera"),
    mjOBJ_LIGHT as Light(b"light"),
    mjOBJ_FLEX as Flex(b"flex"),
    mjOBJ_MESH as Mesh(b"mesh"),
    mjOBJ_SKIN as Skin(b"skin"),
    mjOBJ_HFIELD as HField(b"hfield"),
    mjOBJ_TEXTURE as Texture(b"texture"),
    mjOBJ_MATERIAL as Material(b"material"),
    mjOBJ_PAIR as Pair(b"pair"),
    mjOBJ_EXCLUDE as Exclude(b"exclude"),
    mjOBJ_EQUALITY as Equality(b"equality"),
    mjOBJ_TENDON as Tendon(b"tendon"),
    mjOBJ_ACTUATOR as Actuator(b"actuator"),
    mjOBJ_SENSOR as Sensor(b"sensor"),
    mjOBJ_NUMERIC as Numeric(b"numeric"),
    mjOBJ_TEXT as Text(b"text"),
    mjOBJ_TUPLE as Tuple(b"tuple"),
    mjOBJ_KEY as Key(b"key"),
    mjOBJ_PLUGIN as Plugin(b"plugin"),
    mjOBJ_FRAME as Frame(b"frame"),
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
        write!(f, "ObjectId<{}>({})", obj::display_name::<O>(), self.index)
    }
}
impl<O: Obj> std::fmt::Display for ObjectId<O> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ObjectId<{}>({})", obj::display_name::<O>(), self.index)
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
