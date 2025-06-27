pub use crate::bindgen::mjtObj;

/// element id, used for flex and mesh objects
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct ElementId(pub(crate) usize);

/// vertex id, used for flex and mesh objects
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct VertexId(pub(crate) usize);

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct SegmentationId(pub(crate) usize);
impl SegmentationId {
    /// convert `SegmentationId` to corresponding object type and index
    pub fn to_object_info(&self) -> (mjtObj, usize) {
        /*
            We know:

                segid = (objtype << 16) | objid

            Then, we'll restore `objtype` and `objid` from `segid`:

                objtype = segid >> 16
                objid = segid & 0xFFFF
        */

        let objtype = self.0 >> 16;
        let objid = self.0 & 0xFFFF;

        (
            mjtObj(objtype as u32),
            objid,
        )
    }

    /// try to convert `SegmentationId` to `ObjectId<O>`
    pub fn to_object_id<O: Obj>(&self) -> Option<ObjectId<O>> {
        let (objtype, objid) = self.to_object_info();
        if objtype == O::TYPE {
            Some(unsafe {ObjectId::new_unchecked(objid)})
        } else {
            None
        }
    }
}

pub struct ObjectId<O: Obj> {
    pub(crate) index: usize,
    _type: std::marker::PhantomData<O>,
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

pub trait Obj: private::Sealed { const TYPE: mjtObj; }

mod private { pub trait Sealed {} }

macro_rules! obj_types {
    ($($name:ident as $type_name:ident ($id_bytes:literal)),* $(,)?) => {
        impl mjtObj {
            /// Rusty-[`mju_type2str`](https://github.com/google-deepmind/mujoco/blob/baf84265b8627e6f868bc92ea6422e4e78dacb9c/src/engine/engine_util_misc.c#L1030)
            pub const fn to_str(&self) -> &'static str {
                match *self {
                    $(Self::$name => unsafe {std::str::from_utf8_unchecked($id_bytes)},)*
                    _ => Self::UNKNOWN.to_str(),
                }
            }
            /// Rusty-[`mju_str2type`](https://github.com/google-deepmind/mujoco/blob/baf84265b8627e6f868bc92ea6422e4e78dacb9c/src/engine/engine_util_misc.c#L1118)
            pub const fn from_str(s: &str) -> Self {
                match s.as_bytes() {
                    $(
                        $id_bytes => Self::$name,
                    )*
                    _ => Self::UNKNOWN,
                }
            }
        }

        pub mod obj {
            $(
                pub struct $type_name;

                impl super::private::Sealed for $type_name {}
                impl super::Obj for $type_name {
                    const TYPE: super::mjtObj = super::mjtObj::$name;
                }
            )*

            pub(crate) fn display_name<O: super::Obj>() -> &'static str {
                match O::TYPE {
                    $(super::mjtObj::$name => stringify!($type_name),)*
                    _ => "Unknown",
                }
            }
        }
    };
}
obj_types! {
    UNKNOWN as Unknown(b"unknown"),
    BODY as Body(b"body"),
    XBODY as XBody(b"xbody"),
    JOINT as Joint(b"joint"),
    DOF as Dof(b"dof"),
    GEOM as Geom(b"geom"),
    SITE as Site(b"site"),
    CAMERA as Camera(b"camera"),
    LIGHT as Light(b"light"),
    FLEX as Flex(b"flex"),
    MESH as Mesh(b"mesh"),
    SKIN as Skin(b"skin"),
    HFIELD as HField(b"hfield"),
    TEXTURE as Texture(b"texture"),
    MATERIAL as Material(b"material"),
    PAIR as Pair(b"pair"),
    EXCLUDE as Exclude(b"exclude"),
    EQUALITY as Equality(b"equality"),
    TENDON as Tendon(b"tendon"),
    ACTUATOR as Actuator(b"actuator"),
    SENSOR as Sensor(b"sensor"),
    NUMERIC as Numeric(b"numeric"),
    TEXT as Text(b"text"),
    TUPLE as Tuple(b"tuple"),
    KEY as Key(b"key"),
    PLUGIN as Plugin(b"plugin"),
    FRAME as Frame(b"frame"),
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
