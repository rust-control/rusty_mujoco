pub use crate::bindgen::{mjtObj, mjtWrap, mjtJoint};

pub trait Element: Obj + element::Sealed {}
mod element {
    pub trait Sealed {}
    impl<S: super::Obj + Sealed> super::Element for S {}
    impl Sealed for super::obj::Flex {}
    impl Sealed for super::obj::Mesh {}
    impl Sealed for super::obj::Skin {}
}

macro_rules! element_ids {
    ($( $data:ident ),* $(,)?) => {
        $(
            #[derive(Debug, Eq, PartialEq, Ord, PartialOrd)]
            pub struct $data<E: Element> {
                index: usize,
                _type: std::marker::PhantomData<E>,
            }

            impl<E: Element> Clone for $data<E> {
                fn clone(&self) -> Self {
                    Self { index: self.index, _type: std::marker::PhantomData }
                }
            }
            impl<E: Element> Copy for $data<E> {}

            impl<E: Element> $data<E> {
                pub fn index(&self) -> usize {
                    self.index
                }
                /// SAFETY: This function should only be used when you are sure that
                /// the index is valid for the type.
                pub unsafe fn new_unchecked(index: usize) -> Self {
                    Self { index, _type: std::marker::PhantomData }
                }
            }
        )*
    };
}
element_ids! {
    ElementId,
    VertexId,
    NodeId,
    EdgeId,
    TexcoordId,
    FaceId,
}

/// id in flattened all-elements (' vertexes) array
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct ElementDataId(usize);
impl ElementDataId {
    pub fn index(&self) -> usize {
        self.0
    }
    /// SAFETY: This function should only be used when you are sure that
    /// the index is valid for a element.
    pub unsafe fn new_unchecked(index: usize) -> Self {
        Self(index)
    }
}

/// id in flattened all-edges array
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct EdgeDataId(usize);
impl EdgeDataId {
    pub fn index(&self) -> usize {
        self.0
    }
    /// SAFETY: This function should only be used when you are sure that
    /// the index is valid for a edge.
    pub unsafe fn new_unchecked(index: usize) -> Self {
        Self(index)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct ShellDataId(usize);
impl ShellDataId {
    pub fn index(&self) -> usize {
        self.0
    }
    /// SAFETY: This function should only be used when you are sure that
    /// the index is valid for a shell.
    pub unsafe fn new_unchecked(index: usize) -> Self {
        Self(index)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct EvPairId(usize);
impl EvPairId {
    pub fn index(&self) -> usize {
        self.0
    }
    /// SAFETY: This function should only be used when you are sure that
    /// the index is valid for an edge-vertex pair.
    pub unsafe fn new_unchecked(index: usize) -> Self {
        Self(index)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct NormalId(usize);
impl NormalId {
    pub fn index(&self) -> usize {
        self.0
    }
    /// SAFETY: This function should only be used when you are sure that
    /// the index is valid for an edge-vertex pair.
    pub unsafe fn new_unchecked(index: usize) -> Self {
        Self(index)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct BoneId(usize);
impl BoneId {
    pub fn index(&self) -> usize {
        self.0
    }
    /// SAFETY: This function should only be used when you are sure that
    /// the index is valid for an edge-vertex pair.
    pub unsafe fn new_unchecked(index: usize) -> Self {
        Self(index)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct BoneVertexId(usize);
impl BoneVertexId {
    pub fn index(&self) -> usize {
        self.0
    }
    /// SAFETY: This function should only be used when you are sure that
    /// the index is valid for an edge-vertex pair.
    pub unsafe fn new_unchecked(index: usize) -> Self {
        Self(index)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct HFieldDataId(usize);
impl HFieldDataId {
    pub fn index(&self) -> usize {
        self.0
    }
    /// SAFETY: This function should only be used when you are sure that
    /// the index is valid for an edge-vertex pair.
    pub unsafe fn new_unchecked(index: usize) -> Self {
        Self(index)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct TexDataId(usize);
impl TexDataId {
    pub fn index(&self) -> usize {
        self.0
    }
    /// SAFETY: This function should only be used when you are sure that
    /// the index is valid for an edge-vertex pair.
    pub unsafe fn new_unchecked(index: usize) -> Self {
        Self(index)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct SegmentationId(usize);
impl SegmentationId {
    pub fn index(&self) -> usize {
        self.0
    }

    /// SAFETY: This function should only be used when you are sure that
    /// the index is valid for a segmentation id.
    pub unsafe fn new_unchecked(index: usize) -> Self {
        Self(index)
    }

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

mod private {
    pub trait Sealed {}
}

pub trait Obj: Sized + private::Sealed {
    const TYPE: mjtObj;
    fn object_id(model: &crate::mjModel, name: &str) -> Option<ObjectId<Self>>;
}

pub struct ObjectId<O: Obj> {
    index: usize,
    _type: std::marker::PhantomData<O>,
}
impl<O: Obj> ObjectId<O> {
    pub fn index(&self) -> usize {
        self.index
    }

    pub fn type_(&self) -> mjtObj {
        O::TYPE
    }

    /// SAFETY: This function should only be used when you are sure that the index is valid for the object type `O: Obj`.
    pub unsafe fn new_unchecked(index: usize) -> Self {
        Self {
            index,
            _type: std::marker::PhantomData,
        }
    }
}

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
                    fn object_id(model: &crate::mjModel, name: &str) -> Option<super::ObjectId<Self>> {
                        crate::mj_name2id::<Self>(model, name)
                    }
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

/* TODO:
remove `Qpos`, `Qvel` and use `[f64; J::QPOS_SIZE]`, `[f64; J::QVEL_SIZE]`
when `generic_const_exprs` language feature is stabilzed */
pub trait Joint: Obj {
    const MJT: mjtJoint;
    const QPOS_SIZE: usize;
    const QVEL_SIZE: usize;
    type Qpos: for<'q> TryFrom<&'q [f64]> + AsRef<[f64]>;
    type Qvel: for<'q> TryFrom<&'q [f64]> + AsRef<[f64]>;
}
pub mod joint {
    use super::{obj, Obj, Joint};
    
    pub struct Free;
    impl super::private::Sealed for Free {}
    impl Obj for Free {
        const TYPE: super::mjtObj = super::mjtObj::JOINT;
        fn object_id(model: &crate::mjModel, name: &str) -> Option<super::ObjectId<Self>> {
            let o = obj::Joint::object_id(model, name)?;
            if model.jnt_type(o) != Self::MJT {return None}
            Some(unsafe { super::ObjectId::<Self>::new_unchecked(o.index) })
        }
    }
    impl super::Joint for Free {
        const MJT: super::mjtJoint = super::mjtJoint::FREE;
        const QPOS_SIZE: usize = 7; // x, y, z, q, qw, qx, qy, qz
        const QVEL_SIZE: usize = 6; // vx, vy, vz, wx, wy, wz
        type Qpos = [f64; Self::QPOS_SIZE];
        type Qvel = [f64; Self::QVEL_SIZE];
    }

    pub struct Ball;
    impl super::private::Sealed for Ball {}
    impl Obj for Ball {
        const TYPE: super::mjtObj = super::mjtObj::JOINT;
        fn object_id(model: &crate::mjModel, name: &str) -> Option<super::ObjectId<Self>> {
            let o = obj::Joint::object_id(model, name)?;
            if model.jnt_type(o) != Self::MJT {return None}
            Some(unsafe { super::ObjectId::<Self>::new_unchecked(o.index) })
        }
    }
    impl super::Joint for Ball {
        const MJT: super::mjtJoint = super::mjtJoint::BALL;
        const QPOS_SIZE: usize = 4; // qw, qx, qy, qz
        const QVEL_SIZE: usize = 3; // ωx, ωy, ωz
        type Qpos = [f64; Self::QPOS_SIZE];
        type Qvel = [f64; Self::QVEL_SIZE];
    }

    pub struct Hinge;
    impl super::private::Sealed for Hinge {}
    impl Obj for Hinge {
        const TYPE: super::mjtObj = super::mjtObj::JOINT;
        fn object_id(model: &crate::mjModel, name: &str) -> Option<super::ObjectId<Self>> {
            let o = obj::Joint::object_id(model, name)?;
            if model.jnt_type(o) != Self::MJT {return None}
            Some(unsafe { super::ObjectId::<Self>::new_unchecked(o.index) })
        }
    }
    impl super::Joint for Hinge {
        const MJT: super::mjtJoint = super::mjtJoint::HINGE;
        const QPOS_SIZE: usize = 1; // angle [rad]
        const QVEL_SIZE: usize = 1; // angular_velocity
        type Qpos = [f64; Self::QPOS_SIZE];
        type Qvel = [f64; Self::QVEL_SIZE];
    }

    pub struct Slide;
    impl super::private::Sealed for Slide {}
    impl Obj for Slide {
        const TYPE: super::mjtObj = super::mjtObj::JOINT;
        fn object_id(model: &crate::mjModel, name: &str) -> Option<super::ObjectId<Self>> {
            let o = obj::Joint::object_id(model, name)?;
            if model.jnt_type(o) != Self::MJT {return None}
            Some(unsafe { super::ObjectId::<Self>::new_unchecked(o.index) })
        }
    }
    impl super::Joint for Slide {
        const MJT: super::mjtJoint = super::mjtJoint::SLIDE;
        const QPOS_SIZE: usize = 1; // position [m]
        const QVEL_SIZE: usize = 1; // linear_velocity
        type Qpos = [f64; Self::QPOS_SIZE];
        type Qvel = [f64; Self::QVEL_SIZE];
    }
}

impl<J: Joint> Into<ObjectId<obj::Joint>> for ObjectId<J> {
    fn into(self) -> ObjectId<obj::Joint> {
        unsafe { ObjectId::new_unchecked(self.index) }
    }
}

/*
 * TODO: use `Wrap` and `WrapObjectId` ?
 */
pub trait Wrap: private::Sealed { const TYPE: mjtWrap; }
pub mod wrap {
    pub struct Joint;
    impl super::private::Sealed for Joint {}
    impl super::Wrap for Joint { const TYPE: super::mjtWrap = super::mjtWrap::JOINT; }

    pub struct Pulley;
    impl super::private::Sealed for Pulley {}
    impl super::Wrap for Pulley { const TYPE: super::mjtWrap = super::mjtWrap::PULLEY; }

    pub struct Site;
    impl super::private::Sealed for Site {}
    impl super::Wrap for Site { const TYPE: super::mjtWrap = super::mjtWrap::SITE; }

    pub struct Sphere;
    impl super::private::Sealed for Sphere {}
    impl super::Wrap for Sphere { const TYPE: super::mjtWrap = super::mjtWrap::SPHERE; }

    pub struct Cylinder;
    impl super::private::Sealed for Cylinder {}
    impl super::Wrap for Cylinder { const TYPE: super::mjtWrap = super::mjtWrap::CYLINDER; }
}
pub struct WrapObjectId<W: Wrap> {
    index: usize,
    _type: std::marker::PhantomData<W>,
}
impl<W: Wrap> WrapObjectId<W> {
    pub fn index(&self) -> usize {
        self.index
    }

    pub fn type_(&self) -> mjtWrap {
        W::TYPE
    }

    /// SAFETY: This function should only be used when you are sure that the index is valid for the wrap type `W: Wrap`.
    pub unsafe fn new_unchecked(index: usize) -> Self {
        Self {
            index,
            _type: std::marker::PhantomData,
        }
    }
}
