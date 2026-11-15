#![allow(
    unused,
    non_camel_case_types,
    non_snake_case,
    non_upper_case_globals,
    clippy::all
)]
pub const mjVERSION_HEADER: u32 = 3011000;
pub const mjMINVAL: f64 = 0.000000000000001;
pub const mjPI: f64 = 3.141592653589793;
pub const mjMAXVAL: f64 = 10000000000.0;
pub const mjMINMU: f64 = 0.00001;
pub const mjMINIMP: f64 = 0.0001;
pub const mjMAXIMP: f64 = 0.9999;
pub const mjMAXCONPAIR: usize = 50;
pub const mjMAXTREEDEPTH: usize = 50;
pub const mjMAXFLEXNODES: usize = 27;
pub const mjMINAWAKE: u32 = 10;
pub const mjNEQDATA: usize = 11;
pub const mjNDYN: usize = 10;
pub const mjNGAIN: usize = 10;
pub const mjNBIAS: usize = 10;
pub const mjNFLUID: usize = 12;
pub const mjNREF: usize = 2;
pub const mjNIMP: usize = 5;
pub const mjNPOLY: usize = 2;
pub const mjNSENS: usize = 3;
pub const mjNSOLVER: usize = 200;
pub const mjNISLAND: usize = 20;
pub const mjNGROUP: usize = 6;
pub const mjMAXLIGHT: usize = 100;
pub const mjMAXOVERLAY: usize = 500;
pub const mjMAXLINE: usize = 100;
pub const mjMAXLINEPNT: usize = 1001;
pub const mjMAXPLANEGRID: usize = 200;
pub const mjNAUX: usize = 10;
pub const mjMAXTEXTURE: usize = 1000;
pub const mjMAXMATERIAL: usize = 1000;
pub const mjMAXUISECT: usize = 10;
pub const mjMAXUIITEM: usize = 200;
pub const mjMAXUITEXT: usize = 300;
pub const mjMAXUINAME: usize = 40;
pub const mjMAXUIMULTI: usize = 35;
pub const mjMAXUIEDIT: usize = 7;
pub const mjMAXUIRECT: usize = 25;
pub const mjSEPCLOSED: u32 = 1000;
pub const mjPRESERVE: u32 = 2000;
pub const mjKEY_ESCAPE: u32 = 256;
pub const mjKEY_ENTER: u32 = 257;
pub const mjKEY_TAB: u32 = 258;
pub const mjKEY_BACKSPACE: u32 = 259;
pub const mjKEY_INSERT: u32 = 260;
pub const mjKEY_DELETE: u32 = 261;
pub const mjKEY_RIGHT: u32 = 262;
pub const mjKEY_LEFT: u32 = 263;
pub const mjKEY_DOWN: u32 = 264;
pub const mjKEY_UP: u32 = 265;
pub const mjKEY_PAGE_UP: u32 = 266;
pub const mjKEY_PAGE_DOWN: u32 = 267;
pub const mjKEY_HOME: u32 = 268;
pub const mjKEY_END: u32 = 269;
pub const mjKEY_F1: u32 = 290;
pub const mjKEY_F2: u32 = 291;
pub const mjKEY_F3: u32 = 292;
pub const mjKEY_F4: u32 = 293;
pub const mjKEY_F5: u32 = 294;
pub const mjKEY_F6: u32 = 295;
pub const mjKEY_F7: u32 = 296;
pub const mjKEY_F8: u32 = 297;
pub const mjKEY_F9: u32 = 298;
pub const mjKEY_F10: u32 = 299;
pub const mjKEY_F11: u32 = 300;
pub const mjKEY_F12: u32 = 301;
pub const mjKEY_NUMPAD_0: u32 = 320;
pub const mjKEY_NUMPAD_9: u32 = 329;
pub type mjtNum = f64;
#[doc = "---------------------------------- byte definition -----------------------------------------------"]
pub type mjtByte = ::core::ffi::c_uchar;
pub type mjtBool = bool;
#[doc = "---------------------------------- size definition -----------------------------------------------"]
pub type mjtSize = i64;
pub const mjNDISABLE: usize = mjtDisableBit::mjNDISABLE.0 as usize;
impl mjtDisableBit {
    #[doc = " entire constraint solver"]
    pub const CONSTRAINT: mjtDisableBit = mjtDisableBit(1);
    #[doc = " equality constraints"]
    pub const EQUALITY: mjtDisableBit = mjtDisableBit(2);
    #[doc = " joint and tendon frictionloss constraints"]
    pub const FRICTIONLOSS: mjtDisableBit = mjtDisableBit(4);
    #[doc = " joint and tendon limit constraints"]
    pub const LIMIT: mjtDisableBit = mjtDisableBit(8);
    #[doc = " contact constraints"]
    pub const CONTACT: mjtDisableBit = mjtDisableBit(16);
    #[doc = " passive spring forces"]
    pub const SPRING: mjtDisableBit = mjtDisableBit(32);
    #[doc = " passive damping forces"]
    pub const DAMPER: mjtDisableBit = mjtDisableBit(64);
    #[doc = " gravitational forces"]
    pub const GRAVITY: mjtDisableBit = mjtDisableBit(128);
    #[doc = " clamp control to specified range"]
    pub const CLAMPCTRL: mjtDisableBit = mjtDisableBit(256);
    #[doc = " warmstart constraint solver"]
    pub const WARMSTART: mjtDisableBit = mjtDisableBit(512);
    #[doc = " remove collisions with parent body"]
    pub const FILTERPARENT: mjtDisableBit = mjtDisableBit(1024);
    #[doc = " apply actuation forces"]
    pub const ACTUATION: mjtDisableBit = mjtDisableBit(2048);
    #[doc = " integrator safety: make ref[0]>=2*timestep"]
    pub const REFSAFE: mjtDisableBit = mjtDisableBit(4096);
    #[doc = " sensors"]
    pub const SENSOR: mjtDisableBit = mjtDisableBit(8192);
    #[doc = " mid-phase collision filtering"]
    pub const MIDPHASE: mjtDisableBit = mjtDisableBit(16384);
    #[doc = " implicit integration of joint damping in Euler integrator"]
    pub const EULERDAMP: mjtDisableBit = mjtDisableBit(32768);
    #[doc = " automatic reset when numerical issues are detected"]
    pub const AUTORESET: mjtDisableBit = mjtDisableBit(65536);
    #[doc = " native convex collision detection"]
    pub const NATIVECCD: mjtDisableBit = mjtDisableBit(131072);
    #[doc = " constraint island discovery"]
    pub const ISLAND: mjtDisableBit = mjtDisableBit(262144);
    #[doc = " multiple CCD contact points"]
    pub const MULTICCD: mjtDisableBit = mjtDisableBit(524288);
    #[doc = " number of disable flags"]
    const mjNDISABLE: mjtDisableBit = mjtDisableBit(20);
}
impl ::core::ops::BitOr<mjtDisableBit> for mjtDisableBit {
    type Output = Self;
    #[inline]
    fn bitor(self, other: Self) -> Self {
        mjtDisableBit(self.0 | other.0)
    }
}
impl ::core::ops::BitOrAssign for mjtDisableBit {
    #[inline]
    fn bitor_assign(&mut self, rhs: mjtDisableBit) {
        self.0 |= rhs.0;
    }
}
impl ::core::ops::BitAnd<mjtDisableBit> for mjtDisableBit {
    type Output = Self;
    #[inline]
    fn bitand(self, other: Self) -> Self {
        mjtDisableBit(self.0 & other.0)
    }
}
impl ::core::ops::BitAndAssign for mjtDisableBit {
    #[inline]
    fn bitand_assign(&mut self, rhs: mjtDisableBit) {
        self.0 &= rhs.0;
    }
}
#[repr(transparent)]
#[doc = "---------------------------------- enum types (mjModel) ------------------------------------------"]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtDisableBit(pub(crate) ::core::ffi::c_uint);
pub const mjNENABLE: usize = mjtEnableBit::mjNENABLE.0 as usize;
impl mjtEnableBit {
    #[doc = " override contact parameters"]
    pub const OVERRIDE: mjtEnableBit = mjtEnableBit(1);
    #[doc = " energy computation"]
    pub const ENERGY: mjtEnableBit = mjtEnableBit(2);
    #[doc = " record solver statistics"]
    pub const FWDINV: mjtEnableBit = mjtEnableBit(4);
    #[doc = " discrete-time inverse dynamics"]
    pub const INVDISCRETE: mjtEnableBit = mjtEnableBit(8);
    #[doc = " sleeping"]
    pub const SLEEP: mjtEnableBit = mjtEnableBit(16);
    #[doc = " exact diagonal of constraint inertia"]
    pub const DIAGEXACT: mjtEnableBit = mjtEnableBit(32);
    #[doc = " number of enable flags"]
    const mjNENABLE: mjtEnableBit = mjtEnableBit(6);
}
impl ::core::ops::BitOr<mjtEnableBit> for mjtEnableBit {
    type Output = Self;
    #[inline]
    fn bitor(self, other: Self) -> Self {
        mjtEnableBit(self.0 | other.0)
    }
}
impl ::core::ops::BitOrAssign for mjtEnableBit {
    #[inline]
    fn bitor_assign(&mut self, rhs: mjtEnableBit) {
        self.0 |= rhs.0;
    }
}
impl ::core::ops::BitAnd<mjtEnableBit> for mjtEnableBit {
    type Output = Self;
    #[inline]
    fn bitand(self, other: Self) -> Self {
        mjtEnableBit(self.0 & other.0)
    }
}
impl ::core::ops::BitAndAssign for mjtEnableBit {
    #[inline]
    fn bitand_assign(&mut self, rhs: mjtEnableBit) {
        self.0 &= rhs.0;
    }
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtEnableBit(pub(crate) ::core::ffi::c_uint);
impl mjtJoint {
    #[doc = " global position and orientation (quat)       (7)"]
    pub const FREE: mjtJoint = mjtJoint(0);
    #[doc = " orientation (quat) relative to parent        (4)"]
    pub const BALL: mjtJoint = mjtJoint(1);
    #[doc = " sliding distance along body-fixed axis       (1)"]
    pub const SLIDE: mjtJoint = mjtJoint(2);
    #[doc = " rotation angle (rad) around body-fixed axis  (1)"]
    pub const HINGE: mjtJoint = mjtJoint(3);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtJoint(pub(crate) ::core::ffi::c_uint);
pub const mjNGEOMTYPES: usize = mjtGeom::mjNGEOMTYPES.0 as usize;
impl mjtGeom {
    #[doc = " plane"]
    pub const PLANE: mjtGeom = mjtGeom(0);
    #[doc = " height field"]
    pub const HFIELD: mjtGeom = mjtGeom(1);
    #[doc = " sphere"]
    pub const SPHERE: mjtGeom = mjtGeom(2);
    #[doc = " capsule"]
    pub const CAPSULE: mjtGeom = mjtGeom(3);
    #[doc = " ellipsoid"]
    pub const ELLIPSOID: mjtGeom = mjtGeom(4);
    #[doc = " cylinder"]
    pub const CYLINDER: mjtGeom = mjtGeom(5);
    #[doc = " box"]
    pub const BOX: mjtGeom = mjtGeom(6);
    #[doc = " mesh"]
    pub const MESH: mjtGeom = mjtGeom(7);
    #[doc = " signed distance field"]
    pub const SDF: mjtGeom = mjtGeom(8);
    #[doc = " arrow"]
    pub const ARROW: mjtGeom = mjtGeom(100);
    #[doc = " arrow without wedges"]
    pub const ARROW1: mjtGeom = mjtGeom(101);
    #[doc = " arrow in both directions"]
    pub const ARROW2: mjtGeom = mjtGeom(102);
    #[doc = " line"]
    pub const LINE: mjtGeom = mjtGeom(103);
    #[doc = " box with line edges"]
    pub const LINEBOX: mjtGeom = mjtGeom(104);
    #[doc = " flex"]
    pub const FLEX: mjtGeom = mjtGeom(105);
    #[doc = " skin"]
    pub const SKIN: mjtGeom = mjtGeom(106);
    #[doc = " text label"]
    pub const LABEL: mjtGeom = mjtGeom(107);
    #[doc = " triangle"]
    pub const TRIANGLE: mjtGeom = mjtGeom(108);
    #[doc = " missing geom type"]
    pub const NONE: mjtGeom = mjtGeom(1001);
    #[doc = " number of regular geom types"]
    const mjNGEOMTYPES: mjtGeom = mjtGeom(9);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtGeom(pub(crate) ::core::ffi::c_uint);
impl mjtProjection {
    #[doc = " perspective"]
    pub const PERSPECTIVE: mjtProjection = mjtProjection(0);
    #[doc = " orthographic"]
    pub const ORTHOGRAPHIC: mjtProjection = mjtProjection(1);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtProjection(pub(crate) ::core::ffi::c_uint);
impl mjtCamLight {
    #[doc = " pos and rot fixed in body"]
    pub const FIXED: mjtCamLight = mjtCamLight(0);
    #[doc = " pos tracks body, rot fixed in global"]
    pub const TRACK: mjtCamLight = mjtCamLight(1);
    #[doc = " pos tracks subtree com, rot fixed in body"]
    pub const TRACKCOM: mjtCamLight = mjtCamLight(2);
    #[doc = " pos fixed in body, rot tracks target body"]
    pub const TARGETBODY: mjtCamLight = mjtCamLight(3);
    #[doc = " pos fixed in body, rot tracks target subtree com"]
    pub const TARGETBODYCOM: mjtCamLight = mjtCamLight(4);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtCamLight(pub(crate) ::core::ffi::c_uint);
impl mjtLightType {
    #[doc = " spot"]
    pub const SPOT: mjtLightType = mjtLightType(0);
    #[doc = " directional"]
    pub const DIRECTIONAL: mjtLightType = mjtLightType(1);
    #[doc = " point"]
    pub const POINT: mjtLightType = mjtLightType(2);
    #[doc = " image-based"]
    pub const IMAGE: mjtLightType = mjtLightType(3);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtLightType(pub(crate) ::core::ffi::c_uint);
impl mjtTexture {
    #[doc = " 2d texture, suitable for planes and hfields"]
    pub const D2: mjtTexture = mjtTexture(0);
    #[doc = " cube texture, suitable for all other geom types"]
    pub const CUBE: mjtTexture = mjtTexture(1);
    #[doc = " cube texture used as skybox"]
    pub const SKYBOX: mjtTexture = mjtTexture(2);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtTexture(pub(crate) ::core::ffi::c_uint);
pub const mjNTEXROLE: usize = mjtTextureRole::mjNTEXROLE.0 as usize;
impl mjtTextureRole {
    #[doc = " unspecified"]
    pub const USER: mjtTextureRole = mjtTextureRole(0);
    #[doc = " base color (albedo)"]
    pub const RGB: mjtTextureRole = mjtTextureRole(1);
    #[doc = " ambient occlusion"]
    pub const OCCLUSION: mjtTextureRole = mjtTextureRole(2);
    #[doc = " roughness"]
    pub const ROUGHNESS: mjtTextureRole = mjtTextureRole(3);
    #[doc = " metallic"]
    pub const METALLIC: mjtTextureRole = mjtTextureRole(4);
    #[doc = " normal (bump) map"]
    pub const NORMAL: mjtTextureRole = mjtTextureRole(5);
    #[doc = " opacity"]
    pub const OPACITY: mjtTextureRole = mjtTextureRole(6);
    #[doc = " light emission"]
    pub const EMISSIVE: mjtTextureRole = mjtTextureRole(7);
    #[doc = " base color, opacity"]
    pub const RGBA: mjtTextureRole = mjtTextureRole(8);
    #[doc = " occlusion, roughness, metallic"]
    pub const ORM: mjtTextureRole = mjtTextureRole(9);
    const mjNTEXROLE: mjtTextureRole = mjtTextureRole(10);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtTextureRole(pub(crate) ::core::ffi::c_uint);
impl mjtColorSpace {
    #[doc = " attempts to autodetect color space, defaults to linear"]
    pub const AUTO: mjtColorSpace = mjtColorSpace(0);
    #[doc = " linear color space"]
    pub const LINEAR: mjtColorSpace = mjtColorSpace(1);
    #[doc = " standard RGB color space"]
    pub const SRGB: mjtColorSpace = mjtColorSpace(2);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtColorSpace(pub(crate) ::core::ffi::c_uint);
impl mjtIntegrator {
    #[doc = " semi-implicit Euler"]
    pub const EULER: mjtIntegrator = mjtIntegrator(0);
    #[doc = " 4th-order Runge Kutta"]
    pub const RK4: mjtIntegrator = mjtIntegrator(1);
    #[doc = " implicit in velocity"]
    pub const IMPLICIT: mjtIntegrator = mjtIntegrator(2);
    #[doc = " implicit in velocity, no rne derivative"]
    pub const IMPLICITFAST: mjtIntegrator = mjtIntegrator(3);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtIntegrator(pub(crate) ::core::ffi::c_uint);
impl mjtCone {
    #[doc = " pyramidal"]
    pub const PYRAMIDAL: mjtCone = mjtCone(0);
    #[doc = " elliptic"]
    pub const ELLIPTIC: mjtCone = mjtCone(1);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtCone(pub(crate) ::core::ffi::c_uint);
impl mjtJacobian {
    #[doc = " dense"]
    pub const DENSE: mjtJacobian = mjtJacobian(0);
    #[doc = " sparse"]
    pub const SPARSE: mjtJacobian = mjtJacobian(1);
    #[doc = " dense if nv<60, sparse otherwise"]
    pub const AUTO: mjtJacobian = mjtJacobian(2);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtJacobian(pub(crate) ::core::ffi::c_uint);
impl mjtSolver {
    #[doc = " PGS    (dual)"]
    pub const PGS: mjtSolver = mjtSolver(0);
    #[doc = " CG     (primal)"]
    pub const CG: mjtSolver = mjtSolver(1);
    #[doc = " Newton (primal)"]
    pub const NEWTON: mjtSolver = mjtSolver(2);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtSolver(pub(crate) ::core::ffi::c_uint);
impl mjtEq {
    #[doc = " connect two bodies at a point (ball joint)"]
    pub const CONNECT: mjtEq = mjtEq(0);
    #[doc = " fix relative position and orientation of two bodies"]
    pub const WELD: mjtEq = mjtEq(1);
    #[doc = " couple the values of two scalar joints with cubic"]
    pub const JOINT: mjtEq = mjtEq(2);
    #[doc = " couple the lengths of two tendons with cubic"]
    pub const TENDON: mjtEq = mjtEq(3);
    #[doc = " fix all edge lengths of a flex"]
    pub const FLEX: mjtEq = mjtEq(4);
    #[doc = " fix all vertex lengths of a flex"]
    pub const FLEXVERT: mjtEq = mjtEq(5);
    #[doc = " constrain strain of a trilinear/quadratic flex (B-bar)"]
    pub const FLEXSTRAIN: mjtEq = mjtEq(6);
    #[doc = " unsupported, will cause an error if used"]
    pub const DISTANCE: mjtEq = mjtEq(7);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtEq(pub(crate) ::core::ffi::c_uint);
impl mjtWrap {
    #[doc = " null object"]
    pub const NONE: mjtWrap = mjtWrap(0);
    #[doc = " constant moment arm"]
    pub const JOINT: mjtWrap = mjtWrap(1);
    #[doc = " pulley used to split tendon"]
    pub const PULLEY: mjtWrap = mjtWrap(2);
    #[doc = " pass through site"]
    pub const SITE: mjtWrap = mjtWrap(3);
    #[doc = " wrap around sphere"]
    pub const SPHERE: mjtWrap = mjtWrap(4);
    #[doc = " wrap around (infinite) cylinder"]
    pub const CYLINDER: mjtWrap = mjtWrap(5);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtWrap(pub(crate) ::core::ffi::c_uint);
impl mjtTrn {
    #[doc = " force on joint"]
    pub const JOINT: mjtTrn = mjtTrn(0);
    #[doc = " force on joint, expressed in parent frame"]
    pub const JOINTINPARENT: mjtTrn = mjtTrn(1);
    #[doc = " force via slider-crank linkage"]
    pub const SLIDERCRANK: mjtTrn = mjtTrn(2);
    #[doc = " force on tendon"]
    pub const TENDON: mjtTrn = mjtTrn(3);
    #[doc = " force on site"]
    pub const SITE: mjtTrn = mjtTrn(4);
    #[doc = " adhesion force on a body's geoms"]
    pub const BODY: mjtTrn = mjtTrn(5);
    #[doc = " torque on a relative orientation (3 force outputs)"]
    pub const SO3: mjtTrn = mjtTrn(6);
    #[doc = " undefined transmission type"]
    pub const UNDEFINED: mjtTrn = mjtTrn(1000);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtTrn(pub(crate) ::core::ffi::c_uint);
impl mjtDyn {
    #[doc = " no internal dynamics; ctrl specifies force"]
    pub const NONE: mjtDyn = mjtDyn(0);
    #[doc = " integrator: da/dt = u"]
    pub const INTEGRATOR: mjtDyn = mjtDyn(1);
    #[doc = " linear filter: da/dt = (u-a) / tau"]
    pub const FILTER: mjtDyn = mjtDyn(2);
    #[doc = " linear filter: da/dt = (u-a) / tau, with exact integration"]
    pub const FILTEREXACT: mjtDyn = mjtDyn(3);
    #[doc = " piecewise linear filter with two time constants"]
    pub const MUSCLE: mjtDyn = mjtDyn(4);
    #[doc = " DC motor electrical dynamics"]
    pub const DCMOTOR: mjtDyn = mjtDyn(5);
    #[doc = " user-defined dynamics type"]
    pub const USER: mjtDyn = mjtDyn(6);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtDyn(pub(crate) ::core::ffi::c_uint);
impl mjtGain {
    #[doc = " fixed gain"]
    pub const FIXED: mjtGain = mjtGain(0);
    #[doc = " const + kp*length + kv*velocity"]
    pub const AFFINE: mjtGain = mjtGain(1);
    #[doc = " muscle FLV curve computed by mju_muscleGain()"]
    pub const MUSCLE: mjtGain = mjtGain(2);
    #[doc = " DC motor gain: K or K/R"]
    pub const DCMOTOR: mjtGain = mjtGain(3);
    #[doc = " geodesic servo on an SO3 transmission: force = kp * log(error)"]
    pub const SO3: mjtGain = mjtGain(4);
    #[doc = " user-defined gain type"]
    pub const USER: mjtGain = mjtGain(5);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtGain(pub(crate) ::core::ffi::c_uint);
impl mjtBias {
    #[doc = " no bias"]
    pub const NONE: mjtBias = mjtBias(0);
    #[doc = " const + kp*length + kv*velocity"]
    pub const AFFINE: mjtBias = mjtBias(1);
    #[doc = " muscle passive force computed by mju_muscleBias()"]
    pub const MUSCLE: mjtBias = mjtBias(2);
    #[doc = " DC motor bias: back-EMF, cogging, LuGre friction"]
    pub const DCMOTOR: mjtBias = mjtBias(3);
    #[doc = " damping term of the SO3 geodesic servo"]
    pub const SO3: mjtBias = mjtBias(4);
    #[doc = " user-defined bias type"]
    pub const USER: mjtBias = mjtBias(5);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtBias(pub(crate) ::core::ffi::c_uint);
impl mjtCtrlChart {
    #[doc = " exponential-map orientation target: 3 controls"]
    pub const EXPMAP: mjtCtrlChart = mjtCtrlChart(1);
    #[doc = " quaternion orientation target: 4 controls"]
    pub const QUAT: mjtCtrlChart = mjtCtrlChart(2);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtCtrlChart(pub(crate) ::core::ffi::c_uint);
pub const mjNOBJECT: usize = mjtObj::mjNOBJECT.0 as usize;
impl mjtObj {
    #[doc = " unknown object type"]
    pub const UNKNOWN: mjtObj = mjtObj(0);
    #[doc = " body"]
    pub const BODY: mjtObj = mjtObj(1);
    #[doc = " body, used to access regular frame instead of i-frame"]
    pub const XBODY: mjtObj = mjtObj(2);
    #[doc = " joint"]
    pub const JOINT: mjtObj = mjtObj(3);
    #[doc = " dof"]
    pub const DOF: mjtObj = mjtObj(4);
    #[doc = " geom"]
    pub const GEOM: mjtObj = mjtObj(5);
    #[doc = " site"]
    pub const SITE: mjtObj = mjtObj(6);
    #[doc = " camera"]
    pub const CAMERA: mjtObj = mjtObj(7);
    #[doc = " light"]
    pub const LIGHT: mjtObj = mjtObj(8);
    #[doc = " flex"]
    pub const FLEX: mjtObj = mjtObj(9);
    #[doc = " mesh"]
    pub const MESH: mjtObj = mjtObj(10);
    #[doc = " skin"]
    pub const SKIN: mjtObj = mjtObj(11);
    #[doc = " heightfield"]
    pub const HFIELD: mjtObj = mjtObj(12);
    #[doc = " texture"]
    pub const TEXTURE: mjtObj = mjtObj(13);
    #[doc = " material for rendering"]
    pub const MATERIAL: mjtObj = mjtObj(14);
    #[doc = " geom pair to include"]
    pub const PAIR: mjtObj = mjtObj(15);
    #[doc = " body pair to exclude"]
    pub const EXCLUDE: mjtObj = mjtObj(16);
    #[doc = " equality constraint"]
    pub const EQUALITY: mjtObj = mjtObj(17);
    #[doc = " tendon"]
    pub const TENDON: mjtObj = mjtObj(18);
    #[doc = " actuator"]
    pub const ACTUATOR: mjtObj = mjtObj(19);
    #[doc = " sensor"]
    pub const SENSOR: mjtObj = mjtObj(20);
    #[doc = " numeric"]
    pub const NUMERIC: mjtObj = mjtObj(21);
    #[doc = " text"]
    pub const TEXT: mjtObj = mjtObj(22);
    #[doc = " tuple"]
    pub const TUPLE: mjtObj = mjtObj(23);
    #[doc = " keyframe"]
    pub const KEY: mjtObj = mjtObj(24);
    #[doc = " plugin instance"]
    pub const PLUGIN: mjtObj = mjtObj(25);
    #[doc = " frame"]
    pub const FRAME: mjtObj = mjtObj(100);
    #[doc = " default"]
    pub const DEFAULT: mjtObj = mjtObj(101);
    #[doc = " entire model"]
    pub const MODEL: mjtObj = mjtObj(102);
    #[doc = " number of object types"]
    const mjNOBJECT: mjtObj = mjtObj(26);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtObj(pub(crate) ::core::ffi::c_uint);
impl mjtSensor {
    #[doc = " scalar contact normal forces summed over sensor zone"]
    pub const TOUCH: mjtSensor = mjtSensor(0);
    #[doc = " 3D linear acceleration, in local frame"]
    pub const ACCELEROMETER: mjtSensor = mjtSensor(1);
    #[doc = " 3D linear velocity, in local frame"]
    pub const VELOCIMETER: mjtSensor = mjtSensor(2);
    #[doc = " 3D angular velocity, in local frame"]
    pub const GYRO: mjtSensor = mjtSensor(3);
    #[doc = " 3D force between site's body and its parent body"]
    pub const FORCE: mjtSensor = mjtSensor(4);
    #[doc = " 3D torque between site's body and its parent body"]
    pub const TORQUE: mjtSensor = mjtSensor(5);
    #[doc = " 3D magnetometer"]
    pub const MAGNETOMETER: mjtSensor = mjtSensor(6);
    #[doc = " scalar distance to nearest geom along z-axis"]
    pub const RANGEFINDER: mjtSensor = mjtSensor(7);
    #[doc = " pixel coordinates of a site in the camera image"]
    pub const CAMPROJECTION: mjtSensor = mjtSensor(8);
    #[doc = " scalar joint position (hinge and slide only)"]
    pub const JOINTPOS: mjtSensor = mjtSensor(9);
    #[doc = " scalar joint velocity (hinge and slide only)"]
    pub const JOINTVEL: mjtSensor = mjtSensor(10);
    #[doc = " scalar tendon position"]
    pub const TENDONPOS: mjtSensor = mjtSensor(11);
    #[doc = " scalar tendon velocity"]
    pub const TENDONVEL: mjtSensor = mjtSensor(12);
    #[doc = " scalar actuator position"]
    pub const ACTUATORPOS: mjtSensor = mjtSensor(13);
    #[doc = " scalar actuator velocity"]
    pub const ACTUATORVEL: mjtSensor = mjtSensor(14);
    #[doc = " scalar actuator force"]
    pub const ACTUATORFRC: mjtSensor = mjtSensor(15);
    #[doc = " scalar actuator force, measured at the joint"]
    pub const JOINTACTFRC: mjtSensor = mjtSensor(16);
    #[doc = " scalar actuator force, measured at the tendon"]
    pub const TENDONACTFRC: mjtSensor = mjtSensor(17);
    #[doc = " 4D ball joint quaternion"]
    pub const BALLQUAT: mjtSensor = mjtSensor(18);
    #[doc = " 3D ball joint angular velocity"]
    pub const BALLANGVEL: mjtSensor = mjtSensor(19);
    #[doc = " joint limit distance-margin"]
    pub const JOINTLIMITPOS: mjtSensor = mjtSensor(20);
    #[doc = " joint limit velocity"]
    pub const JOINTLIMITVEL: mjtSensor = mjtSensor(21);
    #[doc = " joint limit force"]
    pub const JOINTLIMITFRC: mjtSensor = mjtSensor(22);
    #[doc = " tendon limit distance-margin"]
    pub const TENDONLIMITPOS: mjtSensor = mjtSensor(23);
    #[doc = " tendon limit velocity"]
    pub const TENDONLIMITVEL: mjtSensor = mjtSensor(24);
    #[doc = " tendon limit force"]
    pub const TENDONLIMITFRC: mjtSensor = mjtSensor(25);
    #[doc = " 3D position"]
    pub const FRAMEPOS: mjtSensor = mjtSensor(26);
    #[doc = " 4D unit quaternion orientation"]
    pub const FRAMEQUAT: mjtSensor = mjtSensor(27);
    #[doc = " 3D unit vector: x-axis of object's frame"]
    pub const FRAMEXAXIS: mjtSensor = mjtSensor(28);
    #[doc = " 3D unit vector: y-axis of object's frame"]
    pub const FRAMEYAXIS: mjtSensor = mjtSensor(29);
    #[doc = " 3D unit vector: z-axis of object's frame"]
    pub const FRAMEZAXIS: mjtSensor = mjtSensor(30);
    #[doc = " 3D linear velocity"]
    pub const FRAMELINVEL: mjtSensor = mjtSensor(31);
    #[doc = " 3D angular velocity"]
    pub const FRAMEANGVEL: mjtSensor = mjtSensor(32);
    #[doc = " 3D linear acceleration"]
    pub const FRAMELINACC: mjtSensor = mjtSensor(33);
    #[doc = " 3D angular acceleration"]
    pub const FRAMEANGACC: mjtSensor = mjtSensor(34);
    #[doc = " 3D center of mass of subtree"]
    pub const SUBTREECOM: mjtSensor = mjtSensor(35);
    #[doc = " 3D linear velocity of subtree"]
    pub const SUBTREELINVEL: mjtSensor = mjtSensor(36);
    #[doc = " 3D angular momentum of subtree"]
    pub const SUBTREEANGMOM: mjtSensor = mjtSensor(37);
    #[doc = " 1 if object is inside a site, 0 otherwise"]
    pub const INSIDESITE: mjtSensor = mjtSensor(38);
    #[doc = " signed distance between two geoms"]
    pub const GEOMDIST: mjtSensor = mjtSensor(39);
    #[doc = " normal direction between two geoms"]
    pub const GEOMNORMAL: mjtSensor = mjtSensor(40);
    #[doc = " segment between two geoms"]
    pub const GEOMFROMTO: mjtSensor = mjtSensor(41);
    #[doc = " contacts which occurred during the simulation"]
    pub const CONTACT: mjtSensor = mjtSensor(42);
    #[doc = " potential energy"]
    pub const E_POTENTIAL: mjtSensor = mjtSensor(43);
    #[doc = " kinetic energy"]
    pub const E_KINETIC: mjtSensor = mjtSensor(44);
    #[doc = " simulation time"]
    pub const CLOCK: mjtSensor = mjtSensor(45);
    #[doc = " tactile sensor"]
    pub const TACTILE: mjtSensor = mjtSensor(46);
    #[doc = " plugin-controlled"]
    pub const PLUGIN: mjtSensor = mjtSensor(47);
    #[doc = " sensor data provided by mjcb_sensor callback"]
    pub const USER: mjtSensor = mjtSensor(48);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtSensor(pub(crate) ::core::ffi::c_uint);
impl mjtStage {
    #[doc = " no computations"]
    pub const NONE: mjtStage = mjtStage(0);
    #[doc = " position-dependent computations"]
    pub const POS: mjtStage = mjtStage(1);
    #[doc = " velocity-dependent computations"]
    pub const VEL: mjtStage = mjtStage(2);
    #[doc = " acceleration/force-dependent computations"]
    pub const ACC: mjtStage = mjtStage(3);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtStage(pub(crate) ::core::ffi::c_uint);
impl mjtDataType {
    #[doc = " real values, no constraints"]
    pub const REAL: mjtDataType = mjtDataType(0);
    #[doc = " positive values; 0 or negative: inactive"]
    pub const POSITIVE: mjtDataType = mjtDataType(1);
    #[doc = " 3D unit vector"]
    pub const AXIS: mjtDataType = mjtDataType(2);
    #[doc = " unit quaternion"]
    pub const QUATERNION: mjtDataType = mjtDataType(3);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtDataType(pub(crate) ::core::ffi::c_uint);
pub const mjNCONDATA: usize = mjtConDataField::mjNCONDATA.0 as usize;
impl mjtConDataField {
    #[doc = " whether a contact was found"]
    pub const FOUND: mjtConDataField = mjtConDataField(0);
    #[doc = " contact force"]
    pub const FORCE: mjtConDataField = mjtConDataField(1);
    #[doc = " contact torque"]
    pub const TORQUE: mjtConDataField = mjtConDataField(2);
    #[doc = " contact penetration distance"]
    pub const DIST: mjtConDataField = mjtConDataField(3);
    #[doc = " contact position"]
    pub const POS: mjtConDataField = mjtConDataField(4);
    #[doc = " contact frame normal"]
    pub const NORMAL: mjtConDataField = mjtConDataField(5);
    #[doc = " contact frame first tangent"]
    pub const TANGENT: mjtConDataField = mjtConDataField(6);
    #[doc = " number of contact sensor data fields"]
    const mjNCONDATA: mjtConDataField = mjtConDataField(7);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtConDataField(pub(crate) ::core::ffi::c_uint);
pub const mjNRAYDATA: usize = mjtRayDataField::mjNRAYDATA.0 as usize;
impl mjtRayDataField {
    #[doc = " distance from ray origin to nearest surface"]
    pub const DIST: mjtRayDataField = mjtRayDataField(0);
    #[doc = " normalized ray direction"]
    pub const DIR: mjtRayDataField = mjtRayDataField(1);
    #[doc = " ray origin"]
    pub const ORIGIN: mjtRayDataField = mjtRayDataField(2);
    #[doc = " point at which ray intersects nearest surface"]
    pub const POINT: mjtRayDataField = mjtRayDataField(3);
    #[doc = " surface normal at intersection point"]
    pub const NORMAL: mjtRayDataField = mjtRayDataField(4);
    #[doc = " depth along z-axis"]
    pub const DEPTH: mjtRayDataField = mjtRayDataField(5);
    #[doc = " number of rangefinder sensor data fields"]
    const mjNRAYDATA: mjtRayDataField = mjtRayDataField(6);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtRayDataField(pub(crate) ::core::ffi::c_uint);
pub const mjNCAMOUT: usize = mjtCamOutBit::mjNCAMOUT.0 as usize;
impl mjtCamOutBit {
    #[doc = " RGB image"]
    pub const RGB: mjtCamOutBit = mjtCamOutBit(1);
    #[doc = " depth image (distance from camera plane)"]
    pub const DEPTH: mjtCamOutBit = mjtCamOutBit(2);
    #[doc = " distance image (distance from camera origin)"]
    pub const DIST: mjtCamOutBit = mjtCamOutBit(4);
    #[doc = " normal image"]
    pub const NORMAL: mjtCamOutBit = mjtCamOutBit(8);
    #[doc = " segmentation image"]
    pub const SEG: mjtCamOutBit = mjtCamOutBit(16);
    #[doc = " number of camera output types"]
    const mjNCAMOUT: mjtCamOutBit = mjtCamOutBit(5);
}
impl ::core::ops::BitOr<mjtCamOutBit> for mjtCamOutBit {
    type Output = Self;
    #[inline]
    fn bitor(self, other: Self) -> Self {
        mjtCamOutBit(self.0 | other.0)
    }
}
impl ::core::ops::BitOrAssign for mjtCamOutBit {
    #[inline]
    fn bitor_assign(&mut self, rhs: mjtCamOutBit) {
        self.0 |= rhs.0;
    }
}
impl ::core::ops::BitAnd<mjtCamOutBit> for mjtCamOutBit {
    type Output = Self;
    #[inline]
    fn bitand(self, other: Self) -> Self {
        mjtCamOutBit(self.0 & other.0)
    }
}
impl ::core::ops::BitAndAssign for mjtCamOutBit {
    #[inline]
    fn bitand_assign(&mut self, rhs: mjtCamOutBit) {
        self.0 &= rhs.0;
    }
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtCamOutBit(pub(crate) ::core::ffi::c_uint);
impl mjtSameFrame {
    #[doc = " no alignment"]
    pub const NONE: mjtSameFrame = mjtSameFrame(0);
    #[doc = " frame is same as body frame"]
    pub const BODY: mjtSameFrame = mjtSameFrame(1);
    #[doc = " frame is same as inertial frame"]
    pub const INERTIA: mjtSameFrame = mjtSameFrame(2);
    #[doc = " frame orientation is same as body orientation"]
    pub const BODYROT: mjtSameFrame = mjtSameFrame(3);
    #[doc = " frame orientation is same as inertia orientation"]
    pub const INERTIAROT: mjtSameFrame = mjtSameFrame(4);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtSameFrame(pub(crate) ::core::ffi::c_uint);
impl mjtSleepPolicy {
    #[doc = " compiler chooses sleep policy"]
    pub const AUTO: mjtSleepPolicy = mjtSleepPolicy(0);
    #[doc = " compiler sleep policy: never"]
    pub const AUTO_NEVER: mjtSleepPolicy = mjtSleepPolicy(1);
    #[doc = " compiler sleep policy: allowed"]
    pub const AUTO_ALLOWED: mjtSleepPolicy = mjtSleepPolicy(2);
    #[doc = " user sleep policy: never"]
    pub const NEVER: mjtSleepPolicy = mjtSleepPolicy(3);
    #[doc = " user sleep policy: allowed"]
    pub const ALLOWED: mjtSleepPolicy = mjtSleepPolicy(4);
    #[doc = " user sleep policy: initialized asleep"]
    pub const INIT: mjtSleepPolicy = mjtSleepPolicy(5);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtSleepPolicy(pub(crate) ::core::ffi::c_uint);
impl mjtLRMode {
    #[doc = " do not process any actuators"]
    pub const NONE: mjtLRMode = mjtLRMode(0);
    #[doc = " process muscle actuators"]
    pub const MUSCLE: mjtLRMode = mjtLRMode(1);
    #[doc = " process muscle and user actuators"]
    pub const MUSCLEUSER: mjtLRMode = mjtLRMode(2);
    #[doc = " process all actuators"]
    pub const ALL: mjtLRMode = mjtLRMode(3);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtLRMode(pub(crate) ::core::ffi::c_uint);
impl mjtFlexSelf {
    #[doc = " no self-collisions"]
    pub const NONE: mjtFlexSelf = mjtFlexSelf(0);
    #[doc = " skip midphase, go directly to narrowphase"]
    pub const NARROW: mjtFlexSelf = mjtFlexSelf(1);
    #[doc = " use BVH in midphase (if midphase enabled)"]
    pub const BVH: mjtFlexSelf = mjtFlexSelf(2);
    #[doc = " use SAP in midphase"]
    pub const SAP: mjtFlexSelf = mjtFlexSelf(3);
    #[doc = " choose between BVH and SAP automatically"]
    pub const AUTO: mjtFlexSelf = mjtFlexSelf(4);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtFlexSelf(pub(crate) ::core::ffi::c_uint);
impl mjtSDFType {
    #[doc = " single SDF"]
    pub const SINGLE: mjtSDFType = mjtSDFType(0);
    #[doc = " max(A, B)"]
    pub const INTERSECTION: mjtSDFType = mjtSDFType(1);
    #[doc = " A - B"]
    pub const MIDSURFACE: mjtSDFType = mjtSDFType(2);
    #[doc = " A + B + abs(max(A, B))"]
    pub const COLLISION: mjtSDFType = mjtSDFType(3);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtSDFType(pub(crate) ::core::ffi::c_uint);
pub const mjNSTATE: usize = mjtState::mjNSTATE.0 as usize;
impl mjtState {
    #[doc = " time"]
    pub const TIME: mjtState = mjtState(1);
    #[doc = " position"]
    pub const QPOS: mjtState = mjtState(2);
    #[doc = " velocity"]
    pub const QVEL: mjtState = mjtState(4);
    #[doc = " actuator activation"]
    pub const ACT: mjtState = mjtState(8);
    #[doc = " history buffers (control, sensor)"]
    pub const HISTORY: mjtState = mjtState(16);
    #[doc = " acceleration used for warmstart"]
    pub const WARMSTART: mjtState = mjtState(32);
    #[doc = " control"]
    pub const CTRL: mjtState = mjtState(64);
    #[doc = " applied generalized force"]
    pub const QFRC_APPLIED: mjtState = mjtState(128);
    #[doc = " applied Cartesian force/torque"]
    pub const XFRC_APPLIED: mjtState = mjtState(256);
    #[doc = " enable/disable constraints"]
    pub const EQ_ACTIVE: mjtState = mjtState(512);
    #[doc = " positions of mocap bodies"]
    pub const MOCAP_POS: mjtState = mjtState(1024);
    #[doc = " orientations of mocap bodies"]
    pub const MOCAP_QUAT: mjtState = mjtState(2048);
    #[doc = " user data"]
    pub const USERDATA: mjtState = mjtState(4096);
    #[doc = " plugin state"]
    pub const PLUGIN: mjtState = mjtState(8192);
    #[doc = " convenience values for commonly used state specifications"]
    pub const PHYSICS: mjtState = mjtState(30);
    #[doc = " convenience values for commonly used state specifications"]
    pub const FULLPHYSICS: mjtState = mjtState(8223);
    #[doc = " convenience values for commonly used state specifications"]
    pub const USER: mjtState = mjtState(8128);
    #[doc = " convenience values for commonly used state specifications"]
    pub const INTEGRATION: mjtState = mjtState(16383);
    #[doc = " number of state elements"]
    const mjNSTATE: mjtState = mjtState(14);
}
#[repr(transparent)]
#[doc = "---------------------------------- enum types (mjData) -------------------------------------------"]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtState(pub(crate) ::core::ffi::c_uint);
impl mjtConstraint {
    #[doc = " equality constraint"]
    pub const EQUALITY: mjtConstraint = mjtConstraint(0);
    #[doc = " dof friction"]
    pub const FRICTION_DOF: mjtConstraint = mjtConstraint(1);
    #[doc = " tendon friction"]
    pub const FRICTION_TENDON: mjtConstraint = mjtConstraint(2);
    #[doc = " joint limit"]
    pub const LIMIT_JOINT: mjtConstraint = mjtConstraint(3);
    #[doc = " tendon limit"]
    pub const LIMIT_TENDON: mjtConstraint = mjtConstraint(4);
    #[doc = " frictionless contact"]
    pub const CONTACT_FRICTIONLESS: mjtConstraint = mjtConstraint(5);
    #[doc = " frictional contact, pyramidal friction cone"]
    pub const CONTACT_PYRAMIDAL: mjtConstraint = mjtConstraint(6);
    #[doc = " frictional contact, elliptic friction cone"]
    pub const CONTACT_ELLIPTIC: mjtConstraint = mjtConstraint(7);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtConstraint(pub(crate) ::core::ffi::c_uint);
impl mjtConstraintState {
    #[doc = " constraint satisfied, zero cost (limit, contact)"]
    pub const SATISFIED: mjtConstraintState = mjtConstraintState(0);
    #[doc = " quadratic cost (equality, friction, limit, contact)"]
    pub const QUADRATIC: mjtConstraintState = mjtConstraintState(1);
    #[doc = " linear cost, negative side (friction)"]
    pub const LINEARNEG: mjtConstraintState = mjtConstraintState(2);
    #[doc = " linear cost, positive side (friction)"]
    pub const LINEARPOS: mjtConstraintState = mjtConstraintState(3);
    #[doc = " squared distance to cone cost (elliptic contact)"]
    pub const CONE: mjtConstraintState = mjtConstraintState(4);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtConstraintState(pub(crate) ::core::ffi::c_uint);
pub const mjNWARNING: usize = mjtWarning::mjNWARNING.0 as usize;
impl mjtWarning {
    #[doc = " (near) singular inertia matrix"]
    pub const INERTIA: mjtWarning = mjtWarning(0);
    #[doc = " too many contacts in contact list"]
    pub const CONTACTFULL: mjtWarning = mjtWarning(1);
    #[doc = " too many constraints"]
    pub const CNSTRFULL: mjtWarning = mjtWarning(2);
    #[doc = " bad number in qpos"]
    pub const BADQPOS: mjtWarning = mjtWarning(3);
    #[doc = " bad number in qvel"]
    pub const BADQVEL: mjtWarning = mjtWarning(4);
    #[doc = " bad number in qacc"]
    pub const BADQACC: mjtWarning = mjtWarning(5);
    #[doc = " bad number in ctrl"]
    pub const BADCTRL: mjtWarning = mjtWarning(6);
    #[doc = " number of warnings"]
    const mjNWARNING: mjtWarning = mjtWarning(7);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtWarning(pub(crate) ::core::ffi::c_uint);
pub const mjNTIMER: usize = mjtTimer::mjNTIMER.0 as usize;
impl mjtTimer {
    #[doc = " step"]
    pub const STEP: mjtTimer = mjtTimer(0);
    #[doc = " forward"]
    pub const FORWARD: mjtTimer = mjtTimer(1);
    #[doc = " inverse"]
    pub const INVERSE: mjtTimer = mjtTimer(2);
    #[doc = " fwdPosition"]
    pub const POSITION: mjtTimer = mjtTimer(3);
    #[doc = " fwdVelocity"]
    pub const VELOCITY: mjtTimer = mjtTimer(4);
    #[doc = " fwdActuation"]
    pub const ACTUATION: mjtTimer = mjtTimer(5);
    #[doc = " fwdConstraint"]
    pub const CONSTRAINT: mjtTimer = mjtTimer(6);
    #[doc = " mj_Euler, mj_implicit"]
    pub const ADVANCE: mjtTimer = mjtTimer(7);
    #[doc = " kinematics, com, tendon, transmission"]
    pub const POS_KINEMATICS: mjtTimer = mjtTimer(8);
    #[doc = " inertia computations"]
    pub const POS_INERTIA: mjtTimer = mjtTimer(9);
    #[doc = " collision detection"]
    pub const POS_COLLISION: mjtTimer = mjtTimer(10);
    #[doc = " make constraints"]
    pub const POS_MAKE: mjtTimer = mjtTimer(11);
    #[doc = " project constraints"]
    pub const POS_PROJECT: mjtTimer = mjtTimer(12);
    #[doc = " broadphase"]
    pub const COL_BROAD: mjtTimer = mjtTimer(13);
    #[doc = " narrowphase"]
    pub const COL_NARROW: mjtTimer = mjtTimer(14);
    #[doc = " number of timers"]
    const mjNTIMER: mjtTimer = mjtTimer(15);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtTimer(pub(crate) ::core::ffi::c_uint);
impl mjtSleepState {
    #[doc = " object is static"]
    pub const STATIC: mjtSleepState = mjtSleepState(-1);
    #[doc = " object is asleep"]
    pub const ASLEEP: mjtSleepState = mjtSleepState(0);
    #[doc = " object is awake"]
    pub const AWAKE: mjtSleepState = mjtSleepState(1);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtSleepState(pub(crate) ::core::ffi::c_int);
impl mjtLogLevel {
    #[doc = " internal engine debug trace (opt-in via topic filtering)"]
    pub const DEBUG: mjtLogLevel = mjtLogLevel(0);
    #[doc = " informational (opt-in via topic filtering)"]
    pub const INFO: mjtLogLevel = mjtLogLevel(1);
    #[doc = " warning"]
    pub const WARNING: mjtLogLevel = mjtLogLevel(2);
    #[doc = " error"]
    pub const ERROR: mjtLogLevel = mjtLogLevel(3);
}
#[repr(transparent)]
#[doc = "---------------------------------- logging -------------------------------------------------------"]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtLogLevel(pub(crate) ::core::ffi::c_uint);
impl mjtLogTopic {
    #[doc = " no topic (always passes filtering)\n INFO topics:"]
    pub const NONE: mjtLogTopic = mjtLogTopic(0);
    #[doc = " timing diagnostics (step)"]
    pub const TIME_STP: mjtLogTopic = mjtLogTopic(1);
    #[doc = " timing diagnostics (compile)\n DEBUG topics:"]
    pub const TIME_CMP: mjtLogTopic = mjtLogTopic(2);
    #[doc = " sleep/wake events"]
    pub const SLEEP: mjtLogTopic = mjtLogTopic(3);
    #[doc = " number of filterable topics"]
    const mjNTOPIC: mjtLogTopic = mjtLogTopic(3);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtLogTopic(pub(crate) ::core::ffi::c_uint);
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjLogMessage {
    #[doc = " mjtLogLevel"]
    pub level: ::core::ffi::c_int,
    #[doc = " mjtLogTopic (0 for error/warning/user)"]
    pub topic: ::core::ffi::c_int,
    #[doc = " message subject (one-liner, printf-formatted)"]
    pub(crate) subject: [::core::ffi::c_char; 1024usize],
    #[doc = " message body (multi-line detail, or NULL)"]
    pub(crate) body: *const ::core::ffi::c_char,
    #[doc = " __func__ or NULL"]
    pub(crate) func: *const ::core::ffi::c_char,
    #[doc = " __FILE__ or NULL"]
    pub(crate) file: *const ::core::ffi::c_char,
    #[doc = " __LINE__ or 0"]
    pub line: ::core::ffi::c_int,
    #[doc = " prepend timestamp to output"]
    pub timestamp: mjtBool,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjLogConfig {
    #[doc = " print to console (default: true)"]
    pub logto_console: mjtBool,
    #[doc = " print to log file (default: true)"]
    pub logto_file: mjtBool,
    #[doc = " log file path (default: \"MUJOCO_LOG.TXT\")"]
    pub(crate) logfile: [::core::ffi::c_char; 1024usize],
    #[doc = " enabled info topic bitmask (default: 0)"]
    pub topics: ::core::ffi::c_int,
}
#[doc = " function type for log handler callback; must be thread-safe, must not call mju_error"]
pub(crate) type mjfLogHandler =
    ::core::option::Option<unsafe extern "C" fn(arg1: *const mjLogMessage)>;
#[doc = "---------------------------------- mjLROpt -------------------------------------------------------"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjLROpt {
    #[doc = " which actuators to process (mjtLRMode)"]
    pub mode: ::core::ffi::c_int,
    #[doc = " use existing length range if available"]
    pub useexisting: ::core::ffi::c_int,
    #[doc = " use joint and tendon limits if available"]
    pub uselimit: ::core::ffi::c_int,
    #[doc = " target acceleration used to compute force"]
    pub accel: mjtNum,
    #[doc = " maximum force; 0: no limit"]
    pub maxforce: mjtNum,
    #[doc = " time constant for velocity reduction; min 0.01"]
    pub timeconst: mjtNum,
    #[doc = " simulation timestep; 0: use mjOption.timestep"]
    pub timestep: mjtNum,
    #[doc = " total simulation time interval"]
    pub inttotal: mjtNum,
    #[doc = " evaluation time interval (at the end)"]
    pub interval: mjtNum,
    #[doc = " convergence tolerance (relative to range)"]
    pub tolrange: mjtNum,
}
#[doc = "---------------------------------- mjCache -------------------------------------------------------"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjCache {
    #[doc = " internal pointer to cache"]
    pub impl_: *mut ::core::ffi::c_void,
}
#[doc = "---------------------------------- mjVFS ---------------------------------------------------------"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjVFS {
    #[doc = " internal pointer to VFS memory"]
    pub impl_: *mut ::core::ffi::c_void,
}
#[doc = "---------------------------------- mjOption ------------------------------------------------------"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjOption {
    #[doc = " timestep"]
    pub timestep: mjtNum,
    #[doc = " ratio of friction-to-normal contact impedance"]
    pub impratio: mjtNum,
    #[doc = " main solver tolerance"]
    pub tolerance: mjtNum,
    #[doc = " CG/Newton linesearch tolerance"]
    pub ls_tolerance: mjtNum,
    #[doc = " noslip solver tolerance"]
    pub noslip_tolerance: mjtNum,
    #[doc = " convex collision solver tolerance"]
    pub ccd_tolerance: mjtNum,
    #[doc = " sleep velocity tolerance"]
    pub sleep_tolerance: mjtNum,
    #[doc = " gravitational acceleration"]
    pub gravity: [mjtNum; 3usize],
    #[doc = " wind (for lift, drag and viscosity)"]
    pub wind: [mjtNum; 3usize],
    #[doc = " global magnetic flux"]
    pub magnetic: [mjtNum; 3usize],
    #[doc = " density of medium"]
    pub density: mjtNum,
    #[doc = " viscosity of medium"]
    pub viscosity: mjtNum,
    #[doc = " margin"]
    pub o_margin: mjtNum,
    #[doc = " solref"]
    pub o_solref: [mjtNum; 2usize],
    #[doc = " solimp"]
    pub o_solimp: [mjtNum; 5usize],
    #[doc = " friction"]
    pub o_friction: [mjtNum; 5usize],
    #[doc = " integration mode (mjtIntegrator)"]
    pub integrator: ::core::ffi::c_int,
    #[doc = " type of friction cone (mjtCone)"]
    pub cone: ::core::ffi::c_int,
    #[doc = " type of Jacobian (mjtJacobian)"]
    pub jacobian: ::core::ffi::c_int,
    #[doc = " solver algorithm (mjtSolver)"]
    pub solver: ::core::ffi::c_int,
    #[doc = " maximum number of main solver iterations"]
    pub iterations: ::core::ffi::c_int,
    #[doc = " maximum number of CG/Newton linesearch iterations"]
    pub ls_iterations: ::core::ffi::c_int,
    #[doc = " maximum number of noslip solver iterations"]
    pub noslip_iterations: ::core::ffi::c_int,
    #[doc = " maximum number of convex collision solver iterations"]
    pub ccd_iterations: ::core::ffi::c_int,
    #[doc = " bit flags for disabling standard features"]
    pub disableflags: ::core::ffi::c_int,
    #[doc = " bit flags for enabling optional features"]
    pub enableflags: ::core::ffi::c_int,
    #[doc = " bit flags for disabling actuators by group id"]
    pub disableactuator: ::core::ffi::c_int,
    #[doc = " number of starting points for gradient descent"]
    pub sdf_initpoints: ::core::ffi::c_int,
    #[doc = " max number of iterations for gradient descent"]
    pub sdf_iterations: ::core::ffi::c_int,
}
#[doc = "---------------------------------- mjVisual ------------------------------------------------------"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjVisual {
    pub global: mjVisual_1,
    pub quality: mjVisual_2,
    pub headlight: mjVisual_3,
    pub map: mjVisual_4,
    pub scale: mjVisual_5,
    pub rgba: mjVisual_6,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjVisual_1 {
    #[doc = " initial camera id (-1: free)"]
    pub(crate) cameraid: ::core::ffi::c_int,
    #[doc = " is the free camera orthographic (0: no, 1: yes)"]
    pub orthographic: ::core::ffi::c_int,
    #[doc = " y field-of-view of free camera (orthographic ? length : degree)"]
    pub fovy: f32,
    #[doc = " inter-pupilary distance for free camera"]
    pub ipd: f32,
    #[doc = " initial azimuth of free camera (degrees)"]
    pub azimuth: f32,
    #[doc = " initial elevation of free camera (degrees)"]
    pub elevation: f32,
    #[doc = " line width for wireframe and ray rendering"]
    pub linewidth: f32,
    #[doc = " glow coefficient for selected body"]
    pub glow: f32,
    #[doc = " initial real-time factor (1: real time)"]
    pub realtime: f32,
    #[doc = " width of offscreen buffer"]
    pub offwidth: ::core::ffi::c_int,
    #[doc = " height of offscreen buffer"]
    pub offheight: ::core::ffi::c_int,
    #[doc = " geom for inertia visualization (0: box, 1: ellipsoid)"]
    pub ellipsoidinertia: ::core::ffi::c_int,
    #[doc = " visualize active bounding volumes (0: no, 1: yes)"]
    pub bvactive: ::core::ffi::c_int,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjVisual_2 {
    #[doc = " size of shadowmap texture"]
    pub shadowsize: ::core::ffi::c_int,
    #[doc = " number of multisamples for offscreen rendering"]
    pub offsamples: ::core::ffi::c_int,
    #[doc = " number of slices for builtin geom drawing"]
    pub numslices: ::core::ffi::c_int,
    #[doc = " number of stacks for builtin geom drawing"]
    pub numstacks: ::core::ffi::c_int,
    #[doc = " number of quads for box rendering"]
    pub numquads: ::core::ffi::c_int,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjVisual_3 {
    #[doc = " ambient rgb (alpha=1)"]
    pub ambient: [f32; 3usize],
    #[doc = " diffuse rgb (alpha=1)"]
    pub diffuse: [f32; 3usize],
    #[doc = " specular rgb (alpha=1)"]
    pub specular: [f32; 3usize],
    #[doc = " is headlight active"]
    pub active: ::core::ffi::c_int,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjVisual_4 {
    #[doc = " mouse perturbation stiffness (space->force)"]
    pub stiffness: f32,
    #[doc = " mouse perturbation stiffness (space->torque)"]
    pub stiffnessrot: f32,
    #[doc = " from force units to space units"]
    pub force: f32,
    #[doc = " from torque units to space units"]
    pub torque: f32,
    #[doc = " scale geom alphas when transparency is enabled"]
    pub alpha: f32,
    #[doc = " OpenGL fog starts at fogstart * mjModel.stat.extent"]
    pub fogstart: f32,
    #[doc = " OpenGL fog ends at fogend * mjModel.stat.extent"]
    pub fogend: f32,
    #[doc = " near clipping plane = znear * mjModel.stat.extent"]
    pub znear: f32,
    #[doc = " far clipping plane = zfar * mjModel.stat.extent"]
    pub zfar: f32,
    #[doc = " haze ratio"]
    pub haze: f32,
    #[doc = " directional light: shadowclip * mjModel.stat.extent"]
    pub shadowclip: f32,
    #[doc = " spot light: shadowscale * light.cutoff"]
    pub shadowscale: f32,
    #[doc = " scale tendon width"]
    pub actuatortendon: f32,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjVisual_5 {
    #[doc = " width of force arrow"]
    pub forcewidth: f32,
    #[doc = " contact width"]
    pub contactwidth: f32,
    #[doc = " contact height"]
    pub contactheight: f32,
    #[doc = " autoconnect capsule width"]
    pub connect: f32,
    #[doc = " com radius"]
    pub com: f32,
    #[doc = " camera object"]
    pub camera: f32,
    #[doc = " light object"]
    pub light: f32,
    #[doc = " selection point"]
    pub selectpoint: f32,
    #[doc = " joint length"]
    pub jointlength: f32,
    #[doc = " joint width"]
    pub jointwidth: f32,
    #[doc = " actuator length"]
    pub actuatorlength: f32,
    #[doc = " actuator width"]
    pub actuatorwidth: f32,
    #[doc = " bodyframe axis length"]
    pub framelength: f32,
    #[doc = " bodyframe axis width"]
    pub framewidth: f32,
    #[doc = " constraint width"]
    pub constraint: f32,
    #[doc = " slidercrank width"]
    pub slidercrank: f32,
    #[doc = " frustum zfar plane"]
    pub frustum: f32,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjVisual_6 {
    #[doc = " fog"]
    pub fog: [f32; 4usize],
    #[doc = " haze"]
    pub haze: [f32; 4usize],
    #[doc = " external force"]
    pub force: [f32; 4usize],
    #[doc = " inertia box"]
    pub inertia: [f32; 4usize],
    #[doc = " joint"]
    pub joint: [f32; 4usize],
    #[doc = " actuator, neutral"]
    pub actuator: [f32; 4usize],
    #[doc = " actuator, negative limit"]
    pub actuatornegative: [f32; 4usize],
    #[doc = " actuator, positive limit"]
    pub actuatorpositive: [f32; 4usize],
    #[doc = " center of mass"]
    pub com: [f32; 4usize],
    #[doc = " camera object"]
    pub camera: [f32; 4usize],
    #[doc = " light object"]
    pub light: [f32; 4usize],
    #[doc = " selection point"]
    pub selectpoint: [f32; 4usize],
    #[doc = " auto connect"]
    pub connect: [f32; 4usize],
    #[doc = " contact point"]
    pub contactpoint: [f32; 4usize],
    #[doc = " contact force"]
    pub contactforce: [f32; 4usize],
    #[doc = " contact friction force"]
    pub contactfriction: [f32; 4usize],
    #[doc = " contact torque"]
    pub contacttorque: [f32; 4usize],
    #[doc = " contact point in gap"]
    pub contactgap: [f32; 4usize],
    #[doc = " rangefinder ray"]
    pub rangefinder: [f32; 4usize],
    #[doc = " constraint"]
    pub constraint: [f32; 4usize],
    #[doc = " slidercrank"]
    pub slidercrank: [f32; 4usize],
    #[doc = " used when crank must be stretched/broken"]
    pub crankbroken: [f32; 4usize],
    #[doc = " camera frustum"]
    pub frustum: [f32; 4usize],
    #[doc = " bounding volume"]
    pub bv: [f32; 4usize],
    #[doc = " active bounding volume"]
    pub bvactive: [f32; 4usize],
}
#[doc = "---------------------------------- mjStatistic ---------------------------------------------------"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjStatistic {
    #[doc = " mean diagonal inertia"]
    pub meaninertia: mjtNum,
    #[doc = " mean body mass"]
    pub meanmass: mjtNum,
    #[doc = " mean body size"]
    pub meansize: mjtNum,
    #[doc = " spatial extent"]
    pub extent: mjtNum,
    #[doc = " center of model"]
    pub center: [mjtNum; 3usize],
}
#[doc = "---------------------------------- mjModel -------------------------------------------------------"]
#[repr(C)]
#[derive(Debug)]
pub struct mjModel {
    #[doc = " number of generalized coordinates = dim(qpos)"]
    pub nq: mjtSize,
    #[doc = " number of degrees of freedom = dim(qvel)"]
    pub nv: mjtSize,
    #[doc = " number of scalar controls = dim(ctrl)"]
    pub nu: mjtSize,
    #[doc = " number of actuators"]
    pub nactuator: mjtSize,
    #[doc = " number of force outputs, derived from transmission type"]
    pub nout: mjtSize,
    #[doc = " number of activation states = dim(act)"]
    pub na: mjtSize,
    #[doc = " number of bodies"]
    pub nbody: mjtSize,
    #[doc = " number of total bounding volumes in all bodies"]
    pub nbvh: mjtSize,
    #[doc = " number of static bounding volumes (aabb stored in mjModel)"]
    pub nbvhstatic: mjtSize,
    #[doc = " number of dynamic bounding volumes (aabb stored in mjData)"]
    pub nbvhdynamic: mjtSize,
    #[doc = " number of total octree cells in all meshes"]
    pub noct: mjtSize,
    #[doc = " number of joints"]
    pub njnt: mjtSize,
    #[doc = " number of kinematic trees under world body"]
    pub ntree: mjtSize,
    #[doc = " number of non-zeros in sparse inertia matrix"]
    pub nM: mjtSize,
    #[doc = " number of non-zeros in sparse body-dof matrix"]
    pub nB: mjtSize,
    #[doc = " number of non-zeros in sparse reduced dof-dof matrix"]
    pub nC: mjtSize,
    #[doc = " number of non-zeros in sparse dof-dof matrix"]
    pub nD: mjtSize,
    #[doc = " number of geoms"]
    pub ngeom: mjtSize,
    #[doc = " number of sites"]
    pub nsite: mjtSize,
    #[doc = " number of cameras"]
    pub ncam: mjtSize,
    #[doc = " number of lights"]
    pub nlight: mjtSize,
    #[doc = " number of flexes"]
    pub nflex: mjtSize,
    #[doc = " number of dofs in all flexes"]
    pub nflexnode: mjtSize,
    #[doc = " number of vertices in all flexes"]
    pub nflexvert: mjtSize,
    #[doc = " number of edges in all flexes"]
    pub nflexedge: mjtSize,
    #[doc = " number of elements in all flexes"]
    pub nflexelem: mjtSize,
    #[doc = " number of element vertex ids in all flexes"]
    pub nflexelemdata: mjtSize,
    #[doc = " number of stiffness parameters in all flexes"]
    pub nflexstiffness: mjtSize,
    #[doc = " number of bending parameters in all flexes"]
    pub nflexbending: mjtSize,
    #[doc = " number of dofs covered by the constant metric factor"]
    pub nefm0dof: mjtSize,
    #[doc = " number of non-zeros in the constant metric factor"]
    pub nefm0L: mjtSize,
    #[doc = " number of element edge ids in all flexes"]
    pub nflexelemedge: mjtSize,
    #[doc = " number of shell fragment vertex ids in all flexes"]
    pub nflexshelldata: mjtSize,
    #[doc = " number of element-vertex pairs in all flexes"]
    pub nflexevpair: mjtSize,
    #[doc = " number of vertices with texture coordinates"]
    pub nflextexcoord: mjtSize,
    #[doc = " number of non-zeros in sparse flexedge Jacobian matrix"]
    pub nJfe: mjtSize,
    #[doc = " number of non-zeros in sparse flexvert Jacobian matrix"]
    pub nJfv: mjtSize,
    #[doc = " number of meshes"]
    pub nmesh: mjtSize,
    #[doc = " number of vertices in all meshes"]
    pub nmeshvert: mjtSize,
    #[doc = " number of normals in all meshes"]
    pub nmeshnormal: mjtSize,
    #[doc = " number of texcoords in all meshes"]
    pub nmeshtexcoord: mjtSize,
    #[doc = " number of triangular faces in all meshes"]
    pub nmeshface: mjtSize,
    #[doc = " number of ints in mesh auxiliary data"]
    pub nmeshgraph: mjtSize,
    #[doc = " number of polygons in all meshes"]
    pub nmeshpoly: mjtSize,
    #[doc = " number of vertices in all polygons"]
    pub nmeshpolyvert: mjtSize,
    #[doc = " number of polygons in vertex map"]
    pub nmeshpolymap: mjtSize,
    #[doc = " number of skins"]
    pub nskin: mjtSize,
    #[doc = " number of vertices in all skins"]
    pub nskinvert: mjtSize,
    #[doc = " number of vertices with texcoords in all skins"]
    pub nskintexvert: mjtSize,
    #[doc = " number of triangular faces in all skins"]
    pub nskinface: mjtSize,
    #[doc = " number of bones in all skins"]
    pub nskinbone: mjtSize,
    #[doc = " number of vertices in all skin bones"]
    pub nskinbonevert: mjtSize,
    #[doc = " number of heightfields"]
    pub nhfield: mjtSize,
    #[doc = " number of data points in all heightfields"]
    pub nhfielddata: mjtSize,
    #[doc = " number of textures"]
    pub ntex: mjtSize,
    #[doc = " number of bytes in texture rgb data"]
    pub ntexdata: mjtSize,
    #[doc = " number of materials"]
    pub nmat: mjtSize,
    #[doc = " number of predefined geom pairs"]
    pub npair: mjtSize,
    #[doc = " number of excluded geom pairs"]
    pub nexclude: mjtSize,
    #[doc = " number of equality constraints"]
    pub neq: mjtSize,
    #[doc = " number of tendons"]
    pub ntendon: mjtSize,
    #[doc = " number of non-zeros in sparse ten_J matrix"]
    pub nJten: mjtSize,
    #[doc = " number of wrap objects in all tendon paths"]
    pub nwrap: mjtSize,
    #[doc = " number of sensors"]
    pub nsensor: mjtSize,
    #[doc = " number of numeric custom fields"]
    pub nnumeric: mjtSize,
    #[doc = " number of mjtNums in all numeric fields"]
    pub nnumericdata: mjtSize,
    #[doc = " number of text custom fields"]
    pub ntext: mjtSize,
    #[doc = " number of mjtBytes in all text fields"]
    pub ntextdata: mjtSize,
    #[doc = " number of tuple custom fields"]
    pub ntuple: mjtSize,
    #[doc = " number of objects in all tuple fields"]
    pub ntupledata: mjtSize,
    #[doc = " number of keyframes"]
    pub nkey: mjtSize,
    #[doc = " number of mocap bodies"]
    pub nmocap: mjtSize,
    #[doc = " number of plugin instances"]
    pub nplugin: mjtSize,
    #[doc = " number of chars in all plugin config attributes"]
    pub npluginattr: mjtSize,
    #[doc = " number of mjtNums in body_user"]
    pub nuser_body: mjtSize,
    #[doc = " number of mjtNums in jnt_user"]
    pub nuser_jnt: mjtSize,
    #[doc = " number of mjtNums in geom_user"]
    pub nuser_geom: mjtSize,
    #[doc = " number of mjtNums in site_user"]
    pub nuser_site: mjtSize,
    #[doc = " number of mjtNums in cam_user"]
    pub nuser_cam: mjtSize,
    #[doc = " number of mjtNums in tendon_user"]
    pub nuser_tendon: mjtSize,
    #[doc = " number of mjtNums in actuator_user"]
    pub nuser_actuator: mjtSize,
    #[doc = " number of mjtNums in sensor_user"]
    pub nuser_sensor: mjtSize,
    #[doc = " number of chars in all names"]
    pub nnames: mjtSize,
    #[doc = " number of chars in all paths"]
    pub npaths: mjtSize,
    #[doc = " number of slots in the names hash map"]
    pub nnames_map: mjtSize,
    #[doc = " number of non-zeros in sparse actuator_moment matrix"]
    pub nJmom: mjtSize,
    #[doc = " number of bodies with nonzero gravcomp"]
    pub ngravcomp: mjtSize,
    #[doc = " number of potential equality-constraint rows"]
    pub nemax: mjtSize,
    #[doc = " number of available rows in constraint Jacobian (legacy)"]
    pub njmax: mjtSize,
    #[doc = " number of potential contacts in contact list (legacy)"]
    pub nconmax: mjtSize,
    #[doc = " maximum number of vertices in a mesh polygon"]
    pub npolygonmax: mjtSize,
    #[doc = " maximum number of edges adjacent to a mesh vertex"]
    pub nmeshdegmax: mjtSize,
    #[doc = " number of mjtNums reserved for the user"]
    pub nuserdata: mjtSize,
    #[doc = " number of mjtNums in sensor data vector"]
    pub nsensordata: mjtSize,
    #[doc = " number of mjtNums in plugin state vector"]
    pub npluginstate: mjtSize,
    #[doc = " number of mjtNums in history buffer"]
    pub nhistory: mjtSize,
    #[doc = " number of bytes in the mjData arena (inclusive of stack)"]
    pub narena: mjtSize,
    #[doc = " number of bytes in buffer"]
    pub nbuffer: mjtSize,
    #[doc = " whether any body has nonzero gravcomp"]
    pub flg_gravcomp: mjtBool,
    #[doc = " whether any geom has nonzero surfacevel"]
    pub flg_surfacevel: mjtBool,
    #[doc = " whether any geom or pair has nonzero adhesion"]
    pub flg_adhesion: mjtBool,
    #[doc = " physics options"]
    pub opt: mjOption,
    #[doc = " visualization options"]
    pub vis: mjVisual,
    #[doc = " model statistics"]
    pub stat: mjStatistic,
    #[doc = " main buffer; all pointers point in it    (nbuffer)"]
    pub buffer: *mut ::core::ffi::c_void,
    #[doc = " qpos values at default pose              (nq x 1)"]
    pub(crate) qpos0: *mut mjtNum,
    #[doc = " reference pose for springs               (nq x 1)"]
    pub(crate) qpos_spring: *mut mjtNum,
    #[doc = " id of body's parent                      (nbody x 1)"]
    pub(crate) body_parentid: *mut ::core::ffi::c_int,
    #[doc = " ancestor that is direct child of world   (nbody x 1)"]
    pub(crate) body_rootid: *mut ::core::ffi::c_int,
    #[doc = " top ancestor with no dofs to this body   (nbody x 1)"]
    pub(crate) body_weldid: *mut ::core::ffi::c_int,
    #[doc = " id of mocap data; -1: none               (nbody x 1)"]
    pub(crate) body_mocapid: *mut ::core::ffi::c_int,
    #[doc = " number of joints for this body           (nbody x 1)"]
    pub(crate) body_jntnum: *mut ::core::ffi::c_int,
    #[doc = " start addr of joints; -1: no joints      (nbody x 1)"]
    pub(crate) body_jntadr: *mut ::core::ffi::c_int,
    #[doc = " number of motion degrees of freedom      (nbody x 1)"]
    pub(crate) body_dofnum: *mut ::core::ffi::c_int,
    #[doc = " start addr of dofs; -1: no dofs          (nbody x 1)"]
    pub(crate) body_dofadr: *mut ::core::ffi::c_int,
    #[doc = " id of body's kinematic tree; -1: static  (nbody x 1)"]
    pub(crate) body_treeid: *mut ::core::ffi::c_int,
    #[doc = " number of geoms                          (nbody x 1)"]
    pub(crate) body_geomnum: *mut ::core::ffi::c_int,
    #[doc = " start addr of geoms; -1: no geoms        (nbody x 1)"]
    pub(crate) body_geomadr: *mut ::core::ffi::c_int,
    #[doc = " 1: diag M; 2: diag M, sliders only       (nbody x 1)"]
    pub(crate) body_simple: *mut mjtByte,
    #[doc = " same frame as inertia (mjtSameframe)     (nbody x 1)"]
    pub(crate) body_sameframe: *mut mjtByte,
    #[doc = " position offset rel. to parent body      (nbody x 3)"]
    pub(crate) body_pos: *mut mjtNum,
    #[doc = " orientation offset rel. to parent body   (nbody x 4)"]
    pub(crate) body_quat: *mut mjtNum,
    #[doc = " local position of center of mass         (nbody x 3)"]
    pub(crate) body_ipos: *mut mjtNum,
    #[doc = " local orientation of inertia ellipsoid   (nbody x 4)"]
    pub(crate) body_iquat: *mut mjtNum,
    #[doc = " mass                                     (nbody x 1)"]
    pub(crate) body_mass: *mut mjtNum,
    #[doc = " mass of subtree starting at this body    (nbody x 1)"]
    pub(crate) body_subtreemass: *mut mjtNum,
    #[doc = " diagonal inertia in ipos/iquat frame     (nbody x 3)"]
    pub(crate) body_inertia: *mut mjtNum,
    #[doc = " mean inv inert in qpos0 (trn, rot)       (nbody x 2)"]
    pub(crate) body_invweight0: *mut mjtNum,
    #[doc = " antigravity force, units of body weight  (nbody x 1)"]
    pub(crate) body_gravcomp: *mut mjtNum,
    #[doc = " MAX over all geom margins+gaps           (nbody x 1)"]
    pub(crate) body_margin: *mut mjtNum,
    #[doc = " user data                                (nbody x nuser_body)"]
    pub(crate) body_user: *mut mjtNum,
    #[doc = " plugin instance id; -1: not in use       (nbody x 1)"]
    pub(crate) body_plugin: *mut ::core::ffi::c_int,
    #[doc = " OR over all geom contypes                (nbody x 1)"]
    pub(crate) body_contype: *mut ::core::ffi::c_int,
    #[doc = " OR over all geom conaffinities           (nbody x 1)"]
    pub(crate) body_conaffinity: *mut ::core::ffi::c_int,
    #[doc = " address of bvh root                      (nbody x 1)"]
    pub(crate) body_bvhadr: *mut ::core::ffi::c_int,
    #[doc = " number of bounding volumes               (nbody x 1)"]
    pub(crate) body_bvhnum: *mut ::core::ffi::c_int,
    #[doc = " depth in the bounding volume hierarchy   (nbvh x 1)"]
    pub(crate) bvh_depth: *mut ::core::ffi::c_int,
    #[doc = " left and right children in tree          (nbvh x 2)"]
    pub(crate) bvh_child: *mut ::core::ffi::c_int,
    #[doc = " geom or elem id of node; -1: non-leaf    (nbvh x 1)"]
    pub(crate) bvh_nodeid: *mut ::core::ffi::c_int,
    #[doc = " local bounding box (center, size)        (nbvhstatic x 6)"]
    pub(crate) bvh_aabb: *mut mjtNum,
    #[doc = " depth in the octree                      (noct x 1)"]
    pub(crate) oct_depth: *mut ::core::ffi::c_int,
    #[doc = " children of octree node                  (noct x 8)"]
    pub(crate) oct_child: *mut ::core::ffi::c_int,
    #[doc = " octree node bounding box (center, size)  (noct x 6)"]
    pub(crate) oct_aabb: *mut mjtNum,
    #[doc = " octree interpolation coefficients        (noct x 8)"]
    pub(crate) oct_coeff: *mut mjtNum,
    #[doc = " type of joint (mjtJoint)                 (njnt x 1)"]
    pub(crate) jnt_type: *mut ::core::ffi::c_int,
    #[doc = " start addr in 'qpos' for joint's data    (njnt x 1)"]
    pub(crate) jnt_qposadr: *mut ::core::ffi::c_int,
    #[doc = " start addr in 'qvel' for joint's data    (njnt x 1)"]
    pub(crate) jnt_dofadr: *mut ::core::ffi::c_int,
    #[doc = " id of joint's body                       (njnt x 1)"]
    pub(crate) jnt_bodyid: *mut ::core::ffi::c_int,
    #[doc = " actuator contributing damping / armature (njnt x 1)"]
    pub(crate) jnt_actuatorid: *mut ::core::ffi::c_int,
    #[doc = " group for visibility                     (njnt x 1)"]
    pub(crate) jnt_group: *mut ::core::ffi::c_int,
    #[doc = " does joint have limits                   (njnt x 1)"]
    pub(crate) jnt_limited: *mut mjtBool,
    #[doc = " does joint have actuator force limits    (njnt x 1)"]
    pub(crate) jnt_actfrclimited: *mut mjtBool,
    #[doc = " is gravcomp force applied via actuators  (njnt x 1)"]
    pub(crate) jnt_actgravcomp: *mut mjtBool,
    #[doc = " constraint solver reference: limit       (njnt x mjNREF)"]
    pub(crate) jnt_solref: *mut mjtNum,
    #[doc = " constraint solver impedance: limit       (njnt x mjNIMP)"]
    pub(crate) jnt_solimp: *mut mjtNum,
    #[doc = " local anchor position                    (njnt x 3)"]
    pub(crate) jnt_pos: *mut mjtNum,
    #[doc = " local joint axis                         (njnt x 3)"]
    pub(crate) jnt_axis: *mut mjtNum,
    #[doc = " linear stiffness coefficient             (njnt x 1)"]
    pub(crate) jnt_stiffness: *mut mjtNum,
    #[doc = " high-order stiffness coefficients        (njnt x mjNPOLY)"]
    pub(crate) jnt_stiffnesspoly: *mut mjtNum,
    #[doc = " joint limits                             (njnt x 2)"]
    pub(crate) jnt_range: *mut mjtNum,
    #[doc = " range of total actuator force            (njnt x 2)"]
    pub(crate) jnt_actfrcrange: *mut mjtNum,
    #[doc = " min distance for limit detection         (njnt x 1)"]
    pub(crate) jnt_margin: *mut mjtNum,
    #[doc = " user data                                (njnt x nuser_jnt)"]
    pub(crate) jnt_user: *mut mjtNum,
    #[doc = " id of dof's body                         (nv x 1)"]
    pub(crate) dof_bodyid: *mut ::core::ffi::c_int,
    #[doc = " id of dof's joint                        (nv x 1)"]
    pub(crate) dof_jntid: *mut ::core::ffi::c_int,
    #[doc = " id of dof's parent; -1: none             (nv x 1)"]
    pub(crate) dof_parentid: *mut ::core::ffi::c_int,
    #[doc = " id of dof's kinematic tree               (nv x 1)"]
    pub(crate) dof_treeid: *mut ::core::ffi::c_int,
    #[doc = " dof address in M-diagonal                (nv x 1)"]
    pub(crate) dof_Madr: *mut ::core::ffi::c_int,
    #[doc = " number of consecutive simple dofs        (nv x 1)"]
    pub(crate) dof_simplenum: *mut ::core::ffi::c_int,
    #[doc = " constraint solver reference:frictionloss (nv x mjNREF)"]
    pub(crate) dof_solref: *mut mjtNum,
    #[doc = " constraint solver impedance:frictionloss (nv x mjNIMP)"]
    pub(crate) dof_solimp: *mut mjtNum,
    #[doc = " dof friction loss                        (nv x 1)"]
    pub(crate) dof_frictionloss: *mut mjtNum,
    #[doc = " dof armature inertia/mass                (nv x 1)"]
    pub(crate) dof_armature: *mut mjtNum,
    #[doc = " linear damping coefficient               (nv x 1)"]
    pub(crate) dof_damping: *mut mjtNum,
    #[doc = " high-order damping coefficients          (nv x mjNPOLY)"]
    pub(crate) dof_dampingpoly: *mut mjtNum,
    #[doc = " diag. inverse inertia in qpos0           (nv x 1)"]
    pub(crate) dof_invweight0: *mut mjtNum,
    #[doc = " diag. inertia in qpos0                   (nv x 1)"]
    pub(crate) dof_M0: *mut mjtNum,
    #[doc = " linear: 1; angular: approx. length scale (nv x 1)"]
    pub(crate) dof_length: *mut mjtNum,
    #[doc = " start addr of bodies                     (ntree x 1)"]
    pub(crate) tree_bodyadr: *mut ::core::ffi::c_int,
    #[doc = " number of bodies in tree                 (ntree x 1)"]
    pub(crate) tree_bodynum: *mut ::core::ffi::c_int,
    #[doc = " start addr of dofs                       (ntree x 1)"]
    pub(crate) tree_dofadr: *mut ::core::ffi::c_int,
    #[doc = " number of dofs in tree                   (ntree x 1)"]
    pub(crate) tree_dofnum: *mut ::core::ffi::c_int,
    #[doc = " sleep policy (mjtSleepPolicy)            (ntree x 1)"]
    pub(crate) tree_sleep_policy: *mut ::core::ffi::c_int,
    #[doc = " geometric type (mjtGeom)                 (ngeom x 1)"]
    pub(crate) geom_type: *mut ::core::ffi::c_int,
    #[doc = " geom contact type                        (ngeom x 1)"]
    pub(crate) geom_contype: *mut ::core::ffi::c_int,
    #[doc = " geom contact affinity                    (ngeom x 1)"]
    pub(crate) geom_conaffinity: *mut ::core::ffi::c_int,
    #[doc = " contact dimensionality (1, 3, 4, 6)      (ngeom x 1)"]
    pub(crate) geom_condim: *mut ::core::ffi::c_int,
    #[doc = " id of geom's body                        (ngeom x 1)"]
    pub(crate) geom_bodyid: *mut ::core::ffi::c_int,
    #[doc = " id of geom's mesh/hfield; -1: none       (ngeom x 1)"]
    pub(crate) geom_dataid: *mut ::core::ffi::c_int,
    #[doc = " material id for rendering; -1: none      (ngeom x 1)"]
    pub(crate) geom_matid: *mut ::core::ffi::c_int,
    #[doc = " group for visibility                     (ngeom x 1)"]
    pub(crate) geom_group: *mut ::core::ffi::c_int,
    #[doc = " geom contact priority                    (ngeom x 1)"]
    pub(crate) geom_priority: *mut ::core::ffi::c_int,
    #[doc = " plugin instance id; -1: not in use       (ngeom x 1)"]
    pub(crate) geom_plugin: *mut ::core::ffi::c_int,
    #[doc = " same frame as body (mjtSameframe)        (ngeom x 1)"]
    pub(crate) geom_sameframe: *mut mjtByte,
    #[doc = " mixing coef for solref/imp in geom pair  (ngeom x 1)"]
    pub(crate) geom_solmix: *mut mjtNum,
    #[doc = " constraint solver reference: contact     (ngeom x mjNREF)"]
    pub(crate) geom_solref: *mut mjtNum,
    #[doc = " constraint solver impedance: contact     (ngeom x mjNIMP)"]
    pub(crate) geom_solimp: *mut mjtNum,
    #[doc = " geom-specific size parameters            (ngeom x 3)"]
    pub(crate) geom_size: *mut mjtNum,
    #[doc = " bounding box, (center, size)             (ngeom x 6)"]
    pub(crate) geom_aabb: *mut mjtNum,
    #[doc = " radius of bounding sphere                (ngeom x 1)"]
    pub(crate) geom_rbound: *mut mjtNum,
    #[doc = " local position offset rel. to body       (ngeom x 3)"]
    pub(crate) geom_pos: *mut mjtNum,
    #[doc = " local orientation offset rel. to body    (ngeom x 4)"]
    pub(crate) geom_quat: *mut mjtNum,
    #[doc = " friction for (slide, spin, roll)         (ngeom x 3)"]
    pub(crate) geom_friction: *mut mjtNum,
    #[doc = " geometric inflation for contact          (ngeom x 1)"]
    pub(crate) geom_margin: *mut mjtNum,
    #[doc = " additional contact detection buffer      (ngeom x 1)"]
    pub(crate) geom_gap: *mut mjtNum,
    #[doc = " surface velocity in local frame: lin,ang (ngeom x 6)"]
    pub(crate) geom_surfacevel: *mut mjtNum,
    #[doc = " adhesive force of contacts               (ngeom x 1)"]
    pub(crate) geom_adhesion: *mut mjtNum,
    #[doc = " fluid interaction parameters             (ngeom x mjNFLUID)"]
    pub(crate) geom_fluid: *mut mjtNum,
    #[doc = " user data                                (ngeom x nuser_geom)"]
    pub(crate) geom_user: *mut mjtNum,
    #[doc = " rgba when material is omitted            (ngeom x 4)"]
    pub(crate) geom_rgba: *mut f32,
    #[doc = " geom type for rendering (mjtGeom)        (nsite x 1)"]
    pub(crate) site_type: *mut ::core::ffi::c_int,
    #[doc = " id of site's body                        (nsite x 1)"]
    pub(crate) site_bodyid: *mut ::core::ffi::c_int,
    #[doc = " material id for rendering; -1: none      (nsite x 1)"]
    pub(crate) site_matid: *mut ::core::ffi::c_int,
    #[doc = " group for visibility                     (nsite x 1)"]
    pub(crate) site_group: *mut ::core::ffi::c_int,
    #[doc = " same frame as body (mjtSameframe)        (nsite x 1)"]
    pub(crate) site_sameframe: *mut mjtByte,
    #[doc = " geom size for rendering                  (nsite x 3)"]
    pub(crate) site_size: *mut mjtNum,
    #[doc = " local position offset rel. to body       (nsite x 3)"]
    pub(crate) site_pos: *mut mjtNum,
    #[doc = " local orientation offset rel. to body    (nsite x 4)"]
    pub(crate) site_quat: *mut mjtNum,
    #[doc = " user data                                (nsite x nuser_site)"]
    pub(crate) site_user: *mut mjtNum,
    #[doc = " rgba when material is omitted            (nsite x 4)"]
    pub(crate) site_rgba: *mut f32,
    #[doc = " camera tracking mode (mjtCamLight)       (ncam x 1)"]
    pub(crate) cam_mode: *mut ::core::ffi::c_int,
    #[doc = " id of camera's body                      (ncam x 1)"]
    pub(crate) cam_bodyid: *mut ::core::ffi::c_int,
    #[doc = " id of targeted body; -1: none            (ncam x 1)"]
    pub(crate) cam_targetbodyid: *mut ::core::ffi::c_int,
    #[doc = " position rel. to body frame              (ncam x 3)"]
    pub(crate) cam_pos: *mut mjtNum,
    #[doc = " orientation rel. to body frame           (ncam x 4)"]
    pub(crate) cam_quat: *mut mjtNum,
    #[doc = " global position rel. to sub-com in qpos0 (ncam x 3)"]
    pub(crate) cam_poscom0: *mut mjtNum,
    #[doc = " global position rel. to body in qpos0    (ncam x 3)"]
    pub(crate) cam_pos0: *mut mjtNum,
    #[doc = " global orientation in qpos0              (ncam x 9)"]
    pub(crate) cam_mat0: *mut mjtNum,
    #[doc = " projection type (mjtProjection)          (ncam x 1)"]
    pub(crate) cam_projection: *mut ::core::ffi::c_int,
    #[doc = " y field-of-view (ortho ? len : deg)      (ncam x 1)"]
    pub(crate) cam_fovy: *mut mjtNum,
    #[doc = " inter-pupilary distance                  (ncam x 1)"]
    pub(crate) cam_ipd: *mut mjtNum,
    #[doc = " resolution: pixels [width, height]       (ncam x 2)"]
    pub(crate) cam_resolution: *mut ::core::ffi::c_int,
    #[doc = " output types (mjtCamOut bit flags)       (ncam x 1)"]
    pub(crate) cam_output: *mut ::core::ffi::c_int,
    #[doc = " sensor size: length [width, height]      (ncam x 2)"]
    pub(crate) cam_sensorsize: *mut f32,
    #[doc = " [focal length; principal point]          (ncam x 4)"]
    pub(crate) cam_intrinsic: *mut f32,
    #[doc = " user data                                (ncam x nuser_cam)"]
    pub(crate) cam_user: *mut mjtNum,
    #[doc = " light tracking mode (mjtCamLight)        (nlight x 1)"]
    pub(crate) light_mode: *mut ::core::ffi::c_int,
    #[doc = " id of light's body                       (nlight x 1)"]
    pub(crate) light_bodyid: *mut ::core::ffi::c_int,
    #[doc = " id of targeted body; -1: none            (nlight x 1)"]
    pub(crate) light_targetbodyid: *mut ::core::ffi::c_int,
    #[doc = " spot, directional, etc. (mjtLightType)   (nlight x 1)"]
    pub(crate) light_type: *mut ::core::ffi::c_int,
    #[doc = " texture id for image lights              (nlight x 1)"]
    pub(crate) light_texid: *mut ::core::ffi::c_int,
    #[doc = " does light cast shadows                  (nlight x 1)"]
    pub(crate) light_castshadow: *mut mjtBool,
    #[doc = " light radius for soft shadows            (nlight x 1)"]
    pub(crate) light_bulbradius: *mut f32,
    #[doc = " intensity, in candela                    (nlight x 1)"]
    pub(crate) light_intensity: *mut f32,
    #[doc = " range of effectiveness                   (nlight x 1)"]
    pub(crate) light_range: *mut f32,
    #[doc = " is light on                              (nlight x 1)"]
    pub(crate) light_active: *mut mjtBool,
    #[doc = " position rel. to body frame              (nlight x 3)"]
    pub(crate) light_pos: *mut mjtNum,
    #[doc = " direction rel. to body frame             (nlight x 3)"]
    pub(crate) light_dir: *mut mjtNum,
    #[doc = " global position rel. to sub-com in qpos0 (nlight x 3)"]
    pub(crate) light_poscom0: *mut mjtNum,
    #[doc = " global position rel. to body in qpos0    (nlight x 3)"]
    pub(crate) light_pos0: *mut mjtNum,
    #[doc = " global direction in qpos0                (nlight x 3)"]
    pub(crate) light_dir0: *mut mjtNum,
    #[doc = " OpenGL attenuation (quadratic model)     (nlight x 3)"]
    pub(crate) light_attenuation: *mut f32,
    #[doc = " OpenGL cutoff                            (nlight x 1)"]
    pub(crate) light_cutoff: *mut f32,
    #[doc = " OpenGL exponent                          (nlight x 1)"]
    pub(crate) light_exponent: *mut f32,
    #[doc = " ambient rgb (alpha=1)                    (nlight x 3)"]
    pub(crate) light_ambient: *mut f32,
    #[doc = " diffuse rgb (alpha=1)                    (nlight x 3)"]
    pub(crate) light_diffuse: *mut f32,
    #[doc = " specular rgb (alpha=1)                   (nlight x 3)"]
    pub(crate) light_specular: *mut f32,
    #[doc = " flex contact type                        (nflex x 1)"]
    pub(crate) flex_contype: *mut ::core::ffi::c_int,
    #[doc = " flex contact affinity                    (nflex x 1)"]
    pub(crate) flex_conaffinity: *mut ::core::ffi::c_int,
    #[doc = " contact dimensionality (1, 3, 4, 6)      (nflex x 1)"]
    pub(crate) flex_condim: *mut ::core::ffi::c_int,
    #[doc = " flex contact priority                    (nflex x 1)"]
    pub(crate) flex_priority: *mut ::core::ffi::c_int,
    #[doc = " mix coef for solref/imp in contact pair  (nflex x 1)"]
    pub(crate) flex_solmix: *mut mjtNum,
    #[doc = " constraint solver reference: contact     (nflex x mjNREF)"]
    pub(crate) flex_solref: *mut mjtNum,
    #[doc = " constraint solver impedance: contact     (nflex x mjNIMP)"]
    pub(crate) flex_solimp: *mut mjtNum,
    #[doc = " friction for (slide, spin, roll)         (nflex x 3)"]
    pub(crate) flex_friction: *mut mjtNum,
    #[doc = " geometric inflation for contact          (nflex x 1)"]
    pub(crate) flex_margin: *mut mjtNum,
    #[doc = " additional contact detection buffer      (nflex x 1)"]
    pub(crate) flex_gap: *mut mjtNum,
    #[doc = " internal flex collision enabled          (nflex x 1)"]
    pub(crate) flex_internal: *mut mjtBool,
    #[doc = " self collision mode (mjtFlexSelf)        (nflex x 1)"]
    pub(crate) flex_selfcollide: *mut ::core::ffi::c_int,
    #[doc = " number of active element layers, 3D only (nflex x 1)"]
    pub(crate) flex_activelayers: *mut ::core::ffi::c_int,
    #[doc = " passive collisions enabled               (nflex x 1)"]
    pub(crate) flex_passive: *mut ::core::ffi::c_int,
    #[doc = " 1: lines, 2: triangles, 3: tetrahedra    (nflex x 1)"]
    pub(crate) flex_dim: *mut ::core::ffi::c_int,
    #[doc = " material id for rendering                (nflex x 1)"]
    pub(crate) flex_matid: *mut ::core::ffi::c_int,
    #[doc = " group for visibility                     (nflex x 1)"]
    pub(crate) flex_group: *mut ::core::ffi::c_int,
    #[doc = " interpolation (0: vertex, 1: nodes)      (nflex x 1)"]
    pub(crate) flex_interp: *mut ::core::ffi::c_int,
    #[doc = " finite cell num per dimension            (nflex x 3)"]
    pub(crate) flex_cellnum: *mut ::core::ffi::c_int,
    #[doc = " first node address                       (nflex x 1)"]
    pub(crate) flex_nodeadr: *mut ::core::ffi::c_int,
    #[doc = " number of nodes                          (nflex x 1)"]
    pub(crate) flex_nodenum: *mut ::core::ffi::c_int,
    #[doc = " first vertex address                     (nflex x 1)"]
    pub(crate) flex_vertadr: *mut ::core::ffi::c_int,
    #[doc = " number of vertices                       (nflex x 1)"]
    pub(crate) flex_vertnum: *mut ::core::ffi::c_int,
    #[doc = " first edge address                       (nflex x 1)"]
    pub(crate) flex_edgeadr: *mut ::core::ffi::c_int,
    #[doc = " number of edges                          (nflex x 1)"]
    pub(crate) flex_edgenum: *mut ::core::ffi::c_int,
    #[doc = " first element address                    (nflex x 1)"]
    pub(crate) flex_elemadr: *mut ::core::ffi::c_int,
    #[doc = " number of elements                       (nflex x 1)"]
    pub(crate) flex_elemnum: *mut ::core::ffi::c_int,
    #[doc = " first element vertex id address          (nflex x 1)"]
    pub(crate) flex_elemdataadr: *mut ::core::ffi::c_int,
    #[doc = " stiffness matrix address                 (nflex x 1)"]
    pub(crate) flex_stiffnessadr: *mut ::core::ffi::c_int,
    #[doc = " first element edge id address            (nflex x 1)"]
    pub(crate) flex_elemedgeadr: *mut ::core::ffi::c_int,
    #[doc = " first bending data address               (nflex x 1)"]
    pub(crate) flex_bendingadr: *mut ::core::ffi::c_int,
    #[doc = " number of shells                         (nflex x 1)"]
    pub(crate) flex_shellnum: *mut ::core::ffi::c_int,
    #[doc = " first shell data address                 (nflex x 1)"]
    pub(crate) flex_shelldataadr: *mut ::core::ffi::c_int,
    #[doc = " first evpair address                     (nflex x 1)"]
    pub(crate) flex_evpairadr: *mut ::core::ffi::c_int,
    #[doc = " number of evpairs                        (nflex x 1)"]
    pub(crate) flex_evpairnum: *mut ::core::ffi::c_int,
    #[doc = " address in flex_texcoord; -1: none       (nflex x 1)"]
    pub(crate) flex_texcoordadr: *mut ::core::ffi::c_int,
    #[doc = " node body ids                            (nflexnode x 1)"]
    pub(crate) flex_nodebodyid: *mut ::core::ffi::c_int,
    #[doc = " vertex body ids                          (nflexvert x 1)"]
    pub(crate) flex_vertbodyid: *mut ::core::ffi::c_int,
    #[doc = " first edge address                       (nflexvert x 1)"]
    pub(crate) flex_vertedgeadr: *mut ::core::ffi::c_int,
    #[doc = " number of edges                          (nflexvert x 1)"]
    pub(crate) flex_vertedgenum: *mut ::core::ffi::c_int,
    #[doc = " edge indices                             (nflexedge x 2)"]
    pub(crate) flex_vertedge: *mut ::core::ffi::c_int,
    #[doc = " edge vertex ids (2 per edge)             (nflexedge x 2)"]
    pub(crate) flex_edge: *mut ::core::ffi::c_int,
    #[doc = " adjacent vertex ids (dim=2 only)         (nflexedge x 2)"]
    pub(crate) flex_edgeflap: *mut ::core::ffi::c_int,
    #[doc = " element vertex ids (dim+1 per elem)      (nflexelemdata x 1)"]
    pub(crate) flex_elem: *mut ::core::ffi::c_int,
    #[doc = " element texture coordinates (dim+1)      (nflexelemdata x 1)"]
    pub(crate) flex_elemtexcoord: *mut ::core::ffi::c_int,
    #[doc = " element edge ids                         (nflexelemedge x 1)"]
    pub(crate) flex_elemedge: *mut ::core::ffi::c_int,
    #[doc = " element distance from surface, 3D only   (nflexelem x 1)"]
    pub(crate) flex_elemlayer: *mut ::core::ffi::c_int,
    #[doc = " shell fragment vertex ids (dim per frag) (nflexshelldata x 1)"]
    pub(crate) flex_shell: *mut ::core::ffi::c_int,
    #[doc = " (element, vertex) collision pairs        (nflexevpair x 2)"]
    pub(crate) flex_evpair: *mut ::core::ffi::c_int,
    #[doc = " vertex positions in local body frames    (nflexvert x 3)"]
    pub(crate) flex_vert: *mut mjtNum,
    #[doc = " vertex positions in qpos0 on [0, 1]^d    (nflexvert x 3)"]
    pub(crate) flex_vert0: *mut mjtNum,
    #[doc = " inverse of reference shape matrix        (nflexvert x 4)"]
    pub(crate) flex_vertmetric: *mut mjtNum,
    #[doc = " node positions in local body frames      (nflexnode x 3)"]
    pub(crate) flex_node: *mut mjtNum,
    #[doc = " Cartesian node positions in qpos0        (nflexnode x 3)"]
    pub(crate) flex_node0: *mut mjtNum,
    #[doc = " edge lengths in qpos0                    (nflexedge x 1)"]
    pub(crate) flexedge_length0: *mut mjtNum,
    #[doc = " edge inv. weight in qpos0                (nflexedge x 1)"]
    pub(crate) flexedge_invweight0: *mut mjtNum,
    #[doc = " radius around primitive element          (nflex x 1)"]
    pub(crate) flex_radius: *mut mjtNum,
    #[doc = " vertex bounding box half sizes in qpos0  (nflex x 3)"]
    pub(crate) flex_size: *mut mjtNum,
    #[doc = " finite element stiffness matrix          (nflexstiffness x 1)"]
    pub(crate) flex_stiffness: *mut mjtNum,
    #[doc = " bending stiffness                        (nflexbending x 1)"]
    pub(crate) flex_bending: *mut mjtNum,
    #[doc = " constant metric factor row->dof address  (nefm0dof x 1)"]
    pub(crate) efm0_dofid: *mut ::core::ffi::c_int,
    #[doc = " constant metric factor row nonzeros      (nefm0dof x 1)"]
    pub(crate) efm0_L_rownnz: *mut ::core::ffi::c_int,
    #[doc = " constant metric factor row addresses     (nefm0dof x 1)"]
    pub(crate) efm0_L_rowadr: *mut ::core::ffi::c_int,
    #[doc = " constant metric factor column indices    (nefm0L x 1)"]
    pub(crate) efm0_L_colind: *mut ::core::ffi::c_int,
    #[doc = " factor of M + (dt^2+dt*d)*K_bend         (nefm0L x 1)"]
    pub(crate) efm0_L: *mut mjtNum,
    #[doc = " Rayleigh's damping coefficient           (nflex x 1)"]
    pub(crate) flex_damping: *mut mjtNum,
    #[doc = " edge stiffness                           (nflex x 1)"]
    pub(crate) flex_edgestiffness: *mut mjtNum,
    #[doc = " edge damping                             (nflex x 1)"]
    pub(crate) flex_edgedamping: *mut mjtNum,
    #[doc = " 0:none, 1:edges, 2:vertices, 3:strain    (nflex x 1)"]
    pub(crate) flex_edgeequality: *mut ::core::ffi::c_int,
    #[doc = " are all vertices in the same body        (nflex x 1)"]
    pub(crate) flex_rigid: *mut mjtBool,
    #[doc = " are both edge vertices in same body      (nflexedge x 1)"]
    pub(crate) flexedge_rigid: *mut mjtBool,
    #[doc = " are all vertex coordinates (0,0,0)       (nflex x 1)"]
    pub(crate) flex_centered: *mut mjtBool,
    #[doc = " render flex skin with flat shading       (nflex x 1)"]
    pub(crate) flex_flatskin: *mut mjtBool,
    #[doc = " address of bvh root; -1: no bvh          (nflex x 1)"]
    pub(crate) flex_bvhadr: *mut ::core::ffi::c_int,
    #[doc = " number of bounding volumes               (nflex x 1)"]
    pub(crate) flex_bvhnum: *mut ::core::ffi::c_int,
    #[doc = " number of non-zeros in Jacobian row      (nflexedge x 1)"]
    pub(crate) flexedge_J_rownnz: *mut ::core::ffi::c_int,
    #[doc = " row start address in colind array        (nflexedge x 1)"]
    pub(crate) flexedge_J_rowadr: *mut ::core::ffi::c_int,
    #[doc = " column indices in sparse Jacobian        (nJfe x 1)"]
    pub(crate) flexedge_J_colind: *mut ::core::ffi::c_int,
    #[doc = " number of non-zeros in Jacobian row      (nflexvert x 2)"]
    pub(crate) flexvert_J_rownnz: *mut ::core::ffi::c_int,
    #[doc = " row start address in colind array        (nflexvert x 2)"]
    pub(crate) flexvert_J_rowadr: *mut ::core::ffi::c_int,
    #[doc = " column indices in sparse Jacobian        (nJfv x 2)"]
    pub(crate) flexvert_J_colind: *mut ::core::ffi::c_int,
    #[doc = " rgba when material is omitted            (nflex x 4)"]
    pub(crate) flex_rgba: *mut f32,
    #[doc = " vertex texture coordinates               (nflextexcoord x 2)"]
    pub(crate) flex_texcoord: *mut f32,
    #[doc = " first vertex address                     (nmesh x 1)"]
    pub(crate) mesh_vertadr: *mut ::core::ffi::c_int,
    #[doc = " number of vertices                       (nmesh x 1)"]
    pub(crate) mesh_vertnum: *mut ::core::ffi::c_int,
    #[doc = " first face address                       (nmesh x 1)"]
    pub(crate) mesh_faceadr: *mut ::core::ffi::c_int,
    #[doc = " number of faces                          (nmesh x 1)"]
    pub(crate) mesh_facenum: *mut ::core::ffi::c_int,
    #[doc = " address of bvh root                      (nmesh x 1)"]
    pub(crate) mesh_bvhadr: *mut ::core::ffi::c_int,
    #[doc = " number of bvh                            (nmesh x 1)"]
    pub(crate) mesh_bvhnum: *mut ::core::ffi::c_int,
    #[doc = " address of octree root                   (nmesh x 1)"]
    pub(crate) mesh_octadr: *mut ::core::ffi::c_int,
    #[doc = " number of octree nodes                   (nmesh x 1)"]
    pub(crate) mesh_octnum: *mut ::core::ffi::c_int,
    #[doc = " first normal address                     (nmesh x 1)"]
    pub(crate) mesh_normaladr: *mut ::core::ffi::c_int,
    #[doc = " number of normals                        (nmesh x 1)"]
    pub(crate) mesh_normalnum: *mut ::core::ffi::c_int,
    #[doc = " texcoord data address; -1: no texcoord   (nmesh x 1)"]
    pub(crate) mesh_texcoordadr: *mut ::core::ffi::c_int,
    #[doc = " number of texcoord                       (nmesh x 1)"]
    pub(crate) mesh_texcoordnum: *mut ::core::ffi::c_int,
    #[doc = " graph data address; -1: no graph         (nmesh x 1)"]
    pub(crate) mesh_graphadr: *mut ::core::ffi::c_int,
    #[doc = " vertex positions for all meshes          (nmeshvert x 3)"]
    pub(crate) mesh_vert: *mut f32,
    #[doc = " normals for all meshes                   (nmeshnormal x 3)"]
    pub(crate) mesh_normal: *mut f32,
    #[doc = " vertex texcoords for all meshes          (nmeshtexcoord x 2)"]
    pub(crate) mesh_texcoord: *mut f32,
    #[doc = " vertex face data                         (nmeshface x 3)"]
    pub(crate) mesh_face: *mut ::core::ffi::c_int,
    #[doc = " normal face data                         (nmeshface x 3)"]
    pub(crate) mesh_facenormal: *mut ::core::ffi::c_int,
    #[doc = " texture face data                        (nmeshface x 3)"]
    pub(crate) mesh_facetexcoord: *mut ::core::ffi::c_int,
    #[doc = " convex graph data                        (nmeshgraph x 1)"]
    pub(crate) mesh_graph: *mut ::core::ffi::c_int,
    #[doc = " scaling applied to asset vertices        (nmesh x 3)"]
    pub(crate) mesh_scale: *mut mjtNum,
    #[doc = " translation applied to asset vertices    (nmesh x 3)"]
    pub(crate) mesh_pos: *mut mjtNum,
    #[doc = " rotation applied to asset vertices       (nmesh x 4)"]
    pub(crate) mesh_quat: *mut mjtNum,
    #[doc = " address of asset path for mesh; -1: none (nmesh x 1)"]
    pub(crate) mesh_pathadr: *mut ::core::ffi::c_int,
    #[doc = " number of polygons per mesh              (nmesh x 1)"]
    pub(crate) mesh_polynum: *mut ::core::ffi::c_int,
    #[doc = " first polygon address per mesh           (nmesh x 1)"]
    pub(crate) mesh_polyadr: *mut ::core::ffi::c_int,
    #[doc = " all polygon normals                      (nmeshpoly x 3)"]
    pub(crate) mesh_polynormal: *mut mjtNum,
    #[doc = " polygon vertex start address             (nmeshpoly x 1)"]
    pub(crate) mesh_polyvertadr: *mut ::core::ffi::c_int,
    #[doc = " number of vertices per polygon           (nmeshpoly x 1)"]
    pub(crate) mesh_polyvertnum: *mut ::core::ffi::c_int,
    #[doc = " all polygon vertices                     (nmeshpolyvert x 1)"]
    pub(crate) mesh_polyvert: *mut ::core::ffi::c_int,
    #[doc = " first polygon address per vertex         (nmeshvert x 1)"]
    pub(crate) mesh_polymapadr: *mut ::core::ffi::c_int,
    #[doc = " number of polygons per vertex            (nmeshvert x 1)"]
    pub(crate) mesh_polymapnum: *mut ::core::ffi::c_int,
    #[doc = " vertex to polygon map                    (nmeshpolymap x 1)"]
    pub(crate) mesh_polymap: *mut ::core::ffi::c_int,
    #[doc = " skin material id; -1: none               (nskin x 1)"]
    pub(crate) skin_matid: *mut ::core::ffi::c_int,
    #[doc = " group for visibility                     (nskin x 1)"]
    pub(crate) skin_group: *mut ::core::ffi::c_int,
    #[doc = " skin rgba                                (nskin x 4)"]
    pub(crate) skin_rgba: *mut f32,
    #[doc = " inflate skin in normal direction         (nskin x 1)"]
    pub(crate) skin_inflate: *mut f32,
    #[doc = " first vertex address                     (nskin x 1)"]
    pub(crate) skin_vertadr: *mut ::core::ffi::c_int,
    #[doc = " number of vertices                       (nskin x 1)"]
    pub(crate) skin_vertnum: *mut ::core::ffi::c_int,
    #[doc = " texcoord data address; -1: no texcoord   (nskin x 1)"]
    pub(crate) skin_texcoordadr: *mut ::core::ffi::c_int,
    #[doc = " first face address                       (nskin x 1)"]
    pub(crate) skin_faceadr: *mut ::core::ffi::c_int,
    #[doc = " number of faces                          (nskin x 1)"]
    pub(crate) skin_facenum: *mut ::core::ffi::c_int,
    #[doc = " first bone in skin                       (nskin x 1)"]
    pub(crate) skin_boneadr: *mut ::core::ffi::c_int,
    #[doc = " number of bones in skin                  (nskin x 1)"]
    pub(crate) skin_bonenum: *mut ::core::ffi::c_int,
    #[doc = " vertex positions for all skin meshes     (nskinvert x 3)"]
    pub(crate) skin_vert: *mut f32,
    #[doc = " vertex texcoords for all skin meshes     (nskintexvert x 2)"]
    pub(crate) skin_texcoord: *mut f32,
    #[doc = " triangle faces for all skin meshes       (nskinface x 3)"]
    pub(crate) skin_face: *mut ::core::ffi::c_int,
    #[doc = " first vertex in each bone                (nskinbone x 1)"]
    pub(crate) skin_bonevertadr: *mut ::core::ffi::c_int,
    #[doc = " number of vertices in each bone          (nskinbone x 1)"]
    pub(crate) skin_bonevertnum: *mut ::core::ffi::c_int,
    #[doc = " bind pos of each bone                    (nskinbone x 3)"]
    pub(crate) skin_bonebindpos: *mut f32,
    #[doc = " bind quat of each bone                   (nskinbone x 4)"]
    pub(crate) skin_bonebindquat: *mut f32,
    #[doc = " body id of each bone                     (nskinbone x 1)"]
    pub(crate) skin_bonebodyid: *mut ::core::ffi::c_int,
    #[doc = " mesh ids of vertices in each bone        (nskinbonevert x 1)"]
    pub(crate) skin_bonevertid: *mut ::core::ffi::c_int,
    #[doc = " weights of vertices in each bone         (nskinbonevert x 1)"]
    pub(crate) skin_bonevertweight: *mut f32,
    #[doc = " address of asset path for skin; -1: none (nskin x 1)"]
    pub(crate) skin_pathadr: *mut ::core::ffi::c_int,
    #[doc = " (x, y, z_top, z_bottom)                  (nhfield x 4)"]
    pub(crate) hfield_size: *mut mjtNum,
    #[doc = " number of rows in grid                   (nhfield x 1)"]
    pub(crate) hfield_nrow: *mut ::core::ffi::c_int,
    #[doc = " number of columns in grid                (nhfield x 1)"]
    pub(crate) hfield_ncol: *mut ::core::ffi::c_int,
    #[doc = " address in hfield_data                   (nhfield x 1)"]
    pub(crate) hfield_adr: *mut ::core::ffi::c_int,
    #[doc = " elevation data                           (nhfielddata x 1)"]
    pub(crate) hfield_data: *mut f32,
    #[doc = " address of hfield asset path; -1: none   (nhfield x 1)"]
    pub(crate) hfield_pathadr: *mut ::core::ffi::c_int,
    #[doc = " texture type (mjtTexture)                (ntex x 1)"]
    pub(crate) tex_type: *mut ::core::ffi::c_int,
    #[doc = " texture colorspace (mjtColorSpace)       (ntex x 1)"]
    pub(crate) tex_colorspace: *mut ::core::ffi::c_int,
    #[doc = " number of rows in texture image          (ntex x 1)"]
    pub(crate) tex_height: *mut ::core::ffi::c_int,
    #[doc = " number of columns in texture image       (ntex x 1)"]
    pub(crate) tex_width: *mut ::core::ffi::c_int,
    #[doc = " number of channels in texture image      (ntex x 1)"]
    pub(crate) tex_nchannel: *mut ::core::ffi::c_int,
    #[doc = " start address in tex_data                (ntex x 1)"]
    pub(crate) tex_adr: *mut mjtSize,
    #[doc = " pixel values                             (ntexdata x 1)"]
    pub(crate) tex_data: *mut mjtByte,
    #[doc = " address of texture asset path; -1: none  (ntex x 1)"]
    pub(crate) tex_pathadr: *mut ::core::ffi::c_int,
    #[doc = " indices of textures; -1: none            (nmat x mjNTEXROLE)"]
    pub(crate) mat_texid: *mut ::core::ffi::c_int,
    #[doc = " make texture cube uniform                (nmat x 1)"]
    pub(crate) mat_texuniform: *mut mjtBool,
    #[doc = " texture repetition for 2d mapping        (nmat x 2)"]
    pub(crate) mat_texrepeat: *mut f32,
    #[doc = " emission (x rgb)                         (nmat x 1)"]
    pub(crate) mat_emission: *mut f32,
    #[doc = " specular (x white)                       (nmat x 1)"]
    pub(crate) mat_specular: *mut f32,
    #[doc = " shininess coef                           (nmat x 1)"]
    pub(crate) mat_shininess: *mut f32,
    #[doc = " reflectance (0: disable)                 (nmat x 1)"]
    pub(crate) mat_reflectance: *mut f32,
    #[doc = " metallic coef                            (nmat x 1)"]
    pub(crate) mat_metallic: *mut f32,
    #[doc = " roughness coef                           (nmat x 1)"]
    pub(crate) mat_roughness: *mut f32,
    #[doc = " rgba                                     (nmat x 4)"]
    pub(crate) mat_rgba: *mut f32,
    #[doc = " contact dimensionality                   (npair x 1)"]
    pub(crate) pair_dim: *mut ::core::ffi::c_int,
    #[doc = " id of geom1                              (npair x 1)"]
    pub(crate) pair_geom1: *mut ::core::ffi::c_int,
    #[doc = " id of geom2                              (npair x 1)"]
    pub(crate) pair_geom2: *mut ::core::ffi::c_int,
    #[doc = " body1 << 16 + body2                      (npair x 1)"]
    pub(crate) pair_signature: *mut ::core::ffi::c_int,
    #[doc = " solver reference: contact normal         (npair x mjNREF)"]
    pub(crate) pair_solref: *mut mjtNum,
    #[doc = " solver reference: contact friction       (npair x mjNREF)"]
    pub(crate) pair_solreffriction: *mut mjtNum,
    #[doc = " solver impedance: contact                (npair x mjNIMP)"]
    pub(crate) pair_solimp: *mut mjtNum,
    #[doc = " geometric inflation for contact          (npair x 1)"]
    pub(crate) pair_margin: *mut mjtNum,
    #[doc = " additional contact detection buffer      (npair x 1)"]
    pub(crate) pair_gap: *mut mjtNum,
    #[doc = " adhesive force of contacts               (npair x 1)"]
    pub(crate) pair_adhesion: *mut mjtNum,
    #[doc = " tangent1, 2, spin, roll1, 2              (npair x 5)"]
    pub(crate) pair_friction: *mut mjtNum,
    #[doc = " body1 << 16 + body2                      (nexclude x 1)"]
    pub(crate) exclude_signature: *mut ::core::ffi::c_int,
    #[doc = " constraint type (mjtEq)                  (neq x 1)"]
    pub(crate) eq_type: *mut ::core::ffi::c_int,
    #[doc = " id of object 1                           (neq x 1)"]
    pub(crate) eq_obj1id: *mut ::core::ffi::c_int,
    #[doc = " id of object 2                           (neq x 1)"]
    pub(crate) eq_obj2id: *mut ::core::ffi::c_int,
    #[doc = " type of both objects (mjtObj)            (neq x 1)"]
    pub(crate) eq_objtype: *mut ::core::ffi::c_int,
    #[doc = " initial enable/disable constraint state  (neq x 1)"]
    pub(crate) eq_active0: *mut mjtBool,
    #[doc = " constraint solver reference              (neq x mjNREF)"]
    pub(crate) eq_solref: *mut mjtNum,
    #[doc = " constraint solver impedance              (neq x mjNIMP)"]
    pub(crate) eq_solimp: *mut mjtNum,
    #[doc = " numeric data for constraint              (neq x mjNEQDATA)"]
    pub(crate) eq_data: *mut mjtNum,
    #[doc = " address of first object in tendon's path (ntendon x 1)"]
    pub(crate) tendon_adr: *mut ::core::ffi::c_int,
    #[doc = " number of objects in tendon's path       (ntendon x 1)"]
    pub(crate) tendon_num: *mut ::core::ffi::c_int,
    #[doc = " material id for rendering                (ntendon x 1)"]
    pub(crate) tendon_matid: *mut ::core::ffi::c_int,
    #[doc = " actuator contributing damping / armature (ntendon x 1)"]
    pub(crate) tendon_actuatorid: *mut ::core::ffi::c_int,
    #[doc = " group for visibility                     (ntendon x 1)"]
    pub(crate) tendon_group: *mut ::core::ffi::c_int,
    #[doc = " number of trees along tendon's path      (ntendon x 1)"]
    pub(crate) tendon_treenum: *mut ::core::ffi::c_int,
    #[doc = " first two trees along tendon's path      (ntendon x 2)"]
    pub(crate) tendon_treeid: *mut ::core::ffi::c_int,
    #[doc = " number of non-zeros in Jacobian row      (ntendon x 1)"]
    pub(crate) ten_J_rownnz: *mut ::core::ffi::c_int,
    #[doc = " row start address in colind array        (ntendon x 1)"]
    pub(crate) ten_J_rowadr: *mut ::core::ffi::c_int,
    #[doc = " column indices in sparse Jacobian        (nJten x 1)"]
    pub(crate) ten_J_colind: *mut ::core::ffi::c_int,
    #[doc = " does tendon have length limits           (ntendon x 1)"]
    pub(crate) tendon_limited: *mut mjtBool,
    #[doc = " does tendon have actuator force limits   (ntendon x 1)"]
    pub(crate) tendon_actfrclimited: *mut mjtBool,
    #[doc = " width for rendering                      (ntendon x 1)"]
    pub(crate) tendon_width: *mut mjtNum,
    #[doc = " constraint solver reference: limit       (ntendon x mjNREF)"]
    pub(crate) tendon_solref_lim: *mut mjtNum,
    #[doc = " constraint solver impedance: limit       (ntendon x mjNIMP)"]
    pub(crate) tendon_solimp_lim: *mut mjtNum,
    #[doc = " constraint solver reference: friction    (ntendon x mjNREF)"]
    pub(crate) tendon_solref_fri: *mut mjtNum,
    #[doc = " constraint solver impedance: friction    (ntendon x mjNIMP)"]
    pub(crate) tendon_solimp_fri: *mut mjtNum,
    #[doc = " tendon length limits                     (ntendon x 2)"]
    pub(crate) tendon_range: *mut mjtNum,
    #[doc = " range of total actuator force            (ntendon x 2)"]
    pub(crate) tendon_actfrcrange: *mut mjtNum,
    #[doc = " min distance for limit detection         (ntendon x 1)"]
    pub(crate) tendon_margin: *mut mjtNum,
    #[doc = " linear stiffness coefficient             (ntendon x 1)"]
    pub(crate) tendon_stiffness: *mut mjtNum,
    #[doc = " high-order stiffness coefficients        (ntendon x mjNPOLY)"]
    pub(crate) tendon_stiffnesspoly: *mut mjtNum,
    #[doc = " linear damping coefficient               (ntendon x 1)"]
    pub(crate) tendon_damping: *mut mjtNum,
    #[doc = " high-order damping coefficients          (ntendon x mjNPOLY)"]
    pub(crate) tendon_dampingpoly: *mut mjtNum,
    #[doc = " inertia associated with tendon velocity  (ntendon x 1)"]
    pub(crate) tendon_armature: *mut mjtNum,
    #[doc = " loss due to friction                     (ntendon x 1)"]
    pub(crate) tendon_frictionloss: *mut mjtNum,
    #[doc = " spring resting length range              (ntendon x 2)"]
    pub(crate) tendon_lengthspring: *mut mjtNum,
    #[doc = " tendon length in qpos0                   (ntendon x 1)"]
    pub(crate) tendon_length0: *mut mjtNum,
    #[doc = " inv. weight in qpos0                     (ntendon x 1)"]
    pub(crate) tendon_invweight0: *mut mjtNum,
    #[doc = " user data                                (ntendon x nuser_tendon)"]
    pub(crate) tendon_user: *mut mjtNum,
    #[doc = " rgba when material is omitted            (ntendon x 4)"]
    pub(crate) tendon_rgba: *mut f32,
    #[doc = " wrap object type (mjtWrap)               (nwrap x 1)"]
    pub(crate) wrap_type: *mut ::core::ffi::c_int,
    #[doc = " object id: geom, site, joint             (nwrap x 1)"]
    pub(crate) wrap_objid: *mut ::core::ffi::c_int,
    #[doc = " divisor, joint coef, or site id          (nwrap x 1)"]
    pub(crate) wrap_prm: *mut mjtNum,
    #[doc = " transmission type (mjtTrn)               (nactuator x 1)"]
    pub(crate) actuator_trntype: *mut ::core::ffi::c_int,
    #[doc = " dynamics type (mjtDyn)                   (nactuator x 1)"]
    pub(crate) actuator_dyntype: *mut ::core::ffi::c_int,
    #[doc = " gain type (mjtGain)                      (nactuator x 1)"]
    pub(crate) actuator_gaintype: *mut ::core::ffi::c_int,
    #[doc = " bias type (mjtBias)                      (nactuator x 1)"]
    pub(crate) actuator_biastype: *mut ::core::ffi::c_int,
    #[doc = " address of first control                 (nactuator x 1)"]
    pub(crate) actuator_ctrladr: *mut ::core::ffi::c_int,
    #[doc = " number of controls                       (nactuator x 1)"]
    pub(crate) actuator_ctrlnum: *mut ::core::ffi::c_int,
    #[doc = " input signature, scoped by gaintype      (nactuator x 1)"]
    pub(crate) actuator_ctrlspec: *mut ::core::ffi::c_int,
    #[doc = " address of first force output            (nactuator x 1)"]
    pub(crate) actuator_outadr: *mut ::core::ffi::c_int,
    #[doc = " number of force outputs, from trntype    (nactuator x 1)"]
    pub(crate) actuator_outnum: *mut ::core::ffi::c_int,
    #[doc = " first activation address; -1: stateless  (nactuator x 1)"]
    pub(crate) actuator_actadr: *mut ::core::ffi::c_int,
    #[doc = " number of activation variables           (nactuator x 1)"]
    pub(crate) actuator_actnum: *mut ::core::ffi::c_int,
    #[doc = " transmission id: joint, tendon, site     (nactuator x 2)"]
    pub(crate) actuator_trnid: *mut ::core::ffi::c_int,
    #[doc = " crank length for slider-crank            (nactuator x 1)"]
    pub(crate) actuator_cranklength: *mut mjtNum,
    #[doc = " dynamics parameters                      (nactuator x mjNDYN)"]
    pub(crate) actuator_dynprm: *mut mjtNum,
    #[doc = " gain parameters                          (nactuator x mjNGAIN)"]
    pub(crate) actuator_gainprm: *mut mjtNum,
    #[doc = " bias parameters                          (nactuator x mjNBIAS)"]
    pub(crate) actuator_biasprm: *mut mjtNum,
    #[doc = " is activation limited                    (nactuator x 1)"]
    pub(crate) actuator_actlimited: *mut mjtBool,
    #[doc = " range of activations                     (nactuator x 2)"]
    pub(crate) actuator_actrange: *mut mjtNum,
    #[doc = " step activation before force             (nactuator x 1)"]
    pub(crate) actuator_actearly: *mut mjtBool,
    #[doc = " history buffer: [nsample, interp]        (nactuator x 2)"]
    pub(crate) actuator_history: *mut ::core::ffi::c_int,
    #[doc = " address in history buffer; -1: none      (nactuator x 1)"]
    pub(crate) actuator_historyadr: *mut ::core::ffi::c_int,
    #[doc = " delay time; 0: no delay                  (nactuator x 1)"]
    pub(crate) actuator_delay: *mut mjtNum,
    #[doc = " linear damping coefficient               (nactuator x 1)"]
    pub(crate) actuator_damping: *mut mjtNum,
    #[doc = " high-order damping coefficients          (nactuator x mjNPOLY)"]
    pub(crate) actuator_dampingpoly: *mut mjtNum,
    #[doc = " armature added to target (joint, tendon) (nactuator x 1)"]
    pub(crate) actuator_armature: *mut mjtNum,
    #[doc = " group for visibility                     (nactuator x 1)"]
    pub(crate) actuator_group: *mut ::core::ffi::c_int,
    #[doc = " user data                                (nactuator x nuser_actuator)"]
    pub(crate) actuator_user: *mut mjtNum,
    #[doc = " plugin instance id; -1: not a plugin     (nactuator x 1)"]
    pub(crate) actuator_plugin: *mut ::core::ffi::c_int,
    #[doc = " is force limited                         (nactuator x 1)"]
    pub(crate) actuator_forcelimited: *mut mjtBool,
    #[doc = " range of forces                          (nactuator x 2)"]
    pub(crate) actuator_forcerange: *mut mjtNum,
    #[doc = " is control limited                       (nu x 1)"]
    pub(crate) actuator_ctrllimited: *mut mjtBool,
    #[doc = " range of controls                        (nu x 2)"]
    pub(crate) actuator_ctrlrange: *mut mjtNum,
    #[doc = " scale length and transmitted force       (nout x 6)"]
    pub(crate) actuator_gear: *mut mjtNum,
    #[doc = " acceleration from unit force in qpos0    (nout x 1)"]
    pub(crate) actuator_acc0: *mut mjtNum,
    #[doc = " actuator length in qpos0                 (nout x 1)"]
    pub(crate) actuator_length0: *mut mjtNum,
    #[doc = " feasible actuator length range           (nout x 2)"]
    pub(crate) actuator_lengthrange: *mut mjtNum,
    #[doc = " sensor type (mjtSensor)                  (nsensor x 1)"]
    pub(crate) sensor_type: *mut ::core::ffi::c_int,
    #[doc = " numeric data type (mjtDataType)          (nsensor x 1)"]
    pub(crate) sensor_datatype: *mut ::core::ffi::c_int,
    #[doc = " required compute stage (mjtStage)        (nsensor x 1)"]
    pub(crate) sensor_needstage: *mut ::core::ffi::c_int,
    #[doc = " type of sensorized object (mjtObj)       (nsensor x 1)"]
    pub(crate) sensor_objtype: *mut ::core::ffi::c_int,
    #[doc = " id of sensorized object                  (nsensor x 1)"]
    pub(crate) sensor_objid: *mut ::core::ffi::c_int,
    #[doc = " type of reference frame (mjtObj)         (nsensor x 1)"]
    pub(crate) sensor_reftype: *mut ::core::ffi::c_int,
    #[doc = " id of reference frame; -1: global frame  (nsensor x 1)"]
    pub(crate) sensor_refid: *mut ::core::ffi::c_int,
    #[doc = " sensor parameters                        (nsensor x mjNSENS)"]
    pub(crate) sensor_intprm: *mut ::core::ffi::c_int,
    #[doc = " number of scalar outputs                 (nsensor x 1)"]
    pub(crate) sensor_dim: *mut ::core::ffi::c_int,
    #[doc = " address in sensor array                  (nsensor x 1)"]
    pub(crate) sensor_adr: *mut ::core::ffi::c_int,
    #[doc = " cutoff for real and positive; 0: ignore  (nsensor x 1)"]
    pub(crate) sensor_cutoff: *mut mjtNum,
    #[doc = " noise standard deviation                 (nsensor x 1)"]
    pub(crate) sensor_noise: *mut mjtNum,
    #[doc = " history buffer: [nsample, interp]        (nsensor x 2)"]
    pub(crate) sensor_history: *mut ::core::ffi::c_int,
    #[doc = " address in history buffer; -1: none      (nsensor x 1)"]
    pub(crate) sensor_historyadr: *mut ::core::ffi::c_int,
    #[doc = " delay time in seconds; 0: no delay       (nsensor x 1)"]
    pub(crate) sensor_delay: *mut mjtNum,
    #[doc = " interval: [period, phase] in seconds     (nsensor x 2)"]
    pub(crate) sensor_interval: *mut mjtNum,
    #[doc = " user data                                (nsensor x nuser_sensor)"]
    pub(crate) sensor_user: *mut mjtNum,
    #[doc = " plugin instance id; -1: not a plugin     (nsensor x 1)"]
    pub(crate) sensor_plugin: *mut ::core::ffi::c_int,
    #[doc = " globally registered plugin slot number   (nplugin x 1)"]
    pub(crate) plugin: *mut ::core::ffi::c_int,
    #[doc = " address in the plugin state array        (nplugin x 1)"]
    pub(crate) plugin_stateadr: *mut ::core::ffi::c_int,
    #[doc = " number of states in the plugin instance  (nplugin x 1)"]
    pub(crate) plugin_statenum: *mut ::core::ffi::c_int,
    #[doc = " config attributes of plugin instances    (npluginattr x 1)"]
    pub(crate) plugin_attr: *mut ::core::ffi::c_char,
    #[doc = " address to each instance's config attrib (nplugin x 1)"]
    pub(crate) plugin_attradr: *mut ::core::ffi::c_int,
    #[doc = " address of field in numeric_data         (nnumeric x 1)"]
    pub(crate) numeric_adr: *mut ::core::ffi::c_int,
    #[doc = " size of numeric field                    (nnumeric x 1)"]
    pub(crate) numeric_size: *mut ::core::ffi::c_int,
    #[doc = " array of all numeric fields              (nnumericdata x 1)"]
    pub(crate) numeric_data: *mut mjtNum,
    #[doc = " address of text in text_data             (ntext x 1)"]
    pub(crate) text_adr: *mut ::core::ffi::c_int,
    #[doc = " size of text field (strlen+1)            (ntext x 1)"]
    pub(crate) text_size: *mut ::core::ffi::c_int,
    #[doc = " array of all text fields (0-terminated)  (ntextdata x 1)"]
    pub(crate) text_data: *mut ::core::ffi::c_char,
    #[doc = " address of text in text_data             (ntuple x 1)"]
    pub(crate) tuple_adr: *mut ::core::ffi::c_int,
    #[doc = " number of objects in tuple               (ntuple x 1)"]
    pub(crate) tuple_size: *mut ::core::ffi::c_int,
    #[doc = " array of object types in all tuples      (ntupledata x 1)"]
    pub(crate) tuple_objtype: *mut ::core::ffi::c_int,
    #[doc = " array of object ids in all tuples        (ntupledata x 1)"]
    pub(crate) tuple_objid: *mut ::core::ffi::c_int,
    #[doc = " array of object params in all tuples     (ntupledata x 1)"]
    pub(crate) tuple_objprm: *mut mjtNum,
    #[doc = " key time                                 (nkey x 1)"]
    pub(crate) key_time: *mut mjtNum,
    #[doc = " key position                             (nkey x nq)"]
    pub(crate) key_qpos: *mut mjtNum,
    #[doc = " key velocity                             (nkey x nv)"]
    pub(crate) key_qvel: *mut mjtNum,
    #[doc = " key activation                           (nkey x na)"]
    pub(crate) key_act: *mut mjtNum,
    #[doc = " key mocap position                       (nkey x nmocap*3)"]
    pub(crate) key_mpos: *mut mjtNum,
    #[doc = " key mocap quaternion                     (nkey x nmocap*4)"]
    pub(crate) key_mquat: *mut mjtNum,
    #[doc = " key control                              (nkey x nu)"]
    pub(crate) key_ctrl: *mut mjtNum,
    #[doc = " body name pointers                       (nbody x 1)"]
    pub(crate) name_bodyadr: *mut ::core::ffi::c_int,
    #[doc = " joint name pointers                      (njnt x 1)"]
    pub(crate) name_jntadr: *mut ::core::ffi::c_int,
    #[doc = " geom name pointers                       (ngeom x 1)"]
    pub(crate) name_geomadr: *mut ::core::ffi::c_int,
    #[doc = " site name pointers                       (nsite x 1)"]
    pub(crate) name_siteadr: *mut ::core::ffi::c_int,
    #[doc = " camera name pointers                     (ncam x 1)"]
    pub(crate) name_camadr: *mut ::core::ffi::c_int,
    #[doc = " light name pointers                      (nlight x 1)"]
    pub(crate) name_lightadr: *mut ::core::ffi::c_int,
    #[doc = " flex name pointers                       (nflex x 1)"]
    pub(crate) name_flexadr: *mut ::core::ffi::c_int,
    #[doc = " mesh name pointers                       (nmesh x 1)"]
    pub(crate) name_meshadr: *mut ::core::ffi::c_int,
    #[doc = " skin name pointers                       (nskin x 1)"]
    pub(crate) name_skinadr: *mut ::core::ffi::c_int,
    #[doc = " hfield name pointers                     (nhfield x 1)"]
    pub(crate) name_hfieldadr: *mut ::core::ffi::c_int,
    #[doc = " texture name pointers                    (ntex x 1)"]
    pub(crate) name_texadr: *mut ::core::ffi::c_int,
    #[doc = " material name pointers                   (nmat x 1)"]
    pub(crate) name_matadr: *mut ::core::ffi::c_int,
    #[doc = " geom pair name pointers                  (npair x 1)"]
    pub(crate) name_pairadr: *mut ::core::ffi::c_int,
    #[doc = " exclude name pointers                    (nexclude x 1)"]
    pub(crate) name_excludeadr: *mut ::core::ffi::c_int,
    #[doc = " equality constraint name pointers        (neq x 1)"]
    pub(crate) name_eqadr: *mut ::core::ffi::c_int,
    #[doc = " tendon name pointers                     (ntendon x 1)"]
    pub(crate) name_tendonadr: *mut ::core::ffi::c_int,
    #[doc = " actuator name pointers                   (nactuator x 1)"]
    pub(crate) name_actuatoradr: *mut ::core::ffi::c_int,
    #[doc = " sensor name pointers                     (nsensor x 1)"]
    pub(crate) name_sensoradr: *mut ::core::ffi::c_int,
    #[doc = " numeric name pointers                    (nnumeric x 1)"]
    pub(crate) name_numericadr: *mut ::core::ffi::c_int,
    #[doc = " text name pointers                       (ntext x 1)"]
    pub(crate) name_textadr: *mut ::core::ffi::c_int,
    #[doc = " tuple name pointers                      (ntuple x 1)"]
    pub(crate) name_tupleadr: *mut ::core::ffi::c_int,
    #[doc = " keyframe name pointers                   (nkey x 1)"]
    pub(crate) name_keyadr: *mut ::core::ffi::c_int,
    #[doc = " plugin instance name pointers            (nplugin x 1)"]
    pub(crate) name_pluginadr: *mut ::core::ffi::c_int,
    #[doc = " names of all objects, 0-terminated       (nnames x 1)"]
    pub(crate) names: *mut ::core::ffi::c_char,
    #[doc = " internal hash map of names               (nnames_map x 1)"]
    pub(crate) names_map: *mut ::core::ffi::c_int,
    #[doc = " paths to assets, 0-terminated            (npaths x 1)"]
    pub(crate) paths: *mut ::core::ffi::c_char,
    #[doc = " body-dof: non-zeros in each row          (nbody x 1)"]
    pub(crate) B_rownnz: *mut ::core::ffi::c_int,
    #[doc = " body-dof: row addresses                  (nbody x 1)"]
    pub(crate) B_rowadr: *mut ::core::ffi::c_int,
    #[doc = " body-dof: column indices                 (nB x 1)"]
    pub(crate) B_colind: *mut ::core::ffi::c_int,
    #[doc = " reduced inertia: non-zeros in each row   (nv x 1)"]
    pub(crate) M_rownnz: *mut ::core::ffi::c_int,
    #[doc = " reduced inertia: row addresses           (nv x 1)"]
    pub(crate) M_rowadr: *mut ::core::ffi::c_int,
    #[doc = " reduced inertia: column indices          (nC x 1)"]
    pub(crate) M_colind: *mut ::core::ffi::c_int,
    #[doc = " index mapping from qM to M               (nC x 1)"]
    pub(crate) mapM2M: *mut ::core::ffi::c_int,
    #[doc = " full inertia: non-zeros in each row      (nv x 1)"]
    pub(crate) D_rownnz: *mut ::core::ffi::c_int,
    #[doc = " full inertia: row addresses              (nv x 1)"]
    pub(crate) D_rowadr: *mut ::core::ffi::c_int,
    #[doc = " full inertia: index of diagonal element  (nv x 1)"]
    pub(crate) D_diag: *mut ::core::ffi::c_int,
    #[doc = " full inertia: column indices             (nD x 1)"]
    pub(crate) D_colind: *mut ::core::ffi::c_int,
    #[doc = " index mapping from M to D                (nD x 1)"]
    pub(crate) mapM2D: *mut ::core::ffi::c_int,
    #[doc = " index mapping from D to M                (nC x 1)"]
    pub(crate) mapD2M: *mut ::core::ffi::c_int,
    #[doc = " also held by the mjSpec that compiled this model"]
    pub signature: u64,
}
#[doc = "------------------------------------- Contact ----------------------------------------------------"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjPreContact {
    pub dist: mjtNum,
    pub pos: [mjtNum; 3usize],
    #[doc = " contact normal of the collision"]
    pub normal: [mjtNum; 3usize],
    #[doc = " first tangent direction"]
    pub tangent: [mjtNum; 3usize],
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjContact {
    #[doc = " distance between nearest points; neg: penetration"]
    pub dist: mjtNum,
    #[doc = " position of contact point: midpoint between geoms"]
    pub pos: [mjtNum; 3usize],
    #[doc = " normal is in [0-2], points from geom[0] to geom[1]"]
    pub frame: [mjtNum; 9usize],
    #[doc = " margin for force generation"]
    pub includemargin: mjtNum,
    #[doc = " tangent1, 2, spin, roll1, 2"]
    pub friction: [mjtNum; 5usize],
    #[doc = " constraint solver reference, normal direction"]
    pub solref: [mjtNum; 2usize],
    #[doc = " constraint solver reference, friction directions"]
    pub solreffriction: [mjtNum; 2usize],
    #[doc = " constraint solver impedance"]
    pub solimp: [mjtNum; 5usize],
    #[doc = " adhesive force along the contact normal"]
    pub adhesion: mjtNum,
    #[doc = " friction of regularized cone, set by mj_makeConstraint"]
    pub mu: mjtNum,
    #[doc = " cone Hessian, set by mj_constraintUpdate"]
    pub H: [mjtNum; 36usize],
    #[doc = " contact space dimensionality: 1, 3, 4 or 6"]
    pub dim: ::core::ffi::c_int,
    #[doc = " id of geom 1; deprecated, use geom[0]"]
    pub geom1: ::core::ffi::c_int,
    #[doc = " id of geom 2; deprecated, use geom[1]"]
    pub geom2: ::core::ffi::c_int,
    #[doc = " geom ids; -1 for flex"]
    pub geom: [::core::ffi::c_int; 2usize],
    #[doc = " flex ids; -1 for geom"]
    pub flex: [::core::ffi::c_int; 2usize],
    #[doc = " element ids; -1 for geom or flex vertex"]
    pub elem: [::core::ffi::c_int; 2usize],
    #[doc = " vertex ids;  -1 for geom or flex element"]
    pub vert: [::core::ffi::c_int; 2usize],
    #[doc = " 0: include, 1: in gap, 2: fused, 3: no dofs, 4: passive"]
    pub exclude: ::core::ffi::c_int,
    #[doc = " address in efc; -1: not included"]
    pub efc_address: ::core::ffi::c_int,
}
#[doc = "---------------------------------- diagnostics ---------------------------------------------------"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjWarningStat {
    #[doc = " info from last warning"]
    pub lastinfo: ::core::ffi::c_int,
    #[doc = " how many times was warning raised"]
    pub number: ::core::ffi::c_int,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjTimerStat {
    #[doc = " cumulative duration"]
    pub duration: mjtNum,
    #[doc = " how many times was timer called"]
    pub number: ::core::ffi::c_int,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjSolverStat {
    #[doc = " cost reduction, scaled by 1/trace(M(qpos0))"]
    pub improvement: mjtNum,
    #[doc = " gradient norm (primal only, scaled)"]
    pub gradient: mjtNum,
    #[doc = " slope in linesearch"]
    pub lineslope: mjtNum,
    #[doc = " number of active constraints"]
    pub nactive: ::core::ffi::c_int,
    #[doc = " number of constraint state changes"]
    pub nchange: ::core::ffi::c_int,
    #[doc = " number of cost evaluations in line search"]
    pub neval: ::core::ffi::c_int,
    #[doc = " number of Cholesky updates in line search"]
    pub nupdate: ::core::ffi::c_int,
}
#[doc = "---------------------------------- mjData --------------------------------------------------------"]
#[repr(C)]
#[derive(Debug)]
pub struct mjData {
    #[doc = " size of the arena in bytes (inclusive of the stack)"]
    pub narena: mjtSize,
    #[doc = " size of main buffer in bytes"]
    pub nbuffer: mjtSize,
    #[doc = " number of plugin instances"]
    pub nplugin: ::core::ffi::c_int,
    #[doc = " first available byte in stack (mutable)"]
    pub pstack: usize,
    #[doc = " value of pstack when mj_markStack was last called (mutable)"]
    pub pbase: usize,
    #[doc = " first available byte in arena"]
    pub parena: usize,
    #[doc = " thread pool pointer"]
    pub threadpool: usize,
    #[doc = " disable stack freeing during threaded execution"]
    pub threadlock: mjtBool,
    #[doc = " maximum stack allocation in bytes (mutable)"]
    pub maxuse_stack: mjtSize,
    #[doc = " maximum arena allocation in bytes"]
    pub maxuse_arena: mjtSize,
    #[doc = " maximum number of contacts"]
    pub maxuse_con: ::core::ffi::c_int,
    #[doc = " maximum number of scalar constraints"]
    pub maxuse_efc: ::core::ffi::c_int,
    #[doc = " solver statistics per island, per iteration"]
    pub solver: [mjSolverStat; 4000usize],
    #[doc = " number of solver iterations, per island"]
    pub solver_niter: [::core::ffi::c_int; 20usize],
    #[doc = " number of nonzeros in solver matrix, per island"]
    pub solver_nnz: [::core::ffi::c_int; 20usize],
    #[doc = " forward-inverse comparison: qfrc, efc"]
    pub solver_fwdinv: [mjtNum; 2usize],
    #[doc = " warning statistics (mutable)"]
    pub warning: [mjWarningStat; 7usize],
    #[doc = " timer statistics"]
    pub timer: [mjTimerStat; 15usize],
    #[doc = " number of detected contacts"]
    pub ncon: ::core::ffi::c_int,
    #[doc = " number of equality constraints"]
    pub ne: ::core::ffi::c_int,
    #[doc = " number of friction constraints"]
    pub nf: ::core::ffi::c_int,
    #[doc = " number of limit constraints"]
    pub nl: ::core::ffi::c_int,
    #[doc = " number of constraints"]
    pub nefc: ::core::ffi::c_int,
    #[doc = " number of non-zeros in constraint Jacobian"]
    pub nJ: ::core::ffi::c_int,
    #[doc = " implicit effective metric M+K: 0 inactive, 1 active, 2 active + preconditioner exact"]
    pub efm_active: ::core::ffi::c_int,
    #[doc = " number of non-zeros in effective-stiffness CSR"]
    pub nefmK: ::core::ffi::c_int,
    #[doc = " number of rows in effective-metric factor"]
    pub nefmdof: ::core::ffi::c_int,
    #[doc = " number of non-zeros in the effective-metric factor"]
    pub nefmL: ::core::ffi::c_int,
    #[doc = " number of non-zeros in constraint inverse inertia square root"]
    pub nY: ::core::ffi::c_int,
    #[doc = " number of non-zeros in constraint inverse inertia matrix"]
    pub nA: ::core::ffi::c_int,
    #[doc = " number of detected constraint islands"]
    pub nisland: ::core::ffi::c_int,
    #[doc = " number of dofs in all islands"]
    pub nidof: ::core::ffi::c_int,
    #[doc = " number of awake trees"]
    pub ntree_awake: ::core::ffi::c_int,
    #[doc = " number of awake dynamic and static bodies"]
    pub nbody_awake: ::core::ffi::c_int,
    #[doc = " number of bodies with awake parents"]
    pub nparent_awake: ::core::ffi::c_int,
    #[doc = " number of awake dofs"]
    pub nv_awake: ::core::ffi::c_int,
    #[doc = " has mj_energyPos been called"]
    pub flg_energypos: mjtBool,
    #[doc = " has mj_energyVel been called"]
    pub flg_energyvel: mjtBool,
    #[doc = " has mj_subtreeVel been called"]
    pub flg_subtreevel: mjtBool,
    #[doc = " has mj_rnePostConstraint been called"]
    pub flg_rnepost: mjtBool,
    #[doc = " simulation time"]
    pub time: mjtNum,
    #[doc = " potential, kinetic energy"]
    pub energy: [mjtNum; 2usize],
    #[doc = " main buffer; all pointers point in it            (nbuffer bytes)"]
    pub buffer: *mut ::core::ffi::c_void,
    #[doc = " arena+stack buffer                               (narena bytes)"]
    pub arena: *mut ::core::ffi::c_void,
    #[doc = " position                                         (nq x 1)"]
    pub(crate) qpos: *mut mjtNum,
    #[doc = " velocity                                         (nv x 1)"]
    pub(crate) qvel: *mut mjtNum,
    #[doc = " actuator activation                              (na x 1)"]
    pub(crate) act: *mut mjtNum,
    #[doc = " history buffer                                   (nhistory x 1)"]
    pub(crate) history: *mut mjtNum,
    #[doc = " acceleration used for warmstart                  (nv x 1)"]
    pub(crate) qacc_warmstart: *mut mjtNum,
    #[doc = " plugin state                                     (npluginstate x 1)"]
    pub(crate) plugin_state: *mut mjtNum,
    #[doc = " control                                          (nu x 1)"]
    pub(crate) ctrl: *mut mjtNum,
    #[doc = " applied generalized force                        (nv x 1)"]
    pub(crate) qfrc_applied: *mut mjtNum,
    #[doc = " applied Cartesian force/torque                   (nbody x 6)"]
    pub(crate) xfrc_applied: *mut mjtNum,
    #[doc = " enable/disable constraints                       (neq x 1)"]
    pub(crate) eq_active: *mut mjtBool,
    #[doc = " positions of mocap bodies                        (nmocap x 3)"]
    pub(crate) mocap_pos: *mut mjtNum,
    #[doc = " orientations of mocap bodies                     (nmocap x 4)"]
    pub(crate) mocap_quat: *mut mjtNum,
    #[doc = " acceleration                                     (nv x 1)"]
    pub(crate) qacc: *mut mjtNum,
    #[doc = " time-derivative of actuator activation           (na x 1)"]
    pub(crate) act_dot: *mut mjtNum,
    #[doc = " user data, not touched by engine                 (nuserdata x 1)"]
    pub(crate) userdata: *mut mjtNum,
    #[doc = " sensor data array                                (nsensordata x 1)"]
    pub(crate) sensordata: *mut mjtNum,
    #[doc = " <0: awake; >=0: index cycle of sleeping trees    (ntree x 1)"]
    pub(crate) tree_asleep: *mut ::core::ffi::c_int,
    #[doc = " copy of m->plugin, required for deletion         (nplugin x 1)"]
    pub(crate) plugin: *mut ::core::ffi::c_int,
    #[doc = " pointer to plugin-managed data structure         (nplugin x 1)"]
    pub(crate) plugin_data: *mut usize,
    #[doc = " Cartesian position of body frame                 (nbody x 3)"]
    pub(crate) xpos: *mut mjtNum,
    #[doc = " Cartesian orientation of body frame              (nbody x 4)"]
    pub(crate) xquat: *mut mjtNum,
    #[doc = " Cartesian orientation of body frame              (nbody x 9)"]
    pub(crate) xmat: *mut mjtNum,
    #[doc = " Cartesian position of body com                   (nbody x 3)"]
    pub(crate) xipos: *mut mjtNum,
    #[doc = " Cartesian orientation of body inertia            (nbody x 9)"]
    pub(crate) ximat: *mut mjtNum,
    #[doc = " Cartesian position of joint anchor               (njnt x 3)"]
    pub(crate) xanchor: *mut mjtNum,
    #[doc = " Cartesian joint axis                             (njnt x 3)"]
    pub(crate) xaxis: *mut mjtNum,
    #[doc = " Cartesian geom position                          (ngeom x 3)"]
    pub(crate) geom_xpos: *mut mjtNum,
    #[doc = " Cartesian geom orientation                       (ngeom x 9)"]
    pub(crate) geom_xmat: *mut mjtNum,
    #[doc = " Cartesian site position                          (nsite x 3)"]
    pub(crate) site_xpos: *mut mjtNum,
    #[doc = " Cartesian site orientation                       (nsite x 9)"]
    pub(crate) site_xmat: *mut mjtNum,
    #[doc = " Cartesian camera position                        (ncam x 3)"]
    pub(crate) cam_xpos: *mut mjtNum,
    #[doc = " Cartesian camera orientation                     (ncam x 9)"]
    pub(crate) cam_xmat: *mut mjtNum,
    #[doc = " Cartesian light position                         (nlight x 3)"]
    pub(crate) light_xpos: *mut mjtNum,
    #[doc = " Cartesian light direction                        (nlight x 3)"]
    pub(crate) light_xdir: *mut mjtNum,
    #[doc = " center of mass of each subtree                   (nbody x 3)"]
    pub(crate) subtree_com: *mut mjtNum,
    #[doc = " com-based motion axis of each dof (rot:lin)      (nv x 6)"]
    pub(crate) cdof: *mut mjtNum,
    #[doc = " com-based body inertia and mass                  (nbody x 10)"]
    pub(crate) cinert: *mut mjtNum,
    #[doc = " Cartesian flex vertex positions                  (nflexvert x 3)"]
    pub(crate) flexvert_xpos: *mut mjtNum,
    #[doc = " flex element bounding boxes (center, size)       (nflexelem x 6)"]
    pub(crate) flexelem_aabb: *mut mjtNum,
    #[doc = " corotated element stiffness (implicit only)      (nflexstiffness x 1)"]
    pub(crate) flexelem_krot: *mut mjtNum,
    #[doc = " flex edge Jacobian                               (nJfe x 1)"]
    pub(crate) flexedge_J: *mut mjtNum,
    #[doc = " flex edge lengths                                (nflexedge x 1)"]
    pub(crate) flexedge_length: *mut mjtNum,
    #[doc = " flex vertex Jacobian                             (nJfv x 2)"]
    pub(crate) flexvert_J: *mut mjtNum,
    #[doc = " flex vertex lengths                              (nflexvert x 2)"]
    pub(crate) flexvert_length: *mut mjtNum,
    #[doc = " global bounding box (center, size)               (nbvhdynamic x 6)"]
    pub(crate) bvh_aabb_dyn: *mut mjtNum,
    #[doc = " start address of tendon's path                   (ntendon x 1)"]
    pub(crate) ten_wrapadr: *mut ::core::ffi::c_int,
    #[doc = " number of wrap points in path                    (ntendon x 1)"]
    pub(crate) ten_wrapnum: *mut ::core::ffi::c_int,
    #[doc = " tendon Jacobian                                  (nJten x 1)"]
    pub(crate) ten_J: *mut mjtNum,
    #[doc = " tendon lengths                                   (ntendon x 1)"]
    pub(crate) ten_length: *mut mjtNum,
    #[doc = " geom id; -1: site; -2: pulley                    (nwrap x 2)"]
    pub(crate) wrap_obj: *mut ::core::ffi::c_int,
    #[doc = " Cartesian 3D points in all paths                 (nwrap x 6)"]
    pub(crate) wrap_xpos: *mut mjtNum,
    #[doc = " actuator lengths, one per force output           (nout x 1)"]
    pub(crate) actuator_length: *mut mjtNum,
    #[doc = " number of non-zeros in actuator_moment row       (nout x 1)"]
    pub(crate) moment_rownnz: *mut ::core::ffi::c_int,
    #[doc = " row start address in colind array                (nout x 1)"]
    pub(crate) moment_rowadr: *mut ::core::ffi::c_int,
    #[doc = " column indices in sparse Jacobian                (nJmom x 1)"]
    pub(crate) moment_colind: *mut ::core::ffi::c_int,
    #[doc = " actuator moments                                 (nJmom x 1)"]
    pub(crate) actuator_moment: *mut mjtNum,
    #[doc = " com-based composite inertia and mass             (nbody x 10)"]
    pub(crate) crb: *mut mjtNum,
    #[doc = " inertia (sparse)                                 (nC x 1)"]
    pub(crate) M: *mut mjtNum,
    #[doc = " L'*D*L factorization of M (sparse)               (nC x 1)"]
    pub(crate) qLD: *mut mjtNum,
    #[doc = " 1/diag(D)                                        (nv x 1)"]
    pub(crate) qLDiagInv: *mut mjtNum,
    #[doc = " was bounding volume checked for collision        (nbvh x 1)"]
    pub(crate) bvh_active: *mut mjtBool,
    #[doc = " is tree awake; 0: asleep; 1: awake               (ntree x 1)"]
    pub(crate) tree_awake: *mut ::core::ffi::c_int,
    #[doc = " body sleep state (mjtSleepState)                 (nbody x 1)"]
    pub(crate) body_awake: *mut ::core::ffi::c_int,
    #[doc = " indices of awake and static bodies               (nbody x 1)"]
    pub(crate) body_awake_ind: *mut ::core::ffi::c_int,
    #[doc = " indices of bodies with awake or static parents   (nbody x 1)"]
    pub(crate) parent_awake_ind: *mut ::core::ffi::c_int,
    #[doc = " indices of awake dofs                            (nv x 1)"]
    pub(crate) dof_awake_ind: *mut ::core::ffi::c_int,
    #[doc = " flex edge velocities                             (nflexedge x 1)"]
    pub(crate) flexedge_velocity: *mut mjtNum,
    #[doc = " tendon velocities                                (ntendon x 1)"]
    pub(crate) ten_velocity: *mut mjtNum,
    #[doc = " actuator velocities, one per force output        (nout x 1)"]
    pub(crate) actuator_velocity: *mut mjtNum,
    #[doc = " com-based velocity (rot:lin)                     (nbody x 6)"]
    pub(crate) cvel: *mut mjtNum,
    #[doc = " time-derivative of cdof (rot:lin)                (nv x 6)"]
    pub(crate) cdof_dot: *mut mjtNum,
    #[doc = " C(qpos,qvel)                                     (nv x 1)"]
    pub(crate) qfrc_bias: *mut mjtNum,
    #[doc = " passive spring force                             (nv x 1)"]
    pub(crate) qfrc_spring: *mut mjtNum,
    #[doc = " passive damper force                             (nv x 1)"]
    pub(crate) qfrc_damper: *mut mjtNum,
    #[doc = " passive gravity compensation force               (nv x 1)"]
    pub(crate) qfrc_gravcomp: *mut mjtNum,
    #[doc = " passive fluid force                              (nv x 1)"]
    pub(crate) qfrc_fluid: *mut mjtNum,
    #[doc = " passive contact adhesion force                   (nv x 1)"]
    pub(crate) qfrc_adhesion: *mut mjtNum,
    #[doc = " total passive force                              (nv x 1)"]
    pub(crate) qfrc_passive: *mut mjtNum,
    #[doc = " linear velocity of subtree com                   (nbody x 3)"]
    pub(crate) subtree_linvel: *mut mjtNum,
    #[doc = " angular momentum about subtree com               (nbody x 3)"]
    pub(crate) subtree_angmom: *mut mjtNum,
    #[doc = " L'*D*L factorization of modified M               (nC x 1)"]
    pub(crate) qH: *mut mjtNum,
    #[doc = " 1/diag(D) of modified M                          (nv x 1)"]
    pub(crate) qHDiagInv: *mut mjtNum,
    #[doc = " d (passive + actuator - bias) / d qvel           (nD x 1)"]
    pub(crate) qDeriv: *mut mjtNum,
    #[doc = " sparse LU of (M - dt*qDeriv)                     (nD x 1)"]
    pub(crate) qLU: *mut mjtNum,
    #[doc = " actuator force in actuation space                (nout x 1)"]
    pub(crate) actuator_force: *mut mjtNum,
    #[doc = " actuator force in joint space                    (nv x 1)"]
    pub(crate) qfrc_actuator: *mut mjtNum,
    #[doc = " net unconstrained force                          (nv x 1)"]
    pub(crate) qfrc_smooth: *mut mjtNum,
    #[doc = " unconstrained acceleration                       (nv x 1)"]
    pub(crate) qacc_smooth: *mut mjtNum,
    #[doc = " constraint force                                 (nv x 1)"]
    pub(crate) qfrc_constraint: *mut mjtNum,
    #[doc = " net external force; should equal:\n qfrc_applied + J'*xfrc_applied + qfrc_actuator   (nv x 1)"]
    pub(crate) qfrc_inverse: *mut mjtNum,
    #[doc = " com-based acceleration                           (nbody x 6)"]
    pub(crate) cacc: *mut mjtNum,
    #[doc = " com-based interaction force with parent          (nbody x 6)"]
    pub(crate) cfrc_int: *mut mjtNum,
    #[doc = " com-based external force on body                 (nbody x 6)"]
    pub(crate) cfrc_ext: *mut mjtNum,
    #[doc = " array of all detected contacts                   (ncon x 1)"]
    pub(crate) contact: *mut mjContact,
    #[doc = " constraint type (mjtConstraint)                  (nefc x 1)"]
    pub(crate) efc_type: *mut ::core::ffi::c_int,
    #[doc = " id of object of specified type                   (nefc x 1)"]
    pub(crate) efc_id: *mut ::core::ffi::c_int,
    #[doc = " number of non-zeros in constraint Jacobian row   (nefc x 1)"]
    pub(crate) efc_J_rownnz: *mut ::core::ffi::c_int,
    #[doc = " row start address in colind array                (nefc x 1)"]
    pub(crate) efc_J_rowadr: *mut ::core::ffi::c_int,
    #[doc = " number of subsequent rows in supernode           (nefc x 1)"]
    pub(crate) efc_J_rowsuper: *mut ::core::ffi::c_int,
    #[doc = " column indices in constraint Jacobian            (nJ x 1)"]
    pub(crate) efc_J_colind: *mut ::core::ffi::c_int,
    #[doc = " constraint Jacobian                              (nJ x 1)"]
    pub(crate) efc_J: *mut mjtNum,
    #[doc = " constraint position (equality, contact)          (nefc x 1)"]
    pub(crate) efc_pos: *mut mjtNum,
    #[doc = " inclusion margin (contact)                       (nefc x 1)"]
    pub(crate) efc_margin: *mut mjtNum,
    #[doc = " frictionloss (friction)                          (nefc x 1)"]
    pub(crate) efc_frictionloss: *mut mjtNum,
    #[doc = " diagonal of A matrix, approximate or exact       (nefc x 1)"]
    pub(crate) efc_diagA: *mut mjtNum,
    #[doc = " stiffness, damping, impedance, imp'              (nefc x 4)"]
    pub(crate) efc_KBIP: *mut mjtNum,
    #[doc = " constraint mass                                  (nefc x 1)"]
    pub(crate) efc_D: *mut mjtNum,
    #[doc = " inverse constraint mass                          (nefc x 1)"]
    pub(crate) efc_R: *mut mjtNum,
    #[doc = " first efc address involving tendon; -1: none     (ntendon x 1)"]
    pub(crate) tendon_efcadr: *mut ::core::ffi::c_int,
    #[doc = " island id of this tree; -1: none                 (ntree x 1)"]
    pub(crate) tree_island: *mut ::core::ffi::c_int,
    #[doc = " number of trees in this island                   (nisland x 1)"]
    pub(crate) island_ntree: *mut ::core::ffi::c_int,
    #[doc = " island start address in itree vector             (nisland x 1)"]
    pub(crate) island_itreeadr: *mut ::core::ffi::c_int,
    #[doc = " map from itree to tree                           (ntree x 1)"]
    pub(crate) map_itree2tree: *mut ::core::ffi::c_int,
    #[doc = " island id of this dof; -1: none                  (nv x 1)"]
    pub(crate) dof_island: *mut ::core::ffi::c_int,
    #[doc = " number of dofs in this island                    (nisland x 1)"]
    pub(crate) island_nv: *mut ::core::ffi::c_int,
    #[doc = " island start address in idof vector              (nisland x 1)"]
    pub(crate) island_idofadr: *mut ::core::ffi::c_int,
    #[doc = " island start address in dof vector               (nisland x 1)"]
    pub(crate) island_dofadr: *mut ::core::ffi::c_int,
    #[doc = " map from dof to idof                             (nv x 1)"]
    pub(crate) map_dof2idof: *mut ::core::ffi::c_int,
    #[doc = " map from idof to dof;  >= nidof: unconstrained   (nv x 1)"]
    pub(crate) map_idof2dof: *mut ::core::ffi::c_int,
    #[doc = " net unconstrained force                          (nidof x 1)"]
    pub(crate) ifrc_smooth: *mut mjtNum,
    #[doc = " unconstrained acceleration                       (nidof x 1)"]
    pub(crate) iacc_smooth: *mut mjtNum,
    #[doc = " acceleration                                     (nidof x 1)"]
    pub(crate) iacc: *mut mjtNum,
    #[doc = " island id of this constraint                     (nefc x 1)"]
    pub(crate) efc_island: *mut ::core::ffi::c_int,
    #[doc = " number of equality constraints in island         (nisland x 1)"]
    pub(crate) island_ne: *mut ::core::ffi::c_int,
    #[doc = " number of friction constraints in island         (nisland x 1)"]
    pub(crate) island_nf: *mut ::core::ffi::c_int,
    #[doc = " number of constraints in island                  (nisland x 1)"]
    pub(crate) island_nefc: *mut ::core::ffi::c_int,
    #[doc = " start address in iefc vector                     (nisland x 1)"]
    pub(crate) island_iefcadr: *mut ::core::ffi::c_int,
    #[doc = " map from efc to iefc                             (nefc x 1)"]
    pub(crate) map_efc2iefc: *mut ::core::ffi::c_int,
    #[doc = " map from iefc to efc                             (nefc x 1)"]
    pub(crate) map_iefc2efc: *mut ::core::ffi::c_int,
    #[doc = " constraint type (mjtConstraint)                  (nefc x 1)"]
    pub(crate) iefc_type: *mut ::core::ffi::c_int,
    #[doc = " id of object of specified type                   (nefc x 1)"]
    pub(crate) iefc_id: *mut ::core::ffi::c_int,
    #[doc = " frictionloss (friction)                          (nefc x 1)"]
    pub(crate) iefc_frictionloss: *mut mjtNum,
    #[doc = " constraint mass                                  (nefc x 1)"]
    pub(crate) iefc_D: *mut mjtNum,
    #[doc = " inverse constraint mass                          (nefc x 1)"]
    pub(crate) iefc_R: *mut mjtNum,
    #[doc = " number of non-zeros in Y row                     (nefc x 1)"]
    pub(crate) efc_Y_rownnz: *mut ::core::ffi::c_int,
    #[doc = " row start address in Y colind array              (nefc x 1)"]
    pub(crate) efc_Y_rowadr: *mut ::core::ffi::c_int,
    #[doc = " column indices in sparse Y                       (nY x 1)"]
    pub(crate) efc_Y_colind: *mut ::core::ffi::c_int,
    #[doc = " whitened Jacobian Y = J*M^(-1/2)                 (nY x 1)"]
    pub(crate) efc_Y: *mut mjtNum,
    #[doc = " number of non-zeros in AR                        (nefc x 1)"]
    pub(crate) efc_AR_rownnz: *mut ::core::ffi::c_int,
    #[doc = " row start address in AR colind array             (nefc x 1)"]
    pub(crate) efc_AR_rowadr: *mut ::core::ffi::c_int,
    #[doc = " column indices in sparse AR                      (nA x 1)"]
    pub(crate) efc_AR_colind: *mut ::core::ffi::c_int,
    #[doc = " J*inv(M)*J' + R                                  (nA x 1)"]
    pub(crate) efc_AR: *mut mjtNum,
    #[doc = " velocity in constraint space: J*qvel             (nefc x 1)"]
    pub(crate) efc_vel: *mut mjtNum,
    #[doc = " reference pseudo-acceleration                    (nefc x 1)"]
    pub(crate) efc_aref: *mut mjtNum,
    #[doc = " smooth-force shift h*K*qvel                      (nv x 1)"]
    pub(crate) efm_c: *mut mjtNum,
    #[doc = " effective-stiffness CSR row nonzeros             (nv x 1)"]
    pub(crate) efm_K_rownnz: *mut ::core::ffi::c_int,
    #[doc = " effective-stiffness CSR row addresses            (nv x 1)"]
    pub(crate) efm_K_rowadr: *mut ::core::ffi::c_int,
    #[doc = " effective-stiffness CSR column indices           (nefmK x 1)"]
    pub(crate) efm_K_colind: *mut ::core::ffi::c_int,
    #[doc = " effective-stiffness CSR values                   (nefmK x 1)"]
    pub(crate) efm_K_val: *mut mjtNum,
    #[doc = " factor row -> dof address                        (nefmdof x 1)"]
    pub(crate) efm_dofid: *mut ::core::ffi::c_int,
    #[doc = " factor row nonzeros                              (nefmdof x 1)"]
    pub(crate) efm_L_rownnz: *mut ::core::ffi::c_int,
    #[doc = " factor row addresses                             (nefmdof x 1)"]
    pub(crate) efm_L_rowadr: *mut ::core::ffi::c_int,
    #[doc = " factor column indices                            (nefmL x 1)"]
    pub(crate) efm_L_colind: *mut ::core::ffi::c_int,
    #[doc = " Cholesky factor of diag(M)+K, covered dofs       (nefmL x 1)"]
    pub(crate) efm_L: *mut mjtNum,
    #[doc = " linear cost term: J*qacc_smooth - aref           (nefc x 1)"]
    pub(crate) efc_b: *mut mjtNum,
    #[doc = " reference pseudo-acceleration                    (nefc x 1)"]
    pub(crate) iefc_aref: *mut mjtNum,
    #[doc = " constraint state (mjtConstraintState)            (nefc x 1)"]
    pub(crate) iefc_state: *mut ::core::ffi::c_int,
    #[doc = " constraint force in constraint space             (nefc x 1)"]
    pub(crate) iefc_force: *mut mjtNum,
    #[doc = " constraint state (mjtConstraintState)            (nefc x 1)"]
    pub(crate) efc_state: *mut ::core::ffi::c_int,
    #[doc = " constraint force in constraint space             (nefc x 1)"]
    pub(crate) efc_force: *mut mjtNum,
    #[doc = " constraint force                                 (nidof x 1)"]
    pub(crate) ifrc_constraint: *mut mjtNum,
    #[doc = " also held by the mjSpec that compiled the model"]
    pub signature: u64,
}
#[doc = " generic MuJoCo function"]
pub(crate) type mjfGeneric =
    ::core::option::Option<unsafe extern "C" fn(m: *const mjModel, d: *mut mjData)>;
#[doc = " contact filter: 1- discard, 0- collide"]
pub(crate) type mjfConFilt = ::core::option::Option<
    unsafe extern "C" fn(
        m: *const mjModel,
        d: *mut mjData,
        geom1: ::core::ffi::c_int,
        geom2: ::core::ffi::c_int,
    ) -> ::core::ffi::c_int,
>;
#[doc = " sensor simulation"]
pub(crate) type mjfSensor = ::core::option::Option<
    unsafe extern "C" fn(m: *const mjModel, d: *mut mjData, stage: ::core::ffi::c_int),
>;
#[doc = " timer"]
pub(crate) type mjfTime = ::core::option::Option<unsafe extern "C" fn() -> mjtNum>;
#[doc = " actuator dynamics, gain, bias"]
pub(crate) type mjfAct = ::core::option::Option<
    unsafe extern "C" fn(m: *const mjModel, d: *const mjData, id: ::core::ffi::c_int) -> mjtNum,
>;
#[doc = " collision detection"]
pub(crate) type mjfCollision = ::core::option::Option<
    unsafe extern "C" fn(
        m: *const mjModel,
        d: *mut mjData,
        con: *mut mjPreContact,
        g1: ::core::ffi::c_int,
        g2: ::core::ffi::c_int,
        margin: mjtNum,
    ) -> ::core::ffi::c_int,
>;
#[doc = " C: opaque types"]
pub type mjString = ::core::ffi::c_void;
pub type mjStringVec = ::core::ffi::c_void;
pub type mjIntVec = ::core::ffi::c_void;
pub type mjIntVecVec = ::core::ffi::c_void;
pub type mjFloatVec = ::core::ffi::c_void;
pub type mjFloatVecVec = ::core::ffi::c_void;
pub type mjDoubleVec = ::core::ffi::c_void;
pub type mjByteVec = ::core::ffi::c_void;
impl mjtGeomInertia {
    #[doc = " mass distributed in the volume"]
    pub const VOLUME: mjtGeomInertia = mjtGeomInertia(0);
    #[doc = " mass distributed on the surface"]
    pub const SHELL: mjtGeomInertia = mjtGeomInertia(1);
}
#[repr(transparent)]
#[doc = "-------------------------------- enum types (mjt) ------------------------------------------------"]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtGeomInertia(pub(crate) ::core::ffi::c_uint);
impl mjtMeshInertia {
    #[doc = " convex mesh inertia"]
    pub const CONVEX: mjtMeshInertia = mjtMeshInertia(0);
    #[doc = " exact mesh inertia"]
    pub const EXACT: mjtMeshInertia = mjtMeshInertia(1);
    #[doc = " legacy mesh inertia"]
    pub const LEGACY: mjtMeshInertia = mjtMeshInertia(2);
    #[doc = " shell mesh inertia"]
    pub const SHELL: mjtMeshInertia = mjtMeshInertia(3);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtMeshInertia(pub(crate) ::core::ffi::c_uint);
impl mjtMeshBuiltin {
    #[doc = " no built-in mesh"]
    pub const NONE: mjtMeshBuiltin = mjtMeshBuiltin(0);
    #[doc = " sphere"]
    pub const SPHERE: mjtMeshBuiltin = mjtMeshBuiltin(1);
    #[doc = " hemisphere"]
    pub const HEMISPHERE: mjtMeshBuiltin = mjtMeshBuiltin(2);
    #[doc = " cone"]
    pub const CONE: mjtMeshBuiltin = mjtMeshBuiltin(3);
    #[doc = " supersphere"]
    pub const SUPERSPHERE: mjtMeshBuiltin = mjtMeshBuiltin(4);
    #[doc = " supertorus"]
    pub const SUPERTORUS: mjtMeshBuiltin = mjtMeshBuiltin(5);
    #[doc = " wedge"]
    pub const WEDGE: mjtMeshBuiltin = mjtMeshBuiltin(6);
    #[doc = " plate"]
    pub const PLATE: mjtMeshBuiltin = mjtMeshBuiltin(7);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtMeshBuiltin(pub(crate) ::core::ffi::c_uint);
impl mjtBuiltin {
    #[doc = " no built-in texture"]
    pub const NONE: mjtBuiltin = mjtBuiltin(0);
    #[doc = " gradient: rgb1->rgb2"]
    pub const GRADIENT: mjtBuiltin = mjtBuiltin(1);
    #[doc = " checker pattern: rgb1, rgb2"]
    pub const CHECKER: mjtBuiltin = mjtBuiltin(2);
    #[doc = " 2d: rgb1; cube: rgb1-up, rgb2-side, rgb3-down"]
    pub const FLAT: mjtBuiltin = mjtBuiltin(3);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtBuiltin(pub(crate) ::core::ffi::c_uint);
impl mjtMark {
    #[doc = " no mark"]
    pub const NONE: mjtMark = mjtMark(0);
    #[doc = " edges"]
    pub const EDGE: mjtMark = mjtMark(1);
    #[doc = " cross"]
    pub const CROSS: mjtMark = mjtMark(2);
    #[doc = " random dots"]
    pub const RANDOM: mjtMark = mjtMark(3);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtMark(pub(crate) ::core::ffi::c_uint);
impl mjtLimited {
    #[doc = " not limited"]
    pub const FALSE: mjtLimited = mjtLimited(0);
    #[doc = " limited"]
    pub const TRUE: mjtLimited = mjtLimited(1);
    #[doc = " limited inferred from presence of range"]
    pub const AUTO: mjtLimited = mjtLimited(2);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtLimited(pub(crate) ::core::ffi::c_uint);
impl mjtAlignFree {
    #[doc = " don't align"]
    pub const FALSE: mjtAlignFree = mjtAlignFree(0);
    #[doc = " align"]
    pub const TRUE: mjtAlignFree = mjtAlignFree(1);
    #[doc = " respect the global compiler flag"]
    pub const AUTO: mjtAlignFree = mjtAlignFree(2);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtAlignFree(pub(crate) ::core::ffi::c_uint);
impl mjtInertiaFromGeom {
    #[doc = " do not use; inertial element required"]
    pub const FALSE: mjtInertiaFromGeom = mjtInertiaFromGeom(0);
    #[doc = " always use; overwrite inertial element"]
    pub const TRUE: mjtInertiaFromGeom = mjtInertiaFromGeom(1);
    #[doc = " use only if inertial element is missing"]
    pub const AUTO: mjtInertiaFromGeom = mjtInertiaFromGeom(2);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtInertiaFromGeom(pub(crate) ::core::ffi::c_uint);
impl mjtOrientation {
    #[doc = " quaternion"]
    pub const QUAT: mjtOrientation = mjtOrientation(0);
    #[doc = " axis and angle"]
    pub const AXISANGLE: mjtOrientation = mjtOrientation(1);
    #[doc = " x and y axes"]
    pub const XYAXES: mjtOrientation = mjtOrientation(2);
    #[doc = " z axis (minimal rotation)"]
    pub const ZAXIS: mjtOrientation = mjtOrientation(3);
    #[doc = " Euler angles"]
    pub const EULER: mjtOrientation = mjtOrientation(4);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtOrientation(pub(crate) ::core::ffi::c_uint);
impl mjtConflict {
    #[doc = " keep parent, warn on conflict"]
    pub const WARNING: mjtConflict = mjtConflict(0);
    #[doc = " merge: min/max/error per field"]
    pub const MERGE: mjtConflict = mjtConflict(1);
    #[doc = " error on any conflict"]
    pub const ERROR: mjtConflict = mjtConflict(2);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtConflict(pub(crate) ::core::ffi::c_uint);
pub const mjNCTIMER: usize = mjtCTimer::mjNCTIMER.0 as usize;
impl mjtCTimer {
    #[doc = " total compile time"]
    pub const TOTAL: mjtCTimer = mjtCTimer(0);
    #[doc = " asset compilation"]
    pub const ASSETS: mjtCTimer = mjtCTimer(1);
    #[doc = " textures"]
    pub const TEXTURE: mjtCTimer = mjtCTimer(2);
    #[doc = " mesh: file loading"]
    pub const MESH_LOAD: mjtCTimer = mjtCTimer(3);
    #[doc = " mesh: convex hull"]
    pub const MESH_HULL: mjtCTimer = mjtCTimer(4);
    #[doc = " mesh: normals and polygons"]
    pub const MESH_POLYGON: mjtCTimer = mjtCTimer(5);
    #[doc = " mesh: volume, CoM, inertia"]
    pub const MESH_INERTIA: mjtCTimer = mjtCTimer(6);
    #[doc = " mesh: bounding volume hierarchy"]
    pub const MESH_BVH: mjtCTimer = mjtCTimer(7);
    #[doc = " mesh: octree and SDF"]
    pub const MESH_OCTREE: mjtCTimer = mjtCTimer(8);
    #[doc = " number of compiler timers"]
    const mjNCTIMER: mjtCTimer = mjtCTimer(9);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtCTimer(pub(crate) ::core::ffi::c_uint);
#[doc = "-------------------------------- attribute structs (mjs) -----------------------------------------"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsElement {
    #[doc = " element type"]
    pub elemtype: mjtObj,
    #[doc = " compilation signature"]
    pub signature: u64,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsCompiler {
    #[doc = " infer \"limited\" attribute based on range"]
    pub autolimits: mjtBool,
    #[doc = " enforce minimum body mass"]
    pub boundmass: f64,
    #[doc = " enforce minimum body diagonal inertia"]
    pub boundinertia: f64,
    #[doc = " rescale masses and inertias; <=0: ignore"]
    pub settotalmass: f64,
    #[doc = " automatically impose A + B >= C rule"]
    pub balanceinertia: mjtBool,
    #[doc = " meshfit to aabb instead of inertia box"]
    pub fitaabb: mjtBool,
    #[doc = " angles in radians or degrees"]
    pub degree: mjtBool,
    #[doc = " sequence for euler rotations"]
    pub(crate) eulerseq: [::core::ffi::c_char; 3usize],
    #[doc = " discard visual geoms in parser"]
    pub discardvisual: mjtBool,
    #[doc = " use multiple threads to speed up compiler"]
    pub usethread: mjtBool,
    #[doc = " fuse static bodies with parent"]
    pub fusestatic: mjtBool,
    #[doc = " use geom inertias"]
    pub inertiafromgeom: mjtInertiaFromGeom,
    #[doc = " range of geom groups used to compute inertia"]
    pub inertiagrouprange: [::core::ffi::c_int; 2usize],
    #[doc = " save explicit inertial clause for all bodies to XML"]
    pub saveinertial: mjtBool,
    #[doc = " align free joints with inertial frame"]
    pub alignfree: mjtBool,
    #[doc = " conflict resolution for attach"]
    pub conflict: mjtConflict,
    #[doc = " options for lengthrange computation"]
    pub LRopt: mjLROpt,
    #[doc = " mesh and hfield directory"]
    pub(crate) meshdir: *mut mjString,
    #[doc = " texture directory"]
    pub(crate) texturedir: *mut mjString,
    #[doc = " bitmask of authored compiler fields"]
    pub authored: u64,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsAuthored {
    #[doc = " authored mjOption fields"]
    pub option: u64,
    #[doc = " individual authored disable flags"]
    pub disableflags: ::core::ffi::c_int,
    #[doc = " individual authored enable flags"]
    pub enableflags: ::core::ffi::c_int,
    #[doc = " individual authored actuator groups"]
    pub disableactuator: ::core::ffi::c_int,
    #[doc = " authored visual.global fields"]
    pub visual_global: u64,
    #[doc = " authored visual.quality fields"]
    pub visual_quality: u64,
    #[doc = " authored visual.headlight fields"]
    pub visual_headlight: u64,
    #[doc = " authored visual.map fields"]
    pub visual_map: u64,
    #[doc = " authored visual.scale fields"]
    pub visual_scale: u64,
    #[doc = " authored visual.rgba fields"]
    pub visual_rgba: u64,
}
#[repr(C)]
#[derive(Debug)]
pub struct mjSpec {
    #[doc = " element type"]
    pub(crate) element: *mut mjsElement,
    #[doc = " model name"]
    pub(crate) modelname: *mut mjString,
    #[doc = " compiler options"]
    pub compiler: mjsCompiler,
    #[doc = " automatically strip paths from mesh files"]
    pub strippath: mjtBool,
    #[doc = " physics options"]
    pub option: mjOption,
    #[doc = " visual options"]
    pub visual: mjVisual,
    #[doc = " statistics override (if defined)"]
    pub stat: mjStatistic,
    #[doc = " number of bytes in arena+stack memory"]
    pub memory: mjtSize,
    #[doc = " max number of equality constraints"]
    pub nemax: ::core::ffi::c_int,
    #[doc = " number of mjtNums in userdata"]
    pub nuserdata: ::core::ffi::c_int,
    #[doc = " number of mjtNums in body_user"]
    pub nuser_body: ::core::ffi::c_int,
    #[doc = " number of mjtNums in jnt_user"]
    pub nuser_jnt: ::core::ffi::c_int,
    #[doc = " number of mjtNums in geom_user"]
    pub nuser_geom: ::core::ffi::c_int,
    #[doc = " number of mjtNums in site_user"]
    pub nuser_site: ::core::ffi::c_int,
    #[doc = " number of mjtNums in cam_user"]
    pub nuser_cam: ::core::ffi::c_int,
    #[doc = " number of mjtNums in tendon_user"]
    pub nuser_tendon: ::core::ffi::c_int,
    #[doc = " number of mjtNums in actuator_user"]
    pub nuser_actuator: ::core::ffi::c_int,
    #[doc = " number of mjtNums in sensor_user"]
    pub nuser_sensor: ::core::ffi::c_int,
    #[doc = " number of keyframes"]
    pub nkey: ::core::ffi::c_int,
    #[doc = " (deprecated) max number of constraints"]
    pub njmax: ::core::ffi::c_int,
    #[doc = " (deprecated) max number of detected contacts"]
    pub nconmax: ::core::ffi::c_int,
    #[doc = " (deprecated) number of mjtNums in mjData stack"]
    pub nstack: mjtSize,
    #[doc = " comment at top of XML"]
    pub(crate) comment: *mut mjString,
    #[doc = " path to model file"]
    pub(crate) modelfiledir: *mut mjString,
    #[doc = " already encountered an implicit plugin sensor/actuator"]
    pub hasImplicitPluginElem: mjtBool,
    #[doc = " authored tracking bitmasks for mjModel structs"]
    pub authored: mjsAuthored,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsOrientation {
    #[doc = " active orientation specifier"]
    pub type_: mjtOrientation,
    #[doc = " axis and angle"]
    pub axisangle: [f64; 4usize],
    #[doc = " x and y axes"]
    pub xyaxes: [f64; 6usize],
    #[doc = " z axis (minimal rotation)"]
    pub zaxis: [f64; 3usize],
    #[doc = " Euler angles"]
    pub euler: [f64; 3usize],
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsPlugin {
    #[doc = " element type"]
    pub(crate) element: *mut mjsElement,
    #[doc = " instance name"]
    pub(crate) name: *mut mjString,
    #[doc = " plugin name"]
    pub(crate) plugin_name: *mut mjString,
    #[doc = " is the plugin active"]
    pub active: mjtBool,
    #[doc = " message appended to compiler errors"]
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsBody {
    #[doc = " element type"]
    pub(crate) element: *mut mjsElement,
    #[doc = " childclass name"]
    pub(crate) childclass: *mut mjString,
    #[doc = " frame position"]
    pub pos: [f64; 3usize],
    #[doc = " frame orientation"]
    pub quat: [f64; 4usize],
    #[doc = " frame alternative orientation"]
    pub alt: mjsOrientation,
    #[doc = " mass"]
    pub mass: f64,
    #[doc = " inertial frame position"]
    pub ipos: [f64; 3usize],
    #[doc = " inertial frame orientation"]
    pub iquat: [f64; 4usize],
    #[doc = " diagonal inertia (in i-frame)"]
    pub inertia: [f64; 3usize],
    #[doc = " inertial frame alternative orientation"]
    pub ialt: mjsOrientation,
    #[doc = " non-axis-aligned inertia matrix"]
    pub fullinertia: [f64; 6usize],
    #[doc = " is this a mocap body"]
    pub mocap: mjtBool,
    #[doc = " gravity compensation"]
    pub gravcomp: f64,
    #[doc = " sleep policy"]
    pub sleep: mjtSleepPolicy,
    #[doc = " simple body optimization (0: false, 1: auto)"]
    pub simple: mjtByte,
    #[doc = " user data"]
    pub(crate) userdata: *mut mjDoubleVec,
    #[doc = " whether to save the body with explicit inertial clause"]
    pub explicitinertial: mjtBool,
    #[doc = " passive force plugin"]
    pub plugin: mjsPlugin,
    #[doc = " message appended to compiler errors"]
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsFrame {
    #[doc = " element type"]
    pub(crate) element: *mut mjsElement,
    #[doc = " childclass name"]
    pub(crate) childclass: *mut mjString,
    #[doc = " position"]
    pub pos: [f64; 3usize],
    #[doc = " orientation"]
    pub quat: [f64; 4usize],
    #[doc = " alternative orientation"]
    pub alt: mjsOrientation,
    #[doc = " message appended to compiler errors"]
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsJoint {
    #[doc = " element type"]
    pub(crate) element: *mut mjsElement,
    #[doc = " joint type"]
    pub type_: mjtJoint,
    #[doc = " anchor position"]
    pub pos: [f64; 3usize],
    #[doc = " joint axis"]
    pub axis: [f64; 3usize],
    #[doc = " value at reference configuration: qpos0"]
    pub ref_: f64,
    #[doc = " align free joint with body com"]
    pub align: mjtAlignFree,
    #[doc = " stiffness coefficients"]
    pub stiffness: [f64; 3usize],
    #[doc = " spring reference value: qpos_spring"]
    pub springref: f64,
    #[doc = " timeconst, dampratio"]
    pub springdamper: [f64; 2usize],
    #[doc = " does joint have limits"]
    pub limited: mjtLimited,
    #[doc = " joint limits"]
    pub range: [f64; 2usize],
    #[doc = " margin value for joint limit detection"]
    pub margin: f64,
    #[doc = " solver reference: joint limits"]
    pub solref_limit: [mjtNum; 2usize],
    #[doc = " solver impedance: joint limits"]
    pub solimp_limit: [mjtNum; 5usize],
    #[doc = " are actuator forces on joint limited"]
    pub actfrclimited: mjtLimited,
    #[doc = " actuator force limits"]
    pub actfrcrange: [f64; 2usize],
    #[doc = " armature inertia (mass for slider)"]
    pub armature: f64,
    #[doc = " damping coefficients"]
    pub damping: [f64; 3usize],
    #[doc = " friction loss"]
    pub frictionloss: f64,
    #[doc = " solver reference: dof friction"]
    pub solref_friction: [mjtNum; 2usize],
    #[doc = " solver impedance: dof friction"]
    pub solimp_friction: [mjtNum; 5usize],
    #[doc = " group"]
    pub group: ::core::ffi::c_int,
    #[doc = " is gravcomp force applied via actuators"]
    pub actgravcomp: mjtBool,
    #[doc = " user data"]
    pub(crate) userdata: *mut mjDoubleVec,
    #[doc = " message appended to compiler errors"]
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsGeom {
    #[doc = " element type"]
    pub(crate) element: *mut mjsElement,
    #[doc = " geom type"]
    pub type_: mjtGeom,
    #[doc = " position"]
    pub pos: [f64; 3usize],
    #[doc = " orientation"]
    pub quat: [f64; 4usize],
    #[doc = " alternative orientation"]
    pub alt: mjsOrientation,
    #[doc = " alternative for capsule, cylinder, box, ellipsoid"]
    pub fromto: [f64; 6usize],
    #[doc = " type-specific size"]
    pub size: [f64; 3usize],
    #[doc = " contact type"]
    pub contype: ::core::ffi::c_int,
    #[doc = " contact affinity"]
    pub conaffinity: ::core::ffi::c_int,
    #[doc = " contact dimensionality"]
    pub condim: ::core::ffi::c_int,
    #[doc = " contact priority"]
    pub priority: ::core::ffi::c_int,
    #[doc = " one-sided friction coefficients: slide, roll, spin"]
    pub friction: [f64; 3usize],
    #[doc = " solver mixing for contact pairs"]
    pub solmix: f64,
    #[doc = " solver reference"]
    pub solref: [mjtNum; 2usize],
    #[doc = " solver impedance"]
    pub solimp: [mjtNum; 5usize],
    #[doc = " margin for contact detection"]
    pub margin: f64,
    #[doc = " additional contact detection buffer"]
    pub gap: f64,
    #[doc = " surface velocity in local frame: linear, angular"]
    pub surfacevel: [f64; 6usize],
    #[doc = " adhesive force of contacts"]
    pub adhesion: f64,
    #[doc = " used to compute density"]
    pub mass: f64,
    #[doc = " used to compute mass and inertia from volume or surface"]
    pub density: f64,
    #[doc = " selects between surface and volume inertia"]
    pub typeinertia: mjtGeomInertia,
    #[doc = " whether ellipsoid-fluid model is active"]
    pub(crate) fluid_ellipsoid: mjtNum,
    #[doc = " ellipsoid-fluid interaction coefs"]
    pub fluid_coefs: [mjtNum; 5usize],
    #[doc = " name of material"]
    pub(crate) material: *mut mjString,
    #[doc = " rgba when material is omitted"]
    pub rgba: [f32; 4usize],
    #[doc = " group"]
    pub group: ::core::ffi::c_int,
    #[doc = " heightfield attached to geom"]
    pub(crate) hfieldname: *mut mjString,
    #[doc = " mesh attached to geom"]
    pub(crate) meshname: *mut mjString,
    #[doc = " scale mesh uniformly"]
    pub fitscale: f64,
    #[doc = " user data"]
    pub(crate) userdata: *mut mjDoubleVec,
    #[doc = " sdf plugin"]
    pub plugin: mjsPlugin,
    #[doc = " message appended to compiler errors"]
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsSite {
    #[doc = " element type"]
    pub(crate) element: *mut mjsElement,
    #[doc = " position"]
    pub pos: [f64; 3usize],
    #[doc = " orientation"]
    pub quat: [f64; 4usize],
    #[doc = " alternative orientation"]
    pub alt: mjsOrientation,
    #[doc = " alternative for capsule, cylinder, box, ellipsoid"]
    pub fromto: [f64; 6usize],
    #[doc = " geom size"]
    pub size: [f64; 3usize],
    #[doc = " geom type"]
    pub type_: mjtGeom,
    #[doc = " name of material"]
    pub(crate) material: *mut mjString,
    #[doc = " group"]
    pub group: ::core::ffi::c_int,
    #[doc = " rgba when material is omitted"]
    pub rgba: [f32; 4usize],
    #[doc = " user data"]
    pub(crate) userdata: *mut mjDoubleVec,
    #[doc = " message appended to compiler errors"]
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsCamera {
    #[doc = " element type"]
    pub(crate) element: *mut mjsElement,
    #[doc = " position"]
    pub pos: [f64; 3usize],
    #[doc = " orientation"]
    pub quat: [f64; 4usize],
    #[doc = " alternative orientation"]
    pub alt: mjsOrientation,
    #[doc = " tracking mode"]
    pub mode: mjtCamLight,
    #[doc = " target body for tracking/targeting"]
    pub(crate) targetbody: *mut mjString,
    #[doc = " camera projection type"]
    pub proj: mjtProjection,
    #[doc = " resolution (pixel)"]
    pub resolution: [::core::ffi::c_int; 2usize],
    #[doc = " bit flags for output type"]
    pub output: ::core::ffi::c_int,
    #[doc = " y-field of view"]
    pub fovy: f64,
    #[doc = " inter-pupillary distance"]
    pub ipd: f64,
    #[doc = " camera intrinsics (length)"]
    pub intrinsic: [f32; 4usize],
    #[doc = " sensor size (length)"]
    pub sensor_size: [f32; 2usize],
    #[doc = " focal length (length)"]
    pub focal_length: [f32; 2usize],
    #[doc = " focal length (pixel)"]
    pub focal_pixel: [f32; 2usize],
    #[doc = " principal point (length)"]
    pub principal_length: [f32; 2usize],
    #[doc = " principal point (pixel)"]
    pub principal_pixel: [f32; 2usize],
    #[doc = " user data"]
    pub(crate) userdata: *mut mjDoubleVec,
    #[doc = " message appended to compiler errors"]
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsLight {
    #[doc = " element type"]
    pub(crate) element: *mut mjsElement,
    #[doc = " position"]
    pub pos: [f64; 3usize],
    #[doc = " direction"]
    pub dir: [f64; 3usize],
    #[doc = " tracking mode"]
    pub mode: mjtCamLight,
    #[doc = " target body for targeting"]
    pub(crate) targetbody: *mut mjString,
    #[doc = " is light active"]
    pub active: mjtBool,
    #[doc = " type of light"]
    pub type_: mjtLightType,
    #[doc = " texture name for image lights"]
    pub(crate) texture: *mut mjString,
    #[doc = " does light cast shadows"]
    pub castshadow: mjtBool,
    #[doc = " bulb radius, for soft shadows"]
    pub bulbradius: f32,
    #[doc = " intensity, in candelas"]
    pub intensity: f32,
    #[doc = " range of effectiveness"]
    pub range: f32,
    #[doc = " OpenGL attenuation (quadratic model)"]
    pub attenuation: [f32; 3usize],
    #[doc = " OpenGL cutoff"]
    pub cutoff: f32,
    #[doc = " OpenGL exponent"]
    pub exponent: f32,
    #[doc = " ambient color"]
    pub ambient: [f32; 3usize],
    #[doc = " diffuse color"]
    pub diffuse: [f32; 3usize],
    #[doc = " specular color"]
    pub specular: [f32; 3usize],
    #[doc = " message appended to compiler errors"]
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsFlex {
    #[doc = " element type"]
    pub(crate) element: *mut mjsElement,
    #[doc = " contact type"]
    pub contype: ::core::ffi::c_int,
    #[doc = " contact affinity"]
    pub conaffinity: ::core::ffi::c_int,
    #[doc = " contact dimensionality"]
    pub condim: ::core::ffi::c_int,
    #[doc = " contact priority"]
    pub priority: ::core::ffi::c_int,
    #[doc = " one-sided friction coefficients: slide, roll, spin"]
    pub friction: [f64; 3usize],
    #[doc = " solver mixing for contact pairs"]
    pub solmix: f64,
    #[doc = " solver reference"]
    pub solref: [mjtNum; 2usize],
    #[doc = " solver impedance"]
    pub solimp: [mjtNum; 5usize],
    #[doc = " margin for contact detection"]
    pub margin: f64,
    #[doc = " additional contact detection buffer"]
    pub gap: f64,
    #[doc = " element dimensionality"]
    pub dim: ::core::ffi::c_int,
    #[doc = " radius around primitive element"]
    pub radius: f64,
    #[doc = " vertex bounding box half sizes in qpos0"]
    pub size: [f64; 3usize],
    #[doc = " enable internal collisions"]
    pub internal: mjtBool,
    #[doc = " render flex skin with flat shading"]
    pub flatskin: mjtBool,
    #[doc = " mode for flex self collision"]
    pub selfcollide: mjtFlexSelf,
    #[doc = " mode for passive collisions"]
    pub passive: ::core::ffi::c_int,
    #[doc = " number of active element layers in 3D"]
    pub activelayers: ::core::ffi::c_int,
    #[doc = " group for visualization"]
    pub group: ::core::ffi::c_int,
    #[doc = " edge stiffness"]
    pub edgestiffness: f64,
    #[doc = " edge damping"]
    pub edgedamping: f64,
    #[doc = " rgba when material is omitted"]
    pub rgba: [f32; 4usize],
    #[doc = " name of material used for rendering"]
    pub(crate) material: *mut mjString,
    #[doc = " Young's modulus"]
    pub young: f64,
    #[doc = " Poisson's ratio"]
    pub poisson: f64,
    #[doc = " Rayleigh's damping"]
    pub damping: f64,
    #[doc = " thickness (2D only)"]
    pub thickness: f64,
    #[doc = " 2D passive forces; 0: none, 1: bending, 2: stretching, 3: both"]
    pub elastic2d: ::core::ffi::c_int,
    #[doc = " grid cell count for finite cell method"]
    pub cellcount: [::core::ffi::c_int; 3usize],
    #[doc = " interpolation order (1: trilinear, 2: quadratic)"]
    pub order: ::core::ffi::c_int,
    #[doc = " node body names"]
    pub(crate) nodebody: *mut mjStringVec,
    #[doc = " vertex body names"]
    pub(crate) vertbody: *mut mjStringVec,
    #[doc = " node positions"]
    pub(crate) node: *mut mjDoubleVec,
    #[doc = " vertex positions"]
    pub(crate) vert: *mut mjDoubleVec,
    #[doc = " element vertex ids"]
    pub(crate) elem: *mut mjIntVec,
    #[doc = " vertex texture coordinates"]
    pub(crate) texcoord: *mut mjFloatVec,
    #[doc = " element texture coordinates"]
    pub(crate) elemtexcoord: *mut mjIntVec,
    #[doc = " message appended to compiler errors"]
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsMesh {
    #[doc = " element type"]
    pub(crate) element: *mut mjsElement,
    #[doc = " content type of file"]
    pub(crate) content_type: *mut mjString,
    #[doc = " mesh file"]
    pub(crate) file: *mut mjString,
    #[doc = " reference position"]
    pub refpos: [f64; 3usize],
    #[doc = " reference orientation"]
    pub refquat: [f64; 4usize],
    #[doc = " rescale mesh"]
    pub scale: [f64; 3usize],
    #[doc = " inertia type (convex, legacy, exact, shell)"]
    pub inertia: mjtMeshInertia,
    #[doc = " do not exclude large-angle faces from normals"]
    pub smoothnormal: mjtBool,
    #[doc = " compute sdf from mesh"]
    pub needsdf: mjtBool,
    #[doc = " maximum vertex count for the convex hull"]
    pub maxhullvert: ::core::ffi::c_int,
    #[doc = " user vertex data"]
    pub(crate) uservert: *mut mjFloatVec,
    #[doc = " user normal data"]
    pub(crate) usernormal: *mut mjFloatVec,
    #[doc = " user texcoord data"]
    pub(crate) usertexcoord: *mut mjFloatVec,
    #[doc = " user vertex indices"]
    pub(crate) userface: *mut mjIntVec,
    #[doc = " user face normal indices"]
    pub(crate) userfacenormal: *mut mjIntVec,
    #[doc = " user texcoord indices"]
    pub(crate) userfacetexcoord: *mut mjIntVec,
    #[doc = " sdf plugin"]
    pub plugin: mjsPlugin,
    #[doc = " name of material"]
    pub(crate) material: *mut mjString,
    #[doc = " max octree depth"]
    pub octree_maxdepth: ::core::ffi::c_int,
    #[doc = " message appended to compiler errors"]
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsHField {
    #[doc = " element type"]
    pub(crate) element: *mut mjsElement,
    #[doc = " content type of file"]
    pub(crate) content_type: *mut mjString,
    #[doc = " file: (nrow, ncol, [elevation data])"]
    pub(crate) file: *mut mjString,
    #[doc = " hfield size (ignore referencing geom size)"]
    pub size: [f64; 4usize],
    #[doc = " number of rows"]
    pub nrow: ::core::ffi::c_int,
    #[doc = " number of columns"]
    pub ncol: ::core::ffi::c_int,
    #[doc = " user-provided elevation data"]
    pub(crate) userdata: *mut mjFloatVec,
    #[doc = " message appended to compiler errors"]
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsSkin {
    #[doc = " element type"]
    pub(crate) element: *mut mjsElement,
    #[doc = " skin file"]
    pub(crate) file: *mut mjString,
    #[doc = " name of material used for rendering"]
    pub(crate) material: *mut mjString,
    #[doc = " rgba when material is omitted"]
    pub rgba: [f32; 4usize],
    #[doc = " inflate in normal direction"]
    pub inflate: f32,
    #[doc = " group for visualization"]
    pub group: ::core::ffi::c_int,
    #[doc = " vertex positions"]
    pub(crate) vert: *mut mjFloatVec,
    #[doc = " texture coordinates"]
    pub(crate) texcoord: *mut mjFloatVec,
    #[doc = " faces"]
    pub(crate) face: *mut mjIntVec,
    #[doc = " body names"]
    pub(crate) bodyname: *mut mjStringVec,
    #[doc = " bind pos"]
    pub(crate) bindpos: *mut mjFloatVec,
    #[doc = " bind quat"]
    pub(crate) bindquat: *mut mjFloatVec,
    #[doc = " vertex ids"]
    pub(crate) vertid: *mut mjIntVecVec,
    #[doc = " vertex weights"]
    pub(crate) vertweight: *mut mjFloatVecVec,
    #[doc = " message appended to compiler errors"]
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsTexture {
    #[doc = " element type"]
    pub(crate) element: *mut mjsElement,
    #[doc = " texture type"]
    pub type_: mjtTexture,
    #[doc = " colorspace"]
    pub colorspace: mjtColorSpace,
    #[doc = " builtin type"]
    pub builtin: mjtBuiltin,
    #[doc = " mark type"]
    pub mark: mjtMark,
    #[doc = " first color for builtin"]
    pub rgb1: [f64; 3usize],
    #[doc = " second color for builtin"]
    pub rgb2: [f64; 3usize],
    #[doc = " mark color"]
    pub markrgb: [f64; 3usize],
    #[doc = " probability of random dots"]
    pub random: f64,
    #[doc = " height in pixels (square for cube and skybox)"]
    pub height: ::core::ffi::c_int,
    #[doc = " width in pixels"]
    pub width: ::core::ffi::c_int,
    #[doc = " number of channels"]
    pub nchannel: ::core::ffi::c_int,
    #[doc = " content type of file"]
    pub(crate) content_type: *mut mjString,
    #[doc = " png file to load; use for all sides of cube"]
    pub(crate) file: *mut mjString,
    #[doc = " size of grid for composite file; (1,1)-repeat"]
    pub gridsize: [::core::ffi::c_int; 2usize],
    #[doc = " row-major: L,R,F,B,U,D for faces; . for unused"]
    pub(crate) gridlayout: [::core::ffi::c_char; 12usize],
    #[doc = " different file for each side of the cube"]
    pub(crate) cubefiles: *mut mjStringVec,
    #[doc = " texture data"]
    pub(crate) data: *mut mjByteVec,
    #[doc = " horizontal flip"]
    pub hflip: mjtBool,
    #[doc = " vertical flip"]
    pub vflip: mjtBool,
    #[doc = " message appended to compiler errors"]
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsMaterial {
    #[doc = " element type"]
    pub(crate) element: *mut mjsElement,
    #[doc = " names of textures (empty: none)"]
    pub(crate) textures: *mut mjStringVec,
    #[doc = " make texture cube uniform"]
    pub texuniform: mjtBool,
    #[doc = " texture repetition for 2D mapping"]
    pub texrepeat: [f32; 2usize],
    #[doc = " emission"]
    pub emission: f32,
    #[doc = " specular"]
    pub specular: f32,
    #[doc = " shininess"]
    pub shininess: f32,
    #[doc = " reflectance"]
    pub reflectance: f32,
    #[doc = " metallic"]
    pub metallic: f32,
    #[doc = " roughness"]
    pub roughness: f32,
    #[doc = " rgba"]
    pub rgba: [f32; 4usize],
    #[doc = " message appended to compiler errors"]
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsPair {
    #[doc = " element type"]
    pub(crate) element: *mut mjsElement,
    #[doc = " name of geom 1"]
    pub(crate) geomname1: *mut mjString,
    #[doc = " name of geom 2"]
    pub(crate) geomname2: *mut mjString,
    #[doc = " contact dimensionality"]
    pub condim: ::core::ffi::c_int,
    #[doc = " solver reference, normal direction"]
    pub solref: [mjtNum; 2usize],
    #[doc = " solver reference, frictional directions"]
    pub solreffriction: [mjtNum; 2usize],
    #[doc = " solver impedance"]
    pub solimp: [mjtNum; 5usize],
    #[doc = " margin for contact detection"]
    pub margin: f64,
    #[doc = " additional contact detection buffer"]
    pub gap: f64,
    #[doc = " adhesive force of contacts"]
    pub adhesion: f64,
    #[doc = " full contact friction"]
    pub friction: [f64; 5usize],
    #[doc = " message appended to errors"]
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsExclude {
    #[doc = " element type"]
    pub(crate) element: *mut mjsElement,
    #[doc = " name of geom 1"]
    pub(crate) bodyname1: *mut mjString,
    #[doc = " name of geom 2"]
    pub(crate) bodyname2: *mut mjString,
    #[doc = " message appended to errors"]
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsEquality {
    #[doc = " element type"]
    pub(crate) element: *mut mjsElement,
    #[doc = " constraint type"]
    pub type_: mjtEq,
    #[doc = " type-dependent data"]
    pub data: [f64; 11usize],
    #[doc = " is equality initially active"]
    pub active: mjtBool,
    #[doc = " name of object 1"]
    pub(crate) name1: *mut mjString,
    #[doc = " name of object 2"]
    pub(crate) name2: *mut mjString,
    #[doc = " type of both objects"]
    pub objtype: mjtObj,
    #[doc = " solver reference"]
    pub solref: [mjtNum; 2usize],
    #[doc = " solver impedance"]
    pub solimp: [mjtNum; 5usize],
    #[doc = " message appended to errors"]
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsTendon {
    #[doc = " element type"]
    pub(crate) element: *mut mjsElement,
    #[doc = " stiffness coefficients"]
    pub stiffness: [f64; 3usize],
    #[doc = " spring resting length; {-1, -1}: use qpos_spring"]
    pub springlength: [f64; 2usize],
    #[doc = " damping coefficients"]
    pub damping: [f64; 3usize],
    #[doc = " friction loss"]
    pub frictionloss: f64,
    #[doc = " solver reference: tendon friction"]
    pub solref_friction: [mjtNum; 2usize],
    #[doc = " solver impedance: tendon friction"]
    pub solimp_friction: [mjtNum; 5usize],
    #[doc = " inertia associated with tendon velocity"]
    pub armature: f64,
    #[doc = " does tendon have limits"]
    pub limited: mjtLimited,
    #[doc = " does tendon have actuator force limits"]
    pub actfrclimited: mjtLimited,
    #[doc = " length limits"]
    pub range: [f64; 2usize],
    #[doc = " actuator force limits"]
    pub actfrcrange: [f64; 2usize],
    #[doc = " margin value for tendon limit detection"]
    pub margin: f64,
    #[doc = " solver reference: tendon limits"]
    pub solref_limit: [mjtNum; 2usize],
    #[doc = " solver impedance: tendon limits"]
    pub solimp_limit: [mjtNum; 5usize],
    #[doc = " name of material for rendering"]
    pub(crate) material: *mut mjString,
    #[doc = " width for rendering"]
    pub width: f64,
    #[doc = " rgba when material is omitted"]
    pub rgba: [f32; 4usize],
    #[doc = " group"]
    pub group: ::core::ffi::c_int,
    #[doc = " user data"]
    pub(crate) userdata: *mut mjDoubleVec,
    #[doc = " message appended to errors"]
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsWrap {
    #[doc = " element type"]
    pub(crate) element: *mut mjsElement,
    #[doc = " wrap type"]
    pub type_: mjtWrap,
    #[doc = " message appended to errors"]
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsActuator {
    #[doc = " element type"]
    pub(crate) element: *mut mjsElement,
    #[doc = " gain type"]
    pub gaintype: mjtGain,
    #[doc = " gain parameters"]
    pub gainprm: [f64; 10usize],
    #[doc = " bias type"]
    pub biastype: mjtBias,
    #[doc = " bias parameters"]
    pub biasprm: [f64; 10usize],
    #[doc = " dynamics type"]
    pub dyntype: mjtDyn,
    #[doc = " dynamics parameters"]
    pub dynprm: [f64; 10usize],
    #[doc = " number of activation variables"]
    pub actdim: ::core::ffi::c_int,
    #[doc = " input signature, scoped by gaintype; 0: type default"]
    pub ctrlspec: ::core::ffi::c_int,
    #[doc = " apply next activations to qfrc"]
    pub actearly: mjtBool,
    #[doc = " transmission type"]
    pub trntype: mjtTrn,
    #[doc = " length and transmitted force scaling"]
    pub gear: [f64; 6usize],
    #[doc = " name of transmission target"]
    pub(crate) target: *mut mjString,
    #[doc = " reference site, for site transmission"]
    pub(crate) refsite: *mut mjString,
    #[doc = " site defining cylinder, for slider-crank"]
    pub(crate) slidersite: *mut mjString,
    #[doc = " crank length, for slider-crank"]
    pub cranklength: f64,
    #[doc = " transmission length range"]
    pub lengthrange: [f64; 2usize],
    #[doc = " automatic range setting for position and intvelocity"]
    pub inheritrange: f64,
    #[doc = " damping coefficients"]
    pub damping: [f64; 3usize],
    #[doc = " armature inertia"]
    pub armature: f64,
    #[doc = " are control limits defined"]
    pub ctrllimited: mjtLimited,
    #[doc = " control range"]
    pub ctrlrange: [f64; 2usize],
    #[doc = " are force limits defined"]
    pub forcelimited: mjtLimited,
    #[doc = " force range"]
    pub forcerange: [f64; 2usize],
    #[doc = " are activation limits defined"]
    pub actlimited: mjtLimited,
    #[doc = " activation range"]
    pub actrange: [f64; 2usize],
    #[doc = " group"]
    pub group: ::core::ffi::c_int,
    #[doc = " number of samples in history buffer"]
    pub nsample: ::core::ffi::c_int,
    #[doc = " interpolation order (0=ZOH, 1=linear, 2=cubic)"]
    pub interp: ::core::ffi::c_int,
    #[doc = " delay time in seconds; 0: no delay"]
    pub delay: f64,
    #[doc = " user data"]
    pub(crate) userdata: *mut mjDoubleVec,
    #[doc = " actuator plugin"]
    pub plugin: mjsPlugin,
    #[doc = " message appended to compiler errors"]
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsSensor {
    #[doc = " element type"]
    pub(crate) element: *mut mjsElement,
    #[doc = " type of sensor"]
    pub type_: mjtSensor,
    #[doc = " type of sensorized object"]
    pub objtype: mjtObj,
    #[doc = " name of sensorized object"]
    pub(crate) objname: *mut mjString,
    #[doc = " type of referenced object"]
    pub reftype: mjtObj,
    #[doc = " name of referenced object"]
    pub(crate) refname: *mut mjString,
    #[doc = " integer parameters"]
    pub intprm: [::core::ffi::c_int; 3usize],
    #[doc = " data type for sensor measurement"]
    pub datatype: mjtDataType,
    #[doc = " compute stage needed to simulate sensor"]
    pub needstage: mjtStage,
    #[doc = " number of scalar outputs"]
    pub dim: ::core::ffi::c_int,
    #[doc = " cutoff for real and positive datatypes"]
    pub cutoff: f64,
    #[doc = " noise stdev"]
    pub noise: f64,
    #[doc = " number of samples in history buffer"]
    pub nsample: ::core::ffi::c_int,
    #[doc = " interpolation order (0=ZOH, 1=linear, 2=cubic)"]
    pub interp: ::core::ffi::c_int,
    #[doc = " delay time in seconds"]
    pub delay: f64,
    #[doc = " [period, time_prev] in seconds"]
    pub interval: [f64; 2usize],
    #[doc = " user data"]
    pub(crate) userdata: *mut mjDoubleVec,
    #[doc = " sensor plugin"]
    pub plugin: mjsPlugin,
    #[doc = " message appended to compiler errors"]
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsNumeric {
    #[doc = " element type"]
    pub(crate) element: *mut mjsElement,
    #[doc = " initialization data"]
    pub(crate) data: *mut mjDoubleVec,
    #[doc = " array size, can be bigger than data size"]
    pub size: ::core::ffi::c_int,
    #[doc = " message appended to compiler errors"]
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsText {
    #[doc = " element type"]
    pub(crate) element: *mut mjsElement,
    #[doc = " text string"]
    pub(crate) data: *mut mjString,
    #[doc = " message appended to compiler errors"]
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsTuple {
    #[doc = " element type"]
    pub(crate) element: *mut mjsElement,
    #[doc = " object types"]
    pub(crate) objtype: *mut mjIntVec,
    #[doc = " object names"]
    pub(crate) objname: *mut mjStringVec,
    #[doc = " object parameters"]
    pub(crate) objprm: *mut mjDoubleVec,
    #[doc = " message appended to compiler errors"]
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsKey {
    #[doc = " element type"]
    pub(crate) element: *mut mjsElement,
    #[doc = " time"]
    pub time: f64,
    #[doc = " qpos"]
    pub(crate) qpos: *mut mjDoubleVec,
    #[doc = " qvel"]
    pub(crate) qvel: *mut mjDoubleVec,
    #[doc = " act"]
    pub(crate) act: *mut mjDoubleVec,
    #[doc = " mocap pos"]
    pub(crate) mpos: *mut mjDoubleVec,
    #[doc = " mocap quat"]
    pub(crate) mquat: *mut mjDoubleVec,
    #[doc = " ctrl"]
    pub(crate) ctrl: *mut mjDoubleVec,
    #[doc = " message appended to compiler errors"]
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsDefault {
    #[doc = " element type"]
    pub(crate) element: *mut mjsElement,
    #[doc = " joint defaults"]
    pub(crate) joint: *mut mjsJoint,
    #[doc = " geom defaults"]
    pub(crate) geom: *mut mjsGeom,
    #[doc = " site defaults"]
    pub(crate) site: *mut mjsSite,
    #[doc = " camera defaults"]
    pub(crate) camera: *mut mjsCamera,
    #[doc = " light defaults"]
    pub(crate) light: *mut mjsLight,
    #[doc = " flex defaults"]
    pub(crate) flex: *mut mjsFlex,
    #[doc = " mesh defaults"]
    pub(crate) mesh: *mut mjsMesh,
    #[doc = " material defaults"]
    pub(crate) material: *mut mjsMaterial,
    #[doc = " pair defaults"]
    pub(crate) pair: *mut mjsPair,
    #[doc = " equality defaults"]
    pub(crate) equality: *mut mjsEquality,
    #[doc = " tendon defaults"]
    pub(crate) tendon: *mut mjsTendon,
    #[doc = " actuator defaults"]
    pub(crate) actuator: *mut mjsActuator,
}
impl mjtCatBit {
    #[doc = " model elements in body 0"]
    pub const STATIC: mjtCatBit = mjtCatBit(1);
    #[doc = " model elements in all other bodies"]
    pub const DYNAMIC: mjtCatBit = mjtCatBit(2);
    #[doc = " decorative geoms"]
    pub const DECOR: mjtCatBit = mjtCatBit(4);
    #[doc = " select all categories"]
    pub const ALL: mjtCatBit = mjtCatBit(7);
}
impl ::core::ops::BitOr<mjtCatBit> for mjtCatBit {
    type Output = Self;
    #[inline]
    fn bitor(self, other: Self) -> Self {
        mjtCatBit(self.0 | other.0)
    }
}
impl ::core::ops::BitOrAssign for mjtCatBit {
    #[inline]
    fn bitor_assign(&mut self, rhs: mjtCatBit) {
        self.0 |= rhs.0;
    }
}
impl ::core::ops::BitAnd<mjtCatBit> for mjtCatBit {
    type Output = Self;
    #[inline]
    fn bitand(self, other: Self) -> Self {
        mjtCatBit(self.0 & other.0)
    }
}
impl ::core::ops::BitAndAssign for mjtCatBit {
    #[inline]
    fn bitand_assign(&mut self, rhs: mjtCatBit) {
        self.0 &= rhs.0;
    }
}
#[repr(transparent)]
#[doc = "---------------------------------- primitive types (mjt) -----------------------------------------"]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtCatBit(pub(crate) ::core::ffi::c_uint);
impl mjtMouse {
    #[doc = " no action"]
    pub const NONE: mjtMouse = mjtMouse(0);
    #[doc = " rotate (orbit) vertically"]
    pub const ROTATE_V: mjtMouse = mjtMouse(1);
    #[doc = " rotate (orbit) horizontally"]
    pub const ROTATE_H: mjtMouse = mjtMouse(2);
    #[doc = " move along vertical plane"]
    pub const MOVE_V: mjtMouse = mjtMouse(3);
    #[doc = " move along horizontal plane"]
    pub const MOVE_H: mjtMouse = mjtMouse(4);
    #[doc = " zoom (towards/away from lookat point)"]
    pub const ZOOM: mjtMouse = mjtMouse(5);
    #[doc = " move (truck, pedestal), vertical plane rel. to target"]
    pub const MOVE_V_REL: mjtMouse = mjtMouse(6);
    #[doc = " move (truck, dolly), horizontal plane rel. to target"]
    pub const MOVE_H_REL: mjtMouse = mjtMouse(7);
    #[doc = " turn (tilt) vertically"]
    pub const TURN_V: mjtMouse = mjtMouse(8);
    #[doc = " turn (pan) horizontally"]
    pub const TURN_H: mjtMouse = mjtMouse(9);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtMouse(pub(crate) ::core::ffi::c_uint);
impl mjtPertBit {
    #[doc = " translation"]
    pub const TRANSLATE: mjtPertBit = mjtPertBit(1);
    #[doc = " rotation"]
    pub const ROTATE: mjtPertBit = mjtPertBit(2);
}
impl ::core::ops::BitOr<mjtPertBit> for mjtPertBit {
    type Output = Self;
    #[inline]
    fn bitor(self, other: Self) -> Self {
        mjtPertBit(self.0 | other.0)
    }
}
impl ::core::ops::BitOrAssign for mjtPertBit {
    #[inline]
    fn bitor_assign(&mut self, rhs: mjtPertBit) {
        self.0 |= rhs.0;
    }
}
impl ::core::ops::BitAnd<mjtPertBit> for mjtPertBit {
    type Output = Self;
    #[inline]
    fn bitand(self, other: Self) -> Self {
        mjtPertBit(self.0 & other.0)
    }
}
impl ::core::ops::BitAndAssign for mjtPertBit {
    #[inline]
    fn bitand_assign(&mut self, rhs: mjtPertBit) {
        self.0 &= rhs.0;
    }
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtPertBit(pub(crate) ::core::ffi::c_uint);
impl mjtCamera {
    #[doc = " free camera"]
    pub const FREE: mjtCamera = mjtCamera(0);
    #[doc = " tracking camera; uses trackbodyid"]
    pub const TRACKING: mjtCamera = mjtCamera(1);
    #[doc = " fixed camera; uses fixedcamid"]
    pub const FIXED: mjtCamera = mjtCamera(2);
    #[doc = " user is responsible for setting OpenGL camera"]
    pub const USER: mjtCamera = mjtCamera(3);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtCamera(pub(crate) ::core::ffi::c_uint);
pub const mjNLABEL: usize = mjtLabel::mjNLABEL.0 as usize;
impl mjtLabel {
    #[doc = " nothing"]
    pub const NONE: mjtLabel = mjtLabel(0);
    #[doc = " body labels"]
    pub const BODY: mjtLabel = mjtLabel(1);
    #[doc = " joint labels"]
    pub const JOINT: mjtLabel = mjtLabel(2);
    #[doc = " geom labels"]
    pub const GEOM: mjtLabel = mjtLabel(3);
    #[doc = " site labels"]
    pub const SITE: mjtLabel = mjtLabel(4);
    #[doc = " camera labels"]
    pub const CAMERA: mjtLabel = mjtLabel(5);
    #[doc = " light labels"]
    pub const LIGHT: mjtLabel = mjtLabel(6);
    #[doc = " tendon labels"]
    pub const TENDON: mjtLabel = mjtLabel(7);
    #[doc = " actuator labels"]
    pub const ACTUATOR: mjtLabel = mjtLabel(8);
    #[doc = " constraint labels"]
    pub const CONSTRAINT: mjtLabel = mjtLabel(9);
    #[doc = " flex labels"]
    pub const FLEX: mjtLabel = mjtLabel(10);
    #[doc = " skin labels"]
    pub const SKIN: mjtLabel = mjtLabel(11);
    #[doc = " selected object"]
    pub const SELECTION: mjtLabel = mjtLabel(12);
    #[doc = " coordinates of selection point"]
    pub const SELPNT: mjtLabel = mjtLabel(13);
    #[doc = " contact information"]
    pub const CONTACTPOINT: mjtLabel = mjtLabel(14);
    #[doc = " magnitude of contact force"]
    pub const CONTACTFORCE: mjtLabel = mjtLabel(15);
    #[doc = " id of island"]
    pub const ISLAND: mjtLabel = mjtLabel(16);
    #[doc = " number of label types"]
    const mjNLABEL: mjtLabel = mjtLabel(17);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtLabel(pub(crate) ::core::ffi::c_uint);
pub const mjNFRAME: usize = mjtFrame::mjNFRAME.0 as usize;
impl mjtFrame {
    #[doc = " no frames"]
    pub const NONE: mjtFrame = mjtFrame(0);
    #[doc = " body frames"]
    pub const BODY: mjtFrame = mjtFrame(1);
    #[doc = " geom frames"]
    pub const GEOM: mjtFrame = mjtFrame(2);
    #[doc = " site frames"]
    pub const SITE: mjtFrame = mjtFrame(3);
    #[doc = " camera frames"]
    pub const CAMERA: mjtFrame = mjtFrame(4);
    #[doc = " light frames"]
    pub const LIGHT: mjtFrame = mjtFrame(5);
    #[doc = " contact frames"]
    pub const CONTACT: mjtFrame = mjtFrame(6);
    #[doc = " world frame"]
    pub const WORLD: mjtFrame = mjtFrame(7);
    #[doc = " number of visualization frames"]
    const mjNFRAME: mjtFrame = mjtFrame(8);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtFrame(pub(crate) ::core::ffi::c_uint);
pub const mjNVISFLAG: usize = mjtVisFlag::mjNVISFLAG.0 as usize;
impl mjtVisFlag {
    #[doc = " mesh convex hull"]
    pub const CONVEXHULL: mjtVisFlag = mjtVisFlag(0);
    #[doc = " textures"]
    pub const TEXTURE: mjtVisFlag = mjtVisFlag(1);
    #[doc = " joints"]
    pub const JOINT: mjtVisFlag = mjtVisFlag(2);
    #[doc = " cameras"]
    pub const CAMERA: mjtVisFlag = mjtVisFlag(3);
    #[doc = " actuators"]
    pub const ACTUATOR: mjtVisFlag = mjtVisFlag(4);
    #[doc = " activations"]
    pub const ACTIVATION: mjtVisFlag = mjtVisFlag(5);
    #[doc = " lights"]
    pub const LIGHT: mjtVisFlag = mjtVisFlag(6);
    #[doc = " tendons"]
    pub const TENDON: mjtVisFlag = mjtVisFlag(7);
    #[doc = " rangefinder sensors"]
    pub const RANGEFINDER: mjtVisFlag = mjtVisFlag(8);
    #[doc = " point constraints"]
    pub const CONSTRAINT: mjtVisFlag = mjtVisFlag(9);
    #[doc = " equivalent inertia boxes"]
    pub const INERTIA: mjtVisFlag = mjtVisFlag(10);
    #[doc = " scale equivalent inertia boxes with mass"]
    pub const SCLINERTIA: mjtVisFlag = mjtVisFlag(11);
    #[doc = " perturbation force"]
    pub const PERTFORCE: mjtVisFlag = mjtVisFlag(12);
    #[doc = " perturbation object"]
    pub const PERTOBJ: mjtVisFlag = mjtVisFlag(13);
    #[doc = " contact points"]
    pub const CONTACTPOINT: mjtVisFlag = mjtVisFlag(14);
    #[doc = " constraint islands"]
    pub const ISLAND: mjtVisFlag = mjtVisFlag(15);
    #[doc = " contact force"]
    pub const CONTACTFORCE: mjtVisFlag = mjtVisFlag(16);
    #[doc = " split contact force into normal and tangent"]
    pub const CONTACTSPLIT: mjtVisFlag = mjtVisFlag(17);
    #[doc = " make dynamic geoms more transparent"]
    pub const TRANSPARENT: mjtVisFlag = mjtVisFlag(18);
    #[doc = " auto connect joints and body coms"]
    pub const AUTOCONNECT: mjtVisFlag = mjtVisFlag(19);
    #[doc = " center of mass"]
    pub const COM: mjtVisFlag = mjtVisFlag(20);
    #[doc = " selection point"]
    pub const SELECT: mjtVisFlag = mjtVisFlag(21);
    #[doc = " static bodies"]
    pub const STATIC: mjtVisFlag = mjtVisFlag(22);
    #[doc = " skin"]
    pub const SKIN: mjtVisFlag = mjtVisFlag(23);
    #[doc = " flex vertices"]
    pub const FLEXVERT: mjtVisFlag = mjtVisFlag(24);
    #[doc = " flex edges"]
    pub const FLEXEDGE: mjtVisFlag = mjtVisFlag(25);
    #[doc = " flex element faces"]
    pub const FLEXFACE: mjtVisFlag = mjtVisFlag(26);
    #[doc = " flex smooth skin (disables the rest)"]
    pub const FLEXSKIN: mjtVisFlag = mjtVisFlag(27);
    #[doc = " body bounding volume hierarchy"]
    pub const BODYBVH: mjtVisFlag = mjtVisFlag(28);
    #[doc = " mesh bounding volume hierarchy"]
    pub const MESHBVH: mjtVisFlag = mjtVisFlag(29);
    #[doc = " iterations of SDF gradient descent"]
    pub const SDFITER: mjtVisFlag = mjtVisFlag(30);
    #[doc = " number of visualization flags"]
    const mjNVISFLAG: mjtVisFlag = mjtVisFlag(31);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtVisFlag(pub(crate) ::core::ffi::c_uint);
pub const mjNRNDFLAG: usize = mjtRndFlag::mjNRNDFLAG.0 as usize;
impl mjtRndFlag {
    #[doc = " shadows"]
    pub const SHADOW: mjtRndFlag = mjtRndFlag(0);
    #[doc = " wireframe"]
    pub const WIREFRAME: mjtRndFlag = mjtRndFlag(1);
    #[doc = " reflections"]
    pub const REFLECTION: mjtRndFlag = mjtRndFlag(2);
    #[doc = " additive transparency"]
    pub const ADDITIVE: mjtRndFlag = mjtRndFlag(3);
    #[doc = " skybox"]
    pub const SKYBOX: mjtRndFlag = mjtRndFlag(4);
    #[doc = " fog"]
    pub const FOG: mjtRndFlag = mjtRndFlag(5);
    #[doc = " haze"]
    pub const HAZE: mjtRndFlag = mjtRndFlag(6);
    #[doc = " depth"]
    pub const DEPTH: mjtRndFlag = mjtRndFlag(7);
    #[doc = " segmentation with random color"]
    pub const SEGMENT: mjtRndFlag = mjtRndFlag(8);
    #[doc = " segmentation with segid+1 color"]
    pub const IDCOLOR: mjtRndFlag = mjtRndFlag(9);
    #[doc = " cull backward faces"]
    pub const CULL_FACE: mjtRndFlag = mjtRndFlag(10);
    #[doc = " number of rendering flags"]
    const mjNRNDFLAG: mjtRndFlag = mjtRndFlag(11);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtRndFlag(pub(crate) ::core::ffi::c_uint);
impl mjtStereo {
    #[doc = " no stereo; use left eye only"]
    pub const NONE: mjtStereo = mjtStereo(0);
    #[doc = " quad buffered; revert to side-by-side if no hardware support"]
    pub const QUADBUFFERED: mjtStereo = mjtStereo(1);
    #[doc = " side-by-side"]
    pub const SIDEBYSIDE: mjtStereo = mjtStereo(2);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtStereo(pub(crate) ::core::ffi::c_uint);
#[doc = "---------------------------------- mjvPerturb ----------------------------------------------------"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjvPerturb {
    #[doc = " selected body id; non-positive: none"]
    pub select: ::core::ffi::c_int,
    #[doc = " selected flex id; negative: none"]
    pub flexselect: ::core::ffi::c_int,
    #[doc = " selected skin id; negative: none"]
    pub skinselect: ::core::ffi::c_int,
    #[doc = " perturbation bitmask (mjtPertBit)"]
    pub active: ::core::ffi::c_int,
    #[doc = " secondary perturbation bitmask (mjtPertBit)"]
    pub active2: ::core::ffi::c_int,
    #[doc = " reference position for selected object"]
    pub refpos: [mjtNum; 3usize],
    #[doc = " reference orientation for selected object"]
    pub refquat: [mjtNum; 4usize],
    #[doc = " reference position for selection point"]
    pub refselpos: [mjtNum; 3usize],
    #[doc = " selection point in object coordinates"]
    pub localpos: [mjtNum; 3usize],
    #[doc = " spatial inertia at selection point"]
    pub localmass: mjtNum,
    #[doc = " relative mouse motion-to-space scaling (set by initPerturb)"]
    pub scale: mjtNum,
}
#[doc = "---------------------------------- mjvCamera -----------------------------------------------------"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjvCamera {
    #[doc = " camera type (mjtCamera)"]
    pub type_: ::core::ffi::c_int,
    #[doc = " fixed camera id"]
    pub(crate) fixedcamid: ::core::ffi::c_int,
    #[doc = " body id to track"]
    pub(crate) trackbodyid: ::core::ffi::c_int,
    #[doc = " lookat point"]
    pub lookat: [mjtNum; 3usize],
    #[doc = " distance to lookat point or tracked body"]
    pub distance: mjtNum,
    #[doc = " camera azimuth (deg)"]
    pub azimuth: mjtNum,
    #[doc = " camera elevation (deg)"]
    pub elevation: mjtNum,
    #[doc = " 0: perspective; 1: orthographic"]
    pub orthographic: ::core::ffi::c_int,
}
#[doc = "---------------------------------- mjvGLCamera ---------------------------------------------------"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjvGLCamera {
    #[doc = " position"]
    pub pos: [f32; 3usize],
    #[doc = " forward direction"]
    pub forward: [f32; 3usize],
    #[doc = " up direction"]
    pub up: [f32; 3usize],
    #[doc = " hor. center (left,right set to match aspect)"]
    pub frustum_center: f32,
    #[doc = " width (not used for rendering)"]
    pub frustum_width: f32,
    #[doc = " bottom"]
    pub frustum_bottom: f32,
    #[doc = " top"]
    pub frustum_top: f32,
    #[doc = " near"]
    pub frustum_near: f32,
    #[doc = " far"]
    pub frustum_far: f32,
    #[doc = " 0: perspective; 1: orthographic"]
    pub orthographic: ::core::ffi::c_int,
}
#[doc = "---------------------------------- mjvGeom -------------------------------------------------------"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjvGeom {
    #[doc = " geom type (mjtGeom)"]
    pub type_: ::core::ffi::c_int,
    #[doc = " mesh, hfield or plane id; -1: none; mesh: 2*id or 2*id+1 (hull)"]
    pub(crate) dataid: ::core::ffi::c_int,
    #[doc = " mujoco object type; mjOBJ_UNKNOWN for decor"]
    pub objtype: ::core::ffi::c_int,
    #[doc = " mujoco object id; -1 for decor"]
    pub(crate) objid: ::core::ffi::c_int,
    #[doc = " visual category"]
    pub category: ::core::ffi::c_int,
    #[doc = " material id; -1: no textured material"]
    pub(crate) matid: ::core::ffi::c_int,
    #[doc = " texture id; -1: none"]
    pub(crate) texid: ::core::ffi::c_int,
    #[doc = " uniform cube mapping"]
    pub texuniform: ::core::ffi::c_int,
    #[doc = " mesh or flex geom has texture coordinates"]
    pub texcoord: ::core::ffi::c_int,
    #[doc = " segmentation id; -1: not shown"]
    pub(crate) segid: ::core::ffi::c_int,
    #[doc = " size parameters"]
    pub size: [f32; 3usize],
    #[doc = " Cartesian position"]
    pub pos: [f32; 3usize],
    #[doc = " Cartesian orientation"]
    pub mat: [f32; 9usize],
    #[doc = " color and transparency"]
    pub rgba: [f32; 4usize],
    #[doc = " emission coef"]
    pub emission: f32,
    #[doc = " specular coef"]
    pub specular: f32,
    #[doc = " shininess coef"]
    pub shininess: f32,
    #[doc = " reflectance coef"]
    pub reflectance: f32,
    #[doc = " texture repetition for 2d mapping"]
    pub texrepeat: [f32; 2usize],
    #[doc = " text label"]
    pub(crate) label: [::core::ffi::c_char; 100usize],
    #[doc = " distance to camera (used by sorter)"]
    pub camdist: f32,
    #[doc = " geom rbound from model, 0 if not model geom"]
    pub modelrbound: f32,
    #[doc = " treat geom as transparent"]
    pub transparent: mjtByte,
}
#[doc = "---------------------------------- mjvLight ------------------------------------------------------"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjvLight {
    #[doc = " light id, -1 for headlight"]
    pub(crate) id: ::core::ffi::c_int,
    #[doc = " position rel. to body frame"]
    pub pos: [f32; 3usize],
    #[doc = " direction rel. to body frame"]
    pub dir: [f32; 3usize],
    #[doc = " type (mjtLightType)"]
    pub type_: ::core::ffi::c_int,
    #[doc = " texture id for image lights"]
    pub(crate) texid: ::core::ffi::c_int,
    #[doc = " OpenGL attenuation (quadratic model)"]
    pub attenuation: [f32; 3usize],
    #[doc = " OpenGL cutoff"]
    pub cutoff: f32,
    #[doc = " OpenGL exponent"]
    pub exponent: f32,
    #[doc = " ambient rgb (alpha=1)"]
    pub ambient: [f32; 3usize],
    #[doc = " diffuse rgb (alpha=1)"]
    pub diffuse: [f32; 3usize],
    #[doc = " specular rgb (alpha=1)"]
    pub specular: [f32; 3usize],
    #[doc = " headlight"]
    pub headlight: mjtByte,
    #[doc = " does light cast shadows"]
    pub castshadow: mjtByte,
    #[doc = " bulb radius for soft shadows"]
    pub bulbradius: f32,
    #[doc = " intensity, in candelas"]
    pub intensity: f32,
    #[doc = " range of effectiveness"]
    pub range: f32,
}
#[doc = "---------------------------------- mjvOption -----------------------------------------------------"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjvOption {
    #[doc = " what objects to label (mjtLabel)"]
    pub label: ::core::ffi::c_int,
    #[doc = " which frame to show (mjtFrame)"]
    pub frame: ::core::ffi::c_int,
    #[doc = " geom visualization by group"]
    pub geomgroup: [mjtByte; 6usize],
    #[doc = " site visualization by group"]
    pub sitegroup: [mjtByte; 6usize],
    #[doc = " joint visualization by group"]
    pub jointgroup: [mjtByte; 6usize],
    #[doc = " tendon visualization by group"]
    pub tendongroup: [mjtByte; 6usize],
    #[doc = " actuator visualization by group"]
    pub actuatorgroup: [mjtByte; 6usize],
    #[doc = " flex visualization by group"]
    pub flexgroup: [mjtByte; 6usize],
    #[doc = " skin visualization by group"]
    pub skingroup: [mjtByte; 6usize],
    #[doc = " visualization flags (indexed by mjtVisFlag)"]
    pub flags: [mjtByte; 31usize],
    #[doc = " depth of the bounding volume hierarchy to be visualized"]
    pub bvh_depth: ::core::ffi::c_int,
    #[doc = " element layer to be visualized for 3D flex"]
    pub flex_layer: ::core::ffi::c_int,
}
#[doc = "---------------------------------- mjvScene ------------------------------------------------------"]
#[repr(C)]
#[derive(Debug)]
pub struct mjvScene {
    #[doc = " size of allocated geom buffer"]
    pub maxgeom: ::core::ffi::c_int,
    #[doc = " number of geoms currently in buffer"]
    pub ngeom: ::core::ffi::c_int,
    #[doc = " buffer for geoms (ngeom)"]
    pub(crate) geoms: *mut mjvGeom,
    #[doc = " buffer for ordering geoms by distance to camera (ngeom)"]
    pub(crate) geomorder: *mut ::core::ffi::c_int,
    #[doc = " number of flexes"]
    pub nflex: ::core::ffi::c_int,
    #[doc = " address of flex edges (nflex)"]
    pub(crate) flexedgeadr: *mut ::core::ffi::c_int,
    #[doc = " number of edges in flex (nflex)"]
    pub(crate) flexedgenum: *mut ::core::ffi::c_int,
    #[doc = " address of flex vertices (nflex)"]
    pub(crate) flexvertadr: *mut ::core::ffi::c_int,
    #[doc = " number of vertices in flex (nflex)"]
    pub(crate) flexvertnum: *mut ::core::ffi::c_int,
    #[doc = " address of flex faces (nflex)"]
    pub(crate) flexfaceadr: *mut ::core::ffi::c_int,
    #[doc = " number of flex faces allocated (nflex)"]
    pub(crate) flexfacenum: *mut ::core::ffi::c_int,
    #[doc = " number of flex faces currently in use (nflex)"]
    pub(crate) flexfaceused: *mut ::core::ffi::c_int,
    #[doc = " flex edge data (2*nflexedge)"]
    pub(crate) flexedge: *mut ::core::ffi::c_int,
    #[doc = " flex vertices (3*nflexvert)"]
    pub(crate) flexvert: *mut f32,
    #[doc = " flex faces vertices (9*sum(flexfacenum))"]
    pub(crate) flexface: *mut f32,
    #[doc = " flex face normals (9*sum(flexfacenum))"]
    pub(crate) flexnormal: *mut f32,
    #[doc = " flex face texture coordinates (6*sum(flexfacenum))"]
    pub(crate) flextexcoord: *mut f32,
    #[doc = " copy of mjVIS_FLEXVERT mjvOption flag"]
    pub flexvertopt: mjtByte,
    #[doc = " copy of mjVIS_FLEXEDGE mjvOption flag"]
    pub flexedgeopt: mjtByte,
    #[doc = " copy of mjVIS_FLEXFACE mjvOption flag"]
    pub flexfaceopt: mjtByte,
    #[doc = " copy of mjVIS_FLEXSKIN mjvOption flag"]
    pub flexskinopt: mjtByte,
    #[doc = " number of skins"]
    pub nskin: ::core::ffi::c_int,
    #[doc = " number of faces in skin (nskin)"]
    pub(crate) skinfacenum: *mut ::core::ffi::c_int,
    #[doc = " address of skin vertices (nskin)"]
    pub(crate) skinvertadr: *mut ::core::ffi::c_int,
    #[doc = " number of vertices in skin (nskin)"]
    pub(crate) skinvertnum: *mut ::core::ffi::c_int,
    #[doc = " skin vertex data (3*nskinvert)"]
    pub(crate) skinvert: *mut f32,
    #[doc = " skin normal data (3*nskinvert)"]
    pub(crate) skinnormal: *mut f32,
    #[doc = " number of lights currently in buffer"]
    pub nlight: ::core::ffi::c_int,
    #[doc = " buffer for lights (nlight)"]
    pub lights: [mjvLight; 100usize],
    #[doc = " left and right camera"]
    pub camera: [mjvGLCamera; 2usize],
    #[doc = " enable model transformation"]
    pub enabletransform: mjtByte,
    #[doc = " model translation"]
    pub translate: [f32; 3usize],
    #[doc = " model quaternion rotation"]
    pub rotate: [f32; 4usize],
    #[doc = " model scaling"]
    pub scale: f32,
    #[doc = " stereoscopic rendering (mjtStereo)"]
    pub stereo: ::core::ffi::c_int,
    #[doc = " rendering flags (indexed by mjtRndFlag)"]
    pub flags: [mjtByte; 11usize],
    #[doc = " frame pixel width; 0: disable framing"]
    pub framewidth: ::core::ffi::c_int,
    #[doc = " frame color"]
    pub framergb: [f32; 3usize],
    #[doc = " 0: ok, 1: geoms exhausted, warning issued"]
    pub status: ::core::ffi::c_int,
}
#[doc = "---------------------------------- mjvFigure -----------------------------------------------------"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjvFigure {
    #[doc = " show legend"]
    pub flg_legend: ::core::ffi::c_int,
    #[doc = " show grid tick labels (x,y)"]
    pub flg_ticklabel: [::core::ffi::c_int; 2usize],
    #[doc = " automatically extend axis ranges to fit data"]
    pub flg_extend: ::core::ffi::c_int,
    #[doc = " isolated line segments (i.e. GL_LINES)"]
    pub flg_barplot: ::core::ffi::c_int,
    #[doc = " vertical selection line"]
    pub flg_selection: ::core::ffi::c_int,
    #[doc = " symmetric y-axis"]
    pub flg_symmetric: ::core::ffi::c_int,
    #[doc = " line width"]
    pub linewidth: f32,
    #[doc = " grid line width"]
    pub gridwidth: f32,
    #[doc = " number of grid points in (x,y)"]
    pub gridsize: [::core::ffi::c_int; 2usize],
    #[doc = " grid line rgb"]
    pub gridrgb: [f32; 3usize],
    #[doc = " figure color and alpha"]
    pub figurergba: [f32; 4usize],
    #[doc = " pane color and alpha"]
    pub panergba: [f32; 4usize],
    #[doc = " legend color and alpha"]
    pub legendrgba: [f32; 4usize],
    #[doc = " text color"]
    pub textrgb: [f32; 3usize],
    #[doc = " line colors"]
    pub linergb: [[f32; 3usize]; 100usize],
    #[doc = " axis ranges; (min>=max) automatic"]
    pub range: [[f32; 2usize]; 2usize],
    #[doc = " x-tick label format for sprintf"]
    pub(crate) xformat: [::core::ffi::c_char; 20usize],
    #[doc = " y-tick label format for sprintf"]
    pub(crate) yformat: [::core::ffi::c_char; 20usize],
    #[doc = " string used to determine min y-tick width"]
    pub(crate) minwidth: [::core::ffi::c_char; 20usize],
    #[doc = " figure title; subplots separated with 2+ spaces"]
    pub(crate) title: [::core::ffi::c_char; 1000usize],
    #[doc = " x-axis label"]
    pub(crate) xlabel: [::core::ffi::c_char; 100usize],
    #[doc = " line names for legend"]
    pub(crate) linename: [[::core::ffi::c_char; 100usize]; 100usize],
    #[doc = " number of lines to offset legend"]
    pub legendoffset: ::core::ffi::c_int,
    #[doc = " selected subplot (for title rendering)"]
    pub subplot: ::core::ffi::c_int,
    #[doc = " if point is in legend rect, highlight line"]
    pub highlight: [::core::ffi::c_int; 2usize],
    #[doc = " if id>=0 and no point, highlight id"]
    pub(crate) highlightid: ::core::ffi::c_int,
    #[doc = " selection line x-value"]
    pub selection: f32,
    #[doc = " number of points in line; (0) disable"]
    pub linepnt: [::core::ffi::c_int; 100usize],
    #[doc = " line data (x,y)"]
    pub linedata: [[f32; 2002usize]; 100usize],
    #[doc = " range of x-axis in pixels"]
    pub xaxispixel: [::core::ffi::c_int; 2usize],
    #[doc = " range of y-axis in pixels"]
    pub yaxispixel: [::core::ffi::c_int; 2usize],
    #[doc = " range of x-axis in data units"]
    pub xaxisdata: [f32; 2usize],
    #[doc = " range of y-axis in data units"]
    pub yaxisdata: [f32; 2usize],
}
#[doc = "---------------------------------- Resource Provider ---------------------------------------------"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjResource {
    #[doc = " name of resource (filename, etc)"]
    pub(crate) name: *mut ::core::ffi::c_char,
    #[doc = " opaque data pointer"]
    pub data: *mut ::core::ffi::c_void,
    #[doc = " pointer to the VFS"]
    pub(crate) vfs: *mut mjVFS,
    #[doc = " timestamp of the resource"]
    pub(crate) timestamp: [::core::ffi::c_char; 512usize],
    #[doc = " pointer to the provider"]
    pub(crate) provider: *const mjpResourceProvider,
}
#[doc = " callback for opening a resource, returns zero on failure.\n Note: If opening fails, the close callback will not be called. Therefore, the\n open callback is responsible for cleaning up any allocated memory before\n returning 0."]
pub(crate) type mjfOpenResource =
    ::core::option::Option<unsafe extern "C" fn(resource: *mut mjResource) -> ::core::ffi::c_int>;
#[doc = " callback for reading a resource\n return number of bytes stored in buffer, return -1 if error"]
pub(crate) type mjfReadResource = ::core::option::Option<
    unsafe extern "C" fn(
        resource: *mut mjResource,
        buffer: *mut *const ::core::ffi::c_void,
    ) -> ::core::ffi::c_int,
>;
#[doc = " callback for closing a resource (responsible for freeing any allocated memory)"]
pub(crate) type mjfCloseResource =
    ::core::option::Option<unsafe extern "C" fn(resource: *mut mjResource)>;
#[doc = " callback for mounting a resource (provider), returns zero on failure"]
pub(crate) type mjfMountResource =
    ::core::option::Option<unsafe extern "C" fn(resource: *mut mjResource) -> ::core::ffi::c_int>;
#[doc = " callback for unmounting a resource (provider), returns zero on failure"]
pub(crate) type mjfUnmountResource =
    ::core::option::Option<unsafe extern "C" fn(resource: *mut mjResource) -> ::core::ffi::c_int>;
#[doc = " callback for checking if the current resource was modified from the time\n specified by the timestamp\n returns 0 if the resource's timestamp matches the provided timestamp\n returns > 0 if the resource is younger than the given timestamp\n returns < 0 if the resource is older than the given timestamp"]
pub(crate) type mjfResourceModified = ::core::option::Option<
    unsafe extern "C" fn(
        resource: *const mjResource,
        timestamp: *const ::core::ffi::c_char,
    ) -> ::core::ffi::c_int,
>;
#[doc = " callback for writing bytes to a resource\n return number of bytes written, return -1 if error"]
pub(crate) type mjfWriteResource = ::core::option::Option<
    unsafe extern "C" fn(
        resource: *mut mjResource,
        buffer: *const ::core::ffi::c_void,
        nbytes: mjtSize,
    ) -> mjtSize,
>;
#[doc = " struct describing a single resource provider"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjpResourceProvider {
    #[doc = " prefix for match against a resource name"]
    pub(crate) prefix: *const ::core::ffi::c_char,
    #[doc = " opening callback"]
    pub open: mjfOpenResource,
    #[doc = " reading callback"]
    pub read: mjfReadResource,
    #[doc = " closing callback"]
    pub close: mjfCloseResource,
    #[doc = " mounting callback (optional)"]
    pub mount: mjfMountResource,
    #[doc = " unmounting callback (optional)"]
    pub unmount: mjfUnmountResource,
    #[doc = " resource modified callback (optional)"]
    pub modified: mjfResourceModified,
    #[doc = " writing callback (optional)"]
    pub write: mjfWriteResource,
    #[doc = " opaque data pointer (resource invariant)"]
    pub data: *mut ::core::ffi::c_void,
}
#[doc = " function pointer types\n return an mjSpec representing the decoded resource."]
pub(crate) type mjfDecode = ::core::option::Option<
    unsafe extern "C" fn(resource: *mut mjResource, vfs: *const mjVFS) -> *mut mjSpec,
>;
#[doc = " return true if the given resource can be decoded."]
pub(crate) type mjfCanDecode =
    ::core::option::Option<unsafe extern "C" fn(resource: *const mjResource) -> ::core::ffi::c_int>;
#[doc = " the struct defining the decoder plugin's interface"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjpDecoder {
    pub(crate) content_type: *const ::core::ffi::c_char,
    pub(crate) extension: *const ::core::ffi::c_char,
    #[doc = " quickly check if this decoder can handle the resource"]
    pub can_decode: mjfCanDecode,
    #[doc = " main decoding function"]
    pub decode: mjfDecode,
}
#[doc = "---------------------------------- Encoder -------------------------------------------------------"]
pub(crate) type mjfEncode = ::core::option::Option<
    unsafe extern "C" fn(
        s: *const mjSpec,
        m: *const mjModel,
        vfs: *const mjVFS,
        resource: *mut mjResource,
    ) -> mjtSize,
>;
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjpEncoder {
    pub(crate) content_type: *const ::core::ffi::c_char,
    pub(crate) extension: *const ::core::ffi::c_char,
    #[doc = "  Function to encode an mjSpec and mjModel to a mjResource."]
    pub encode: mjfEncode,
    #[doc = " Function to close/free the resource."]
    pub close_resource: mjfCloseResource,
}
impl mjtPluginCapabilityBit {
    #[doc = " actuator forces"]
    pub const ACTUATOR: mjtPluginCapabilityBit = mjtPluginCapabilityBit(1);
    #[doc = " sensor measurements"]
    pub const SENSOR: mjtPluginCapabilityBit = mjtPluginCapabilityBit(2);
    #[doc = " passive forces"]
    pub const PASSIVE: mjtPluginCapabilityBit = mjtPluginCapabilityBit(4);
    #[doc = " signed distance fields"]
    pub const SDF: mjtPluginCapabilityBit = mjtPluginCapabilityBit(8);
}
impl ::core::ops::BitOr<mjtPluginCapabilityBit> for mjtPluginCapabilityBit {
    type Output = Self;
    #[inline]
    fn bitor(self, other: Self) -> Self {
        mjtPluginCapabilityBit(self.0 | other.0)
    }
}
impl ::core::ops::BitOrAssign for mjtPluginCapabilityBit {
    #[inline]
    fn bitor_assign(&mut self, rhs: mjtPluginCapabilityBit) {
        self.0 |= rhs.0;
    }
}
impl ::core::ops::BitAnd<mjtPluginCapabilityBit> for mjtPluginCapabilityBit {
    type Output = Self;
    #[inline]
    fn bitand(self, other: Self) -> Self {
        mjtPluginCapabilityBit(self.0 & other.0)
    }
}
impl ::core::ops::BitAndAssign for mjtPluginCapabilityBit {
    #[inline]
    fn bitand_assign(&mut self, rhs: mjtPluginCapabilityBit) {
        self.0 &= rhs.0;
    }
}
#[repr(transparent)]
#[doc = "---------------------------------- Plugins -------------------------------------------------------"]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtPluginCapabilityBit(pub(crate) ::core::ffi::c_uint);
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjpPlugin {
    #[doc = " globally unique name identifying the plugin"]
    pub(crate) name: *const ::core::ffi::c_char,
    #[doc = " number of configuration attributes"]
    pub nattribute: ::core::ffi::c_int,
    #[doc = " name of configuration attributes"]
    pub(crate) attributes: *const *const ::core::ffi::c_char,
    #[doc = " plugin capabilities: bitfield of mjtPluginCapabilityBit"]
    pub capabilityflags: ::core::ffi::c_int,
    #[doc = " sensor computation stage (mjtStage)"]
    pub needstage: ::core::ffi::c_int,
    #[doc = " number of mjtNums needed to store the state of a plugin instance (required)"]
    pub(crate) nstate: ::core::option::Option<
        unsafe extern "C" fn(m: *const mjModel, instance: ::core::ffi::c_int) -> ::core::ffi::c_int,
    >,
    #[doc = " dimension of the specified sensor's output (required only for sensor plugins)"]
    pub(crate) nsensordata: ::core::option::Option<
        unsafe extern "C" fn(
            m: *const mjModel,
            instance: ::core::ffi::c_int,
            sensor_id: ::core::ffi::c_int,
        ) -> ::core::ffi::c_int,
    >,
    #[doc = " called when a new mjData is being created (required), returns 0 on success or -1 on failure"]
    pub(crate) init: ::core::option::Option<
        unsafe extern "C" fn(
            m: *const mjModel,
            d: *mut mjData,
            instance: ::core::ffi::c_int,
        ) -> ::core::ffi::c_int,
    >,
    #[doc = " called when an mjData is being freed (optional)"]
    pub(crate) destroy:
        ::core::option::Option<unsafe extern "C" fn(d: *mut mjData, instance: ::core::ffi::c_int)>,
    #[doc = " called when an mjData is being copied (optional)"]
    pub(crate) copy: ::core::option::Option<
        unsafe extern "C" fn(
            dest: *mut mjData,
            m: *const mjModel,
            src: *const mjData,
            instance: ::core::ffi::c_int,
        ),
    >,
    #[doc = " called when an mjData is being reset (required)"]
    pub(crate) reset: ::core::option::Option<
        unsafe extern "C" fn(
            m: *const mjModel,
            plugin_state: *mut mjtNum,
            plugin_data: *mut ::core::ffi::c_void,
            instance: ::core::ffi::c_int,
        ),
    >,
    #[doc = " called when the plugin needs to update its outputs (required)"]
    pub(crate) compute: ::core::option::Option<
        unsafe extern "C" fn(
            m: *const mjModel,
            d: *mut mjData,
            instance: ::core::ffi::c_int,
            capability_bit: ::core::ffi::c_int,
        ),
    >,
    #[doc = " called when time integration occurs (optional)"]
    pub(crate) advance: ::core::option::Option<
        unsafe extern "C" fn(m: *const mjModel, d: *mut mjData, instance: ::core::ffi::c_int),
    >,
    #[doc = " called by mjv_updateScene (optional)"]
    pub(crate) visualize: ::core::option::Option<
        unsafe extern "C" fn(
            m: *const mjModel,
            d: *mut mjData,
            opt: *const mjvOption,
            scn: *mut mjvScene,
            instance: ::core::ffi::c_int,
        ),
    >,
    #[doc = " updates the actuator plugin's entries in act_dot\n called after native act_dot is computed and before the compute callback"]
    pub(crate) actuator_act_dot: ::core::option::Option<
        unsafe extern "C" fn(m: *const mjModel, d: *mut mjData, instance: ::core::ffi::c_int),
    >,
    #[doc = " signed distance from the surface"]
    pub(crate) sdf_distance: ::core::option::Option<
        unsafe extern "C" fn(
            point: *const [mjtNum; 3usize],
            d: *const mjData,
            instance: ::core::ffi::c_int,
        ) -> mjtNum,
    >,
    #[doc = " gradient of distance with respect to local coordinates"]
    pub(crate) sdf_gradient: ::core::option::Option<
        unsafe extern "C" fn(
            gradient: *mut [mjtNum; 3usize],
            point: *const [mjtNum; 3usize],
            d: *const mjData,
            instance: ::core::ffi::c_int,
        ),
    >,
    #[doc = " called during compilation for marching cubes"]
    pub(crate) sdf_staticdistance: ::core::option::Option<
        unsafe extern "C" fn(point: *const [mjtNum; 3usize], attributes: *const mjtNum) -> mjtNum,
    >,
    #[doc = " convert attributes and provide defaults if not present"]
    pub(crate) sdf_attribute: ::core::option::Option<
        unsafe extern "C" fn(
            attribute: *mut [mjtNum; 0usize],
            name: *mut [*const ::core::ffi::c_char; 0usize],
            value: *mut [*const ::core::ffi::c_char; 0usize],
        ),
    >,
    #[doc = " bounding box of implicit surface"]
    pub(crate) sdf_aabb: ::core::option::Option<
        unsafe extern "C" fn(aabb: *mut [mjtNum; 6usize], attributes: *const mjtNum),
    >,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjSDF {
    pub(crate) plugin: *mut *const mjpPlugin,
    pub(crate) id: *mut ::core::ffi::c_int,
    pub type_: mjtSDFType,
    pub(crate) relpos: *mut mjtNum,
    pub(crate) relmat: *mut mjtNum,
    pub(crate) geomtype: *mut mjtGeom,
}
#[doc = " function pointer type for mj_loadAllPluginLibraries callback"]
pub(crate) type mjfPluginLibraryLoadCallback = ::core::option::Option<
    unsafe extern "C" fn(
        filename: *const ::core::ffi::c_char,
        first: ::core::ffi::c_int,
        count: ::core::ffi::c_int,
    ),
>;
impl mjtGridPos {
    #[doc = " top left"]
    pub const TOPLEFT: mjtGridPos = mjtGridPos(0);
    #[doc = " top right"]
    pub const TOPRIGHT: mjtGridPos = mjtGridPos(1);
    #[doc = " bottom left"]
    pub const BOTTOMLEFT: mjtGridPos = mjtGridPos(2);
    #[doc = " bottom right"]
    pub const BOTTOMRIGHT: mjtGridPos = mjtGridPos(3);
    #[doc = " top center"]
    pub const TOP: mjtGridPos = mjtGridPos(4);
    #[doc = " bottom center"]
    pub const BOTTOM: mjtGridPos = mjtGridPos(5);
    #[doc = " left center"]
    pub const LEFT: mjtGridPos = mjtGridPos(6);
    #[doc = " right center"]
    pub const RIGHT: mjtGridPos = mjtGridPos(7);
}
#[repr(transparent)]
#[doc = "---------------------------------- primitive types (mjt) -----------------------------------------"]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtGridPos(pub(crate) ::core::ffi::c_uint);
impl mjtFramebuffer {
    #[doc = " default/window buffer"]
    pub const WINDOW: mjtFramebuffer = mjtFramebuffer(0);
    #[doc = " offscreen buffer"]
    pub const OFFSCREEN: mjtFramebuffer = mjtFramebuffer(1);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtFramebuffer(pub(crate) ::core::ffi::c_uint);
impl mjtDepthMap {
    #[doc = " standard depth map; 0: znear, 1: zfar"]
    pub const ZERONEAR: mjtDepthMap = mjtDepthMap(0);
    #[doc = " reversed depth map; 1: znear, 0: zfar"]
    pub const ZEROFAR: mjtDepthMap = mjtDepthMap(1);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtDepthMap(pub(crate) ::core::ffi::c_uint);
impl mjtFontScale {
    #[doc = " 50% scale, suitable for low-res rendering"]
    pub const X50: mjtFontScale = mjtFontScale(50);
    #[doc = " normal scale, suitable in the absence of DPI scaling"]
    pub const X100: mjtFontScale = mjtFontScale(100);
    #[doc = " 150% scale"]
    pub const X150: mjtFontScale = mjtFontScale(150);
    #[doc = " 200% scale"]
    pub const X200: mjtFontScale = mjtFontScale(200);
    #[doc = " 250% scale"]
    pub const X250: mjtFontScale = mjtFontScale(250);
    #[doc = " 300% scale"]
    pub const X300: mjtFontScale = mjtFontScale(300);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtFontScale(pub(crate) ::core::ffi::c_uint);
impl mjtFont {
    #[doc = " normal font"]
    pub const NORMAL: mjtFont = mjtFont(0);
    #[doc = " normal font with shadow (for higher contrast)"]
    pub const SHADOW: mjtFont = mjtFont(1);
    #[doc = " big font (for user alerts)"]
    pub const BIG: mjtFont = mjtFont(2);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtFont(pub(crate) ::core::ffi::c_uint);
impl mjrPixelFormat {
    #[doc = " unknown/unspecified"]
    pub const UNKNOWN: mjrPixelFormat = mjrPixelFormat(0);
    #[doc = " 1 channel, 8 bit"]
    pub const R8: mjrPixelFormat = mjrPixelFormat(1);
    #[doc = " 3 channels, 8 bits per channel"]
    pub const RGB8: mjrPixelFormat = mjrPixelFormat(2);
    #[doc = " 4 channels, 8 bits per channel"]
    pub const RGBA8: mjrPixelFormat = mjrPixelFormat(3);
    #[doc = " 1 channel, 32 bit float"]
    pub const R32F: mjrPixelFormat = mjrPixelFormat(4);
    #[doc = " 1 channel, 32 bit float, for depth buffers"]
    pub const DEPTH32F: mjrPixelFormat = mjrPixelFormat(5);
    #[doc = " ktx compressed data"]
    pub const KTX: mjrPixelFormat = mjrPixelFormat(6);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjrPixelFormat(pub(crate) ::core::ffi::c_uint);
impl mjrVertexAttributeUsage {
    #[doc = " vertex position"]
    pub const POSITION: mjrVertexAttributeUsage = mjrVertexAttributeUsage(0);
    #[doc = " vertex normal"]
    pub const NORMAL: mjrVertexAttributeUsage = mjrVertexAttributeUsage(1);
    #[doc = " vertex tangents"]
    pub const TANGENTS: mjrVertexAttributeUsage = mjrVertexAttributeUsage(2);
    #[doc = " vertex texture coordinates"]
    pub const UV: mjrVertexAttributeUsage = mjrVertexAttributeUsage(3);
    #[doc = " vertex color"]
    pub const COLOR: mjrVertexAttributeUsage = mjrVertexAttributeUsage(4);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjrVertexAttributeUsage(pub(crate) ::core::ffi::c_uint);
impl mjrVertexAttributeType {
    #[doc = " 2D 32-bit float vector"]
    pub const FLOAT2: mjrVertexAttributeType = mjrVertexAttributeType(0);
    #[doc = " 3D 32-bit float vector"]
    pub const FLOAT3: mjrVertexAttributeType = mjrVertexAttributeType(1);
    #[doc = " 4D 32-bit float vector"]
    pub const FLOAT4: mjrVertexAttributeType = mjrVertexAttributeType(2);
    #[doc = " 4D unsigned 8-bit byte vector"]
    pub const UBYTE4: mjrVertexAttributeType = mjrVertexAttributeType(3);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjrVertexAttributeType(pub(crate) ::core::ffi::c_uint);
impl mjrIndexType {
    #[doc = " 16-bit unsigned integer"]
    pub const U16: mjrIndexType = mjrIndexType(0);
    #[doc = " 32-bit unsigned integer"]
    pub const U32: mjrIndexType = mjrIndexType(1);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjrIndexType(pub(crate) ::core::ffi::c_uint);
impl mjrMeshPrimitiveType {
    #[doc = " triangles"]
    pub const TRIANGLES: mjrMeshPrimitiveType = mjrMeshPrimitiveType(0);
    #[doc = " lines"]
    pub const LINES: mjrMeshPrimitiveType = mjrMeshPrimitiveType(1);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjrMeshPrimitiveType(pub(crate) ::core::ffi::c_uint);
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjrRect {
    #[doc = " left (usually 0)"]
    pub left: ::core::ffi::c_int,
    #[doc = " bottom (usually 0)"]
    pub bottom: ::core::ffi::c_int,
    #[doc = " width (usually buffer width)"]
    pub width: ::core::ffi::c_int,
    #[doc = " height (usually buffer height)"]
    pub height: ::core::ffi::c_int,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjrRendererInfo {
    #[doc = " renderer family: classic, filament, noop"]
    pub(crate) renderer: *const ::core::ffi::c_char,
    #[doc = " graphics backend: opengl, vulkan; empty if uninitialized"]
    pub(crate) backend: *const ::core::ffi::c_char,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjrVertexAttribute {
    #[doc = " position, normal, etc [mjrVertexAttributeUsage]"]
    pub usage: ::core::ffi::c_int,
    #[doc = " float3, ubyte4, etc. [mjrVertexAttributeType]"]
    pub type_: ::core::ffi::c_int,
}
pub use self::mjtColorSpace as mjrColorSpace;
pub use self::mjtLightType as mjrLightType;
#[doc = " alias non-rendering types into mjr namespace"]
pub use self::mjtTexture as mjrSamplerType;
#[doc = "---------------------------------- mjvGLCamera ---------------------------------------------------"]
pub type mjrCamera = mjvGLCamera;
#[doc = "---------------------------------- mjrContext ----------------------------------------------------"]
#[repr(C)]
#[derive(Debug)]
pub struct mjrContext {
    #[doc = " line width for wireframe rendering"]
    pub lineWidth: f32,
    #[doc = " clipping radius for directional lights"]
    pub shadowClip: f32,
    #[doc = " fraction of light cutoff for spot lights"]
    pub shadowScale: f32,
    #[doc = " fog start = stat.extent * vis.map.fogstart"]
    pub fogStart: f32,
    #[doc = " fog end = stat.extent * vis.map.fogend"]
    pub fogEnd: f32,
    #[doc = " fog rgba"]
    pub fogRGBA: [f32; 4usize],
    #[doc = " size of shadow map texture"]
    pub shadowSize: ::core::ffi::c_int,
    #[doc = " width of offscreen buffer"]
    pub offWidth: ::core::ffi::c_int,
    #[doc = " height of offscreen buffer"]
    pub offHeight: ::core::ffi::c_int,
    #[doc = " number of offscreen buffer multisamples"]
    pub offSamples: ::core::ffi::c_int,
    #[doc = " font scale"]
    pub fontScale: ::core::ffi::c_int,
    #[doc = " auxiliary buffer width"]
    pub auxWidth: [::core::ffi::c_int; 10usize],
    #[doc = " auxiliary buffer height"]
    pub auxHeight: [::core::ffi::c_int; 10usize],
    #[doc = " auxiliary buffer multisamples"]
    pub auxSamples: [::core::ffi::c_int; 10usize],
    #[doc = " offscreen framebuffer object"]
    pub offFBO: ::core::ffi::c_uint,
    #[doc = " offscreen framebuffer for resolving multisamples"]
    pub offFBO_r: ::core::ffi::c_uint,
    #[doc = " offscreen color buffer"]
    pub offColor: ::core::ffi::c_uint,
    #[doc = " offscreen color buffer for resolving multisamples"]
    pub offColor_r: ::core::ffi::c_uint,
    #[doc = " offscreen depth and stencil buffer"]
    pub offDepthStencil: ::core::ffi::c_uint,
    #[doc = " offscreen depth and stencil buffer for multisamples"]
    pub offDepthStencil_r: ::core::ffi::c_uint,
    #[doc = " shadow map framebuffer object"]
    pub shadowFBO: ::core::ffi::c_uint,
    #[doc = " shadow map texture"]
    pub shadowTex: ::core::ffi::c_uint,
    #[doc = " auxiliary framebuffer object"]
    pub auxFBO: [::core::ffi::c_uint; 10usize],
    #[doc = " auxiliary framebuffer object for resolving"]
    pub auxFBO_r: [::core::ffi::c_uint; 10usize],
    #[doc = " auxiliary color buffer"]
    pub auxColor: [::core::ffi::c_uint; 10usize],
    #[doc = " auxiliary color buffer for resolving"]
    pub auxColor_r: [::core::ffi::c_uint; 10usize],
    #[doc = " material texture ids (-1: no texture)"]
    pub(crate) mat_texid: [::core::ffi::c_int; 10000usize],
    #[doc = " uniform cube mapping"]
    pub mat_texuniform: [::core::ffi::c_int; 1000usize],
    #[doc = " texture repetition for 2d mapping"]
    pub mat_texrepeat: [f32; 2000usize],
    #[doc = " number of allocated textures"]
    pub ntexture: ::core::ffi::c_int,
    #[doc = " type of texture (mjtTexture) (ntexture)"]
    pub textureType: [::core::ffi::c_int; 1000usize],
    #[doc = " texture names"]
    pub texture: [::core::ffi::c_uint; 1000usize],
    #[doc = " all planes from model"]
    pub basePlane: ::core::ffi::c_uint,
    #[doc = " all meshes from model"]
    pub baseMesh: ::core::ffi::c_uint,
    #[doc = " all height fields from model"]
    pub baseHField: ::core::ffi::c_uint,
    #[doc = " all builtin geoms, with quality from model"]
    pub baseBuiltin: ::core::ffi::c_uint,
    #[doc = " normal font"]
    pub baseFontNormal: ::core::ffi::c_uint,
    #[doc = " shadow font"]
    pub baseFontShadow: ::core::ffi::c_uint,
    #[doc = " big font"]
    pub baseFontBig: ::core::ffi::c_uint,
    #[doc = " all planes from model"]
    pub rangePlane: ::core::ffi::c_int,
    #[doc = " all meshes from model"]
    pub rangeMesh: ::core::ffi::c_int,
    #[doc = " all hfields from model"]
    pub rangeHField: ::core::ffi::c_int,
    #[doc = " all builtin geoms, with quality from model"]
    pub rangeBuiltin: ::core::ffi::c_int,
    #[doc = " all characters in font"]
    pub rangeFont: ::core::ffi::c_int,
    #[doc = " number of skins"]
    pub nskin: ::core::ffi::c_int,
    #[doc = " skin vertex position VBOs (nskin)"]
    pub(crate) skinvertVBO: *mut ::core::ffi::c_uint,
    #[doc = " skin vertex normal VBOs (nskin)"]
    pub(crate) skinnormalVBO: *mut ::core::ffi::c_uint,
    #[doc = " skin vertex texture coordinate VBOs (nskin)"]
    pub(crate) skintexcoordVBO: *mut ::core::ffi::c_uint,
    #[doc = " skin face index VBOs (nskin)"]
    pub(crate) skinfaceVBO: *mut ::core::ffi::c_uint,
    #[doc = " character widths: normal and shadow"]
    pub charWidth: [::core::ffi::c_int; 127usize],
    #[doc = " character widths: big"]
    pub charWidthBig: [::core::ffi::c_int; 127usize],
    #[doc = " character heights: normal and shadow"]
    pub charHeight: ::core::ffi::c_int,
    #[doc = " character heights: big"]
    pub charHeightBig: ::core::ffi::c_int,
    #[doc = " is OpenGL initialized"]
    pub glInitialized: ::core::ffi::c_int,
    #[doc = " is default/window framebuffer available"]
    pub windowAvailable: ::core::ffi::c_int,
    #[doc = " number of samples for default/window framebuffer"]
    pub windowSamples: ::core::ffi::c_int,
    #[doc = " is stereo available for default/window framebuffer"]
    pub windowStereo: ::core::ffi::c_int,
    #[doc = " is default/window framebuffer double buffered"]
    pub windowDoublebuffer: ::core::ffi::c_int,
    #[doc = " currently active framebuffer: mjFB_WINDOW or mjFB_OFFSCREEN"]
    pub currentBuffer: ::core::ffi::c_int,
    #[doc = " default color pixel format for mjr_readPixels"]
    pub readPixelFormat: ::core::ffi::c_int,
    #[doc = " depth mapping: mjDEPTH_ZERONEAR or mjDEPTH_ZEROFAR"]
    pub readDepthMap: ::core::ffi::c_int,
}
impl mjtButton {
    #[doc = " no button"]
    pub const NONE: mjtButton = mjtButton(0);
    #[doc = " left button"]
    pub const LEFT: mjtButton = mjtButton(1);
    #[doc = " right button"]
    pub const RIGHT: mjtButton = mjtButton(2);
    #[doc = " middle button"]
    pub const MIDDLE: mjtButton = mjtButton(3);
}
#[repr(transparent)]
#[doc = "---------------------------------- primitive types (mjt) -----------------------------------------"]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtButton(pub(crate) ::core::ffi::c_uint);
impl mjtEvent {
    #[doc = " no event"]
    pub const NONE: mjtEvent = mjtEvent(0);
    #[doc = " mouse move"]
    pub const MOVE: mjtEvent = mjtEvent(1);
    #[doc = " mouse button press"]
    pub const PRESS: mjtEvent = mjtEvent(2);
    #[doc = " mouse button release"]
    pub const RELEASE: mjtEvent = mjtEvent(3);
    #[doc = " scroll"]
    pub const SCROLL: mjtEvent = mjtEvent(4);
    #[doc = " key press"]
    pub const KEY: mjtEvent = mjtEvent(5);
    #[doc = " resize"]
    pub const RESIZE: mjtEvent = mjtEvent(6);
    #[doc = " redraw"]
    pub const REDRAW: mjtEvent = mjtEvent(7);
    #[doc = " files drop"]
    pub const FILESDROP: mjtEvent = mjtEvent(8);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtEvent(pub(crate) ::core::ffi::c_uint);
pub const mjNITEM: usize = mjtItem::mjNITEM.0 as usize;
impl mjtItem {
    #[doc = " end of definition list (not an item)"]
    pub const END: mjtItem = mjtItem(-2);
    #[doc = " section (not an item)"]
    pub const SECTION: mjtItem = mjtItem(-1);
    #[doc = " separator"]
    pub const SEPARATOR: mjtItem = mjtItem(0);
    #[doc = " static text"]
    pub const STATIC: mjtItem = mjtItem(1);
    #[doc = " button"]
    pub const BUTTON: mjtItem = mjtItem(2);
    #[doc = " check box, int value"]
    pub const CHECKINT: mjtItem = mjtItem(3);
    #[doc = " check box, mjtByte value"]
    pub const CHECKBYTE: mjtItem = mjtItem(4);
    #[doc = " radio group"]
    pub const RADIO: mjtItem = mjtItem(5);
    #[doc = " radio group, single line"]
    pub const RADIOLINE: mjtItem = mjtItem(6);
    #[doc = " selection box"]
    pub const SELECT: mjtItem = mjtItem(7);
    #[doc = " slider, int value"]
    pub const SLIDERINT: mjtItem = mjtItem(8);
    #[doc = " slider, mjtNum value"]
    pub const SLIDERNUM: mjtItem = mjtItem(9);
    #[doc = " editable array, int values"]
    pub const EDITINT: mjtItem = mjtItem(10);
    #[doc = " editable array, mjtNum values"]
    pub const EDITNUM: mjtItem = mjtItem(11);
    #[doc = " editable array, float values"]
    pub const EDITFLOAT: mjtItem = mjtItem(12);
    #[doc = " editable text"]
    pub const EDITTXT: mjtItem = mjtItem(13);
    #[doc = " number of item types"]
    const mjNITEM: mjtItem = mjtItem(14);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtItem(pub(crate) ::core::ffi::c_int);
impl mjtSection {
    #[doc = " closed state (regular section)"]
    pub const CLOSED: mjtSection = mjtSection(0);
    #[doc = " open state (regular section)"]
    pub const OPEN: mjtSection = mjtSection(1);
    #[doc = " fixed section: always open, no title"]
    pub const FIXED: mjtSection = mjtSection(2);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtSection(pub(crate) ::core::ffi::c_uint);
#[doc = " predicate function: set enable/disable based on item category"]
pub(crate) type mjfItemEnable = ::core::option::Option<
    unsafe extern "C" fn(
        category: ::core::ffi::c_int,
        data: *mut ::core::ffi::c_void,
    ) -> ::core::ffi::c_int,
>;
#[doc = "---------------------------------- mjuiState -----------------------------------------------------"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjuiState {
    #[doc = " number of rectangles used"]
    pub nrect: ::core::ffi::c_int,
    #[doc = " rectangles (index 0: entire window)"]
    pub rect: [mjrRect; 25usize],
    #[doc = " pointer to user data (for callbacks)"]
    pub userdata: *mut ::core::ffi::c_void,
    #[doc = " (type mjtEvent)"]
    pub type_: ::core::ffi::c_int,
    #[doc = " is left button down"]
    pub left: ::core::ffi::c_int,
    #[doc = " is right button down"]
    pub right: ::core::ffi::c_int,
    #[doc = " is middle button down"]
    pub middle: ::core::ffi::c_int,
    #[doc = " is last press a double click"]
    pub doubleclick: ::core::ffi::c_int,
    #[doc = " which button was pressed (mjtButton)"]
    pub button: ::core::ffi::c_int,
    #[doc = " time of last button press"]
    pub buttontime: f64,
    #[doc = " x position"]
    pub x: f64,
    #[doc = " y position"]
    pub y: f64,
    #[doc = " x displacement"]
    pub dx: f64,
    #[doc = " y displacement"]
    pub dy: f64,
    #[doc = " x scroll"]
    pub sx: f64,
    #[doc = " y scroll"]
    pub sy: f64,
    #[doc = " is control down"]
    pub control: ::core::ffi::c_int,
    #[doc = " is shift down"]
    pub shift: ::core::ffi::c_int,
    #[doc = " is alt down"]
    pub alt: ::core::ffi::c_int,
    #[doc = " which key was pressed"]
    pub key: ::core::ffi::c_int,
    #[doc = " time of last key press"]
    pub keytime: f64,
    #[doc = " which rectangle contains mouse"]
    pub mouserect: ::core::ffi::c_int,
    #[doc = " which rectangle is dragged with mouse"]
    pub dragrect: ::core::ffi::c_int,
    #[doc = " which button started drag (mjtButton)"]
    pub dragbutton: ::core::ffi::c_int,
    #[doc = " number of files dropped"]
    pub dropcount: ::core::ffi::c_int,
    #[doc = " paths to files dropped"]
    pub(crate) droppaths: *mut *const ::core::ffi::c_char,
}
#[doc = "---------------------------------- mjuiThemeSpacing ----------------------------------------------"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjuiThemeSpacing {
    #[doc = " total width"]
    pub total: ::core::ffi::c_int,
    #[doc = " scrollbar width"]
    pub scroll: ::core::ffi::c_int,
    #[doc = " label width"]
    pub label: ::core::ffi::c_int,
    #[doc = " section gap"]
    pub section: ::core::ffi::c_int,
    #[doc = " corner radius for section"]
    pub cornersect: ::core::ffi::c_int,
    #[doc = " corner radius for separator"]
    pub cornersep: ::core::ffi::c_int,
    #[doc = " item side gap"]
    pub itemside: ::core::ffi::c_int,
    #[doc = " item middle gap"]
    pub(crate) itemmid: ::core::ffi::c_int,
    #[doc = " item vertical gap"]
    pub itemver: ::core::ffi::c_int,
    #[doc = " text horizontal gap"]
    pub texthor: ::core::ffi::c_int,
    #[doc = " text vertical gap"]
    pub textver: ::core::ffi::c_int,
    #[doc = " number of pixels to scroll"]
    pub linescroll: ::core::ffi::c_int,
    #[doc = " number of multisamples"]
    pub samples: ::core::ffi::c_int,
}
#[doc = "---------------------------------- mjuiThemeColor ------------------------------------------------"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjuiThemeColor {
    #[doc = " master background"]
    pub master: [f32; 3usize],
    #[doc = " scrollbar thumb"]
    pub thumb: [f32; 3usize],
    #[doc = " section title"]
    pub secttitle: [f32; 3usize],
    #[doc = " section title: bottom color"]
    pub secttitle2: [f32; 3usize],
    #[doc = " section title with unchecked box"]
    pub secttitleuncheck: [f32; 3usize],
    #[doc = " section title with unchecked box: bottom color"]
    pub secttitleuncheck2: [f32; 3usize],
    #[doc = " section title with checked box"]
    pub secttitlecheck: [f32; 3usize],
    #[doc = " section title with checked box: bottom color"]
    pub secttitlecheck2: [f32; 3usize],
    #[doc = " section font"]
    pub sectfont: [f32; 3usize],
    #[doc = " section symbol"]
    pub sectsymbol: [f32; 3usize],
    #[doc = " section pane"]
    pub sectpane: [f32; 3usize],
    #[doc = " separator title"]
    pub separator: [f32; 3usize],
    #[doc = " separator title: bottom color"]
    pub separator2: [f32; 3usize],
    #[doc = " shortcut background"]
    pub shortcut: [f32; 3usize],
    #[doc = " font active"]
    pub fontactive: [f32; 3usize],
    #[doc = " font inactive"]
    pub fontinactive: [f32; 3usize],
    #[doc = " decor inactive"]
    pub decorinactive: [f32; 3usize],
    #[doc = " inactive slider color 2"]
    pub decorinactive2: [f32; 3usize],
    #[doc = " button"]
    pub button: [f32; 3usize],
    #[doc = " check"]
    pub check: [f32; 3usize],
    #[doc = " radio"]
    pub radio: [f32; 3usize],
    #[doc = " select"]
    pub select: [f32; 3usize],
    #[doc = " select pane"]
    pub select2: [f32; 3usize],
    #[doc = " slider"]
    pub slider: [f32; 3usize],
    #[doc = " slider color 2"]
    pub slider2: [f32; 3usize],
    #[doc = " edit"]
    pub edit: [f32; 3usize],
    #[doc = " edit invalid"]
    pub edit2: [f32; 3usize],
    #[doc = " edit cursor"]
    pub cursor: [f32; 3usize],
}
#[doc = "---------------------------------- mjuiItem ------------------------------------------------------"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjuiItemSingle {
    #[doc = " 0: none, 1: control, 2: shift; 4: alt"]
    pub modifier: ::core::ffi::c_int,
    #[doc = " shortcut key; 0: undefined"]
    pub shortcut: ::core::ffi::c_int,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjuiItemMulti {
    #[doc = " number of elements in group"]
    pub nelem: ::core::ffi::c_int,
    #[doc = " element names"]
    pub(crate) name: [[::core::ffi::c_char; 40usize]; 35usize],
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjuiItemSlider {
    #[doc = " slider range"]
    pub range: [f64; 2usize],
    #[doc = " number of range divisions"]
    pub divisions: f64,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjuiItemEdit {
    #[doc = " number of elements in list"]
    pub nelem: ::core::ffi::c_int,
    #[doc = " element range (min>=max: ignore)"]
    pub range: [[f64; 2usize]; 7usize],
}
#[repr(C)]
#[derive(Copy, Clone)]
pub struct mjuiItem {
    #[doc = " type (mjtItem)"]
    pub type_: ::core::ffi::c_int,
    #[doc = " name"]
    pub(crate) name: [::core::ffi::c_char; 40usize],
    #[doc = " 0: disable, 1: enable, 2+: use predicate"]
    pub state: ::core::ffi::c_int,
    #[doc = " data pointer (type-specific)"]
    pub pdata: *mut ::core::ffi::c_void,
    #[doc = " id of section containing item"]
    pub(crate) sectionid: ::core::ffi::c_int,
    #[doc = " id of item within section"]
    pub(crate) itemid: ::core::ffi::c_int,
    #[doc = " user-supplied id (for event handling)"]
    pub(crate) userid: ::core::ffi::c_int,
    pub __bindgen_anon_1: mjuiItem_1,
    #[doc = " rectangle occupied by item"]
    pub rect: mjrRect,
    #[doc = " item skipped due to closed separator"]
    pub skip: ::core::ffi::c_int,
}
#[doc = " type-specific properties"]
#[repr(C)]
#[derive(Copy, Clone)]
pub(crate) union mjuiItem_1 {
    #[doc = " check and button"]
    pub single: mjuiItemSingle,
    #[doc = " static, radio and select"]
    pub multi: mjuiItemMulti,
    #[doc = " slider"]
    pub slider: mjuiItemSlider,
    #[doc = " edit"]
    pub edit: mjuiItemEdit,
}
#[doc = "---------------------------------- mjuiSection ---------------------------------------------------"]
#[repr(C)]
#[derive(Copy, Clone)]
pub struct mjuiSection {
    #[doc = " name"]
    pub(crate) name: [::core::ffi::c_char; 40usize],
    #[doc = " section state (mjtSection)"]
    pub state: ::core::ffi::c_int,
    #[doc = " 0: none, 1: control, 2: shift; 4: alt"]
    pub modifier: ::core::ffi::c_int,
    #[doc = " shortcut key; 0: undefined"]
    pub shortcut: ::core::ffi::c_int,
    #[doc = " 0: none, 1: unchecked, 2: checked"]
    pub checkbox: ::core::ffi::c_int,
    #[doc = " number of items in use"]
    pub nitem: ::core::ffi::c_int,
    #[doc = " preallocated array of items"]
    pub item: [mjuiItem; 200usize],
    #[doc = " rectangle occupied by title"]
    pub rtitle: mjrRect,
    #[doc = " rectangle occupied by content"]
    pub rcontent: mjrRect,
    #[doc = " last mouse click over this section"]
    pub lastclick: ::core::ffi::c_int,
}
#[doc = "---------------------------------- mjUI ----------------------------------------------------------"]
#[repr(C)]
#[derive(Copy, Clone)]
pub struct mjUI {
    #[doc = " UI theme spacing"]
    pub spacing: mjuiThemeSpacing,
    #[doc = " UI theme color"]
    pub color: mjuiThemeColor,
    #[doc = " callback to set item state programmatically"]
    pub predicate: mjfItemEnable,
    #[doc = " pointer to user data (passed to predicate)"]
    pub userdata: *mut ::core::ffi::c_void,
    #[doc = " index of this ui rectangle in mjuiState"]
    pub(crate) rectid: ::core::ffi::c_int,
    #[doc = " aux buffer index of this ui"]
    pub(crate) auxid: ::core::ffi::c_int,
    #[doc = " number of radio columns (0 defaults to 2)"]
    pub radiocol: ::core::ffi::c_int,
    #[doc = " width"]
    pub width: ::core::ffi::c_int,
    #[doc = " current height"]
    pub height: ::core::ffi::c_int,
    #[doc = " height when all sections open"]
    pub maxheight: ::core::ffi::c_int,
    #[doc = " scroll from top of UI"]
    pub scroll: ::core::ffi::c_int,
    #[doc = " 0: none, -1: scroll, otherwise 1+section"]
    pub mousesect: ::core::ffi::c_int,
    #[doc = " item within section"]
    pub mouseitem: ::core::ffi::c_int,
    #[doc = " help button down: print shortcuts"]
    pub mousehelp: ::core::ffi::c_int,
    #[doc = " number of mouse clicks over UI"]
    pub mouseclicks: ::core::ffi::c_int,
    #[doc = " 0: none, otherwise 1+section"]
    pub mousesectcheck: ::core::ffi::c_int,
    #[doc = " 0: none, otherwise 1+section"]
    pub editsect: ::core::ffi::c_int,
    #[doc = " item within section"]
    pub edititem: ::core::ffi::c_int,
    #[doc = " cursor position"]
    pub editcursor: ::core::ffi::c_int,
    #[doc = " horizontal scroll"]
    pub editscroll: ::core::ffi::c_int,
    #[doc = " current text"]
    pub(crate) edittext: [::core::ffi::c_char; 300usize],
    #[doc = " pointer to changed edit in last mjui_event"]
    pub(crate) editchanged: *mut mjuiItem,
    #[doc = " number of sections in use"]
    pub nsect: ::core::ffi::c_int,
    #[doc = " preallocated array of sections"]
    pub sect: [mjuiSection; 10usize],
}
#[doc = "---------------------------------- mjuiDef -------------------------------------------------------"]
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjuiDef {
    #[doc = " type (mjtItem); -1: section"]
    pub type_: ::core::ffi::c_int,
    #[doc = " name"]
    pub(crate) name: [::core::ffi::c_char; 40usize],
    #[doc = " state"]
    pub state: ::core::ffi::c_int,
    #[doc = " pointer to data"]
    pub pdata: *mut ::core::ffi::c_void,
    #[doc = " string with type-specific properties"]
    pub(crate) other: [::core::ffi::c_char; 300usize],
    #[doc = " int with type-specific properties"]
    pub otherint: ::core::ffi::c_int,
}
pub(crate) mod functions {
    use super::*;
    unsafe extern "C" {
        #[doc = " user memory handlers"]
        pub(crate) static mut mju_user_malloc:
            ::core::option::Option<unsafe extern "C" fn(arg1: usize) -> *mut ::core::ffi::c_void>;
        pub(crate) static mut mju_user_free:
            ::core::option::Option<unsafe extern "C" fn(arg1: *mut ::core::ffi::c_void)>;
        #[doc = " legacy error/warning handlers (deprecated: prefer mju_setLogHandler)"]
        pub(crate) static mut mju_user_error:
            ::core::option::Option<unsafe extern "C" fn(arg1: *const ::core::ffi::c_char)>;
        pub(crate) static mut mju_user_warning:
            ::core::option::Option<unsafe extern "C" fn(arg1: *const ::core::ffi::c_char)>;
        #[doc = " callbacks extending computation pipeline"]
        pub(crate) static mut mjcb_passive: mjfGeneric;
        pub(crate) static mut mjcb_control: mjfGeneric;
        pub(crate) static mut mjcb_contactfilter: mjfConFilt;
        pub(crate) static mut mjcb_sensor: mjfSensor;
        pub(crate) static mut mjcb_time: mjfTime;
        pub(crate) static mut mjcb_act_dyn: mjfAct;
        pub(crate) static mut mjcb_act_gain: mjfAct;
        pub(crate) static mut mjcb_act_bias: mjfAct;
        #[doc = " collision function table"]
        pub(crate) static mut mjCOLLISIONFUNC: [[mjfCollision; 9usize]; 9usize];
        #[doc = " string names"]
        pub(crate) static mut mjDISABLESTRING: [*const ::core::ffi::c_char; 20usize];
        pub(crate) static mut mjENABLESTRING: [*const ::core::ffi::c_char; 6usize];
        pub(crate) static mut mjTIMERSTRING: [*const ::core::ffi::c_char; 15usize];
        pub(crate) static mut mjLABELSTRING: [*const ::core::ffi::c_char; 17usize];
        pub(crate) static mut mjFRAMESTRING: [*const ::core::ffi::c_char; 8usize];
        pub(crate) static mut mjVISSTRING: [[*const ::core::ffi::c_char; 3usize]; 31usize];
        pub(crate) static mut mjRNDSTRING: [[*const ::core::ffi::c_char; 3usize]; 11usize];
        pub(crate) static mut mjTOPICSTRING: [*const ::core::ffi::c_char; 3usize];
        #[doc = " Initialize an empty VFS, mj_deleteVFS must be called to deallocate the VFS."]
        pub(crate) fn mj_defaultVFS(vfs: *mut mjVFS);
        #[doc = " Mount a ResourceProvider to handle file operations under the given path; return 0: success,\n 2: repeated name, -1: invalid resource provider."]
        pub(crate) fn mj_mountVFS(
            vfs: *mut mjVFS,
            filepath: *const ::core::ffi::c_char,
            provider: *const mjpResourceProvider,
        ) -> ::core::ffi::c_int;
        #[doc = " Unmount a previously mounted ResourceProvider; return 0: success, -1: not found in VFS."]
        pub(crate) fn mj_unmountVFS(
            vfs: *mut mjVFS,
            filename: *const ::core::ffi::c_char,
        ) -> ::core::ffi::c_int;
        #[doc = " Add file to VFS; return 0: success, 2: repeated name, -1: failed to load."]
        pub(crate) fn mj_addFileVFS(
            vfs: *mut mjVFS,
            directory: *const ::core::ffi::c_char,
            filename: *const ::core::ffi::c_char,
        ) -> ::core::ffi::c_int;
        #[doc = " Add file to VFS from buffer; return 0: success, 2: repeated name, -1: failed to load."]
        pub(crate) fn mj_addBufferVFS(
            vfs: *mut mjVFS,
            name: *const ::core::ffi::c_char,
            buffer: *const ::core::ffi::c_void,
            nbuffer: ::core::ffi::c_int,
        ) -> ::core::ffi::c_int;
        #[doc = " Delete file from VFS; return 0: success, -1: not found in VFS."]
        pub(crate) fn mj_deleteFileVFS(
            vfs: *mut mjVFS,
            filename: *const ::core::ffi::c_char,
        ) -> ::core::ffi::c_int;
        #[doc = " Check if buffer exists in VFS; return 1: exists, 0: not found."]
        pub(crate) fn mj_containsBufferVFS(
            vfs: *mut mjVFS,
            name: *const ::core::ffi::c_char,
        ) -> ::core::ffi::c_int;
        #[doc = " Check if file exists in VFS; return 1: exists, 0: not found."]
        pub(crate) fn mj_containsFileVFS(
            vfs: *mut mjVFS,
            directory: *const ::core::ffi::c_char,
            filename: *const ::core::ffi::c_char,
        ) -> ::core::ffi::c_int;
        #[doc = " Delete all files from VFS and deallocates VFS internal memory."]
        pub(crate) fn mj_deleteVFS(vfs: *mut mjVFS);
        #[doc = " Get the current size of the asset cache in bytes."]
        pub(crate) fn mj_getCacheSize(cache: *const mjCache) -> usize;
        #[doc = " Get the capacity of the asset cache in bytes."]
        pub(crate) fn mj_getCacheCapacity(cache: *const mjCache) -> usize;
        #[doc = " Set the capacity of the asset cache in bytes (0 to disable); return the new capacity."]
        pub(crate) fn mj_setCacheCapacity(cache: *mut mjCache, size: usize) -> usize;
        #[doc = " Get the internal asset cache used by the compiler."]
        pub(crate) fn mj_getCache() -> *mut mjCache;
        #[doc = " Clear the asset cache."]
        pub(crate) fn mj_clearCache(cache: *mut mjCache);
        #[doc = " Parse XML file in MJCF or URDF format, compile it; return low-level model.\n If vfs is not NULL, look up files in vfs before reading from disk.\n If error is not NULL, it must have size error_sz.\n Nullable: vfs, error"]
        pub(crate) fn mj_loadXML(
            filename: *const ::core::ffi::c_char,
            vfs: *const mjVFS,
            error: *mut ::core::ffi::c_char,
            error_sz: ::core::ffi::c_int,
        ) -> *mut mjModel;
        #[doc = " Parse spec from XML file.\n Nullable: vfs, error"]
        pub(crate) fn mj_parseXML(
            filename: *const ::core::ffi::c_char,
            vfs: *const mjVFS,
            error: *mut ::core::ffi::c_char,
            error_sz: ::core::ffi::c_int,
        ) -> *mut mjSpec;
        #[doc = " Parse spec from XML string.\n Nullable: vfs, error"]
        pub(crate) fn mj_parseXMLString(
            xml: *const ::core::ffi::c_char,
            vfs: *const mjVFS,
            error: *mut ::core::ffi::c_char,
            error_sz: ::core::ffi::c_int,
        ) -> *mut mjSpec;
        #[doc = " Parse spec from a file.\n Nullable: vfs, error"]
        pub(crate) fn mj_parse(
            filename: *const ::core::ffi::c_char,
            content_type: *const ::core::ffi::c_char,
            vfs: *const mjVFS,
            error: *mut ::core::ffi::c_char,
            error_sz: ::core::ffi::c_int,
        ) -> *mut mjSpec;
        #[doc = " Encode spec/model to a file using a registered encoder.\n Returns the number of bytes written on success, -1 on failure.\n Nullable: m, vfs, error"]
        pub(crate) fn mj_encode(
            s: *const mjSpec,
            m: *const mjModel,
            filename: *const ::core::ffi::c_char,
            content_type: *const ::core::ffi::c_char,
            vfs: *const mjVFS,
            error: *mut ::core::ffi::c_char,
            error_sz: ::core::ffi::c_int,
        ) -> mjtSize;
        #[doc = " Compile spec to model.\n Nullable: vfs"]
        pub(crate) fn mj_compile(s: *mut mjSpec, vfs: *const mjVFS) -> *mut mjModel;
        #[doc = " Copy real-valued arrays from model to spec; return 1 on success."]
        pub(crate) fn mj_copyBack(s: *mut mjSpec, m: *const mjModel) -> ::core::ffi::c_int;
        #[doc = " Recompile spec to model, preserving the state; return 0 on success.\n Nullable: vfs"]
        pub(crate) fn mj_recompile(
            s: *mut mjSpec,
            vfs: *const mjVFS,
            m: *mut mjModel,
            d: *mut mjData,
        ) -> ::core::ffi::c_int;
        #[doc = " Update XML data structures with info from low-level model created with mj_loadXML, save as MJCF.\n If error is not NULL, it must have size error_sz.\n Nullable: error"]
        pub(crate) fn mj_saveLastXML(
            filename: *const ::core::ffi::c_char,
            m: *const mjModel,
            error: *mut ::core::ffi::c_char,
            error_sz: ::core::ffi::c_int,
        ) -> ::core::ffi::c_int;
        #[doc = " Free last XML model if loaded. Called internally at each load."]
        pub(crate) fn mj_freeLastXML();
        #[doc = " Save spec to XML string; return 0 on success, -1 on failure.\n If length of the output buffer is too small; return the required size.\n Nullable: error"]
        pub(crate) fn mj_saveXMLString(
            s: *const mjSpec,
            xml: *mut ::core::ffi::c_char,
            xml_sz: ::core::ffi::c_int,
            error: *mut ::core::ffi::c_char,
            error_sz: ::core::ffi::c_int,
        ) -> ::core::ffi::c_int;
        #[doc = " Save spec to XML file; return 0 on success, -1 otherwise.\n Nullable: error"]
        pub(crate) fn mj_saveXML(
            s: *const mjSpec,
            filename: *const ::core::ffi::c_char,
            error: *mut ::core::ffi::c_char,
            error_sz: ::core::ffi::c_int,
        ) -> ::core::ffi::c_int;
        #[doc = " Given MJCF filename, fills dependencies with a list of all other asset files it depends on.\n The search is recursive, and the list includes the filename itself."]
        pub(crate) fn mju_getXMLDependencies(
            filename: *const ::core::ffi::c_char,
            dependencies: *mut mjStringVec,
        );
        #[doc = " Advance simulation, use control callback to obtain external force and control."]
        pub(crate) fn mj_step(m: *const mjModel, d: *mut mjData);
        #[doc = " Advance simulation in two steps: before external force and control is set by user."]
        pub(crate) fn mj_step1(m: *const mjModel, d: *mut mjData);
        #[doc = " Advance simulation in two steps: after external force and control is set by user."]
        pub(crate) fn mj_step2(m: *const mjModel, d: *mut mjData);
        #[doc = " Forward dynamics: same as mj_step but do not integrate in time."]
        pub(crate) fn mj_forward(m: *const mjModel, d: *mut mjData);
        #[doc = " Inverse dynamics: qacc must be set before calling."]
        pub(crate) fn mj_inverse(m: *const mjModel, d: *mut mjData);
        #[doc = " Forward dynamics with skip; skipstage is mjtStage."]
        pub(crate) fn mj_forwardSkip(
            m: *const mjModel,
            d: *mut mjData,
            skipstage: ::core::ffi::c_int,
            skipsensor: ::core::ffi::c_int,
        );
        #[doc = " Inverse dynamics with skip; skipstage is mjtStage."]
        pub(crate) fn mj_inverseSkip(
            m: *const mjModel,
            d: *mut mjData,
            skipstage: ::core::ffi::c_int,
            skipsensor: ::core::ffi::c_int,
        );
        #[doc = " Set default options for length range computation."]
        pub(crate) fn mj_defaultLROpt(opt: *mut mjLROpt);
        #[doc = " Set solver parameters to default values.\n Nullable: solref, solimp"]
        pub(crate) fn mj_defaultSolRefImp(solref: *mut mjtNum, solimp: *mut mjtNum);
        #[doc = " Set physics options to default values."]
        pub(crate) fn mj_defaultOption(opt: *mut mjOption);
        #[doc = " Set visual options to default values."]
        pub(crate) fn mj_defaultVisual(vis: *mut mjVisual);
        #[doc = " Copy mjModel, allocate new if dest is NULL.\n Nullable: dest"]
        pub(crate) fn mj_copyModel(dest: *mut mjModel, src: *const mjModel) -> *mut mjModel;
        #[doc = " Save model to binary MJB file or memory buffer; buffer has precedence when given.\n Nullable: filename, buffer"]
        pub(crate) fn mj_saveModel(
            m: *const mjModel,
            filename: *const ::core::ffi::c_char,
            buffer: *mut ::core::ffi::c_void,
            buffer_sz: ::core::ffi::c_int,
        );
        #[doc = " Load model from binary MJB file.\n If vfs is not NULL, look up file in vfs before reading from disk.\n Nullable: vfs"]
        pub(crate) fn mj_loadModel(
            filename: *const ::core::ffi::c_char,
            vfs: *const mjVFS,
        ) -> *mut mjModel;
        #[doc = " Load model from memory buffer."]
        pub(crate) fn mj_loadModelBuffer(
            buffer: *const ::core::ffi::c_void,
            buffer_sz: ::core::ffi::c_int,
        ) -> *mut mjModel;
        #[doc = " Free memory allocation in model."]
        pub(crate) fn mj_deleteModel(m: *mut mjModel);
        #[doc = " Return size of buffer needed to hold model."]
        pub(crate) fn mj_sizeModel(m: *const mjModel) -> mjtSize;
        #[doc = " Allocate mjData corresponding to given model.\n If the model buffer is unallocated the initial configuration will not be set."]
        pub(crate) fn mj_makeData(m: *const mjModel) -> *mut mjData;
        #[doc = " Copy mjData.\n m is only required to contain the size fields from MJMODEL_INTS."]
        pub(crate) fn mj_copyData(
            dest: *mut mjData,
            m: *const mjModel,
            src: *const mjData,
        ) -> *mut mjData;
        #[doc = " Copy mjData, skip large arrays not required for visualization."]
        pub(crate) fn mjv_copyData(
            dest: *mut mjData,
            m: *const mjModel,
            src: *const mjData,
        ) -> *mut mjData;
        #[doc = " Reset ctrl to neutral values: zero, except quaternion inputs which reset to the identity."]
        pub(crate) fn mj_resetCtrl(m: *const mjModel, d: *mut mjData);
        #[doc = " Reset data to defaults."]
        pub(crate) fn mj_resetData(m: *const mjModel, d: *mut mjData);
        #[doc = " Reset data to defaults, fill everything else with debug_value."]
        pub(crate) fn mj_resetDataDebug(
            m: *const mjModel,
            d: *mut mjData,
            debug_value: ::core::ffi::c_uchar,
        );
        #[doc = " Reset data. If 0 <= key < nkey, set fields from specified keyframe."]
        pub(crate) fn mj_resetDataKeyframe(
            m: *const mjModel,
            d: *mut mjData,
            key: ::core::ffi::c_int,
        );
        #[doc = " Mark a new frame on the mjData stack."]
        pub(crate) fn mj_markStack(d: *mut mjData);
        #[doc = " Free the current mjData stack frame. All pointers returned by mj_stackAlloc since the last call\n to mj_markStack must no longer be used afterwards."]
        pub(crate) fn mj_freeStack(d: *mut mjData);
        #[doc = " Allocate a number of bytes on mjData stack at a specific alignment.\n Call mju_error on stack overflow."]
        pub(crate) fn mj_stackAllocByte(
            d: *mut mjData,
            bytes: usize,
            alignment: usize,
        ) -> *mut ::core::ffi::c_void;
        #[doc = " Allocate array of mjtNums on mjData stack. Call mju_error on stack overflow."]
        pub(crate) fn mj_stackAllocNum(d: *mut mjData, size: usize) -> *mut mjtNum;
        #[doc = " Allocate array of ints on mjData stack. Call mju_error on stack overflow."]
        pub(crate) fn mj_stackAllocInt(d: *mut mjData, size: usize) -> *mut ::core::ffi::c_int;
        #[doc = " Free memory allocation in mjData."]
        pub(crate) fn mj_deleteData(d: *mut mjData);
        #[doc = " Reset all callbacks to NULL pointers (NULL is the default)."]
        pub(crate) fn mj_resetCallbacks();
        #[doc = " Set constant fields of mjModel, corresponding to qpos0 configuration."]
        pub(crate) fn mj_setConst(m: *mut mjModel, d: *mut mjData);
        #[doc = " Set actuator_lengthrange for specified actuator; return 1 if ok, 0 if error.\n Nullable: error"]
        pub(crate) fn mj_setLengthRange(
            m: *mut mjModel,
            d: *mut mjData,
            index: ::core::ffi::c_int,
            opt: *const mjLROpt,
            error: *mut ::core::ffi::c_char,
            error_sz: ::core::ffi::c_int,
        ) -> ::core::ffi::c_int;
        #[doc = " Create empty spec."]
        pub(crate) fn mj_makeSpec() -> *mut mjSpec;
        #[doc = " Copy spec."]
        pub(crate) fn mj_copySpec(s: *const mjSpec) -> *mut mjSpec;
        #[doc = " Free memory allocation in mjSpec."]
        pub(crate) fn mj_deleteSpec(s: *mut mjSpec);
        #[doc = " Activate plugin; return 0 on success."]
        pub(crate) fn mjs_activatePlugin(
            s: *mut mjSpec,
            name: *const ::core::ffi::c_char,
        ) -> ::core::ffi::c_int;
        #[doc = " Turn deep copy on or off attach; return 0 on success."]
        pub(crate) fn mjs_setDeepCopy(
            s: *mut mjSpec,
            deepcopy: ::core::ffi::c_int,
        ) -> ::core::ffi::c_int;
        #[doc = " Print mjModel to text file, specifying format.\n float_format must be a valid printf-style format string for a single float value."]
        pub(crate) fn mj_printFormattedModel(
            m: *const mjModel,
            filename: *const ::core::ffi::c_char,
            float_format: *const ::core::ffi::c_char,
        );
        #[doc = " Print model to text file."]
        pub(crate) fn mj_printModel(m: *const mjModel, filename: *const ::core::ffi::c_char);
        #[doc = " Print mjData to text file, specifying format.\n float_format must be a valid printf-style format string for a single float value."]
        pub(crate) fn mj_printFormattedData(
            m: *const mjModel,
            d: *const mjData,
            filename: *const ::core::ffi::c_char,
            float_format: *const ::core::ffi::c_char,
        );
        #[doc = " Print data to text file."]
        pub(crate) fn mj_printData(
            m: *const mjModel,
            d: *const mjData,
            filename: *const ::core::ffi::c_char,
        );
        #[doc = " Print matrix to screen."]
        pub(crate) fn mju_printMat(
            mat: *const mjtNum,
            nr: ::core::ffi::c_int,
            nc: ::core::ffi::c_int,
        );
        #[doc = " Print sparse matrix to screen."]
        pub(crate) fn mju_printMatSparse(
            mat: *const mjtNum,
            nr: ::core::ffi::c_int,
            rownnz: *const ::core::ffi::c_int,
            rowadr: *const ::core::ffi::c_int,
            colind: *const ::core::ffi::c_int,
        );
        #[doc = " Print internal XML schema as plain text or HTML, with style-padding or &nbsp;."]
        pub(crate) fn mj_printSchema(
            filename: *const ::core::ffi::c_char,
            buffer: *mut ::core::ffi::c_char,
            buffer_sz: ::core::ffi::c_int,
            flg_html: ::core::ffi::c_int,
            flg_pad: ::core::ffi::c_int,
        ) -> ::core::ffi::c_int;
        #[doc = " Print scene to text file."]
        pub(crate) fn mj_printScene(s: *const mjvScene, filename: *const ::core::ffi::c_char);
        #[doc = " Print scene to text file, specifying format.\n float_format must be a valid printf-style format string for a single float value."]
        pub(crate) fn mj_printFormattedScene(
            s: *const mjvScene,
            filename: *const ::core::ffi::c_char,
            float_format: *const ::core::ffi::c_char,
        );
        #[doc = " Run all kinematics-like computations (kinematics, comPos, camlight, flex, tendon)."]
        pub(crate) fn mj_fwdKinematics(m: *const mjModel, d: *mut mjData);
        #[doc = " Run position-dependent computations."]
        pub(crate) fn mj_fwdPosition(m: *const mjModel, d: *mut mjData);
        #[doc = " Run velocity-dependent computations."]
        pub(crate) fn mj_fwdVelocity(m: *const mjModel, d: *mut mjData);
        #[doc = " Compute actuator force qfrc_actuator."]
        pub(crate) fn mj_fwdActuation(m: *const mjModel, d: *mut mjData);
        #[doc = " Add up all non-constraint forces, compute qacc_smooth."]
        pub(crate) fn mj_fwdAcceleration(m: *const mjModel, d: *mut mjData);
        #[doc = " Run selected constraint solver."]
        pub(crate) fn mj_fwdConstraint(m: *const mjModel, d: *mut mjData);
        #[doc = " Euler integrator, semi-implicit in velocity."]
        pub(crate) fn mj_Euler(m: *const mjModel, d: *mut mjData);
        #[doc = " Runge-Kutta explicit order-N integrator."]
        pub(crate) fn mj_RungeKutta(m: *const mjModel, d: *mut mjData, N: ::core::ffi::c_int);
        #[doc = " Implicit-in-velocity integrators."]
        pub(crate) fn mj_implicit(m: *const mjModel, d: *mut mjData);
        #[doc = " Run position-dependent computations in inverse dynamics."]
        pub(crate) fn mj_invPosition(m: *const mjModel, d: *mut mjData);
        #[doc = " Run velocity-dependent computations in inverse dynamics."]
        pub(crate) fn mj_invVelocity(m: *const mjModel, d: *mut mjData);
        #[doc = " Apply the analytical formula for inverse constraint dynamics."]
        pub(crate) fn mj_invConstraint(m: *const mjModel, d: *mut mjData);
        #[doc = " Compare forward and inverse dynamics, save results in fwdinv."]
        pub(crate) fn mj_compareFwdInv(m: *const mjModel, d: *mut mjData);
        #[doc = " Evaluate position-dependent sensors."]
        pub(crate) fn mj_sensorPos(m: *const mjModel, d: *mut mjData);
        #[doc = " Evaluate velocity-dependent sensors."]
        pub(crate) fn mj_sensorVel(m: *const mjModel, d: *mut mjData);
        #[doc = " Evaluate acceleration and force-dependent sensors."]
        pub(crate) fn mj_sensorAcc(m: *const mjModel, d: *mut mjData);
        #[doc = " Evaluate position-dependent energy (potential)."]
        pub(crate) fn mj_energyPos(m: *const mjModel, d: *mut mjData);
        #[doc = " Evaluate velocity-dependent energy (kinetic)."]
        pub(crate) fn mj_energyVel(m: *const mjModel, d: *mut mjData);
        #[doc = " Check qpos, reset if any element is too big or nan."]
        pub(crate) fn mj_checkPos(m: *const mjModel, d: *mut mjData);
        #[doc = " Check qvel, reset if any element is too big or nan."]
        pub(crate) fn mj_checkVel(m: *const mjModel, d: *mut mjData);
        #[doc = " Check qacc, reset if any element is too big or nan."]
        pub(crate) fn mj_checkAcc(m: *const mjModel, d: *mut mjData);
        #[doc = " Run forward kinematics."]
        pub(crate) fn mj_kinematics(m: *const mjModel, d: *mut mjData);
        #[doc = " Map inertias and motion dofs to global frame centered at CoM."]
        pub(crate) fn mj_comPos(m: *const mjModel, d: *mut mjData);
        #[doc = " Compute camera and light positions and orientations."]
        pub(crate) fn mj_camlight(m: *const mjModel, d: *mut mjData);
        #[doc = " Compute flex-related quantities."]
        pub(crate) fn mj_flex(m: *const mjModel, d: *mut mjData);
        #[doc = " Compute tendon lengths, velocities and moment arms."]
        pub(crate) fn mj_tendon(m: *const mjModel, d: *mut mjData);
        #[doc = " Compute actuator transmission lengths and moments."]
        pub(crate) fn mj_transmission(m: *const mjModel, d: *mut mjData);
        #[doc = " Run composite rigid body inertia algorithm (CRB)."]
        pub(crate) fn mj_crb(m: *const mjModel, d: *mut mjData);
        #[doc = " Make inertia matrix."]
        pub(crate) fn mj_makeM(m: *const mjModel, d: *mut mjData);
        #[doc = " Compute sparse L'*D*L factorizaton of inertia matrix."]
        pub(crate) fn mj_factorM(m: *const mjModel, d: *mut mjData);
        #[doc = " Solve linear system M * x = y using factorization:  x = inv(L'*D*L)*y"]
        pub(crate) fn mj_solveM(
            m: *const mjModel,
            d: *mut mjData,
            x: *mut mjtNum,
            y: *const mjtNum,
            n: ::core::ffi::c_int,
        );
        #[doc = " Half of linear solve:  x = sqrt(inv(D))*inv(L')*y"]
        pub(crate) fn mj_solveM2(
            m: *const mjModel,
            d: *mut mjData,
            x: *mut mjtNum,
            y: *const mjtNum,
            sqrtInvD: *const mjtNum,
            n: ::core::ffi::c_int,
        );
        #[doc = " Compute cvel, cdof_dot."]
        pub(crate) fn mj_comVel(m: *const mjModel, d: *mut mjData);
        #[doc = " Compute qfrc_passive from spring-dampers, gravity compensation and fluid forces."]
        pub(crate) fn mj_passive(m: *const mjModel, d: *mut mjData);
        #[doc = " Sub-tree linear velocity and angular momentum: compute subtree_linvel, subtree_angmom."]
        pub(crate) fn mj_subtreeVel(m: *const mjModel, d: *mut mjData);
        #[doc = " RNE: compute M(qpos)*qacc + C(qpos,qvel); flg_acc=0 removes inertial term."]
        pub(crate) fn mj_rne(
            m: *const mjModel,
            d: *mut mjData,
            flg_acc: ::core::ffi::c_int,
            result: *mut mjtNum,
        );
        #[doc = " RNE with complete data: compute cacc, cfrc_ext, cfrc_int."]
        pub(crate) fn mj_rnePostConstraint(m: *const mjModel, d: *mut mjData);
        #[doc = " Return the maximum number of contacts that can be generated between two geoms.\n If has_margin is -1, then the margin is pulled from the model, otherwise if has_margin > 0\n indicates that the geoms have a positive margin."]
        pub(crate) fn mj_maxContact(
            m: *const mjModel,
            g1: ::core::ffi::c_int,
            g2: ::core::ffi::c_int,
            has_margin: ::core::ffi::c_int,
        ) -> ::core::ffi::c_int;
        #[doc = " Run collision detection."]
        pub(crate) fn mj_collision(m: *const mjModel, d: *mut mjData);
        #[doc = " Construct constraints."]
        pub(crate) fn mj_makeConstraint(m: *const mjModel, d: *mut mjData);
        #[doc = " Find constraint islands."]
        pub(crate) fn mj_island(m: *const mjModel, d: *mut mjData);
        #[doc = " Compute inverse constraint inertia efc_AR."]
        pub(crate) fn mj_projectConstraint(m: *const mjModel, d: *mut mjData);
        #[doc = " Compute efc_vel, efc_aref."]
        pub(crate) fn mj_referenceConstraint(m: *const mjModel, d: *mut mjData);
        #[doc = " Compute efc_state, efc_force, qfrc_constraint, and (optionally) cone Hessians.\n If cost is not NULL, set *cost = s(jar) where jar = Jac*qacc-aref.\n Nullable: cost"]
        pub(crate) fn mj_constraintUpdate(
            m: *const mjModel,
            d: *mut mjData,
            jar: *const mjtNum,
            cost: *mut [mjtNum; 1usize],
            flg_coneHessian: ::core::ffi::c_int,
        );
        #[doc = " Return size of state signature."]
        pub(crate) fn mj_stateSize(
            m: *const mjModel,
            sig: ::core::ffi::c_int,
        ) -> ::core::ffi::c_int;
        #[doc = " Get state."]
        pub(crate) fn mj_getState(
            m: *const mjModel,
            d: *const mjData,
            state: *mut mjtNum,
            sig: ::core::ffi::c_int,
        );
        #[doc = " Extract a subset of components from a state previously obtained via mj_getState."]
        pub(crate) fn mj_extractState(
            m: *const mjModel,
            src: *const mjtNum,
            srcsig: ::core::ffi::c_int,
            dst: *mut mjtNum,
            dstsig: ::core::ffi::c_int,
        );
        #[doc = " Set state."]
        pub(crate) fn mj_setState(
            m: *const mjModel,
            d: *mut mjData,
            state: *const mjtNum,
            sig: ::core::ffi::c_int,
        );
        #[doc = " Copy state from src to dst."]
        pub(crate) fn mj_copyState(
            m: *const mjModel,
            src: *const mjData,
            dst: *mut mjData,
            sig: ::core::ffi::c_int,
        );
        #[doc = " Read ctrl value for actuator at given time.\n Returns d->ctrl[id] if no history, otherwise reads from history buffer.\n interp: 0=zero-order-hold, 1=linear, 2=cubic spline."]
        pub(crate) fn mj_readCtrl(
            m: *const mjModel,
            d: *const mjData,
            id: ::core::ffi::c_int,
            time: mjtNum,
            interp: ::core::ffi::c_int,
        ) -> mjtNum;
        #[doc = " Read sensor value from history buffer at given time.\n Returns pointer to sensordata (no history) or history buffer (exact match),\n or NULL if interpolation performed (writes to result).\n interp: 0=zero-order-hold, 1=linear, 2=cubic spline."]
        pub(crate) fn mj_readSensor(
            m: *const mjModel,
            d: *const mjData,
            id: ::core::ffi::c_int,
            time: mjtNum,
            result: *mut mjtNum,
            interp: ::core::ffi::c_int,
        ) -> *const mjtNum;
        #[doc = " Initialize history buffer for actuator; if times is NULL, uses existing buffer timestamps.\n Nullable: times"]
        pub(crate) fn mj_initCtrlHistory(
            m: *const mjModel,
            d: *mut mjData,
            id: ::core::ffi::c_int,
            times: *const mjtNum,
            values: *const mjtNum,
        );
        #[doc = " Initialize history buffer for sensor; if times is NULL, uses existing buffer timestamps.\n phase sets the user slot (last computation time for interval sensors).\n Nullable: times"]
        pub(crate) fn mj_initSensorHistory(
            m: *const mjModel,
            d: *mut mjData,
            id: ::core::ffi::c_int,
            times: *const mjtNum,
            values: *const mjtNum,
            phase: mjtNum,
        );
        #[doc = " Copy current state to the k-th model keyframe."]
        pub(crate) fn mj_setKeyframe(m: *mut mjModel, d: *const mjData, k: ::core::ffi::c_int);
        #[doc = " Add contact to d->contact list; return 0 if success; 1 if buffer full."]
        pub(crate) fn mj_addContact(
            m: *const mjModel,
            d: *mut mjData,
            con: *const mjContact,
        ) -> ::core::ffi::c_int;
        #[doc = " Determine type of friction cone."]
        pub(crate) fn mj_isPyramidal(m: *const mjModel) -> ::core::ffi::c_int;
        #[doc = " Determine type of constraint Jacobian."]
        pub(crate) fn mj_isSparse(m: *const mjModel) -> ::core::ffi::c_int;
        #[doc = " Determine type of solver (PGS is dual, CG and Newton are primal)."]
        pub(crate) fn mj_isDual(m: *const mjModel) -> ::core::ffi::c_int;
        #[doc = " Multiply dense or sparse constraint Jacobian by vector."]
        pub(crate) fn mj_mulJacVec(
            m: *const mjModel,
            d: *const mjData,
            res: *mut mjtNum,
            vec: *const mjtNum,
        );
        #[doc = " Multiply dense or sparse constraint Jacobian transpose by vector."]
        pub(crate) fn mj_mulJacTVec(
            m: *const mjModel,
            d: *const mjData,
            res: *mut mjtNum,
            vec: *const mjtNum,
        );
        #[doc = " Compute 3/6-by-nv end-effector Jacobian of global point attached to given body.\n Nullable: jacp, jacr"]
        pub(crate) fn mj_jac(
            m: *const mjModel,
            d: *const mjData,
            jacp: *mut mjtNum,
            jacr: *mut mjtNum,
            point: *const [mjtNum; 3usize],
            body: ::core::ffi::c_int,
        );
        #[doc = " Compute body frame end-effector Jacobian.\n Nullable: jacp, jacr"]
        pub(crate) fn mj_jacBody(
            m: *const mjModel,
            d: *const mjData,
            jacp: *mut mjtNum,
            jacr: *mut mjtNum,
            body: ::core::ffi::c_int,
        );
        #[doc = " Compute body center-of-mass end-effector Jacobian.\n Nullable: jacp, jacr"]
        pub(crate) fn mj_jacBodyCom(
            m: *const mjModel,
            d: *const mjData,
            jacp: *mut mjtNum,
            jacr: *mut mjtNum,
            body: ::core::ffi::c_int,
        );
        #[doc = " Compute subtree center-of-mass end-effector Jacobian."]
        pub(crate) fn mj_jacSubtreeCom(
            m: *const mjModel,
            d: *mut mjData,
            jacp: *mut mjtNum,
            body: ::core::ffi::c_int,
        );
        #[doc = " Compute geom end-effector Jacobian.\n Nullable: jacp, jacr"]
        pub(crate) fn mj_jacGeom(
            m: *const mjModel,
            d: *const mjData,
            jacp: *mut mjtNum,
            jacr: *mut mjtNum,
            geom: ::core::ffi::c_int,
        );
        #[doc = " Compute site end-effector Jacobian.\n Nullable: jacp, jacr"]
        pub(crate) fn mj_jacSite(
            m: *const mjModel,
            d: *const mjData,
            jacp: *mut mjtNum,
            jacr: *mut mjtNum,
            site: ::core::ffi::c_int,
        );
        #[doc = " Compute translation end-effector Jacobian of point, and rotation Jacobian of axis.\n Nullable: jacPoint, jacAxis"]
        pub(crate) fn mj_jacPointAxis(
            m: *const mjModel,
            d: *mut mjData,
            jacPoint: *mut mjtNum,
            jacAxis: *mut mjtNum,
            point: *const [mjtNum; 3usize],
            axis: *const [mjtNum; 3usize],
            body: ::core::ffi::c_int,
        );
        #[doc = " Compute 3/6-by-nv Jacobian time derivative of global point attached to given body.\n Nullable: jacp, jacr"]
        pub(crate) fn mj_jacDot(
            m: *const mjModel,
            d: *const mjData,
            jacp: *mut mjtNum,
            jacr: *mut mjtNum,
            point: *const [mjtNum; 3usize],
            body: ::core::ffi::c_int,
        );
        #[doc = " Compute subtree angular momentum matrix."]
        pub(crate) fn mj_angmomMat(
            m: *const mjModel,
            d: *mut mjData,
            mat: *mut mjtNum,
            body: ::core::ffi::c_int,
        );
        #[doc = " Get id of object with the specified mjtObj type and name; return -1 if id not found."]
        pub(crate) fn mj_name2id(
            m: *const mjModel,
            type_: ::core::ffi::c_int,
            name: *const ::core::ffi::c_char,
        ) -> ::core::ffi::c_int;
        #[doc = " Get name of object with the specified mjtObj type and id; return NULL if name not found."]
        pub(crate) fn mj_id2name(
            m: *const mjModel,
            type_: ::core::ffi::c_int,
            id: ::core::ffi::c_int,
        ) -> *const ::core::ffi::c_char;
        #[doc = " Get name of actuator input, determined by the actuator type and input signature;\n return NULL if the actuator type defines no input names."]
        pub(crate) fn mj_actuatorInputName(
            m: *const mjModel,
            id: ::core::ffi::c_int,
            input: ::core::ffi::c_int,
        ) -> *const ::core::ffi::c_char;
        #[doc = " Convert sparse inertia matrix into full (i.e. dense) matrix."]
        pub(crate) fn mj_fullM(m: *const mjModel, d: *const mjData, dst: *mut mjtNum);
        #[doc = " Multiply vector by inertia matrix."]
        pub(crate) fn mj_mulM(
            m: *const mjModel,
            d: *const mjData,
            res: *mut mjtNum,
            vec: *const mjtNum,
        );
        #[doc = " Multiply vector by (inertia matrix)^(1/2)."]
        pub(crate) fn mj_mulM2(
            m: *const mjModel,
            d: *const mjData,
            res: *mut mjtNum,
            vec: *const mjtNum,
        );
        #[doc = " Add inertia matrix to destination matrix (lower triangle only).\n Destination can be sparse or dense when all int* are NULL.\n Nullable: rownnz, rowadr, colind"]
        pub(crate) fn mj_addM(
            m: *const mjModel,
            d: *mut mjData,
            dst: *mut mjtNum,
            rownnz: *mut ::core::ffi::c_int,
            rowadr: *mut ::core::ffi::c_int,
            colind: *mut ::core::ffi::c_int,
        );
        #[doc = " Apply Cartesian force and torque (outside xfrc_applied mechanism).\n Nullable: force, torque"]
        pub(crate) fn mj_applyFT(
            m: *const mjModel,
            d: *mut mjData,
            force: *const [mjtNum; 3usize],
            torque: *const [mjtNum; 3usize],
            point: *const [mjtNum; 3usize],
            body: ::core::ffi::c_int,
            qfrc_target: *mut mjtNum,
        );
        #[doc = " Compute object 6D velocity (rot:lin) in object-centered frame, world/local orientation."]
        pub(crate) fn mj_objectVelocity(
            m: *const mjModel,
            d: *const mjData,
            objtype: ::core::ffi::c_int,
            objid: ::core::ffi::c_int,
            res: *mut [mjtNum; 6usize],
            flg_local: ::core::ffi::c_int,
        );
        #[doc = " Compute object 6D acceleration (rot:lin) in object-centered frame, world/local orientation."]
        pub(crate) fn mj_objectAcceleration(
            m: *const mjModel,
            d: *const mjData,
            objtype: ::core::ffi::c_int,
            objid: ::core::ffi::c_int,
            res: *mut [mjtNum; 6usize],
            flg_local: ::core::ffi::c_int,
        );
        #[doc = " Return smallest signed distance between two geoms and optionally segment from geom1 to geom2.\n Nullable: fromto"]
        pub(crate) fn mj_geomDistance(
            m: *const mjModel,
            d: *mut mjData,
            geom1: ::core::ffi::c_int,
            geom2: ::core::ffi::c_int,
            distmax: mjtNum,
            fromto: *mut [mjtNum; 6usize],
        ) -> mjtNum;
        #[doc = " Extract 6D force:torque given contact id, in the contact frame."]
        pub(crate) fn mj_contactForce(
            m: *const mjModel,
            d: *const mjData,
            id: ::core::ffi::c_int,
            result: *mut [mjtNum; 6usize],
        );
        #[doc = " Compute velocity by finite-differencing two positions."]
        pub(crate) fn mj_differentiatePos(
            m: *const mjModel,
            qvel: *mut mjtNum,
            dt: mjtNum,
            qpos1: *const mjtNum,
            qpos2: *const mjtNum,
        );
        #[doc = " Integrate position with given velocity."]
        pub(crate) fn mj_integratePos(
            m: *const mjModel,
            qpos: *mut mjtNum,
            qvel: *const mjtNum,
            dt: mjtNum,
        );
        #[doc = " Normalize all quaternions in qpos-type vector."]
        pub(crate) fn mj_normalizeQuat(m: *const mjModel, qpos: *mut mjtNum);
        #[doc = " Map from body local to global Cartesian coordinates, sameframe takes values from mjtSameFrame."]
        pub(crate) fn mj_local2Global(
            d: *mut mjData,
            xpos: *mut [mjtNum; 3usize],
            xmat: *mut [mjtNum; 9usize],
            pos: *const [mjtNum; 3usize],
            quat: *const [mjtNum; 4usize],
            body: ::core::ffi::c_int,
            sameframe: mjtByte,
        );
        #[doc = " Sum all body masses."]
        pub(crate) fn mj_getTotalmass(m: *const mjModel) -> mjtNum;
        #[doc = " Scale body masses and inertias to achieve specified total mass."]
        pub(crate) fn mj_setTotalmass(m: *mut mjModel, newmass: mjtNum);
        #[doc = " Return a config attribute value of a plugin instance;\n NULL: invalid plugin instance ID or attribute name"]
        pub(crate) fn mj_getPluginConfig(
            m: *const mjModel,
            plugin_id: ::core::ffi::c_int,
            attrib: *const ::core::ffi::c_char,
        ) -> *const ::core::ffi::c_char;
        #[doc = " Load a dynamic library. The dynamic library is assumed to register one or more plugins."]
        pub(crate) fn mj_loadPluginLibrary(path: *const ::core::ffi::c_char);
        #[doc = " Scan a directory and load all dynamic libraries. Dynamic libraries in the specified directory\n are assumed to register one or more plugins. Optionally, if a callback is specified, it is called\n for each dynamic library encountered that registers plugins."]
        pub(crate) fn mj_loadAllPluginLibraries(
            directory: *const ::core::ffi::c_char,
            callback: mjfPluginLibraryLoadCallback,
        );
        #[doc = " Return version number: 1.0.2 is encoded as 102."]
        pub(crate) fn mj_version() -> ::core::ffi::c_int;
        #[doc = " Return the current version of MuJoCo as a null-terminated string."]
        pub(crate) fn mj_versionString() -> *const ::core::ffi::c_char;
        #[doc = " Intersect ray (pnt+x*vec, x>=0) with visible geoms, except geoms in bodyexclude.\n Return distance (x) to nearest surface, or -1 if no intersection.\n geomgroup, flg_static are as in mjvOption; geomgroup==NULL skips group exclusion.\n Nullable: geomgroup, geomid, normal"]
        pub(crate) fn mj_ray(
            m: *const mjModel,
            d: *const mjData,
            pnt: *const [mjtNum; 3usize],
            vec: *const [mjtNum; 3usize],
            geomgroup: *const mjtByte,
            flg_static: mjtBool,
            bodyexclude: ::core::ffi::c_int,
            geomid: *mut [::core::ffi::c_int; 1usize],
            normal: *mut [mjtNum; 3usize],
        ) -> mjtNum;
        #[doc = " Intersect multiple rays emanating from a single point, compute normals if given.\n Similar semantics to mj_ray, but vec, normal and dist are arrays.\n Geoms further than cutoff are ignored.\n Nullable: geomgroup, geomid, normal"]
        pub(crate) fn mj_multiRay(
            m: *const mjModel,
            d: *mut mjData,
            pnt: *const [mjtNum; 3usize],
            vec: *const mjtNum,
            geomgroup: *const mjtByte,
            flg_static: mjtBool,
            bodyexclude: ::core::ffi::c_int,
            geomid: *mut ::core::ffi::c_int,
            dist: *mut mjtNum,
            normal: *mut mjtNum,
            nray: ::core::ffi::c_int,
            cutoff: mjtNum,
        );
        #[doc = " Intersect ray with hfield; return nearest distance or -1 if no intersection.\n Nullable: normal"]
        pub(crate) fn mj_rayHfield(
            m: *const mjModel,
            d: *const mjData,
            geomid: ::core::ffi::c_int,
            pnt: *const [mjtNum; 3usize],
            vec: *const [mjtNum; 3usize],
            normal: *mut [mjtNum; 3usize],
        ) -> mjtNum;
        #[doc = " Intersect ray with mesh; return nearest distance or -1 if no intersection.\n Nullable: normal"]
        pub(crate) fn mj_rayMesh(
            m: *const mjModel,
            d: *const mjData,
            geomid: ::core::ffi::c_int,
            pnt: *const [mjtNum; 3usize],
            vec: *const [mjtNum; 3usize],
            normal: *mut [mjtNum; 3usize],
        ) -> mjtNum;
        #[doc = " Intersect ray with pure geom; return nearest distance or -1 if no intersection.\n Nullable: normal"]
        pub(crate) fn mju_rayGeom(
            pos: *const [mjtNum; 3usize],
            mat: *const [mjtNum; 9usize],
            size: *const [mjtNum; 3usize],
            pnt: *const [mjtNum; 3usize],
            vec: *const [mjtNum; 3usize],
            geomtype: ::core::ffi::c_int,
            normal: *mut [mjtNum; 3usize],
        ) -> mjtNum;
        #[doc = " Intersect ray with flex; return nearest distance or -1 if no intersection,\n and also output nearest vertex id and surface normal.\n Nullable: vertid, normal"]
        pub(crate) fn mj_rayFlex(
            m: *const mjModel,
            d: *const mjData,
            flex_layer: ::core::ffi::c_int,
            flg_vert: mjtBool,
            flg_edge: mjtBool,
            flg_face: mjtBool,
            flg_skin: mjtBool,
            flexid: ::core::ffi::c_int,
            pnt: *const [mjtNum; 3usize],
            vec: *const [mjtNum; 3usize],
            vertid: *mut [::core::ffi::c_int; 1usize],
            normal: *mut [mjtNum; 3usize],
        ) -> mjtNum;
        #[doc = " Intersect ray with skin; return nearest distance or -1 if no intersection,\n and also output nearest vertex id.\n Nullable: vertid"]
        pub(crate) fn mju_raySkin(
            nface: ::core::ffi::c_int,
            nvert: ::core::ffi::c_int,
            face: *const ::core::ffi::c_int,
            vert: *const f32,
            pnt: *const [mjtNum; 3usize],
            vec: *const [mjtNum; 3usize],
            vertid: *mut [::core::ffi::c_int; 1usize],
        ) -> mjtNum;
        #[doc = " Set default camera."]
        pub(crate) fn mjv_defaultCamera(cam: *mut mjvCamera);
        #[doc = " Set default free camera."]
        pub(crate) fn mjv_defaultFreeCamera(m: *const mjModel, cam: *mut mjvCamera);
        #[doc = " Set default perturbation."]
        pub(crate) fn mjv_defaultPerturb(pert: *mut mjvPerturb);
        #[doc = " Transform pose from room to model space."]
        pub(crate) fn mjv_room2model(
            modelpos: *mut [mjtNum; 3usize],
            modelquat: *mut [mjtNum; 4usize],
            roompos: *const [mjtNum; 3usize],
            roomquat: *const [mjtNum; 4usize],
            scn: *const mjvScene,
        );
        #[doc = " Transform pose from model to room space."]
        pub(crate) fn mjv_model2room(
            roompos: *mut [mjtNum; 3usize],
            roomquat: *mut [mjtNum; 4usize],
            modelpos: *const [mjtNum; 3usize],
            modelquat: *const [mjtNum; 4usize],
            scn: *const mjvScene,
        );
        #[doc = " Get camera info in model space; average left and right OpenGL cameras."]
        pub(crate) fn mjv_cameraInModel(
            headpos: *mut [mjtNum; 3usize],
            forward: *mut [mjtNum; 3usize],
            up: *mut [mjtNum; 3usize],
            scn: *const mjvScene,
        );
        #[doc = " Get camera info in room space; average left and right OpenGL cameras."]
        pub(crate) fn mjv_cameraInRoom(
            headpos: *mut [mjtNum; 3usize],
            forward: *mut [mjtNum; 3usize],
            up: *mut [mjtNum; 3usize],
            scn: *const mjvScene,
        );
        #[doc = " Get frustum height at unit distance from camera; average left and right OpenGL cameras."]
        pub(crate) fn mjv_frustumHeight(scn: *const mjvScene) -> mjtNum;
        #[doc = " Rotate 3D vec in horizontal plane by angle between (0,1) and (forward_x,forward_y)."]
        pub(crate) fn mjv_alignToCamera(
            res: *mut [mjtNum; 3usize],
            vec: *const [mjtNum; 3usize],
            forward: *const [mjtNum; 3usize],
        );
        #[doc = " Move camera with mouse; action is mjtMouse."]
        pub(crate) fn mjv_moveCamera(
            m: *const mjModel,
            action: ::core::ffi::c_int,
            reldx: mjtNum,
            reldy: mjtNum,
            cam: *mut mjvCamera,
        );
        #[doc = " Move perturb object with mouse; action is mjtMouse."]
        pub(crate) fn mjv_movePerturb(
            m: *const mjModel,
            d: *const mjData,
            action: ::core::ffi::c_int,
            reldx: mjtNum,
            reldy: mjtNum,
            scn: *const mjvScene,
            pert: *mut mjvPerturb,
        );
        #[doc = " Move model with mouse; action is mjtMouse."]
        pub(crate) fn mjv_moveModel(
            m: *const mjModel,
            action: ::core::ffi::c_int,
            reldx: mjtNum,
            reldy: mjtNum,
            roomup: *const [mjtNum; 3usize],
            scn: *mut mjvScene,
        );
        #[doc = " Copy perturb pos,quat from selected body; set scale for perturbation."]
        pub(crate) fn mjv_initPerturb(
            m: *const mjModel,
            d: *mut mjData,
            scn: *const mjvScene,
            pert: *mut mjvPerturb,
        );
        #[doc = " Set perturb pos,quat in d->mocap when selected body is mocap, and in d->qpos otherwise.\n Write d->qpos only if flg_paused and subtree root for selected body has free joint."]
        pub(crate) fn mjv_applyPerturbPose(
            m: *const mjModel,
            d: *mut mjData,
            pert: *const mjvPerturb,
            flg_paused: ::core::ffi::c_int,
        );
        #[doc = " Set perturb force,torque in d->xfrc_applied, if selected body is dynamic."]
        pub(crate) fn mjv_applyPerturbForce(
            m: *const mjModel,
            d: *mut mjData,
            pert: *const mjvPerturb,
        );
        #[doc = " Return the average of two OpenGL cameras."]
        pub(crate) fn mjv_averageCamera(
            cam1: *const mjvGLCamera,
            cam2: *const mjvGLCamera,
        ) -> mjvGLCamera;
        #[doc = " Select geom, flex or skin with mouse; return bodyid; -1: none selected.\n Nullable: geomid, flexid, skinid"]
        pub(crate) fn mjv_select(
            m: *const mjModel,
            d: *const mjData,
            vopt: *const mjvOption,
            aspectratio: mjtNum,
            relx: mjtNum,
            rely: mjtNum,
            scn: *const mjvScene,
            selpnt: *mut [mjtNum; 3usize],
            geomid: *mut [::core::ffi::c_int; 1usize],
            flexid: *mut [::core::ffi::c_int; 1usize],
            skinid: *mut [::core::ffi::c_int; 1usize],
        ) -> ::core::ffi::c_int;
        #[doc = " Set default visualization options."]
        pub(crate) fn mjv_defaultOption(opt: *mut mjvOption);
        #[doc = " Set default figure."]
        pub(crate) fn mjv_defaultFigure(fig: *mut mjvFigure);
        #[doc = " Initialize given geom fields when not NULL, set the rest to their default values.\n Nullable: size, pos, mat, rgba"]
        pub(crate) fn mjv_initGeom(
            geom: *mut mjvGeom,
            type_: ::core::ffi::c_int,
            size: *const [mjtNum; 3usize],
            pos: *const [mjtNum; 3usize],
            mat: *const [mjtNum; 9usize],
            rgba: *const [f32; 4usize],
        );
        #[doc = " Set (type, size, pos, mat) for connector-type geom between given points.\n Assume that mjv_initGeom was already called to set all other properties.\n Width of mjGEOM_LINE is denominated in pixels."]
        pub(crate) fn mjv_connector(
            geom: *mut mjvGeom,
            type_: ::core::ffi::c_int,
            width: mjtNum,
            from: *const [mjtNum; 3usize],
            to: *const [mjtNum; 3usize],
        );
        #[doc = " Set default abstract scene."]
        pub(crate) fn mjv_defaultScene(scn: *mut mjvScene);
        #[doc = " Allocate resources in abstract scene."]
        pub(crate) fn mjv_makeScene(
            m: *const mjModel,
            scn: *mut mjvScene,
            maxgeom: ::core::ffi::c_int,
        );
        #[doc = " Free abstract scene."]
        pub(crate) fn mjv_freeScene(scn: *mut mjvScene);
        #[doc = " Update entire scene given model state."]
        pub(crate) fn mjv_updateScene(
            m: *const mjModel,
            d: *mut mjData,
            opt: *const mjvOption,
            pert: *const mjvPerturb,
            cam: *mut mjvCamera,
            catmask: ::core::ffi::c_int,
            scn: *mut mjvScene,
        );
        #[doc = " Copy mjModel, skip large arrays not required for abstract visualization.\n Nullable: dest"]
        pub(crate) fn mjv_copyModel(dest: *mut mjModel, src: *const mjModel);
        #[doc = " Add geoms from selected categories."]
        pub(crate) fn mjv_addGeoms(
            m: *const mjModel,
            d: *mut mjData,
            opt: *const mjvOption,
            pert: *const mjvPerturb,
            catmask: ::core::ffi::c_int,
            scn: *mut mjvScene,
        );
        #[doc = " Make list of lights."]
        pub(crate) fn mjv_makeLights(m: *const mjModel, d: *const mjData, scn: *mut mjvScene);
        #[doc = " Update camera."]
        pub(crate) fn mjv_updateCamera(
            m: *const mjModel,
            d: *const mjData,
            cam: *mut mjvCamera,
            scn: *mut mjvScene,
        );
        #[doc = " Update skins."]
        pub(crate) fn mjv_updateSkin(m: *const mjModel, d: *const mjData, scn: *mut mjvScene);
        #[doc = " Compute camera position and forward, up, and right vectors.\n Nullable: headpos, forward, up, right"]
        pub(crate) fn mjv_cameraFrame(
            headpos: *mut [mjtNum; 3usize],
            forward: *mut [mjtNum; 3usize],
            up: *mut [mjtNum; 3usize],
            right: *mut [mjtNum; 3usize],
            d: *const mjData,
            cam: *const mjvCamera,
        );
        #[doc = " Compute camera frustum: vertical, horizontal, and clip planes.\n Nullable: zver, zhor, zclip"]
        pub(crate) fn mjv_cameraFrustum(
            zver: *mut [f32; 2usize],
            zhor: *mut [f32; 2usize],
            zclip: *mut [f32; 2usize],
            m: *const mjModel,
            cam: *const mjvCamera,
        );
        #[doc = " Set default mjrContext."]
        pub(crate) fn mjr_defaultContext(con: *mut mjrContext);
        #[doc = " Set default mjrRendererInfo."]
        pub(crate) fn mjr_defaultRendererInfo(info: *mut mjrRendererInfo);
        #[doc = " Get active renderer information."]
        pub(crate) fn mjr_getRendererInfo(info: *mut mjrRendererInfo);
        #[doc = " Allocate resources in custom OpenGL context; fontscale is mjtFontScale."]
        pub(crate) fn mjr_makeContext(
            m: *const mjModel,
            con: *mut mjrContext,
            fontscale: ::core::ffi::c_int,
        );
        #[doc = " Change font of existing context."]
        pub(crate) fn mjr_changeFont(fontscale: ::core::ffi::c_int, con: *mut mjrContext);
        #[doc = " Add Aux buffer with given index to context; free previous Aux buffer."]
        pub(crate) fn mjr_addAux(
            index: ::core::ffi::c_int,
            width: ::core::ffi::c_int,
            height: ::core::ffi::c_int,
            samples: ::core::ffi::c_int,
            con: *mut mjrContext,
        );
        #[doc = " Free resources in custom OpenGL context, set to default."]
        pub(crate) fn mjr_freeContext(con: *mut mjrContext);
        #[doc = " Resize offscreen buffers."]
        pub(crate) fn mjr_resizeOffscreen(
            width: ::core::ffi::c_int,
            height: ::core::ffi::c_int,
            con: *mut mjrContext,
        );
        #[doc = " Upload texture to GPU, overwriting previous upload if any."]
        pub(crate) fn mjr_uploadTexture(
            m: *const mjModel,
            con: *const mjrContext,
            texid: ::core::ffi::c_int,
        );
        #[doc = " Upload mesh to GPU, overwriting previous upload if any."]
        pub(crate) fn mjr_uploadMesh(
            m: *const mjModel,
            con: *const mjrContext,
            meshid: ::core::ffi::c_int,
        );
        #[doc = " Upload height field to GPU, overwriting previous upload if any."]
        pub(crate) fn mjr_uploadHField(
            m: *const mjModel,
            con: *const mjrContext,
            hfieldid: ::core::ffi::c_int,
        );
        #[doc = " Make con->currentBuffer current again."]
        pub(crate) fn mjr_restoreBuffer(con: *const mjrContext);
        #[doc = " Set OpenGL framebuffer for rendering: mjFB_WINDOW or mjFB_OFFSCREEN.\n If only one buffer is available, set that buffer and ignore framebuffer argument."]
        pub(crate) fn mjr_setBuffer(framebuffer: ::core::ffi::c_int, con: *mut mjrContext);
        #[doc = " Read pixels from current OpenGL framebuffer to client buffer.\n Viewport is in OpenGL framebuffer; client buffer starts at (0,0)."]
        pub(crate) fn mjr_readPixels(
            rgb: *mut ::core::ffi::c_uchar,
            depth: *mut f32,
            viewport: mjrRect,
            con: *const mjrContext,
        );
        #[doc = " Draw pixels from client buffer to current OpenGL framebuffer.\n Viewport is in OpenGL framebuffer; client buffer starts at (0,0)."]
        pub(crate) fn mjr_drawPixels(
            rgb: *const ::core::ffi::c_uchar,
            depth: *const f32,
            viewport: mjrRect,
            con: *const mjrContext,
        );
        #[doc = " Blit from src viewpoint in current framebuffer to dst viewport in other framebuffer.\n If src, dst have different size and flg_depth==0, color is interpolated with GL_LINEAR."]
        pub(crate) fn mjr_blitBuffer(
            src: mjrRect,
            dst: mjrRect,
            flg_color: ::core::ffi::c_int,
            flg_depth: ::core::ffi::c_int,
            con: *const mjrContext,
        );
        #[doc = " Set Aux buffer for custom OpenGL rendering (call restoreBuffer when done)."]
        pub(crate) fn mjr_setAux(index: ::core::ffi::c_int, con: *const mjrContext);
        #[doc = " Blit from Aux buffer to con->currentBuffer."]
        pub(crate) fn mjr_blitAux(
            index: ::core::ffi::c_int,
            src: mjrRect,
            left: ::core::ffi::c_int,
            bottom: ::core::ffi::c_int,
            con: *const mjrContext,
        );
        #[doc = " Draw text at (x,y) in relative coordinates; font is mjtFont."]
        pub(crate) fn mjr_text(
            font: ::core::ffi::c_int,
            txt: *const ::core::ffi::c_char,
            con: *const mjrContext,
            x: f32,
            y: f32,
            r: f32,
            g: f32,
            b: f32,
        );
        #[doc = " Draw text overlay; font is mjtFont; gridpos is mjtGridPos."]
        pub(crate) fn mjr_overlay(
            font: ::core::ffi::c_int,
            gridpos: ::core::ffi::c_int,
            viewport: mjrRect,
            overlay: *const ::core::ffi::c_char,
            overlay2: *const ::core::ffi::c_char,
            con: *const mjrContext,
        );
        #[doc = " Get maximum viewport for active buffer."]
        pub(crate) fn mjr_maxViewport(con: *const mjrContext) -> mjrRect;
        #[doc = " Draw rectangle."]
        pub(crate) fn mjr_rectangle(viewport: mjrRect, r: f32, g: f32, b: f32, a: f32);
        #[doc = " Draw rectangle with centered text."]
        pub(crate) fn mjr_label(
            viewport: mjrRect,
            font: ::core::ffi::c_int,
            txt: *const ::core::ffi::c_char,
            r: f32,
            g: f32,
            b: f32,
            a: f32,
            rt: f32,
            gt: f32,
            bt: f32,
            con: *const mjrContext,
        );
        #[doc = " Draw 2D figure."]
        pub(crate) fn mjr_figure(viewport: mjrRect, fig: *mut mjvFigure, con: *const mjrContext);
        #[doc = " Render 3D scene."]
        pub(crate) fn mjr_render(viewport: mjrRect, scn: *mut mjvScene, con: *const mjrContext);
        #[doc = " Call glFinish."]
        pub(crate) fn mjr_finish();
        #[doc = " Call glGetError and return result."]
        pub(crate) fn mjr_getError() -> ::core::ffi::c_int;
        #[doc = " Find first rectangle containing mouse, -1: not found."]
        pub(crate) fn mjr_findRect(
            x: ::core::ffi::c_int,
            y: ::core::ffi::c_int,
            nrect: ::core::ffi::c_int,
            rect: *const mjrRect,
        ) -> ::core::ffi::c_int;
        #[doc = " Get builtin UI theme spacing (ind: 0-1)."]
        pub(crate) fn mjui_themeSpacing(ind: ::core::ffi::c_int) -> mjuiThemeSpacing;
        #[doc = " Get builtin UI theme color (ind: 0-3)."]
        pub(crate) fn mjui_themeColor(ind: ::core::ffi::c_int) -> mjuiThemeColor;
        #[doc = " Add definitions to UI."]
        pub(crate) fn mjui_add(ui: *mut mjUI, def: *const mjuiDef);
        #[doc = " Add definitions to UI section."]
        pub(crate) fn mjui_addToSection(
            ui: *mut mjUI,
            sect: ::core::ffi::c_int,
            def: *const mjuiDef,
        );
        #[doc = " Compute UI sizes."]
        pub(crate) fn mjui_resize(ui: *mut mjUI, con: *const mjrContext);
        #[doc = " Update specific section/item; -1: update all."]
        pub(crate) fn mjui_update(
            section: ::core::ffi::c_int,
            item: ::core::ffi::c_int,
            ui: *const mjUI,
            state: *const mjuiState,
            con: *const mjrContext,
        );
        #[doc = " Handle UI event; return pointer to changed item, NULL if no change."]
        pub(crate) fn mjui_event(
            ui: *mut mjUI,
            state: *mut mjuiState,
            con: *const mjrContext,
        ) -> *mut mjuiItem;
        #[doc = " Copy UI image to current buffer."]
        pub(crate) fn mjui_render(ui: *mut mjUI, state: *const mjuiState, con: *const mjrContext);
        #[doc = " Main error function; does not return to caller."]
        pub(crate) fn mju_error(msg: *const ::core::ffi::c_char, ...);
        #[doc = " Main warning function; returns to caller."]
        pub(crate) fn mju_warning(msg: *const ::core::ffi::c_char, ...);
        #[doc = " Clear user error and memory handlers."]
        pub(crate) fn mju_clearHandlers();
        #[doc = " Set the active log handler; return the previous handler.\n If handler is NULL, restore the default handler."]
        pub(crate) fn mju_setLogHandler(handler: mjfLogHandler) -> mjfLogHandler;
        #[doc = " Get default handler configuration."]
        pub(crate) fn mju_getLogConfig() -> mjLogConfig;
        #[doc = " Set default handler configuration."]
        pub(crate) fn mju_setLogConfig(config: mjLogConfig);
        #[doc = " Log an info message with optional topic filtering."]
        pub(crate) fn mju_info(topic: ::core::ffi::c_int, msg: *const ::core::ffi::c_char, ...);
        #[doc = " Dispatch a structured log message to the active handler."]
        pub(crate) fn mju_message(msg: *const mjLogMessage);
        #[doc = " Allocate memory; byte-align on 64; pad size to multiple of 64."]
        pub(crate) fn mju_malloc(size: usize) -> *mut ::core::ffi::c_void;
        #[doc = " Free memory, using free() by default."]
        pub(crate) fn mju_free(ptr: *mut ::core::ffi::c_void);
        #[doc = " High-level warning function: count warnings in mjData, print only the first."]
        pub(crate) fn mj_warning(
            d: *mut mjData,
            warning: ::core::ffi::c_int,
            info: ::core::ffi::c_int,
        );
        #[doc = " Write [datetime, type: message] to MUJOCO_LOG.TXT."]
        pub(crate) fn mju_writeLog(
            type_: *const ::core::ffi::c_char,
            msg: *const ::core::ffi::c_char,
        );
        #[doc = " Get compiler error message from spec."]
        pub(crate) fn mjs_getError(s: *mut mjSpec) -> *const ::core::ffi::c_char;
        #[doc = " Get compiler timing diagnostics from spec, returns pointer to array of size mjNCTIMER."]
        pub(crate) fn mjs_getTimer(s: *mut mjSpec) -> *const f64;
        #[doc = " Return 1 if compiler error is a warning. Deprecated: use mjs_numWarnings(s) > 0."]
        pub(crate) fn mjs_isWarning(s: *mut mjSpec) -> ::core::ffi::c_int;
        #[doc = " Get number of warnings accumulated in the spec."]
        pub(crate) fn mjs_numWarnings(spec: *const mjSpec) -> ::core::ffi::c_int;
        #[doc = " Get the i-th warning message (returns nullptr if index out of bounds)."]
        pub(crate) fn mjs_getWarning(
            spec: *const mjSpec,
            index: ::core::ffi::c_int,
        ) -> *const ::core::ffi::c_char;
        #[doc = " Set res = 0."]
        pub(crate) fn mju_zero3(res: *mut [mjtNum; 3usize]);
        #[doc = " Set res = vec."]
        pub(crate) fn mju_copy3(res: *mut [mjtNum; 3usize], data: *const [mjtNum; 3usize]);
        #[doc = " Set res = vec*scl."]
        pub(crate) fn mju_scl3(
            res: *mut [mjtNum; 3usize],
            vec: *const [mjtNum; 3usize],
            scl: mjtNum,
        );
        #[doc = " Set res = vec1 + vec2."]
        pub(crate) fn mju_add3(
            res: *mut [mjtNum; 3usize],
            vec1: *const [mjtNum; 3usize],
            vec2: *const [mjtNum; 3usize],
        );
        #[doc = " Set res = vec1 - vec2."]
        pub(crate) fn mju_sub3(
            res: *mut [mjtNum; 3usize],
            vec1: *const [mjtNum; 3usize],
            vec2: *const [mjtNum; 3usize],
        );
        #[doc = " Set res = res + vec."]
        pub(crate) fn mju_addTo3(res: *mut [mjtNum; 3usize], vec: *const [mjtNum; 3usize]);
        #[doc = " Set res = res - vec."]
        pub(crate) fn mju_subFrom3(res: *mut [mjtNum; 3usize], vec: *const [mjtNum; 3usize]);
        #[doc = " Set res = res + vec*scl."]
        pub(crate) fn mju_addToScl3(
            res: *mut [mjtNum; 3usize],
            vec: *const [mjtNum; 3usize],
            scl: mjtNum,
        );
        #[doc = " Set res = vec1 + vec2*scl."]
        pub(crate) fn mju_addScl3(
            res: *mut [mjtNum; 3usize],
            vec1: *const [mjtNum; 3usize],
            vec2: *const [mjtNum; 3usize],
            scl: mjtNum,
        );
        #[doc = " Normalize vector; return length before normalization."]
        pub(crate) fn mju_normalize3(vec: *mut [mjtNum; 3usize]) -> mjtNum;
        #[doc = " Return vector length (without normalizing the vector)."]
        pub(crate) fn mju_norm3(vec: *const [mjtNum; 3usize]) -> mjtNum;
        #[doc = " Return dot-product of vec1 and vec2."]
        pub(crate) fn mju_dot3(
            vec1: *const [mjtNum; 3usize],
            vec2: *const [mjtNum; 3usize],
        ) -> mjtNum;
        #[doc = " Return Cartesian distance between 3D vectors pos1 and pos2."]
        pub(crate) fn mju_dist3(
            pos1: *const [mjtNum; 3usize],
            pos2: *const [mjtNum; 3usize],
        ) -> mjtNum;
        #[doc = " Multiply 3-by-3 matrix by vector: res = mat * vec."]
        pub(crate) fn mju_mulMatVec3(
            res: *mut [mjtNum; 3usize],
            mat: *const [mjtNum; 9usize],
            vec: *const [mjtNum; 3usize],
        );
        #[doc = " Multiply transposed 3-by-3 matrix by vector: res = mat' * vec."]
        pub(crate) fn mju_mulMatTVec3(
            res: *mut [mjtNum; 3usize],
            mat: *const [mjtNum; 9usize],
            vec: *const [mjtNum; 3usize],
        );
        #[doc = " Compute cross-product: res = cross(a, b)."]
        pub(crate) fn mju_cross(
            res: *mut [mjtNum; 3usize],
            a: *const [mjtNum; 3usize],
            b: *const [mjtNum; 3usize],
        );
        #[doc = " Set res = 0."]
        pub(crate) fn mju_zero4(res: *mut [mjtNum; 4usize]);
        #[doc = " Set res = (1,0,0,0)."]
        pub(crate) fn mju_unit4(res: *mut [mjtNum; 4usize]);
        #[doc = " Set res = vec."]
        pub(crate) fn mju_copy4(res: *mut [mjtNum; 4usize], data: *const [mjtNum; 4usize]);
        #[doc = " Normalize vector; return length before normalization."]
        pub(crate) fn mju_normalize4(vec: *mut [mjtNum; 4usize]) -> mjtNum;
        #[doc = " Set res = 0."]
        pub(crate) fn mju_zero(res: *mut mjtNum, n: ::core::ffi::c_int);
        #[doc = " Set res = val."]
        pub(crate) fn mju_fill(res: *mut mjtNum, val: mjtNum, n: ::core::ffi::c_int);
        #[doc = " Set res = vec."]
        pub(crate) fn mju_copy(res: *mut mjtNum, vec: *const mjtNum, n: ::core::ffi::c_int);
        #[doc = " Return sum(vec)."]
        pub(crate) fn mju_sum(vec: *const mjtNum, n: ::core::ffi::c_int) -> mjtNum;
        #[doc = " Return L1 norm: sum(abs(vec))."]
        pub(crate) fn mju_L1(vec: *const mjtNum, n: ::core::ffi::c_int) -> mjtNum;
        #[doc = " Set res = vec*scl."]
        pub(crate) fn mju_scl(
            res: *mut mjtNum,
            vec: *const mjtNum,
            scl: mjtNum,
            n: ::core::ffi::c_int,
        );
        #[doc = " Set res = vec1 + vec2."]
        pub(crate) fn mju_add(
            res: *mut mjtNum,
            vec1: *const mjtNum,
            vec2: *const mjtNum,
            n: ::core::ffi::c_int,
        );
        #[doc = " Set res = vec1 - vec2."]
        pub(crate) fn mju_sub(
            res: *mut mjtNum,
            vec1: *const mjtNum,
            vec2: *const mjtNum,
            n: ::core::ffi::c_int,
        );
        #[doc = " Set res = res + vec."]
        pub(crate) fn mju_addTo(res: *mut mjtNum, vec: *const mjtNum, n: ::core::ffi::c_int);
        #[doc = " Set res = res - vec."]
        pub(crate) fn mju_subFrom(res: *mut mjtNum, vec: *const mjtNum, n: ::core::ffi::c_int);
        #[doc = " Set res = res + vec*scl."]
        pub(crate) fn mju_addToScl(
            res: *mut mjtNum,
            vec: *const mjtNum,
            scl: mjtNum,
            n: ::core::ffi::c_int,
        );
        #[doc = " Set res = vec1 + vec2*scl."]
        pub(crate) fn mju_addScl(
            res: *mut mjtNum,
            vec1: *const mjtNum,
            vec2: *const mjtNum,
            scl: mjtNum,
            n: ::core::ffi::c_int,
        );
        #[doc = " Normalize vector; return length before normalization."]
        pub(crate) fn mju_normalize(res: *mut mjtNum, n: ::core::ffi::c_int) -> mjtNum;
        #[doc = " Return vector length (without normalizing vector)."]
        pub(crate) fn mju_norm(res: *const mjtNum, n: ::core::ffi::c_int) -> mjtNum;
        #[doc = " Return dot-product of vec1 and vec2."]
        pub(crate) fn mju_dot(
            vec1: *const mjtNum,
            vec2: *const mjtNum,
            n: ::core::ffi::c_int,
        ) -> mjtNum;
        #[doc = " Multiply matrix and vector: res = mat * vec."]
        pub(crate) fn mju_mulMatVec(
            res: *mut mjtNum,
            mat: *const mjtNum,
            vec: *const mjtNum,
            nr: ::core::ffi::c_int,
            nc: ::core::ffi::c_int,
        );
        #[doc = " Multiply transposed matrix and vector: res = mat' * vec."]
        pub(crate) fn mju_mulMatTVec(
            res: *mut mjtNum,
            mat: *const mjtNum,
            vec: *const mjtNum,
            nr: ::core::ffi::c_int,
            nc: ::core::ffi::c_int,
        );
        #[doc = " Multiply square matrix with vectors on both sides: return vec1' * mat * vec2."]
        pub(crate) fn mju_mulVecMatVec(
            vec1: *const mjtNum,
            mat: *const mjtNum,
            vec2: *const mjtNum,
            n: ::core::ffi::c_int,
        ) -> mjtNum;
        #[doc = " Transpose matrix: res = mat'."]
        pub(crate) fn mju_transpose(
            res: *mut mjtNum,
            mat: *const mjtNum,
            nr: ::core::ffi::c_int,
            nc: ::core::ffi::c_int,
        );
        #[doc = " Symmetrize square matrix res = (mat + mat')/2."]
        pub(crate) fn mju_symmetrize(res: *mut mjtNum, mat: *const mjtNum, n: ::core::ffi::c_int);
        #[doc = " Set mat to the identity matrix."]
        pub(crate) fn mju_eye(mat: *mut mjtNum, n: ::core::ffi::c_int);
        #[doc = " Multiply matrices: res = mat1 * mat2."]
        pub(crate) fn mju_mulMatMat(
            res: *mut mjtNum,
            mat1: *const mjtNum,
            mat2: *const mjtNum,
            r1: ::core::ffi::c_int,
            c1: ::core::ffi::c_int,
            c2: ::core::ffi::c_int,
        );
        #[doc = " Multiply matrices, second argument transposed: res = mat1 * mat2'."]
        pub(crate) fn mju_mulMatMatT(
            res: *mut mjtNum,
            mat1: *const mjtNum,
            mat2: *const mjtNum,
            r1: ::core::ffi::c_int,
            c1: ::core::ffi::c_int,
            r2: ::core::ffi::c_int,
        );
        #[doc = " Multiply matrices, first argument transposed: res = mat1' * mat2."]
        pub(crate) fn mju_mulMatTMat(
            res: *mut mjtNum,
            mat1: *const mjtNum,
            mat2: *const mjtNum,
            r1: ::core::ffi::c_int,
            c1: ::core::ffi::c_int,
            c2: ::core::ffi::c_int,
        );
        #[doc = " Set res = mat' * diag * mat if diag is not NULL, and res = mat' * mat otherwise."]
        pub(crate) fn mju_sqrMatTD(
            res: *mut mjtNum,
            mat: *const mjtNum,
            diag: *const mjtNum,
            nr: ::core::ffi::c_int,
            nc: ::core::ffi::c_int,
        );
        #[doc = " Coordinate transform of 6D motion or force vector in rotation:translation format.\n rotnew2old is 3-by-3, NULL means no rotation; flg_force specifies force or motion type.\n Nullable: rotnew2old"]
        pub(crate) fn mju_transformSpatial(
            res: *mut [mjtNum; 6usize],
            vec: *const [mjtNum; 6usize],
            flg_force: ::core::ffi::c_int,
            newpos: *const [mjtNum; 3usize],
            oldpos: *const [mjtNum; 3usize],
            rotnew2old: *const [mjtNum; 9usize],
        );
        #[doc = " Convert matrix from dense to sparse.\n  nnz is size of res and colind; return 1 if too small, 0 otherwise."]
        pub(crate) fn mju_dense2sparse(
            res: *mut mjtNum,
            mat: *const mjtNum,
            nr: ::core::ffi::c_int,
            nc: ::core::ffi::c_int,
            rownnz: *mut ::core::ffi::c_int,
            rowadr: *mut ::core::ffi::c_int,
            colind: *mut ::core::ffi::c_int,
            nnz: ::core::ffi::c_int,
        ) -> ::core::ffi::c_int;
        #[doc = " Convert matrix from sparse to dense."]
        pub(crate) fn mju_sparse2dense(
            res: *mut mjtNum,
            mat: *const mjtNum,
            nr: ::core::ffi::c_int,
            nc: ::core::ffi::c_int,
            rownnz: *const ::core::ffi::c_int,
            rowadr: *const ::core::ffi::c_int,
            colind: *const ::core::ffi::c_int,
        );
        #[doc = " Convert lower-triangular symmetric CSR matrix to full dense matrix."]
        pub(crate) fn mju_sym2dense(
            res: *mut mjtNum,
            mat: *const mjtNum,
            n: ::core::ffi::c_int,
            rownnz: *const ::core::ffi::c_int,
            rowadr: *const ::core::ffi::c_int,
            colind: *const ::core::ffi::c_int,
        );
        #[doc = " Rotate vector by quaternion."]
        pub(crate) fn mju_rotVecQuat(
            res: *mut [mjtNum; 3usize],
            vec: *const [mjtNum; 3usize],
            quat: *const [mjtNum; 4usize],
        );
        #[doc = " Conjugate quaternion, corresponding to opposite rotation."]
        pub(crate) fn mju_negQuat(res: *mut [mjtNum; 4usize], quat: *const [mjtNum; 4usize]);
        #[doc = " Multiply quaternions."]
        pub(crate) fn mju_mulQuat(
            res: *mut [mjtNum; 4usize],
            quat1: *const [mjtNum; 4usize],
            quat2: *const [mjtNum; 4usize],
        );
        #[doc = " Multiply quaternion and axis."]
        pub(crate) fn mju_mulQuatAxis(
            res: *mut [mjtNum; 4usize],
            quat: *const [mjtNum; 4usize],
            axis: *const [mjtNum; 3usize],
        );
        #[doc = " Convert axisAngle to quaternion."]
        pub(crate) fn mju_axisAngle2Quat(
            res: *mut [mjtNum; 4usize],
            axis: *const [mjtNum; 3usize],
            angle: mjtNum,
        );
        #[doc = " Convert quaternion (corresponding to orientation difference) to 3D velocity."]
        pub(crate) fn mju_quat2Vel(
            res: *mut [mjtNum; 3usize],
            quat: *const [mjtNum; 4usize],
            dt: mjtNum,
        );
        #[doc = " Subtract quaternions, express as 3D velocity: qb*quat(res) = qa."]
        pub(crate) fn mju_subQuat(
            res: *mut [mjtNum; 3usize],
            qa: *const [mjtNum; 4usize],
            qb: *const [mjtNum; 4usize],
        );
        #[doc = " Convert quaternion to 3D rotation matrix."]
        pub(crate) fn mju_quat2Mat(res: *mut [mjtNum; 9usize], quat: *const [mjtNum; 4usize]);
        #[doc = " Convert 3D rotation matrix to quaternion."]
        pub(crate) fn mju_mat2Quat(quat: *mut [mjtNum; 4usize], mat: *const [mjtNum; 9usize]);
        #[doc = " Compute time-derivative of quaternion, given 3D rotational velocity."]
        pub(crate) fn mju_derivQuat(
            res: *mut [mjtNum; 4usize],
            quat: *const [mjtNum; 4usize],
            vel: *const [mjtNum; 3usize],
        );
        #[doc = " Integrate quaternion given 3D angular velocity."]
        pub(crate) fn mju_quatIntegrate(
            quat: *mut [mjtNum; 4usize],
            vel: *const [mjtNum; 3usize],
            scale: mjtNum,
        );
        #[doc = " Construct quaternion performing rotation from z-axis to given vector."]
        pub(crate) fn mju_quatZ2Vec(quat: *mut [mjtNum; 4usize], vec: *const [mjtNum; 3usize]);
        #[doc = " Extract 3D rotation from an arbitrary 3x3 matrix by refining the input quaternion.\n Return the number of iterations required to converge."]
        pub(crate) fn mju_mat2Rot(
            quat: *mut [mjtNum; 4usize],
            mat: *const [mjtNum; 9usize],
        ) -> ::core::ffi::c_int;
        #[doc = " Convert sequence of Euler angles (radians) to quaternion.\n seq[0,1,2] must be in 'xyzXYZ', lower/upper-case mean intrinsic/extrinsic rotations."]
        pub(crate) fn mju_euler2Quat(
            quat: *mut [mjtNum; 4usize],
            euler: *const [mjtNum; 3usize],
            seq: *const ::core::ffi::c_char,
        );
        #[doc = " Multiply two poses."]
        pub(crate) fn mju_mulPose(
            posres: *mut [mjtNum; 3usize],
            quatres: *mut [mjtNum; 4usize],
            pos1: *const [mjtNum; 3usize],
            quat1: *const [mjtNum; 4usize],
            pos2: *const [mjtNum; 3usize],
            quat2: *const [mjtNum; 4usize],
        );
        #[doc = " Conjugate pose, corresponding to the opposite spatial transformation."]
        pub(crate) fn mju_negPose(
            posres: *mut [mjtNum; 3usize],
            quatres: *mut [mjtNum; 4usize],
            pos: *const [mjtNum; 3usize],
            quat: *const [mjtNum; 4usize],
        );
        #[doc = " Transform vector by pose."]
        pub(crate) fn mju_trnVecPose(
            res: *mut [mjtNum; 3usize],
            pos: *const [mjtNum; 3usize],
            quat: *const [mjtNum; 4usize],
            vec: *const [mjtNum; 3usize],
        );
        #[doc = " Cholesky decomposition: mat = L*L'; return rank, decomposition performed in-place into mat."]
        pub(crate) fn mju_cholFactor(
            mat: *mut mjtNum,
            n: ::core::ffi::c_int,
            mindiag: mjtNum,
        ) -> ::core::ffi::c_int;
        #[doc = " Solve (mat*mat') * res = vec, where mat is a Cholesky factor."]
        pub(crate) fn mju_cholSolve(
            res: *mut mjtNum,
            mat: *const mjtNum,
            vec: *const mjtNum,
            n: ::core::ffi::c_int,
        );
        #[doc = " Cholesky rank-one update: L*L' +/- x*x'; return rank."]
        pub(crate) fn mju_cholUpdate(
            mat: *mut mjtNum,
            x: *mut mjtNum,
            n: ::core::ffi::c_int,
            flg_plus: ::core::ffi::c_int,
        ) -> ::core::ffi::c_int;
        #[doc = " Band-dense Cholesky decomposition.\n  Return minimum value in the factorized diagonal, or 0 if rank-deficient.\n  mat has (ntotal-ndense) x nband + ndense x ntotal elements.\n  The first (ntotal-ndense) x nband store the band part, left of diagonal, inclusive.\n  The second ndense x ntotal store the band part as entire dense rows.\n  Add diagadd+diagmul*mat_ii to diagonal before factorization."]
        pub(crate) fn mju_cholFactorBand(
            mat: *mut mjtNum,
            ntotal: ::core::ffi::c_int,
            nband: ::core::ffi::c_int,
            ndense: ::core::ffi::c_int,
            diagadd: mjtNum,
            diagmul: mjtNum,
        ) -> mjtNum;
        #[doc = " Solve (mat*mat')*res = vec where mat is a band-dense Cholesky factor."]
        pub(crate) fn mju_cholSolveBand(
            res: *mut mjtNum,
            mat: *const mjtNum,
            vec: *const mjtNum,
            ntotal: ::core::ffi::c_int,
            nband: ::core::ffi::c_int,
            ndense: ::core::ffi::c_int,
        );
        #[doc = " Convert banded matrix to dense matrix, fill upper triangle if flg_sym>0."]
        pub(crate) fn mju_band2Dense(
            res: *mut mjtNum,
            mat: *const mjtNum,
            ntotal: ::core::ffi::c_int,
            nband: ::core::ffi::c_int,
            ndense: ::core::ffi::c_int,
            flg_sym: mjtBool,
        );
        #[doc = " Convert dense matrix to banded matrix."]
        pub(crate) fn mju_dense2Band(
            res: *mut mjtNum,
            mat: *const mjtNum,
            ntotal: ::core::ffi::c_int,
            nband: ::core::ffi::c_int,
            ndense: ::core::ffi::c_int,
        );
        #[doc = " Multiply band-diagonal matrix with nvec vectors, include upper triangle if flg_sym>0."]
        pub(crate) fn mju_bandMulMatVec(
            res: *mut mjtNum,
            mat: *const mjtNum,
            vec: *const mjtNum,
            ntotal: ::core::ffi::c_int,
            nband: ::core::ffi::c_int,
            ndense: ::core::ffi::c_int,
            nvec: ::core::ffi::c_int,
            flg_sym: mjtBool,
        );
        #[doc = " Address of diagonal element i in band-dense matrix representation."]
        pub(crate) fn mju_bandDiag(
            i: ::core::ffi::c_int,
            ntotal: ::core::ffi::c_int,
            nband: ::core::ffi::c_int,
            ndense: ::core::ffi::c_int,
        ) -> ::core::ffi::c_int;
        #[doc = " Eigenvalue decomposition of symmetric 3x3 matrix, mat = eigvec * diag(eigval) * eigvec'."]
        pub(crate) fn mju_eig3(
            eigval: *mut [mjtNum; 3usize],
            eigvec: *mut [mjtNum; 9usize],
            quat: *mut [mjtNum; 4usize],
            mat: *const [mjtNum; 9usize],
        ) -> ::core::ffi::c_int;
        #[doc = " minimize 0.5*x'*H*x + x'*g  s.t. lower <= x <= upper; return rank or -1 if failed\n   inputs:\n     n           - problem dimension\n     H           - SPD matrix                n*n\n     g           - bias vector               n\n     lower       - lower bounds              n\n     upper       - upper bounds              n\n     res         - solution warmstart        n\n   return value:\n     nfree <= n  - rank of unconstrained subspace, -1 if failure\n   outputs (required):\n     res         - solution                  n\n     R           - subspace Cholesky factor  nfree*nfree    allocated: n*(n+7)\n   outputs (optional):\n     index       - set of free dimensions    nfree          allocated: n\n   notes:\n     the initial value of res is used to warmstart the solver\n     R must have allocatd size n*(n+7), but only nfree*nfree values are used in output\n     index (if given) must have allocated size n, but only nfree values are used in output\n     only the lower triangles of H and R and are read from and written to, respectively\n     the convenience function mju_boxQPmalloc allocates the required data structures\n Nullable: index, lower, upper"]
        pub(crate) fn mju_boxQP(
            res: *mut mjtNum,
            R: *mut mjtNum,
            index: *mut ::core::ffi::c_int,
            H: *const mjtNum,
            g: *const mjtNum,
            n: ::core::ffi::c_int,
            lower: *const mjtNum,
            upper: *const mjtNum,
        ) -> ::core::ffi::c_int;
        #[doc = " allocate heap memory for box-constrained Quadratic Program\n   as in mju_boxQP, index, lower, and upper are optional\n   free all pointers with mju_free()"]
        pub(crate) fn mju_boxQPmalloc(
            res: *mut *mut mjtNum,
            R: *mut *mut mjtNum,
            index: *mut *mut ::core::ffi::c_int,
            H: *mut *mut mjtNum,
            g: *mut *mut mjtNum,
            n: ::core::ffi::c_int,
            lower: *mut *mut mjtNum,
            upper: *mut *mut mjtNum,
        );
        #[doc = " Muscle active force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax)."]
        pub(crate) fn mju_muscleGain(
            len: mjtNum,
            vel: mjtNum,
            lengthrange: *const [mjtNum; 2usize],
            acc0: mjtNum,
            prm: *const [mjtNum; 9usize],
        ) -> mjtNum;
        #[doc = " Muscle passive force, prm = (range[2], force, scale, lmin, lmax, vmax, fpmax, fvmax)."]
        pub(crate) fn mju_muscleBias(
            len: mjtNum,
            lengthrange: *const [mjtNum; 2usize],
            acc0: mjtNum,
            prm: *const [mjtNum; 9usize],
        ) -> mjtNum;
        #[doc = " Muscle activation dynamics, prm = (tau_act, tau_deact, smoothing_width)."]
        pub(crate) fn mju_muscleDynamics(
            ctrl: mjtNum,
            act: mjtNum,
            prm: *const [mjtNum; 3usize],
        ) -> mjtNum;
        #[doc = " Convert contact force to pyramid representation."]
        pub(crate) fn mju_encodePyramid(
            pyramid: *mut mjtNum,
            force: *const mjtNum,
            mu: *const mjtNum,
            dim: ::core::ffi::c_int,
        );
        #[doc = " Convert pyramid representation to contact force."]
        pub(crate) fn mju_decodePyramid(
            force: *mut mjtNum,
            pyramid: *const mjtNum,
            mu: *const mjtNum,
            dim: ::core::ffi::c_int,
        );
        #[doc = " Integrate spring-damper analytically; return pos(dt)."]
        pub(crate) fn mju_springDamper(
            pos0: mjtNum,
            vel0: mjtNum,
            Kp: mjtNum,
            Kv: mjtNum,
            dt: mjtNum,
        ) -> mjtNum;
        #[doc = " Return min(a,b) with single evaluation of a and b."]
        pub(crate) fn mju_min(a: mjtNum, b: mjtNum) -> mjtNum;
        #[doc = " Return max(a,b) with single evaluation of a and b."]
        pub(crate) fn mju_max(a: mjtNum, b: mjtNum) -> mjtNum;
        #[doc = " Clip x to the range [min, max]."]
        pub(crate) fn mju_clip(x: mjtNum, min: mjtNum, max: mjtNum) -> mjtNum;
        #[doc = " Return sign of x: +1, -1 or 0."]
        pub(crate) fn mju_sign(x: mjtNum) -> mjtNum;
        #[doc = " Round x to nearest integer."]
        pub(crate) fn mju_round(x: mjtNum) -> ::core::ffi::c_int;
        #[doc = " Convert type id (mjtObj) to type name."]
        pub(crate) fn mju_type2Str(type_: ::core::ffi::c_int) -> *const ::core::ffi::c_char;
        #[doc = " Convert type name to type id (mjtObj)."]
        pub(crate) fn mju_str2Type(str_: *const ::core::ffi::c_char) -> ::core::ffi::c_int;
        #[doc = " Return human readable number of bytes using standard letter suffix."]
        pub(crate) fn mju_writeNumBytes(nbytes: usize) -> *const ::core::ffi::c_char;
        #[doc = " Construct a warning message given the warning type and info."]
        pub(crate) fn mju_warningText(
            warning: ::core::ffi::c_int,
            info: usize,
        ) -> *const ::core::ffi::c_char;
        #[doc = " Return 1 if nan or abs(x)>mjMAXVAL, 0 otherwise. Used by check functions."]
        pub(crate) fn mju_isBad(x: mjtNum) -> ::core::ffi::c_int;
        #[doc = " Return 1 if all elements are 0."]
        pub(crate) fn mju_isZero(vec: *const mjtNum, n: ::core::ffi::c_int) -> ::core::ffi::c_int;
        #[doc = " Standard normal random number generator (optional second number)."]
        pub(crate) fn mju_standardNormal(num2: *mut mjtNum) -> mjtNum;
        #[doc = " Convert from float to mjtNum."]
        pub(crate) fn mju_f2n(res: *mut mjtNum, vec: *const f32, n: ::core::ffi::c_int);
        #[doc = " Convert from mjtNum to float."]
        pub(crate) fn mju_n2f(res: *mut f32, vec: *const mjtNum, n: ::core::ffi::c_int);
        #[doc = " Convert from double to mjtNum."]
        pub(crate) fn mju_d2n(res: *mut mjtNum, vec: *const f64, n: ::core::ffi::c_int);
        #[doc = " Convert from mjtNum to double."]
        pub(crate) fn mju_n2d(res: *mut f64, vec: *const mjtNum, n: ::core::ffi::c_int);
        #[doc = " Insertion sort, resulting list is in increasing order."]
        pub(crate) fn mju_insertionSort(list: *mut mjtNum, n: ::core::ffi::c_int);
        #[doc = " Integer insertion sort, resulting list is in increasing order."]
        pub(crate) fn mju_insertionSortInt(list: *mut ::core::ffi::c_int, n: ::core::ffi::c_int);
        #[doc = " Generate Halton sequence."]
        pub(crate) fn mju_Halton(index: ::core::ffi::c_int, base: ::core::ffi::c_int) -> mjtNum;
        #[doc = " Call strncpy, then set dst[n-1] = 0."]
        pub(crate) fn mju_strncpy(
            dst: *mut ::core::ffi::c_char,
            src: *const ::core::ffi::c_char,
            n: ::core::ffi::c_int,
        ) -> *mut ::core::ffi::c_char;
        #[doc = " Sigmoid function over 0<=x<=1 using quintic polynomial."]
        pub(crate) fn mju_sigmoid(x: mjtNum) -> mjtNum;
        #[doc = " get sdf from geom id"]
        pub(crate) fn mjc_getSDF(m: *const mjModel, id: ::core::ffi::c_int) -> *const mjpPlugin;
        #[doc = " signed distance function"]
        pub(crate) fn mjc_distance(
            m: *const mjModel,
            d: *const mjData,
            s: *const mjSDF,
            x: *const [mjtNum; 3usize],
        ) -> mjtNum;
        #[doc = " gradient of sdf"]
        pub(crate) fn mjc_gradient(
            m: *const mjModel,
            d: *const mjData,
            s: *const mjSDF,
            gradient: *mut [mjtNum; 3usize],
            x: *const [mjtNum; 3usize],
        );
        #[doc = " Finite differenced transition matrices (control theory notation)\n   d(x_next) = A*dx + B*du\n   d(sensor) = C*dx + D*du\n   required output matrix dimensions:\n      A: (2*nv+na x 2*nv+na)\n      B: (2*nv+na x nu)\n      D: (nsensordata x 2*nv+na)\n      C: (nsensordata x nu)\n Nullable: A, B, C, D"]
        pub(crate) fn mjd_transitionFD(
            m: *const mjModel,
            d: *mut mjData,
            eps: mjtNum,
            flg_centered: mjtBool,
            A: *mut mjtNum,
            B: *mut mjtNum,
            C: *mut mjtNum,
            D: *mut mjtNum,
        );
        #[doc = " Finite differenced Jacobians of (force, sensors) = mj_inverse(state, acceleration)\n   All outputs are optional. Output dimensions (transposed w.r.t Control Theory convention):\n     DfDq: (nv x nv)\n     DfDv: (nv x nv)\n     DfDa: (nv x nv)\n     DsDq: (nv x nsensordata)\n     DsDv: (nv x nsensordata)\n     DsDa: (nv x nsensordata)\n     DmDq: (nv x nC)\n   single-letter shortcuts:\n     inputs: q=qpos, v=qvel, a=qacc\n     outputs: f=qfrc_inverse, s=sensordata, m=M\n   notes:\n     optionally computes mass matrix Jacobian DmDq\n     flg_actuation specifies whether to subtract qfrc_actuator from qfrc_inverse\n Nullable: DfDq, DfDv, DfDa, DsDq, DsDv, DsDa, DmDq"]
        pub(crate) fn mjd_inverseFD(
            m: *const mjModel,
            d: *mut mjData,
            eps: mjtNum,
            flg_actuation: mjtBool,
            DfDq: *mut mjtNum,
            DfDv: *mut mjtNum,
            DfDa: *mut mjtNum,
            DsDq: *mut mjtNum,
            DsDv: *mut mjtNum,
            DsDa: *mut mjtNum,
            DmDq: *mut mjtNum,
        );
        #[doc = " Derivatives of mju_subQuat.\n Nullable: Da, Db"]
        pub(crate) fn mjd_subQuat(
            qa: *const [mjtNum; 4usize],
            qb: *const [mjtNum; 4usize],
            Da: *mut [mjtNum; 9usize],
            Db: *mut [mjtNum; 9usize],
        );
        #[doc = " Derivatives of mju_quatIntegrate.\n Nullable: Dquat, Dvel, Dscale"]
        pub(crate) fn mjd_quatIntegrate(
            vel: *const [mjtNum; 3usize],
            scale: mjtNum,
            Dquat: *mut [mjtNum; 9usize],
            Dvel: *mut [mjtNum; 9usize],
            Dscale: *mut [mjtNum; 3usize],
        );
        #[doc = " Set default plugin definition."]
        pub(crate) fn mjp_defaultPlugin(plugin: *mut mjpPlugin);
        #[doc = " Globally register a plugin. This function is thread-safe.\n If an identical mjpPlugin is already registered, this function does nothing.\n If a non-identical mjpPlugin with the same name is already registered, an mju_error is raised.\n Two mjpPlugins are considered identical if all member function pointers and numbers are equal,\n and the name and attribute strings are all identical, however the char pointers to the strings\n need not be the same."]
        pub(crate) fn mjp_registerPlugin(plugin: *const mjpPlugin) -> ::core::ffi::c_int;
        #[doc = " Return the number of globally registered plugins."]
        pub(crate) fn mjp_pluginCount() -> ::core::ffi::c_int;
        #[doc = " Look up a plugin by name. If slot is not NULL, also write its registered slot number into it."]
        pub(crate) fn mjp_getPlugin(
            name: *const ::core::ffi::c_char,
            slot: *mut ::core::ffi::c_int,
        ) -> *const mjpPlugin;
        #[doc = " Look up a plugin by the registered slot number that was returned by mjp_registerPlugin."]
        pub(crate) fn mjp_getPluginAtSlot(slot: ::core::ffi::c_int) -> *const mjpPlugin;
        #[doc = " Set default resource provider definition."]
        pub(crate) fn mjp_defaultResourceProvider(provider: *mut mjpResourceProvider);
        #[doc = " Globally register a resource provider in a thread-safe manner. The provider must have a prefix\n that is not a sub-prefix or super-prefix of any current registered providers.\n Return a slot number >= 0 on success, -1 on failure."]
        pub(crate) fn mjp_registerResourceProvider(
            provider: *const mjpResourceProvider,
        ) -> ::core::ffi::c_int;
        #[doc = " Return the number of globally registered resource providers."]
        pub(crate) fn mjp_resourceProviderCount() -> ::core::ffi::c_int;
        #[doc = " Return the resource provider with the prefix that matches against the resource name.\n If no match, return NULL."]
        pub(crate) fn mjp_getResourceProvider(
            resource_name: *const ::core::ffi::c_char,
        ) -> *const mjpResourceProvider;
        #[doc = " Look up a resource provider by slot number returned by mjp_registerResourceProvider.\n If invalid slot number, return NULL."]
        pub(crate) fn mjp_getResourceProviderAtSlot(
            slot: ::core::ffi::c_int,
        ) -> *const mjpResourceProvider;
        #[doc = " Globally register a decoder. This function is thread-safe.\n If an identical mjpDecoder is already registered, this function does nothing.\n If a non-identical mjpDecoder with the same name is already registered, an mju_error is raised."]
        pub(crate) fn mjp_registerDecoder(decoder: *const mjpDecoder);
        #[doc = " Set default resource decoder definition."]
        pub(crate) fn mjp_defaultDecoder(decoder: *mut mjpDecoder);
        #[doc = " Return the resource provider with the prefix that matches against the resource name.\n If no match, return NULL."]
        pub(crate) fn mjp_findDecoder(
            resource: *const mjResource,
            content_type: *const ::core::ffi::c_char,
        ) -> *const mjpDecoder;
        #[doc = " Globally register an encoder. This function is thread-safe.\n If an identical mjpEncoder is already registered, this function does nothing.\n If a non-identical mjpEncoder with the same name is already registered, an mju_error is raised."]
        pub(crate) fn mjp_registerEncoder(encoder: *const mjpEncoder);
        #[doc = " Set default resource encoder definition."]
        pub(crate) fn mjp_defaultEncoder(encoder: *mut mjpEncoder);
        #[doc = " Return the encoder that matches against the content type or filename extension.\n If no match, return NULL."]
        pub(crate) fn mjp_findEncoder(
            filename: *const ::core::ffi::c_char,
            content_type: *const ::core::ffi::c_char,
        ) -> *const mjpEncoder;
        #[doc = " Open a resource; if the name doesn't have a prefix matching a registered resource provider,\n then the OS filesystem is used.\n Nullable: dir, vfs, error"]
        pub(crate) fn mju_openResource(
            dir: *const ::core::ffi::c_char,
            name: *const ::core::ffi::c_char,
            vfs: *const mjVFS,
            error: *mut ::core::ffi::c_char,
            nerror: usize,
        ) -> *mut mjResource;
        #[doc = " Close a resource; no-op if resource is NULL."]
        pub(crate) fn mju_closeResource(resource: *mut mjResource);
        #[doc = " Set buffer to bytes read from the resource and return number of bytes in buffer;\n return negative value if error."]
        pub(crate) fn mju_readResource(
            resource: *mut mjResource,
            buffer: *mut *const ::core::ffi::c_void,
        ) -> ::core::ffi::c_int;
        #[doc = " Write resource data via its resource provider, return bytes written or -1 on error.\n Nullable: vfs, error"]
        pub(crate) fn mju_writeResource(
            name: *const ::core::ffi::c_char,
            buffer: *const ::core::ffi::c_void,
            nbytes: mjtSize,
            vfs: *const mjVFS,
            error: *mut ::core::ffi::c_char,
            nerror: usize,
        ) -> mjtSize;
        #[doc = " For a resource with a name partitioned as {dir}{filename}, get the dir and ndir pointers."]
        pub(crate) fn mju_getResourceDir(
            resource: *mut mjResource,
            dir: *mut *const ::core::ffi::c_char,
            ndir: *mut ::core::ffi::c_int,
        );
        #[doc = " Compare resource timestamp to provided timestamp.\n Return 0 if timestamps match, >0 if resource is newer, <0 if resource is older."]
        pub(crate) fn mju_isModifiedResource(
            resource: *const mjResource,
            timestamp: *const ::core::ffi::c_char,
        ) -> ::core::ffi::c_int;
        #[doc = " Find the decoder for a resource and return the decoded spec.\n The caller takes ownership of the spec and is responsible for cleaning it up.\n Nullable: vfs"]
        pub(crate) fn mju_decodeResource(
            resource: *mut mjResource,
            content_type: *const ::core::ffi::c_char,
            vfs: *const mjVFS,
        ) -> *mut mjSpec;
        #[doc = " Create a thread pool with nthread worker threads."]
        pub(crate) fn mju_threadpool(d: *mut mjData, nthread: ::core::ffi::c_int);
        #[doc = " Attach child to a parent; return the attached element if success or NULL otherwise."]
        pub(crate) fn mjs_attach(
            parent: *mut mjsElement,
            child: *const mjsElement,
            prefix: *const ::core::ffi::c_char,
            suffix: *const ::core::ffi::c_char,
        ) -> *mut mjsElement;
        #[doc = " Add child body to body; return child.\n Nullable: def"]
        pub(crate) fn mjs_addBody(body: *mut mjsBody, def: *const mjsDefault) -> *mut mjsBody;
        #[doc = " Add site to body; return site spec.\n Nullable: def"]
        pub(crate) fn mjs_addSite(body: *mut mjsBody, def: *const mjsDefault) -> *mut mjsSite;
        #[doc = " Add joint to body.\n Nullable: def"]
        pub(crate) fn mjs_addJoint(body: *mut mjsBody, def: *const mjsDefault) -> *mut mjsJoint;
        #[doc = " Add freejoint to body."]
        pub(crate) fn mjs_addFreeJoint(body: *mut mjsBody) -> *mut mjsJoint;
        #[doc = " Add geom to body.\n Nullable: def"]
        pub(crate) fn mjs_addGeom(body: *mut mjsBody, def: *const mjsDefault) -> *mut mjsGeom;
        #[doc = " Add camera to body.\n Nullable: def"]
        pub(crate) fn mjs_addCamera(body: *mut mjsBody, def: *const mjsDefault) -> *mut mjsCamera;
        #[doc = " Add light to body.\n Nullable: def"]
        pub(crate) fn mjs_addLight(body: *mut mjsBody, def: *const mjsDefault) -> *mut mjsLight;
        #[doc = " Add frame to body."]
        pub(crate) fn mjs_addFrame(body: *mut mjsBody, parentframe: *mut mjsFrame)
        -> *mut mjsFrame;
        #[doc = " Remove object corresponding to the given element; return 0 on success."]
        pub(crate) fn mjs_delete(spec: *mut mjSpec, element: *mut mjsElement)
        -> ::core::ffi::c_int;
        #[doc = " Add actuator.\n Nullable: def"]
        pub(crate) fn mjs_addActuator(s: *mut mjSpec, def: *const mjsDefault) -> *mut mjsActuator;
        #[doc = " Add sensor."]
        pub(crate) fn mjs_addSensor(s: *mut mjSpec) -> *mut mjsSensor;
        #[doc = " Add flex."]
        pub(crate) fn mjs_addFlex(s: *mut mjSpec) -> *mut mjsFlex;
        #[doc = " Add flexcomp: create flex with auto-generated bodies/joints, return flex spec.\n Nullable: type, dof, count, cellcount, spacing, scale, pos, quat, origin, file, vfs"]
        pub(crate) fn mjs_makeFlex(
            body: *mut mjsBody,
            name: *const ::core::ffi::c_char,
            type_: *const ::core::ffi::c_char,
            dim: ::core::ffi::c_int,
            dof: *const ::core::ffi::c_char,
            count: *const [::core::ffi::c_int; 3usize],
            cellcount: *const [::core::ffi::c_int; 3usize],
            spacing: *const [f64; 3usize],
            scale: *const [f64; 3usize],
            radius: f64,
            mass: f64,
            inertiabox: f64,
            equality: ::core::ffi::c_int,
            rigid: ::core::ffi::c_int,
            flatskin: ::core::ffi::c_int,
            elastic2d: ::core::ffi::c_int,
            pos: *const [f64; 3usize],
            quat: *const [f64; 4usize],
            origin: *const [f64; 3usize],
            file: *const ::core::ffi::c_char,
            vfs: *const mjVFS,
        ) -> *mut mjsFlex;
        #[doc = " Add contact pair.\n Nullable: def"]
        pub(crate) fn mjs_addPair(s: *mut mjSpec, def: *const mjsDefault) -> *mut mjsPair;
        #[doc = " Add excluded body pair."]
        pub(crate) fn mjs_addExclude(s: *mut mjSpec) -> *mut mjsExclude;
        #[doc = " Add equality.\n Nullable: def"]
        pub(crate) fn mjs_addEquality(s: *mut mjSpec, def: *const mjsDefault) -> *mut mjsEquality;
        #[doc = " Add tendon.\n Nullable: def"]
        pub(crate) fn mjs_addTendon(s: *mut mjSpec, def: *const mjsDefault) -> *mut mjsTendon;
        #[doc = " Wrap site using tendon."]
        pub(crate) fn mjs_wrapSite(
            tendon: *mut mjsTendon,
            name: *const ::core::ffi::c_char,
        ) -> *mut mjsWrap;
        #[doc = " Wrap geom using tendon."]
        pub(crate) fn mjs_wrapGeom(
            tendon: *mut mjsTendon,
            name: *const ::core::ffi::c_char,
            sidesite: *const ::core::ffi::c_char,
        ) -> *mut mjsWrap;
        #[doc = " Wrap joint using tendon."]
        pub(crate) fn mjs_wrapJoint(
            tendon: *mut mjsTendon,
            name: *const ::core::ffi::c_char,
            coef: f64,
        ) -> *mut mjsWrap;
        #[doc = " Wrap pulley using tendon."]
        pub(crate) fn mjs_wrapPulley(tendon: *mut mjsTendon, divisor: f64) -> *mut mjsWrap;
        #[doc = " Add numeric."]
        pub(crate) fn mjs_addNumeric(s: *mut mjSpec) -> *mut mjsNumeric;
        #[doc = " Add text."]
        pub(crate) fn mjs_addText(s: *mut mjSpec) -> *mut mjsText;
        #[doc = " Add tuple."]
        pub(crate) fn mjs_addTuple(s: *mut mjSpec) -> *mut mjsTuple;
        #[doc = " Add keyframe."]
        pub(crate) fn mjs_addKey(s: *mut mjSpec) -> *mut mjsKey;
        #[doc = " Add plugin."]
        pub(crate) fn mjs_addPlugin(s: *mut mjSpec) -> *mut mjsPlugin;
        #[doc = " Add default.\n Nullable: parent"]
        pub(crate) fn mjs_addDefault(
            s: *mut mjSpec,
            classname: *const ::core::ffi::c_char,
            parent: *const mjsDefault,
        ) -> *mut mjsDefault;
        #[doc = " Set actuator to motor; return error if any."]
        pub(crate) fn mjs_setToMotor(actuator: *mut mjsActuator) -> *const ::core::ffi::c_char;
        #[doc = " Set actuator to position; return error if any."]
        pub(crate) fn mjs_setToPosition(
            actuator: *mut mjsActuator,
            kp: f64,
            kv: *mut [f64; 1usize],
            dampratio: *mut [f64; 1usize],
            timeconst: *mut [f64; 1usize],
            inheritrange: f64,
        ) -> *const ::core::ffi::c_char;
        #[doc = " Set actuator to integrated velocity; return error if any."]
        pub(crate) fn mjs_setToIntVelocity(
            actuator: *mut mjsActuator,
            kp: f64,
            kv: *mut [f64; 1usize],
            dampratio: *mut [f64; 1usize],
            timeconst: *mut [f64; 1usize],
            inheritrange: f64,
        ) -> *const ::core::ffi::c_char;
        #[doc = " Set actuator to velocity servo; return error if any."]
        pub(crate) fn mjs_setToVelocity(
            actuator: *mut mjsActuator,
            kv: f64,
        ) -> *const ::core::ffi::c_char;
        #[doc = " Set actuator to orientation servo."]
        pub(crate) fn mjs_setToOrientation(
            actuator: *mut mjsActuator,
            kp: f64,
            kv: *mut [f64; 1usize],
            dampratio: *mut [f64; 1usize],
            ctrlspec: ::core::ffi::c_int,
        ) -> *const ::core::ffi::c_char;
        #[doc = " Set actuator to activate damper; return error if any."]
        pub(crate) fn mjs_setToDamper(
            actuator: *mut mjsActuator,
            kv: f64,
        ) -> *const ::core::ffi::c_char;
        #[doc = " Set actuator to hydraulic or pneumatic cylinder; return error if any."]
        pub(crate) fn mjs_setToCylinder(
            actuator: *mut mjsActuator,
            timeconst: f64,
            bias: f64,
            area: f64,
            diameter: f64,
        ) -> *const ::core::ffi::c_char;
        #[doc = " Set actuator to muscle; return error if any.a"]
        pub(crate) fn mjs_setToMuscle(
            actuator: *mut mjsActuator,
            timeconst: *mut [f64; 2usize],
            tausmooth: f64,
            range: *mut [f64; 2usize],
            force: f64,
            scale: f64,
            lmin: f64,
            lmax: f64,
            vmax: f64,
            fpmax: f64,
            fvmax: f64,
        ) -> *const ::core::ffi::c_char;
        #[doc = " Set actuator to active adhesion; return error if any."]
        pub(crate) fn mjs_setToAdhesion(
            actuator: *mut mjsActuator,
            gain: f64,
        ) -> *const ::core::ffi::c_char;
        #[doc = " Set actuator to DC motor; return error if any.\n Nullable: motorconst, nominal, saturation, inductance, cogging, controller, thermal, lugre"]
        pub(crate) fn mjs_setToDCMotor(
            actuator: *mut mjsActuator,
            motorconst: *mut [f64; 2usize],
            resistance: f64,
            nominal: *mut [f64; 3usize],
            saturation: *mut [f64; 3usize],
            inductance: *mut [f64; 2usize],
            cogging: *mut [f64; 3usize],
            controller: *mut [f64; 6usize],
            thermal: *mut [f64; 6usize],
            lugre: *mut [f64; 5usize],
            input_mode: ::core::ffi::c_int,
        ) -> *const ::core::ffi::c_char;
        #[doc = " Add mesh.\n Nullable: def"]
        pub(crate) fn mjs_addMesh(s: *mut mjSpec, def: *const mjsDefault) -> *mut mjsMesh;
        #[doc = " Add height field."]
        pub(crate) fn mjs_addHField(s: *mut mjSpec) -> *mut mjsHField;
        #[doc = " Add skin."]
        pub(crate) fn mjs_addSkin(s: *mut mjSpec) -> *mut mjsSkin;
        #[doc = " Add texture."]
        pub(crate) fn mjs_addTexture(s: *mut mjSpec) -> *mut mjsTexture;
        #[doc = " Add material.\n Nullable: def"]
        pub(crate) fn mjs_addMaterial(s: *mut mjSpec, def: *const mjsDefault) -> *mut mjsMaterial;
        #[doc = " Sets the vertices and normals of a mesh."]
        pub(crate) fn mjs_makeMesh(
            mesh: *mut mjsMesh,
            builtin: mjtMeshBuiltin,
            params: *mut f64,
            nparams: ::core::ffi::c_int,
        ) -> ::core::ffi::c_int;
        #[doc = " Get spec from body."]
        pub(crate) fn mjs_getSpec(element: *const mjsElement) -> *mut mjSpec;
        #[doc = " get spec that originally defined an element\n contrary to mjs_getSpec, this does not change after attachment"]
        pub(crate) fn mjs_getOriginSpec(element: *const mjsElement) -> *mut mjSpec;
        #[doc = " Get compiler associated with element's origin spec."]
        pub(crate) fn mjs_getCompiler(element: *const mjsElement) -> *mut mjsCompiler;
        #[doc = " Find spec (model asset) by name."]
        pub(crate) fn mjs_findSpec(
            spec: *const mjSpec,
            name: *const ::core::ffi::c_char,
        ) -> *mut mjSpec;
        #[doc = " Find body in spec by name."]
        pub(crate) fn mjs_findBody(
            s: *const mjSpec,
            name: *const ::core::ffi::c_char,
        ) -> *mut mjsBody;
        #[doc = " Find element in spec by name."]
        pub(crate) fn mjs_findElement(
            s: *const mjSpec,
            type_: mjtObj,
            name: *const ::core::ffi::c_char,
        ) -> *mut mjsElement;
        #[doc = " Find child body by name."]
        pub(crate) fn mjs_findChild(
            body: *const mjsBody,
            name: *const ::core::ffi::c_char,
        ) -> *mut mjsBody;
        #[doc = " Get parent body."]
        pub(crate) fn mjs_getParent(element: *const mjsElement) -> *mut mjsBody;
        #[doc = " Get parent frame."]
        pub(crate) fn mjs_getFrame(element: *const mjsElement) -> *mut mjsFrame;
        #[doc = " Find frame by name."]
        pub(crate) fn mjs_findFrame(
            s: *const mjSpec,
            name: *const ::core::ffi::c_char,
        ) -> *mut mjsFrame;
        #[doc = " Get default corresponding to an element."]
        pub(crate) fn mjs_getDefault(element: *const mjsElement) -> *mut mjsDefault;
        #[doc = " Find default in model by class name."]
        pub(crate) fn mjs_findDefault(
            s: *const mjSpec,
            classname: *const ::core::ffi::c_char,
        ) -> *mut mjsDefault;
        #[doc = " Get global default from model."]
        pub(crate) fn mjs_getSpecDefault(s: *const mjSpec) -> *mut mjsDefault;
        #[doc = " Get element id."]
        pub(crate) fn mjs_getId(element: *const mjsElement) -> ::core::ffi::c_int;
        #[doc = " Return body's first child of given type. If recurse is nonzero, also search the body's subtree."]
        pub(crate) fn mjs_firstChild(
            body: *const mjsBody,
            type_: mjtObj,
            recurse: ::core::ffi::c_int,
        ) -> *mut mjsElement;
        #[doc = " Return body's next child of the same type; return NULL if child is last.\n If recurse is nonzero, also search the body's subtree."]
        pub(crate) fn mjs_nextChild(
            body: *const mjsBody,
            child: *const mjsElement,
            recurse: ::core::ffi::c_int,
        ) -> *mut mjsElement;
        #[doc = " Return spec's first element of selected type."]
        pub(crate) fn mjs_firstElement(s: *const mjSpec, type_: mjtObj) -> *mut mjsElement;
        #[doc = " Return spec's next element; return NULL if element is last."]
        pub(crate) fn mjs_nextElement(
            s: *const mjSpec,
            element: *const mjsElement,
        ) -> *mut mjsElement;
        #[doc = " Get wrapped element in tendon path."]
        pub(crate) fn mjs_getWrapTarget(wrap: *const mjsWrap) -> *mut mjsElement;
        #[doc = " Get wrapped element side site in tendon path if it has one, nullptr otherwise."]
        pub(crate) fn mjs_getWrapSideSite(wrap: *const mjsWrap) -> *mut mjsSite;
        #[doc = " Get divisor of mjsWrap wrapping a puller."]
        pub(crate) fn mjs_getWrapDivisor(wrap: *const mjsWrap) -> f64;
        #[doc = " Get coefficient of mjsWrap wrapping a joint."]
        pub(crate) fn mjs_getWrapCoef(wrap: *const mjsWrap) -> f64;
        #[doc = " Set element's name; return 0 on success."]
        pub(crate) fn mjs_setName(
            element: *mut mjsElement,
            name: *const ::core::ffi::c_char,
        ) -> ::core::ffi::c_int;
        #[doc = " Copy buffer."]
        pub(crate) fn mjs_setBuffer(
            dest: *mut mjByteVec,
            array: *const ::core::ffi::c_void,
            size: ::core::ffi::c_int,
        );
        #[doc = " Copy text to string."]
        pub(crate) fn mjs_setString(dest: *mut mjString, text: *const ::core::ffi::c_char);
        #[doc = " Split text to entries and copy to string vector."]
        pub(crate) fn mjs_setStringVec(dest: *mut mjStringVec, text: *const ::core::ffi::c_char);
        #[doc = " Set entry in string vector."]
        pub(crate) fn mjs_setInStringVec(
            dest: *mut mjStringVec,
            i: ::core::ffi::c_int,
            text: *const ::core::ffi::c_char,
        ) -> mjtBool;
        #[doc = " Append text entry to string vector."]
        pub(crate) fn mjs_appendString(dest: *mut mjStringVec, text: *const ::core::ffi::c_char);
        #[doc = " Copy int array to vector."]
        pub(crate) fn mjs_setInt(
            dest: *mut mjIntVec,
            array: *const ::core::ffi::c_int,
            size: ::core::ffi::c_int,
        );
        #[doc = " Append int array to vector of arrays."]
        pub(crate) fn mjs_appendIntVec(
            dest: *mut mjIntVecVec,
            array: *const ::core::ffi::c_int,
            size: ::core::ffi::c_int,
        );
        #[doc = " Copy float array to vector."]
        pub(crate) fn mjs_setFloat(
            dest: *mut mjFloatVec,
            array: *const f32,
            size: ::core::ffi::c_int,
        );
        #[doc = " Append float array to vector of arrays."]
        pub(crate) fn mjs_appendFloatVec(
            dest: *mut mjFloatVecVec,
            array: *const f32,
            size: ::core::ffi::c_int,
        );
        #[doc = " Copy double array to vector."]
        pub(crate) fn mjs_setDouble(
            dest: *mut mjDoubleVec,
            array: *const f64,
            size: ::core::ffi::c_int,
        );
        #[doc = " Set plugin attributes."]
        pub(crate) fn mjs_setPluginAttributes(
            plugin: *mut mjsPlugin,
            attributes: *mut ::core::ffi::c_void,
        );
        #[doc = " Get element's name."]
        pub(crate) fn mjs_getName(element: *mut mjsElement) -> *mut mjString;
        #[doc = " Get string contents."]
        pub(crate) fn mjs_getString(source: *const mjString) -> *const ::core::ffi::c_char;
        #[doc = " Get double array contents and optionally its size.\n Nullable: size"]
        pub(crate) fn mjs_getDouble(
            source: *const mjDoubleVec,
            size: *mut ::core::ffi::c_int,
        ) -> *const f64;
        #[doc = " Get number of elements a tendon wraps."]
        pub(crate) fn mjs_getWrapNum(tendonspec: *const mjsTendon) -> ::core::ffi::c_int;
        #[doc = " Get mjsWrap element at position i in the tendon path."]
        pub(crate) fn mjs_getWrap(
            tendonspec: *const mjsTendon,
            i: ::core::ffi::c_int,
        ) -> *mut mjsWrap;
        #[doc = " Get plugin attributes."]
        pub(crate) fn mjs_getPluginAttributes(
            plugin: *const mjsPlugin,
        ) -> *const ::core::ffi::c_void;
        #[doc = " Set element's default."]
        pub(crate) fn mjs_setDefault(element: *mut mjsElement, def: *const mjsDefault);
        #[doc = " Set element's enclosing frame; return 0 on success."]
        pub(crate) fn mjs_setFrame(
            dest: *mut mjsElement,
            frame: *mut mjsFrame,
        ) -> ::core::ffi::c_int;
        #[doc = " Resolve alternative orientations to quat; return error if any."]
        pub(crate) fn mjs_resolveOrientation(
            quat: *mut [f64; 4usize],
            degree: mjtByte,
            sequence: *const ::core::ffi::c_char,
            orientation: *const mjsOrientation,
        ) -> *const ::core::ffi::c_char;
        #[doc = " Transform body into a frame."]
        pub(crate) fn mjs_bodyToFrame(body: *mut *mut mjsBody) -> *mut mjsFrame;
        #[doc = " Set user payload, overriding the existing value for the specified key if present."]
        pub(crate) fn mjs_setUserValue(
            element: *mut mjsElement,
            key: *const ::core::ffi::c_char,
            data: *const ::core::ffi::c_void,
        );
        #[doc = " Set user payload, overriding the existing value for the specified key if\n present. This version differs from mjs_setUserValue in that it takes a\n cleanup function that will be called when the user payload is deleted."]
        pub(crate) fn mjs_setUserValueWithCleanup(
            element: *mut mjsElement,
            key: *const ::core::ffi::c_char,
            data: *const ::core::ffi::c_void,
            cleanup: ::core::option::Option<unsafe extern "C" fn(arg1: *const ::core::ffi::c_void)>,
        );
        #[doc = " Return user payload or NULL if none found."]
        pub(crate) fn mjs_getUserValue(
            element: *mut mjsElement,
            key: *const ::core::ffi::c_char,
        ) -> *const ::core::ffi::c_void;
        #[doc = " Delete user payload."]
        pub(crate) fn mjs_deleteUserValue(
            element: *mut mjsElement,
            key: *const ::core::ffi::c_char,
        );
        #[doc = " Return sensor dimension."]
        pub(crate) fn mjs_sensorDim(sensor: *const mjsSensor) -> ::core::ffi::c_int;
        #[doc = " Default spec attributes."]
        pub(crate) fn mjs_defaultSpec(spec: *mut mjSpec);
        #[doc = " Default orientation attributes."]
        pub(crate) fn mjs_defaultOrientation(orient: *mut mjsOrientation);
        #[doc = " Default body attributes."]
        pub(crate) fn mjs_defaultBody(body: *mut mjsBody);
        #[doc = " Default frame attributes."]
        pub(crate) fn mjs_defaultFrame(frame: *mut mjsFrame);
        #[doc = " Default joint attributes."]
        pub(crate) fn mjs_defaultJoint(joint: *mut mjsJoint);
        #[doc = " Default geom attributes."]
        pub(crate) fn mjs_defaultGeom(geom: *mut mjsGeom);
        #[doc = " Default site attributes."]
        pub(crate) fn mjs_defaultSite(site: *mut mjsSite);
        #[doc = " Default camera attributes."]
        pub(crate) fn mjs_defaultCamera(camera: *mut mjsCamera);
        #[doc = " Default light attributes."]
        pub(crate) fn mjs_defaultLight(light: *mut mjsLight);
        #[doc = " Default flex attributes."]
        pub(crate) fn mjs_defaultFlex(flex: *mut mjsFlex);
        #[doc = " Default mesh attributes."]
        pub(crate) fn mjs_defaultMesh(mesh: *mut mjsMesh);
        #[doc = " Default height field attributes."]
        pub(crate) fn mjs_defaultHField(hfield: *mut mjsHField);
        #[doc = " Default skin attributes."]
        pub(crate) fn mjs_defaultSkin(skin: *mut mjsSkin);
        #[doc = " Default texture attributes."]
        pub(crate) fn mjs_defaultTexture(texture: *mut mjsTexture);
        #[doc = " Default material attributes."]
        pub(crate) fn mjs_defaultMaterial(material: *mut mjsMaterial);
        #[doc = " Default pair attributes."]
        pub(crate) fn mjs_defaultPair(pair: *mut mjsPair);
        #[doc = " Default equality attributes."]
        pub(crate) fn mjs_defaultEquality(equality: *mut mjsEquality);
        #[doc = " Default tendon attributes."]
        pub(crate) fn mjs_defaultTendon(tendon: *mut mjsTendon);
        #[doc = " Default actuator attributes."]
        pub(crate) fn mjs_defaultActuator(actuator: *mut mjsActuator);
        #[doc = " Default sensor attributes."]
        pub(crate) fn mjs_defaultSensor(sensor: *mut mjsSensor);
        #[doc = " Default numeric attributes."]
        pub(crate) fn mjs_defaultNumeric(numeric: *mut mjsNumeric);
        #[doc = " Default text attributes."]
        pub(crate) fn mjs_defaultText(text: *mut mjsText);
        #[doc = " Default tuple attributes."]
        pub(crate) fn mjs_defaultTuple(tuple: *mut mjsTuple);
        #[doc = " Default keyframe attributes."]
        pub(crate) fn mjs_defaultKey(key: *mut mjsKey);
        #[doc = " Default plugin attributes."]
        pub(crate) fn mjs_defaultPlugin(plugin: *mut mjsPlugin);
        #[doc = " Safely cast an element as mjsBody, or return NULL if the element is not an mjsBody."]
        pub(crate) fn mjs_asBody(element: *mut mjsElement) -> *mut mjsBody;
        #[doc = " Safely cast an element as mjsGeom, or return NULL if the element is not an mjsGeom."]
        pub(crate) fn mjs_asGeom(element: *mut mjsElement) -> *mut mjsGeom;
        #[doc = " Safely cast an element as mjsJoint, or return NULL if the element is not an mjsJoint."]
        pub(crate) fn mjs_asJoint(element: *mut mjsElement) -> *mut mjsJoint;
        #[doc = " Safely cast an element as mjsSite, or return NULL if the element is not an mjsSite."]
        pub(crate) fn mjs_asSite(element: *mut mjsElement) -> *mut mjsSite;
        #[doc = " Safely cast an element as mjsCamera, or return NULL if the element is not an mjsCamera."]
        pub(crate) fn mjs_asCamera(element: *mut mjsElement) -> *mut mjsCamera;
        #[doc = " Safely cast an element as mjsLight, or return NULL if the element is not an mjsLight."]
        pub(crate) fn mjs_asLight(element: *mut mjsElement) -> *mut mjsLight;
        #[doc = " Safely cast an element as mjsFrame, or return NULL if the element is not an mjsFrame."]
        pub(crate) fn mjs_asFrame(element: *mut mjsElement) -> *mut mjsFrame;
        #[doc = " Safely cast an element as mjsActuator, or return NULL if the element is not an mjsActuator."]
        pub(crate) fn mjs_asActuator(element: *mut mjsElement) -> *mut mjsActuator;
        #[doc = " Safely cast an element as mjsSensor, or return NULL if the element is not an mjsSensor."]
        pub(crate) fn mjs_asSensor(element: *mut mjsElement) -> *mut mjsSensor;
        #[doc = " Safely cast an element as mjsFlex, or return NULL if the element is not an mjsFlex."]
        pub(crate) fn mjs_asFlex(element: *mut mjsElement) -> *mut mjsFlex;
        #[doc = " Safely cast an element as mjsPair, or return NULL if the element is not an mjsPair."]
        pub(crate) fn mjs_asPair(element: *mut mjsElement) -> *mut mjsPair;
        #[doc = " Safely cast an element as mjsEquality, or return NULL if the element is not an mjsEquality."]
        pub(crate) fn mjs_asEquality(element: *mut mjsElement) -> *mut mjsEquality;
        #[doc = " Safely cast an element as mjsExclude, or return NULL if the element is not an mjsExclude."]
        pub(crate) fn mjs_asExclude(element: *mut mjsElement) -> *mut mjsExclude;
        #[doc = " Safely cast an element as mjsTendon, or return NULL if the element is not an mjsTendon."]
        pub(crate) fn mjs_asTendon(element: *mut mjsElement) -> *mut mjsTendon;
        #[doc = " Safely cast an element as mjsNumeric, or return NULL if the element is not an mjsNumeric."]
        pub(crate) fn mjs_asNumeric(element: *mut mjsElement) -> *mut mjsNumeric;
        #[doc = " Safely cast an element as mjsText, or return NULL if the element is not an mjsText."]
        pub(crate) fn mjs_asText(element: *mut mjsElement) -> *mut mjsText;
        #[doc = " Safely cast an element as mjsTuple, or return NULL if the element is not an mjsTuple."]
        pub(crate) fn mjs_asTuple(element: *mut mjsElement) -> *mut mjsTuple;
        #[doc = " Safely cast an element as mjsKey, or return NULL if the element is not an mjsKey."]
        pub(crate) fn mjs_asKey(element: *mut mjsElement) -> *mut mjsKey;
        #[doc = " Safely cast an element as mjsMesh, or return NULL if the element is not an mjsMesh."]
        pub(crate) fn mjs_asMesh(element: *mut mjsElement) -> *mut mjsMesh;
        #[doc = " Safely cast an element as mjsHField, or return NULL if the element is not an mjsHField."]
        pub(crate) fn mjs_asHField(element: *mut mjsElement) -> *mut mjsHField;
        #[doc = " Safely cast an element as mjsSkin, or return NULL if the element is not an mjsSkin."]
        pub(crate) fn mjs_asSkin(element: *mut mjsElement) -> *mut mjsSkin;
        #[doc = " Safely cast an element as mjsTexture, or return NULL if the element is not an mjsTexture."]
        pub(crate) fn mjs_asTexture(element: *mut mjsElement) -> *mut mjsTexture;
        #[doc = " Safely cast an element as mjsMaterial, or return NULL if the element is not an mjsMaterial."]
        pub(crate) fn mjs_asMaterial(element: *mut mjsElement) -> *mut mjsMaterial;
        #[doc = " Safely cast an element as mjsPlugin, or return NULL if the element is not an mjsPlugin."]
        pub(crate) fn mjs_asPlugin(element: *mut mjsElement) -> *mut mjsPlugin;
    }
}
