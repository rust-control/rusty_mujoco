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
pub type mjtByte = ::core::ffi::c_uchar;
pub type mjtBool = bool;
pub type mjtSize = i64;
pub const mjNDISABLE: usize = mjtDisableBit::mjNDISABLE.0 as usize;
impl mjtDisableBit {
    pub const CONSTRAINT: mjtDisableBit = mjtDisableBit(1);
    pub const EQUALITY: mjtDisableBit = mjtDisableBit(2);
    pub const FRICTIONLOSS: mjtDisableBit = mjtDisableBit(4);
    pub const LIMIT: mjtDisableBit = mjtDisableBit(8);
    pub const CONTACT: mjtDisableBit = mjtDisableBit(16);
    pub const SPRING: mjtDisableBit = mjtDisableBit(32);
    pub const DAMPER: mjtDisableBit = mjtDisableBit(64);
    pub const GRAVITY: mjtDisableBit = mjtDisableBit(128);
    pub const CLAMPCTRL: mjtDisableBit = mjtDisableBit(256);
    pub const WARMSTART: mjtDisableBit = mjtDisableBit(512);
    pub const FILTERPARENT: mjtDisableBit = mjtDisableBit(1024);
    pub const ACTUATION: mjtDisableBit = mjtDisableBit(2048);
    pub const REFSAFE: mjtDisableBit = mjtDisableBit(4096);
    pub const SENSOR: mjtDisableBit = mjtDisableBit(8192);
    pub const MIDPHASE: mjtDisableBit = mjtDisableBit(16384);
    pub const EULERDAMP: mjtDisableBit = mjtDisableBit(32768);
    pub const AUTORESET: mjtDisableBit = mjtDisableBit(65536);
    pub const NATIVECCD: mjtDisableBit = mjtDisableBit(131072);
    pub const ISLAND: mjtDisableBit = mjtDisableBit(262144);
    pub const MULTICCD: mjtDisableBit = mjtDisableBit(524288);
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
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtDisableBit(::core::ffi::c_uint);
pub const mjNENABLE: usize = mjtEnableBit::mjNENABLE.0 as usize;
impl mjtEnableBit {
    pub const OVERRIDE: mjtEnableBit = mjtEnableBit(1);
    pub const ENERGY: mjtEnableBit = mjtEnableBit(2);
    pub const FWDINV: mjtEnableBit = mjtEnableBit(4);
    pub const INVDISCRETE: mjtEnableBit = mjtEnableBit(8);
    pub const SLEEP: mjtEnableBit = mjtEnableBit(16);
    pub const DIAGEXACT: mjtEnableBit = mjtEnableBit(32);
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
pub struct mjtEnableBit(::core::ffi::c_uint);
impl mjtJoint {
    pub const FREE: mjtJoint = mjtJoint(0);
    pub const BALL: mjtJoint = mjtJoint(1);
    pub const SLIDE: mjtJoint = mjtJoint(2);
    pub const HINGE: mjtJoint = mjtJoint(3);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtJoint(::core::ffi::c_uint);
pub const mjNGEOMTYPES: usize = mjtGeom::mjNGEOMTYPES.0 as usize;
impl mjtGeom {
    pub const PLANE: mjtGeom = mjtGeom(0);
    pub const HFIELD: mjtGeom = mjtGeom(1);
    pub const SPHERE: mjtGeom = mjtGeom(2);
    pub const CAPSULE: mjtGeom = mjtGeom(3);
    pub const ELLIPSOID: mjtGeom = mjtGeom(4);
    pub const CYLINDER: mjtGeom = mjtGeom(5);
    pub const BOX: mjtGeom = mjtGeom(6);
    pub const MESH: mjtGeom = mjtGeom(7);
    pub const SDF: mjtGeom = mjtGeom(8);
    pub const ARROW: mjtGeom = mjtGeom(100);
    pub const ARROW1: mjtGeom = mjtGeom(101);
    pub const ARROW2: mjtGeom = mjtGeom(102);
    pub const LINE: mjtGeom = mjtGeom(103);
    pub const LINEBOX: mjtGeom = mjtGeom(104);
    pub const FLEX: mjtGeom = mjtGeom(105);
    pub const SKIN: mjtGeom = mjtGeom(106);
    pub const LABEL: mjtGeom = mjtGeom(107);
    pub const TRIANGLE: mjtGeom = mjtGeom(108);
    pub const NONE: mjtGeom = mjtGeom(1001);
    const mjNGEOMTYPES: mjtGeom = mjtGeom(9);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtGeom(::core::ffi::c_uint);
impl mjtProjection {
    pub const PERSPECTIVE: mjtProjection = mjtProjection(0);
    pub const ORTHOGRAPHIC: mjtProjection = mjtProjection(1);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtProjection(::core::ffi::c_uint);
impl mjtCamLight {
    pub const FIXED: mjtCamLight = mjtCamLight(0);
    pub const TRACK: mjtCamLight = mjtCamLight(1);
    pub const TRACKCOM: mjtCamLight = mjtCamLight(2);
    pub const TARGETBODY: mjtCamLight = mjtCamLight(3);
    pub const TARGETBODYCOM: mjtCamLight = mjtCamLight(4);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtCamLight(::core::ffi::c_uint);
impl mjtLightType {
    pub const SPOT: mjtLightType = mjtLightType(0);
    pub const DIRECTIONAL: mjtLightType = mjtLightType(1);
    pub const POINT: mjtLightType = mjtLightType(2);
    pub const IMAGE: mjtLightType = mjtLightType(3);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtLightType(::core::ffi::c_uint);
impl mjtTexture {
    pub const D2: mjtTexture = mjtTexture(0);
    pub const CUBE: mjtTexture = mjtTexture(1);
    pub const SKYBOX: mjtTexture = mjtTexture(2);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtTexture(::core::ffi::c_uint);
pub const mjNTEXROLE: usize = mjtTextureRole::mjNTEXROLE.0 as usize;
impl mjtTextureRole {
    pub const USER: mjtTextureRole = mjtTextureRole(0);
    pub const RGB: mjtTextureRole = mjtTextureRole(1);
    pub const OCCLUSION: mjtTextureRole = mjtTextureRole(2);
    pub const ROUGHNESS: mjtTextureRole = mjtTextureRole(3);
    pub const METALLIC: mjtTextureRole = mjtTextureRole(4);
    pub const NORMAL: mjtTextureRole = mjtTextureRole(5);
    pub const OPACITY: mjtTextureRole = mjtTextureRole(6);
    pub const EMISSIVE: mjtTextureRole = mjtTextureRole(7);
    pub const RGBA: mjtTextureRole = mjtTextureRole(8);
    pub const ORM: mjtTextureRole = mjtTextureRole(9);
    const mjNTEXROLE: mjtTextureRole = mjtTextureRole(10);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtTextureRole(::core::ffi::c_uint);
impl mjtColorSpace {
    pub const AUTO: mjtColorSpace = mjtColorSpace(0);
    pub const LINEAR: mjtColorSpace = mjtColorSpace(1);
    pub const SRGB: mjtColorSpace = mjtColorSpace(2);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtColorSpace(::core::ffi::c_uint);
impl mjtIntegrator {
    pub const EULER: mjtIntegrator = mjtIntegrator(0);
    pub const RK4: mjtIntegrator = mjtIntegrator(1);
    pub const IMPLICIT: mjtIntegrator = mjtIntegrator(2);
    pub const IMPLICITFAST: mjtIntegrator = mjtIntegrator(3);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtIntegrator(::core::ffi::c_uint);
impl mjtCone {
    pub const PYRAMIDAL: mjtCone = mjtCone(0);
    pub const ELLIPTIC: mjtCone = mjtCone(1);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtCone(::core::ffi::c_uint);
impl mjtJacobian {
    pub const DENSE: mjtJacobian = mjtJacobian(0);
    pub const SPARSE: mjtJacobian = mjtJacobian(1);
    pub const AUTO: mjtJacobian = mjtJacobian(2);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtJacobian(::core::ffi::c_uint);
impl mjtSolver {
    pub const PGS: mjtSolver = mjtSolver(0);
    pub const CG: mjtSolver = mjtSolver(1);
    pub const NEWTON: mjtSolver = mjtSolver(2);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtSolver(::core::ffi::c_uint);
impl mjtEq {
    pub const CONNECT: mjtEq = mjtEq(0);
    pub const WELD: mjtEq = mjtEq(1);
    pub const JOINT: mjtEq = mjtEq(2);
    pub const TENDON: mjtEq = mjtEq(3);
    pub const FLEX: mjtEq = mjtEq(4);
    pub const FLEXVERT: mjtEq = mjtEq(5);
    pub const FLEXSTRAIN: mjtEq = mjtEq(6);
    pub const DISTANCE: mjtEq = mjtEq(7);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtEq(::core::ffi::c_uint);
impl mjtWrap {
    pub const NONE: mjtWrap = mjtWrap(0);
    pub const JOINT: mjtWrap = mjtWrap(1);
    pub const PULLEY: mjtWrap = mjtWrap(2);
    pub const SITE: mjtWrap = mjtWrap(3);
    pub const SPHERE: mjtWrap = mjtWrap(4);
    pub const CYLINDER: mjtWrap = mjtWrap(5);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtWrap(::core::ffi::c_uint);
impl mjtTrn {
    pub const JOINT: mjtTrn = mjtTrn(0);
    pub const JOINTINPARENT: mjtTrn = mjtTrn(1);
    pub const SLIDERCRANK: mjtTrn = mjtTrn(2);
    pub const TENDON: mjtTrn = mjtTrn(3);
    pub const SITE: mjtTrn = mjtTrn(4);
    pub const BODY: mjtTrn = mjtTrn(5);
    pub const SO3: mjtTrn = mjtTrn(6);
    pub const UNDEFINED: mjtTrn = mjtTrn(1000);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtTrn(::core::ffi::c_uint);
impl mjtDyn {
    pub const NONE: mjtDyn = mjtDyn(0);
    pub const INTEGRATOR: mjtDyn = mjtDyn(1);
    pub const FILTER: mjtDyn = mjtDyn(2);
    pub const FILTEREXACT: mjtDyn = mjtDyn(3);
    pub const MUSCLE: mjtDyn = mjtDyn(4);
    pub const DCMOTOR: mjtDyn = mjtDyn(5);
    pub const USER: mjtDyn = mjtDyn(6);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtDyn(::core::ffi::c_uint);
impl mjtGain {
    pub const FIXED: mjtGain = mjtGain(0);
    pub const AFFINE: mjtGain = mjtGain(1);
    pub const MUSCLE: mjtGain = mjtGain(2);
    pub const DCMOTOR: mjtGain = mjtGain(3);
    pub const SO3: mjtGain = mjtGain(4);
    pub const USER: mjtGain = mjtGain(5);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtGain(::core::ffi::c_uint);
impl mjtBias {
    pub const NONE: mjtBias = mjtBias(0);
    pub const AFFINE: mjtBias = mjtBias(1);
    pub const MUSCLE: mjtBias = mjtBias(2);
    pub const DCMOTOR: mjtBias = mjtBias(3);
    pub const SO3: mjtBias = mjtBias(4);
    pub const USER: mjtBias = mjtBias(5);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtBias(::core::ffi::c_uint);
impl mjtCtrlChart {
    pub const EXPMAP: mjtCtrlChart = mjtCtrlChart(1);
    pub const QUAT: mjtCtrlChart = mjtCtrlChart(2);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtCtrlChart(::core::ffi::c_uint);
pub const mjNOBJECT: usize = mjtObj::mjNOBJECT.0 as usize;
impl mjtObj {
    pub const UNKNOWN: mjtObj = mjtObj(0);
    pub const BODY: mjtObj = mjtObj(1);
    pub const XBODY: mjtObj = mjtObj(2);
    pub const JOINT: mjtObj = mjtObj(3);
    pub const DOF: mjtObj = mjtObj(4);
    pub const GEOM: mjtObj = mjtObj(5);
    pub const SITE: mjtObj = mjtObj(6);
    pub const CAMERA: mjtObj = mjtObj(7);
    pub const LIGHT: mjtObj = mjtObj(8);
    pub const FLEX: mjtObj = mjtObj(9);
    pub const MESH: mjtObj = mjtObj(10);
    pub const SKIN: mjtObj = mjtObj(11);
    pub const HFIELD: mjtObj = mjtObj(12);
    pub const TEXTURE: mjtObj = mjtObj(13);
    pub const MATERIAL: mjtObj = mjtObj(14);
    pub const PAIR: mjtObj = mjtObj(15);
    pub const EXCLUDE: mjtObj = mjtObj(16);
    pub const EQUALITY: mjtObj = mjtObj(17);
    pub const TENDON: mjtObj = mjtObj(18);
    pub const ACTUATOR: mjtObj = mjtObj(19);
    pub const SENSOR: mjtObj = mjtObj(20);
    pub const NUMERIC: mjtObj = mjtObj(21);
    pub const TEXT: mjtObj = mjtObj(22);
    pub const TUPLE: mjtObj = mjtObj(23);
    pub const KEY: mjtObj = mjtObj(24);
    pub const PLUGIN: mjtObj = mjtObj(25);
    pub const FRAME: mjtObj = mjtObj(100);
    pub const DEFAULT: mjtObj = mjtObj(101);
    pub const MODEL: mjtObj = mjtObj(102);
    const mjNOBJECT: mjtObj = mjtObj(26);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtObj(::core::ffi::c_uint);
impl mjtSensor {
    pub const TOUCH: mjtSensor = mjtSensor(0);
    pub const ACCELEROMETER: mjtSensor = mjtSensor(1);
    pub const VELOCIMETER: mjtSensor = mjtSensor(2);
    pub const GYRO: mjtSensor = mjtSensor(3);
    pub const FORCE: mjtSensor = mjtSensor(4);
    pub const TORQUE: mjtSensor = mjtSensor(5);
    pub const MAGNETOMETER: mjtSensor = mjtSensor(6);
    pub const RANGEFINDER: mjtSensor = mjtSensor(7);
    pub const CAMPROJECTION: mjtSensor = mjtSensor(8);
    pub const JOINTPOS: mjtSensor = mjtSensor(9);
    pub const JOINTVEL: mjtSensor = mjtSensor(10);
    pub const TENDONPOS: mjtSensor = mjtSensor(11);
    pub const TENDONVEL: mjtSensor = mjtSensor(12);
    pub const ACTUATORPOS: mjtSensor = mjtSensor(13);
    pub const ACTUATORVEL: mjtSensor = mjtSensor(14);
    pub const ACTUATORFRC: mjtSensor = mjtSensor(15);
    pub const JOINTACTFRC: mjtSensor = mjtSensor(16);
    pub const TENDONACTFRC: mjtSensor = mjtSensor(17);
    pub const BALLQUAT: mjtSensor = mjtSensor(18);
    pub const BALLANGVEL: mjtSensor = mjtSensor(19);
    pub const JOINTLIMITPOS: mjtSensor = mjtSensor(20);
    pub const JOINTLIMITVEL: mjtSensor = mjtSensor(21);
    pub const JOINTLIMITFRC: mjtSensor = mjtSensor(22);
    pub const TENDONLIMITPOS: mjtSensor = mjtSensor(23);
    pub const TENDONLIMITVEL: mjtSensor = mjtSensor(24);
    pub const TENDONLIMITFRC: mjtSensor = mjtSensor(25);
    pub const FRAMEPOS: mjtSensor = mjtSensor(26);
    pub const FRAMEQUAT: mjtSensor = mjtSensor(27);
    pub const FRAMEXAXIS: mjtSensor = mjtSensor(28);
    pub const FRAMEYAXIS: mjtSensor = mjtSensor(29);
    pub const FRAMEZAXIS: mjtSensor = mjtSensor(30);
    pub const FRAMELINVEL: mjtSensor = mjtSensor(31);
    pub const FRAMEANGVEL: mjtSensor = mjtSensor(32);
    pub const FRAMELINACC: mjtSensor = mjtSensor(33);
    pub const FRAMEANGACC: mjtSensor = mjtSensor(34);
    pub const SUBTREECOM: mjtSensor = mjtSensor(35);
    pub const SUBTREELINVEL: mjtSensor = mjtSensor(36);
    pub const SUBTREEANGMOM: mjtSensor = mjtSensor(37);
    pub const INSIDESITE: mjtSensor = mjtSensor(38);
    pub const GEOMDIST: mjtSensor = mjtSensor(39);
    pub const GEOMNORMAL: mjtSensor = mjtSensor(40);
    pub const GEOMFROMTO: mjtSensor = mjtSensor(41);
    pub const CONTACT: mjtSensor = mjtSensor(42);
    pub const E_POTENTIAL: mjtSensor = mjtSensor(43);
    pub const E_KINETIC: mjtSensor = mjtSensor(44);
    pub const CLOCK: mjtSensor = mjtSensor(45);
    pub const TACTILE: mjtSensor = mjtSensor(46);
    pub const PLUGIN: mjtSensor = mjtSensor(47);
    pub const USER: mjtSensor = mjtSensor(48);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtSensor(::core::ffi::c_uint);
impl mjtStage {
    pub const NONE: mjtStage = mjtStage(0);
    pub const POS: mjtStage = mjtStage(1);
    pub const VEL: mjtStage = mjtStage(2);
    pub const ACC: mjtStage = mjtStage(3);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtStage(::core::ffi::c_uint);
impl mjtDataType {
    pub const REAL: mjtDataType = mjtDataType(0);
    pub const POSITIVE: mjtDataType = mjtDataType(1);
    pub const AXIS: mjtDataType = mjtDataType(2);
    pub const QUATERNION: mjtDataType = mjtDataType(3);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtDataType(::core::ffi::c_uint);
pub const mjNCONDATA: usize = mjtConDataField::mjNCONDATA.0 as usize;
impl mjtConDataField {
    pub const FOUND: mjtConDataField = mjtConDataField(0);
    pub const FORCE: mjtConDataField = mjtConDataField(1);
    pub const TORQUE: mjtConDataField = mjtConDataField(2);
    pub const DIST: mjtConDataField = mjtConDataField(3);
    pub const POS: mjtConDataField = mjtConDataField(4);
    pub const NORMAL: mjtConDataField = mjtConDataField(5);
    pub const TANGENT: mjtConDataField = mjtConDataField(6);
    const mjNCONDATA: mjtConDataField = mjtConDataField(7);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtConDataField(::core::ffi::c_uint);
pub const mjNRAYDATA: usize = mjtRayDataField::mjNRAYDATA.0 as usize;
impl mjtRayDataField {
    pub const DIST: mjtRayDataField = mjtRayDataField(0);
    pub const DIR: mjtRayDataField = mjtRayDataField(1);
    pub const ORIGIN: mjtRayDataField = mjtRayDataField(2);
    pub const POINT: mjtRayDataField = mjtRayDataField(3);
    pub const NORMAL: mjtRayDataField = mjtRayDataField(4);
    pub const DEPTH: mjtRayDataField = mjtRayDataField(5);
    const mjNRAYDATA: mjtRayDataField = mjtRayDataField(6);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtRayDataField(::core::ffi::c_uint);
pub const mjNCAMOUT: usize = mjtCamOutBit::mjNCAMOUT.0 as usize;
impl mjtCamOutBit {
    pub const RGB: mjtCamOutBit = mjtCamOutBit(1);
    pub const DEPTH: mjtCamOutBit = mjtCamOutBit(2);
    pub const DIST: mjtCamOutBit = mjtCamOutBit(4);
    pub const NORMAL: mjtCamOutBit = mjtCamOutBit(8);
    pub const SEG: mjtCamOutBit = mjtCamOutBit(16);
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
pub struct mjtCamOutBit(::core::ffi::c_uint);
impl mjtSameFrame {
    pub const NONE: mjtSameFrame = mjtSameFrame(0);
    pub const BODY: mjtSameFrame = mjtSameFrame(1);
    pub const INERTIA: mjtSameFrame = mjtSameFrame(2);
    pub const BODYROT: mjtSameFrame = mjtSameFrame(3);
    pub const INERTIAROT: mjtSameFrame = mjtSameFrame(4);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtSameFrame(::core::ffi::c_uint);
impl mjtSleepPolicy {
    pub const AUTO: mjtSleepPolicy = mjtSleepPolicy(0);
    pub const AUTO_NEVER: mjtSleepPolicy = mjtSleepPolicy(1);
    pub const AUTO_ALLOWED: mjtSleepPolicy = mjtSleepPolicy(2);
    pub const NEVER: mjtSleepPolicy = mjtSleepPolicy(3);
    pub const ALLOWED: mjtSleepPolicy = mjtSleepPolicy(4);
    pub const INIT: mjtSleepPolicy = mjtSleepPolicy(5);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtSleepPolicy(::core::ffi::c_uint);
impl mjtLRMode {
    pub const NONE: mjtLRMode = mjtLRMode(0);
    pub const MUSCLE: mjtLRMode = mjtLRMode(1);
    pub const MUSCLEUSER: mjtLRMode = mjtLRMode(2);
    pub const ALL: mjtLRMode = mjtLRMode(3);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtLRMode(::core::ffi::c_uint);
impl mjtFlexSelf {
    pub const NONE: mjtFlexSelf = mjtFlexSelf(0);
    pub const NARROW: mjtFlexSelf = mjtFlexSelf(1);
    pub const BVH: mjtFlexSelf = mjtFlexSelf(2);
    pub const SAP: mjtFlexSelf = mjtFlexSelf(3);
    pub const AUTO: mjtFlexSelf = mjtFlexSelf(4);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtFlexSelf(::core::ffi::c_uint);
impl mjtSDFType {
    pub const SINGLE: mjtSDFType = mjtSDFType(0);
    pub const INTERSECTION: mjtSDFType = mjtSDFType(1);
    pub const MIDSURFACE: mjtSDFType = mjtSDFType(2);
    pub const COLLISION: mjtSDFType = mjtSDFType(3);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtSDFType(::core::ffi::c_uint);
pub const mjNSTATE: usize = mjtState::mjNSTATE.0 as usize;
impl mjtState {
    pub const TIME: mjtState = mjtState(1);
    pub const QPOS: mjtState = mjtState(2);
    pub const QVEL: mjtState = mjtState(4);
    pub const ACT: mjtState = mjtState(8);
    pub const HISTORY: mjtState = mjtState(16);
    pub const WARMSTART: mjtState = mjtState(32);
    pub const CTRL: mjtState = mjtState(64);
    pub const QFRC_APPLIED: mjtState = mjtState(128);
    pub const XFRC_APPLIED: mjtState = mjtState(256);
    pub const EQ_ACTIVE: mjtState = mjtState(512);
    pub const MOCAP_POS: mjtState = mjtState(1024);
    pub const MOCAP_QUAT: mjtState = mjtState(2048);
    pub const USERDATA: mjtState = mjtState(4096);
    pub const PLUGIN: mjtState = mjtState(8192);
    pub const PHYSICS: mjtState = mjtState(30);
    pub const FULLPHYSICS: mjtState = mjtState(8223);
    pub const USER: mjtState = mjtState(8128);
    pub const INTEGRATION: mjtState = mjtState(16383);
    const mjNSTATE: mjtState = mjtState(14);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtState(::core::ffi::c_uint);
impl mjtConstraint {
    pub const EQUALITY: mjtConstraint = mjtConstraint(0);
    pub const FRICTION_DOF: mjtConstraint = mjtConstraint(1);
    pub const FRICTION_TENDON: mjtConstraint = mjtConstraint(2);
    pub const LIMIT_JOINT: mjtConstraint = mjtConstraint(3);
    pub const LIMIT_TENDON: mjtConstraint = mjtConstraint(4);
    pub const CONTACT_FRICTIONLESS: mjtConstraint = mjtConstraint(5);
    pub const CONTACT_PYRAMIDAL: mjtConstraint = mjtConstraint(6);
    pub const CONTACT_ELLIPTIC: mjtConstraint = mjtConstraint(7);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtConstraint(::core::ffi::c_uint);
impl mjtConstraintState {
    pub const SATISFIED: mjtConstraintState = mjtConstraintState(0);
    pub const QUADRATIC: mjtConstraintState = mjtConstraintState(1);
    pub const LINEARNEG: mjtConstraintState = mjtConstraintState(2);
    pub const LINEARPOS: mjtConstraintState = mjtConstraintState(3);
    pub const CONE: mjtConstraintState = mjtConstraintState(4);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtConstraintState(::core::ffi::c_uint);
pub const mjNWARNING: usize = mjtWarning::mjNWARNING.0 as usize;
impl mjtWarning {
    pub const INERTIA: mjtWarning = mjtWarning(0);
    pub const CONTACTFULL: mjtWarning = mjtWarning(1);
    pub const CNSTRFULL: mjtWarning = mjtWarning(2);
    pub const BADQPOS: mjtWarning = mjtWarning(3);
    pub const BADQVEL: mjtWarning = mjtWarning(4);
    pub const BADQACC: mjtWarning = mjtWarning(5);
    pub const BADCTRL: mjtWarning = mjtWarning(6);
    const mjNWARNING: mjtWarning = mjtWarning(7);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtWarning(::core::ffi::c_uint);
pub const mjNTIMER: usize = mjtTimer::mjNTIMER.0 as usize;
impl mjtTimer {
    pub const STEP: mjtTimer = mjtTimer(0);
    pub const FORWARD: mjtTimer = mjtTimer(1);
    pub const INVERSE: mjtTimer = mjtTimer(2);
    pub const POSITION: mjtTimer = mjtTimer(3);
    pub const VELOCITY: mjtTimer = mjtTimer(4);
    pub const ACTUATION: mjtTimer = mjtTimer(5);
    pub const CONSTRAINT: mjtTimer = mjtTimer(6);
    pub const ADVANCE: mjtTimer = mjtTimer(7);
    pub const POS_KINEMATICS: mjtTimer = mjtTimer(8);
    pub const POS_INERTIA: mjtTimer = mjtTimer(9);
    pub const POS_COLLISION: mjtTimer = mjtTimer(10);
    pub const POS_MAKE: mjtTimer = mjtTimer(11);
    pub const POS_PROJECT: mjtTimer = mjtTimer(12);
    pub const COL_BROAD: mjtTimer = mjtTimer(13);
    pub const COL_NARROW: mjtTimer = mjtTimer(14);
    const mjNTIMER: mjtTimer = mjtTimer(15);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtTimer(::core::ffi::c_uint);
impl mjtSleepState {
    pub const STATIC: mjtSleepState = mjtSleepState(-1);
    pub const ASLEEP: mjtSleepState = mjtSleepState(0);
    pub const AWAKE: mjtSleepState = mjtSleepState(1);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtSleepState(::core::ffi::c_int);
impl mjtLogLevel {
    pub const DEBUG: mjtLogLevel = mjtLogLevel(0);
    pub const INFO: mjtLogLevel = mjtLogLevel(1);
    pub const WARNING: mjtLogLevel = mjtLogLevel(2);
    pub const ERROR: mjtLogLevel = mjtLogLevel(3);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtLogLevel(::core::ffi::c_uint);
impl mjtLogTopic {
    pub const NONE: mjtLogTopic = mjtLogTopic(0);
    pub const TIME_STP: mjtLogTopic = mjtLogTopic(1);
    pub const TIME_CMP: mjtLogTopic = mjtLogTopic(2);
    pub const SLEEP: mjtLogTopic = mjtLogTopic(3);
    const mjNTOPIC: mjtLogTopic = mjtLogTopic(3);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtLogTopic(::core::ffi::c_uint);
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjLogMessage {
    pub level: ::core::ffi::c_int,
    pub topic: ::core::ffi::c_int,
    pub(crate) subject: [::core::ffi::c_char; 1024usize],
    pub(crate) body: *const ::core::ffi::c_char,
    pub(crate) func: *const ::core::ffi::c_char,
    pub(crate) file: *const ::core::ffi::c_char,
    pub line: ::core::ffi::c_int,
    pub timestamp: mjtBool,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjLogConfig {
    pub logto_console: mjtBool,
    pub logto_file: mjtBool,
    pub(crate) logfile: [::core::ffi::c_char; 1024usize],
    pub topics: ::core::ffi::c_int,
}
pub(crate) type mjfLogHandler =
    ::core::option::Option<unsafe extern "C" fn(arg1: *const mjLogMessage)>;
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjLROpt {
    pub mode: ::core::ffi::c_int,
    pub useexisting: ::core::ffi::c_int,
    pub uselimit: ::core::ffi::c_int,
    pub accel: mjtNum,
    pub maxforce: mjtNum,
    pub timeconst: mjtNum,
    pub timestep: mjtNum,
    pub inttotal: mjtNum,
    pub interval: mjtNum,
    pub tolrange: mjtNum,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjCache {
    pub impl_: *mut ::core::ffi::c_void,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjVFS {
    pub impl_: *mut ::core::ffi::c_void,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjOption {
    pub timestep: mjtNum,
    pub impratio: mjtNum,
    pub tolerance: mjtNum,
    pub ls_tolerance: mjtNum,
    pub noslip_tolerance: mjtNum,
    pub ccd_tolerance: mjtNum,
    pub sleep_tolerance: mjtNum,
    pub gravity: [mjtNum; 3usize],
    pub wind: [mjtNum; 3usize],
    pub magnetic: [mjtNum; 3usize],
    pub density: mjtNum,
    pub viscosity: mjtNum,
    pub o_margin: mjtNum,
    pub o_solref: [mjtNum; 2usize],
    pub o_solimp: [mjtNum; 5usize],
    pub o_friction: [mjtNum; 5usize],
    pub integrator: ::core::ffi::c_int,
    pub cone: ::core::ffi::c_int,
    pub jacobian: ::core::ffi::c_int,
    pub solver: ::core::ffi::c_int,
    pub iterations: ::core::ffi::c_int,
    pub ls_iterations: ::core::ffi::c_int,
    pub noslip_iterations: ::core::ffi::c_int,
    pub ccd_iterations: ::core::ffi::c_int,
    pub disableflags: ::core::ffi::c_int,
    pub enableflags: ::core::ffi::c_int,
    pub disableactuator: ::core::ffi::c_int,
    pub sdf_initpoints: ::core::ffi::c_int,
    pub sdf_iterations: ::core::ffi::c_int,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjVisual {
    pub(crate) global: mjVisual___bindgen_ty_1,
    pub(crate) quality: mjVisual___bindgen_ty_2,
    pub(crate) headlight: mjVisual___bindgen_ty_3,
    pub(crate) map: mjVisual___bindgen_ty_4,
    pub(crate) scale: mjVisual___bindgen_ty_5,
    pub(crate) rgba: mjVisual___bindgen_ty_6,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub(crate) struct mjVisual___bindgen_ty_1 {
    pub(crate) cameraid: ::core::ffi::c_int,
    pub(crate) orthographic: ::core::ffi::c_int,
    pub(crate) fovy: f32,
    pub(crate) ipd: f32,
    pub(crate) azimuth: f32,
    pub(crate) elevation: f32,
    pub(crate) linewidth: f32,
    pub(crate) glow: f32,
    pub(crate) realtime: f32,
    pub(crate) offwidth: ::core::ffi::c_int,
    pub(crate) offheight: ::core::ffi::c_int,
    pub(crate) ellipsoidinertia: ::core::ffi::c_int,
    pub(crate) bvactive: ::core::ffi::c_int,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub(crate) struct mjVisual___bindgen_ty_2 {
    pub(crate) shadowsize: ::core::ffi::c_int,
    pub(crate) offsamples: ::core::ffi::c_int,
    pub(crate) numslices: ::core::ffi::c_int,
    pub(crate) numstacks: ::core::ffi::c_int,
    pub(crate) numquads: ::core::ffi::c_int,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub(crate) struct mjVisual___bindgen_ty_3 {
    pub(crate) ambient: [f32; 3usize],
    pub(crate) diffuse: [f32; 3usize],
    pub(crate) specular: [f32; 3usize],
    pub(crate) active: ::core::ffi::c_int,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub(crate) struct mjVisual___bindgen_ty_4 {
    pub(crate) stiffness: f32,
    pub(crate) stiffnessrot: f32,
    pub(crate) force: f32,
    pub(crate) torque: f32,
    pub(crate) alpha: f32,
    pub(crate) fogstart: f32,
    pub(crate) fogend: f32,
    pub(crate) znear: f32,
    pub(crate) zfar: f32,
    pub(crate) haze: f32,
    pub(crate) shadowclip: f32,
    pub(crate) shadowscale: f32,
    pub(crate) actuatortendon: f32,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub(crate) struct mjVisual___bindgen_ty_5 {
    pub(crate) forcewidth: f32,
    pub(crate) contactwidth: f32,
    pub(crate) contactheight: f32,
    pub(crate) connect: f32,
    pub(crate) com: f32,
    pub(crate) camera: f32,
    pub(crate) light: f32,
    pub(crate) selectpoint: f32,
    pub(crate) jointlength: f32,
    pub(crate) jointwidth: f32,
    pub(crate) actuatorlength: f32,
    pub(crate) actuatorwidth: f32,
    pub(crate) framelength: f32,
    pub(crate) framewidth: f32,
    pub(crate) constraint: f32,
    pub(crate) slidercrank: f32,
    pub(crate) frustum: f32,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub(crate) struct mjVisual___bindgen_ty_6 {
    pub(crate) fog: [f32; 4usize],
    pub(crate) haze: [f32; 4usize],
    pub(crate) force: [f32; 4usize],
    pub(crate) inertia: [f32; 4usize],
    pub(crate) joint: [f32; 4usize],
    pub(crate) actuator: [f32; 4usize],
    pub(crate) actuatornegative: [f32; 4usize],
    pub(crate) actuatorpositive: [f32; 4usize],
    pub(crate) com: [f32; 4usize],
    pub(crate) camera: [f32; 4usize],
    pub(crate) light: [f32; 4usize],
    pub(crate) selectpoint: [f32; 4usize],
    pub(crate) connect: [f32; 4usize],
    pub(crate) contactpoint: [f32; 4usize],
    pub(crate) contactforce: [f32; 4usize],
    pub(crate) contactfriction: [f32; 4usize],
    pub(crate) contacttorque: [f32; 4usize],
    pub(crate) contactgap: [f32; 4usize],
    pub(crate) rangefinder: [f32; 4usize],
    pub(crate) constraint: [f32; 4usize],
    pub(crate) slidercrank: [f32; 4usize],
    pub(crate) crankbroken: [f32; 4usize],
    pub(crate) frustum: [f32; 4usize],
    pub(crate) bv: [f32; 4usize],
    pub(crate) bvactive: [f32; 4usize],
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjStatistic {
    pub meaninertia: mjtNum,
    pub meanmass: mjtNum,
    pub meansize: mjtNum,
    pub extent: mjtNum,
    pub center: [mjtNum; 3usize],
}
#[repr(C)]
#[derive(Debug)]
pub struct mjModel {
    pub nq: mjtSize,
    pub nv: mjtSize,
    pub nu: mjtSize,
    pub nactuator: mjtSize,
    pub nout: mjtSize,
    pub na: mjtSize,
    pub nbody: mjtSize,
    pub nbvh: mjtSize,
    pub nbvhstatic: mjtSize,
    pub nbvhdynamic: mjtSize,
    pub noct: mjtSize,
    pub njnt: mjtSize,
    pub ntree: mjtSize,
    pub nM: mjtSize,
    pub nB: mjtSize,
    pub nC: mjtSize,
    pub nD: mjtSize,
    pub ngeom: mjtSize,
    pub nsite: mjtSize,
    pub ncam: mjtSize,
    pub nlight: mjtSize,
    pub nflex: mjtSize,
    pub nflexnode: mjtSize,
    pub nflexvert: mjtSize,
    pub nflexedge: mjtSize,
    pub nflexelem: mjtSize,
    pub nflexelemdata: mjtSize,
    pub nflexstiffness: mjtSize,
    pub nflexbending: mjtSize,
    pub nefm0dof: mjtSize,
    pub nefm0L: mjtSize,
    pub nflexelemedge: mjtSize,
    pub nflexshelldata: mjtSize,
    pub nflexevpair: mjtSize,
    pub nflextexcoord: mjtSize,
    pub nJfe: mjtSize,
    pub nJfv: mjtSize,
    pub nmesh: mjtSize,
    pub nmeshvert: mjtSize,
    pub nmeshnormal: mjtSize,
    pub nmeshtexcoord: mjtSize,
    pub nmeshface: mjtSize,
    pub nmeshgraph: mjtSize,
    pub nmeshpoly: mjtSize,
    pub nmeshpolyvert: mjtSize,
    pub nmeshpolymap: mjtSize,
    pub nskin: mjtSize,
    pub nskinvert: mjtSize,
    pub nskintexvert: mjtSize,
    pub nskinface: mjtSize,
    pub nskinbone: mjtSize,
    pub nskinbonevert: mjtSize,
    pub nhfield: mjtSize,
    pub nhfielddata: mjtSize,
    pub ntex: mjtSize,
    pub ntexdata: mjtSize,
    pub nmat: mjtSize,
    pub npair: mjtSize,
    pub nexclude: mjtSize,
    pub neq: mjtSize,
    pub ntendon: mjtSize,
    pub nJten: mjtSize,
    pub nwrap: mjtSize,
    pub nsensor: mjtSize,
    pub nnumeric: mjtSize,
    pub nnumericdata: mjtSize,
    pub ntext: mjtSize,
    pub ntextdata: mjtSize,
    pub ntuple: mjtSize,
    pub ntupledata: mjtSize,
    pub nkey: mjtSize,
    pub nmocap: mjtSize,
    pub nplugin: mjtSize,
    pub npluginattr: mjtSize,
    pub nuser_body: mjtSize,
    pub nuser_jnt: mjtSize,
    pub nuser_geom: mjtSize,
    pub nuser_site: mjtSize,
    pub nuser_cam: mjtSize,
    pub nuser_tendon: mjtSize,
    pub nuser_actuator: mjtSize,
    pub nuser_sensor: mjtSize,
    pub nnames: mjtSize,
    pub npaths: mjtSize,
    pub nnames_map: mjtSize,
    pub nJmom: mjtSize,
    pub ngravcomp: mjtSize,
    pub nemax: mjtSize,
    pub njmax: mjtSize,
    pub nconmax: mjtSize,
    pub npolygonmax: mjtSize,
    pub nmeshdegmax: mjtSize,
    pub nuserdata: mjtSize,
    pub nsensordata: mjtSize,
    pub npluginstate: mjtSize,
    pub nhistory: mjtSize,
    pub narena: mjtSize,
    pub nbuffer: mjtSize,
    pub flg_gravcomp: mjtBool,
    pub flg_surfacevel: mjtBool,
    pub flg_adhesion: mjtBool,
    pub opt: mjOption,
    pub vis: mjVisual,
    pub stat: mjStatistic,
    pub buffer: *mut ::core::ffi::c_void,
    pub(crate) qpos0: *mut mjtNum,
    pub(crate) qpos_spring: *mut mjtNum,
    pub(crate) body_parentid: *mut ::core::ffi::c_int,
    pub(crate) body_rootid: *mut ::core::ffi::c_int,
    pub(crate) body_weldid: *mut ::core::ffi::c_int,
    pub(crate) body_mocapid: *mut ::core::ffi::c_int,
    pub(crate) body_jntnum: *mut ::core::ffi::c_int,
    pub(crate) body_jntadr: *mut ::core::ffi::c_int,
    pub(crate) body_dofnum: *mut ::core::ffi::c_int,
    pub(crate) body_dofadr: *mut ::core::ffi::c_int,
    pub(crate) body_treeid: *mut ::core::ffi::c_int,
    pub(crate) body_geomnum: *mut ::core::ffi::c_int,
    pub(crate) body_geomadr: *mut ::core::ffi::c_int,
    pub(crate) body_simple: *mut mjtByte,
    pub(crate) body_sameframe: *mut mjtByte,
    pub(crate) body_pos: *mut mjtNum,
    pub(crate) body_quat: *mut mjtNum,
    pub(crate) body_ipos: *mut mjtNum,
    pub(crate) body_iquat: *mut mjtNum,
    pub(crate) body_mass: *mut mjtNum,
    pub(crate) body_subtreemass: *mut mjtNum,
    pub(crate) body_inertia: *mut mjtNum,
    pub(crate) body_invweight0: *mut mjtNum,
    pub(crate) body_gravcomp: *mut mjtNum,
    pub(crate) body_margin: *mut mjtNum,
    pub(crate) body_user: *mut mjtNum,
    pub(crate) body_plugin: *mut ::core::ffi::c_int,
    pub(crate) body_contype: *mut ::core::ffi::c_int,
    pub(crate) body_conaffinity: *mut ::core::ffi::c_int,
    pub(crate) body_bvhadr: *mut ::core::ffi::c_int,
    pub(crate) body_bvhnum: *mut ::core::ffi::c_int,
    pub(crate) bvh_depth: *mut ::core::ffi::c_int,
    pub(crate) bvh_child: *mut ::core::ffi::c_int,
    pub(crate) bvh_nodeid: *mut ::core::ffi::c_int,
    pub(crate) bvh_aabb: *mut mjtNum,
    pub(crate) oct_depth: *mut ::core::ffi::c_int,
    pub(crate) oct_child: *mut ::core::ffi::c_int,
    pub(crate) oct_aabb: *mut mjtNum,
    pub(crate) oct_coeff: *mut mjtNum,
    pub(crate) jnt_type: *mut ::core::ffi::c_int,
    pub(crate) jnt_qposadr: *mut ::core::ffi::c_int,
    pub(crate) jnt_dofadr: *mut ::core::ffi::c_int,
    pub(crate) jnt_bodyid: *mut ::core::ffi::c_int,
    pub(crate) jnt_actuatorid: *mut ::core::ffi::c_int,
    pub(crate) jnt_group: *mut ::core::ffi::c_int,
    pub(crate) jnt_limited: *mut mjtBool,
    pub(crate) jnt_actfrclimited: *mut mjtBool,
    pub(crate) jnt_actgravcomp: *mut mjtBool,
    pub(crate) jnt_solref: *mut mjtNum,
    pub(crate) jnt_solimp: *mut mjtNum,
    pub(crate) jnt_pos: *mut mjtNum,
    pub(crate) jnt_axis: *mut mjtNum,
    pub(crate) jnt_stiffness: *mut mjtNum,
    pub(crate) jnt_stiffnesspoly: *mut mjtNum,
    pub(crate) jnt_range: *mut mjtNum,
    pub(crate) jnt_actfrcrange: *mut mjtNum,
    pub(crate) jnt_margin: *mut mjtNum,
    pub(crate) jnt_user: *mut mjtNum,
    pub(crate) dof_bodyid: *mut ::core::ffi::c_int,
    pub(crate) dof_jntid: *mut ::core::ffi::c_int,
    pub(crate) dof_parentid: *mut ::core::ffi::c_int,
    pub(crate) dof_treeid: *mut ::core::ffi::c_int,
    pub(crate) dof_Madr: *mut ::core::ffi::c_int,
    pub(crate) dof_simplenum: *mut ::core::ffi::c_int,
    pub(crate) dof_solref: *mut mjtNum,
    pub(crate) dof_solimp: *mut mjtNum,
    pub(crate) dof_frictionloss: *mut mjtNum,
    pub(crate) dof_armature: *mut mjtNum,
    pub(crate) dof_damping: *mut mjtNum,
    pub(crate) dof_dampingpoly: *mut mjtNum,
    pub(crate) dof_invweight0: *mut mjtNum,
    pub(crate) dof_M0: *mut mjtNum,
    pub(crate) dof_length: *mut mjtNum,
    pub(crate) tree_bodyadr: *mut ::core::ffi::c_int,
    pub(crate) tree_bodynum: *mut ::core::ffi::c_int,
    pub(crate) tree_dofadr: *mut ::core::ffi::c_int,
    pub(crate) tree_dofnum: *mut ::core::ffi::c_int,
    pub(crate) tree_sleep_policy: *mut ::core::ffi::c_int,
    pub(crate) geom_type: *mut ::core::ffi::c_int,
    pub(crate) geom_contype: *mut ::core::ffi::c_int,
    pub(crate) geom_conaffinity: *mut ::core::ffi::c_int,
    pub(crate) geom_condim: *mut ::core::ffi::c_int,
    pub(crate) geom_bodyid: *mut ::core::ffi::c_int,
    pub(crate) geom_dataid: *mut ::core::ffi::c_int,
    pub(crate) geom_matid: *mut ::core::ffi::c_int,
    pub(crate) geom_group: *mut ::core::ffi::c_int,
    pub(crate) geom_priority: *mut ::core::ffi::c_int,
    pub(crate) geom_plugin: *mut ::core::ffi::c_int,
    pub(crate) geom_sameframe: *mut mjtByte,
    pub(crate) geom_solmix: *mut mjtNum,
    pub(crate) geom_solref: *mut mjtNum,
    pub(crate) geom_solimp: *mut mjtNum,
    pub(crate) geom_size: *mut mjtNum,
    pub(crate) geom_aabb: *mut mjtNum,
    pub(crate) geom_rbound: *mut mjtNum,
    pub(crate) geom_pos: *mut mjtNum,
    pub(crate) geom_quat: *mut mjtNum,
    pub(crate) geom_friction: *mut mjtNum,
    pub(crate) geom_margin: *mut mjtNum,
    pub(crate) geom_gap: *mut mjtNum,
    pub(crate) geom_surfacevel: *mut mjtNum,
    pub(crate) geom_adhesion: *mut mjtNum,
    pub(crate) geom_fluid: *mut mjtNum,
    pub(crate) geom_user: *mut mjtNum,
    pub(crate) geom_rgba: *mut f32,
    pub(crate) site_type: *mut ::core::ffi::c_int,
    pub(crate) site_bodyid: *mut ::core::ffi::c_int,
    pub(crate) site_matid: *mut ::core::ffi::c_int,
    pub(crate) site_group: *mut ::core::ffi::c_int,
    pub(crate) site_sameframe: *mut mjtByte,
    pub(crate) site_size: *mut mjtNum,
    pub(crate) site_pos: *mut mjtNum,
    pub(crate) site_quat: *mut mjtNum,
    pub(crate) site_user: *mut mjtNum,
    pub(crate) site_rgba: *mut f32,
    pub(crate) cam_mode: *mut ::core::ffi::c_int,
    pub(crate) cam_bodyid: *mut ::core::ffi::c_int,
    pub(crate) cam_targetbodyid: *mut ::core::ffi::c_int,
    pub(crate) cam_pos: *mut mjtNum,
    pub(crate) cam_quat: *mut mjtNum,
    pub(crate) cam_poscom0: *mut mjtNum,
    pub(crate) cam_pos0: *mut mjtNum,
    pub(crate) cam_mat0: *mut mjtNum,
    pub(crate) cam_projection: *mut ::core::ffi::c_int,
    pub(crate) cam_fovy: *mut mjtNum,
    pub(crate) cam_ipd: *mut mjtNum,
    pub(crate) cam_resolution: *mut ::core::ffi::c_int,
    pub(crate) cam_output: *mut ::core::ffi::c_int,
    pub(crate) cam_sensorsize: *mut f32,
    pub(crate) cam_intrinsic: *mut f32,
    pub(crate) cam_user: *mut mjtNum,
    pub(crate) light_mode: *mut ::core::ffi::c_int,
    pub(crate) light_bodyid: *mut ::core::ffi::c_int,
    pub(crate) light_targetbodyid: *mut ::core::ffi::c_int,
    pub(crate) light_type: *mut ::core::ffi::c_int,
    pub(crate) light_texid: *mut ::core::ffi::c_int,
    pub(crate) light_castshadow: *mut mjtBool,
    pub(crate) light_bulbradius: *mut f32,
    pub(crate) light_intensity: *mut f32,
    pub(crate) light_range: *mut f32,
    pub(crate) light_active: *mut mjtBool,
    pub(crate) light_pos: *mut mjtNum,
    pub(crate) light_dir: *mut mjtNum,
    pub(crate) light_poscom0: *mut mjtNum,
    pub(crate) light_pos0: *mut mjtNum,
    pub(crate) light_dir0: *mut mjtNum,
    pub(crate) light_attenuation: *mut f32,
    pub(crate) light_cutoff: *mut f32,
    pub(crate) light_exponent: *mut f32,
    pub(crate) light_ambient: *mut f32,
    pub(crate) light_diffuse: *mut f32,
    pub(crate) light_specular: *mut f32,
    pub(crate) flex_contype: *mut ::core::ffi::c_int,
    pub(crate) flex_conaffinity: *mut ::core::ffi::c_int,
    pub(crate) flex_condim: *mut ::core::ffi::c_int,
    pub(crate) flex_priority: *mut ::core::ffi::c_int,
    pub(crate) flex_solmix: *mut mjtNum,
    pub(crate) flex_solref: *mut mjtNum,
    pub(crate) flex_solimp: *mut mjtNum,
    pub(crate) flex_friction: *mut mjtNum,
    pub(crate) flex_margin: *mut mjtNum,
    pub(crate) flex_gap: *mut mjtNum,
    pub(crate) flex_internal: *mut mjtBool,
    pub(crate) flex_selfcollide: *mut ::core::ffi::c_int,
    pub(crate) flex_activelayers: *mut ::core::ffi::c_int,
    pub(crate) flex_passive: *mut ::core::ffi::c_int,
    pub(crate) flex_dim: *mut ::core::ffi::c_int,
    pub(crate) flex_matid: *mut ::core::ffi::c_int,
    pub(crate) flex_group: *mut ::core::ffi::c_int,
    pub(crate) flex_interp: *mut ::core::ffi::c_int,
    pub(crate) flex_cellnum: *mut ::core::ffi::c_int,
    pub(crate) flex_nodeadr: *mut ::core::ffi::c_int,
    pub(crate) flex_nodenum: *mut ::core::ffi::c_int,
    pub(crate) flex_vertadr: *mut ::core::ffi::c_int,
    pub(crate) flex_vertnum: *mut ::core::ffi::c_int,
    pub(crate) flex_edgeadr: *mut ::core::ffi::c_int,
    pub(crate) flex_edgenum: *mut ::core::ffi::c_int,
    pub(crate) flex_elemadr: *mut ::core::ffi::c_int,
    pub(crate) flex_elemnum: *mut ::core::ffi::c_int,
    pub(crate) flex_elemdataadr: *mut ::core::ffi::c_int,
    pub(crate) flex_stiffnessadr: *mut ::core::ffi::c_int,
    pub(crate) flex_elemedgeadr: *mut ::core::ffi::c_int,
    pub(crate) flex_bendingadr: *mut ::core::ffi::c_int,
    pub(crate) flex_shellnum: *mut ::core::ffi::c_int,
    pub(crate) flex_shelldataadr: *mut ::core::ffi::c_int,
    pub(crate) flex_evpairadr: *mut ::core::ffi::c_int,
    pub(crate) flex_evpairnum: *mut ::core::ffi::c_int,
    pub(crate) flex_texcoordadr: *mut ::core::ffi::c_int,
    pub(crate) flex_nodebodyid: *mut ::core::ffi::c_int,
    pub(crate) flex_vertbodyid: *mut ::core::ffi::c_int,
    pub(crate) flex_vertedgeadr: *mut ::core::ffi::c_int,
    pub(crate) flex_vertedgenum: *mut ::core::ffi::c_int,
    pub(crate) flex_vertedge: *mut ::core::ffi::c_int,
    pub(crate) flex_edge: *mut ::core::ffi::c_int,
    pub(crate) flex_edgeflap: *mut ::core::ffi::c_int,
    pub(crate) flex_elem: *mut ::core::ffi::c_int,
    pub(crate) flex_elemtexcoord: *mut ::core::ffi::c_int,
    pub(crate) flex_elemedge: *mut ::core::ffi::c_int,
    pub(crate) flex_elemlayer: *mut ::core::ffi::c_int,
    pub(crate) flex_shell: *mut ::core::ffi::c_int,
    pub(crate) flex_evpair: *mut ::core::ffi::c_int,
    pub(crate) flex_vert: *mut mjtNum,
    pub(crate) flex_vert0: *mut mjtNum,
    pub(crate) flex_vertmetric: *mut mjtNum,
    pub(crate) flex_node: *mut mjtNum,
    pub(crate) flex_node0: *mut mjtNum,
    pub(crate) flexedge_length0: *mut mjtNum,
    pub(crate) flexedge_invweight0: *mut mjtNum,
    pub(crate) flex_radius: *mut mjtNum,
    pub(crate) flex_size: *mut mjtNum,
    pub(crate) flex_stiffness: *mut mjtNum,
    pub(crate) flex_bending: *mut mjtNum,
    pub(crate) efm0_dofid: *mut ::core::ffi::c_int,
    pub(crate) efm0_L_rownnz: *mut ::core::ffi::c_int,
    pub(crate) efm0_L_rowadr: *mut ::core::ffi::c_int,
    pub(crate) efm0_L_colind: *mut ::core::ffi::c_int,
    pub(crate) efm0_L: *mut mjtNum,
    pub(crate) flex_damping: *mut mjtNum,
    pub(crate) flex_edgestiffness: *mut mjtNum,
    pub(crate) flex_edgedamping: *mut mjtNum,
    pub(crate) flex_edgeequality: *mut ::core::ffi::c_int,
    pub(crate) flex_rigid: *mut mjtBool,
    pub(crate) flexedge_rigid: *mut mjtBool,
    pub(crate) flex_centered: *mut mjtBool,
    pub(crate) flex_flatskin: *mut mjtBool,
    pub(crate) flex_bvhadr: *mut ::core::ffi::c_int,
    pub(crate) flex_bvhnum: *mut ::core::ffi::c_int,
    pub(crate) flexedge_J_rownnz: *mut ::core::ffi::c_int,
    pub(crate) flexedge_J_rowadr: *mut ::core::ffi::c_int,
    pub(crate) flexedge_J_colind: *mut ::core::ffi::c_int,
    pub(crate) flexvert_J_rownnz: *mut ::core::ffi::c_int,
    pub(crate) flexvert_J_rowadr: *mut ::core::ffi::c_int,
    pub(crate) flexvert_J_colind: *mut ::core::ffi::c_int,
    pub(crate) flex_rgba: *mut f32,
    pub(crate) flex_texcoord: *mut f32,
    pub(crate) mesh_vertadr: *mut ::core::ffi::c_int,
    pub(crate) mesh_vertnum: *mut ::core::ffi::c_int,
    pub(crate) mesh_faceadr: *mut ::core::ffi::c_int,
    pub(crate) mesh_facenum: *mut ::core::ffi::c_int,
    pub(crate) mesh_bvhadr: *mut ::core::ffi::c_int,
    pub(crate) mesh_bvhnum: *mut ::core::ffi::c_int,
    pub(crate) mesh_octadr: *mut ::core::ffi::c_int,
    pub(crate) mesh_octnum: *mut ::core::ffi::c_int,
    pub(crate) mesh_normaladr: *mut ::core::ffi::c_int,
    pub(crate) mesh_normalnum: *mut ::core::ffi::c_int,
    pub(crate) mesh_texcoordadr: *mut ::core::ffi::c_int,
    pub(crate) mesh_texcoordnum: *mut ::core::ffi::c_int,
    pub(crate) mesh_graphadr: *mut ::core::ffi::c_int,
    pub(crate) mesh_vert: *mut f32,
    pub(crate) mesh_normal: *mut f32,
    pub(crate) mesh_texcoord: *mut f32,
    pub(crate) mesh_face: *mut ::core::ffi::c_int,
    pub(crate) mesh_facenormal: *mut ::core::ffi::c_int,
    pub(crate) mesh_facetexcoord: *mut ::core::ffi::c_int,
    pub(crate) mesh_graph: *mut ::core::ffi::c_int,
    pub(crate) mesh_scale: *mut mjtNum,
    pub(crate) mesh_pos: *mut mjtNum,
    pub(crate) mesh_quat: *mut mjtNum,
    pub(crate) mesh_pathadr: *mut ::core::ffi::c_int,
    pub(crate) mesh_polynum: *mut ::core::ffi::c_int,
    pub(crate) mesh_polyadr: *mut ::core::ffi::c_int,
    pub(crate) mesh_polynormal: *mut mjtNum,
    pub(crate) mesh_polyvertadr: *mut ::core::ffi::c_int,
    pub(crate) mesh_polyvertnum: *mut ::core::ffi::c_int,
    pub(crate) mesh_polyvert: *mut ::core::ffi::c_int,
    pub(crate) mesh_polymapadr: *mut ::core::ffi::c_int,
    pub(crate) mesh_polymapnum: *mut ::core::ffi::c_int,
    pub(crate) mesh_polymap: *mut ::core::ffi::c_int,
    pub(crate) skin_matid: *mut ::core::ffi::c_int,
    pub(crate) skin_group: *mut ::core::ffi::c_int,
    pub(crate) skin_rgba: *mut f32,
    pub(crate) skin_inflate: *mut f32,
    pub(crate) skin_vertadr: *mut ::core::ffi::c_int,
    pub(crate) skin_vertnum: *mut ::core::ffi::c_int,
    pub(crate) skin_texcoordadr: *mut ::core::ffi::c_int,
    pub(crate) skin_faceadr: *mut ::core::ffi::c_int,
    pub(crate) skin_facenum: *mut ::core::ffi::c_int,
    pub(crate) skin_boneadr: *mut ::core::ffi::c_int,
    pub(crate) skin_bonenum: *mut ::core::ffi::c_int,
    pub(crate) skin_vert: *mut f32,
    pub(crate) skin_texcoord: *mut f32,
    pub(crate) skin_face: *mut ::core::ffi::c_int,
    pub(crate) skin_bonevertadr: *mut ::core::ffi::c_int,
    pub(crate) skin_bonevertnum: *mut ::core::ffi::c_int,
    pub(crate) skin_bonebindpos: *mut f32,
    pub(crate) skin_bonebindquat: *mut f32,
    pub(crate) skin_bonebodyid: *mut ::core::ffi::c_int,
    pub(crate) skin_bonevertid: *mut ::core::ffi::c_int,
    pub(crate) skin_bonevertweight: *mut f32,
    pub(crate) skin_pathadr: *mut ::core::ffi::c_int,
    pub(crate) hfield_size: *mut mjtNum,
    pub(crate) hfield_nrow: *mut ::core::ffi::c_int,
    pub(crate) hfield_ncol: *mut ::core::ffi::c_int,
    pub(crate) hfield_adr: *mut ::core::ffi::c_int,
    pub(crate) hfield_data: *mut f32,
    pub(crate) hfield_pathadr: *mut ::core::ffi::c_int,
    pub(crate) tex_type: *mut ::core::ffi::c_int,
    pub(crate) tex_colorspace: *mut ::core::ffi::c_int,
    pub(crate) tex_height: *mut ::core::ffi::c_int,
    pub(crate) tex_width: *mut ::core::ffi::c_int,
    pub(crate) tex_nchannel: *mut ::core::ffi::c_int,
    pub(crate) tex_adr: *mut mjtSize,
    pub(crate) tex_data: *mut mjtByte,
    pub(crate) tex_pathadr: *mut ::core::ffi::c_int,
    pub(crate) mat_texid: *mut ::core::ffi::c_int,
    pub(crate) mat_texuniform: *mut mjtBool,
    pub(crate) mat_texrepeat: *mut f32,
    pub(crate) mat_emission: *mut f32,
    pub(crate) mat_specular: *mut f32,
    pub(crate) mat_shininess: *mut f32,
    pub(crate) mat_reflectance: *mut f32,
    pub(crate) mat_metallic: *mut f32,
    pub(crate) mat_roughness: *mut f32,
    pub(crate) mat_rgba: *mut f32,
    pub(crate) pair_dim: *mut ::core::ffi::c_int,
    pub(crate) pair_geom1: *mut ::core::ffi::c_int,
    pub(crate) pair_geom2: *mut ::core::ffi::c_int,
    pub(crate) pair_signature: *mut ::core::ffi::c_int,
    pub(crate) pair_solref: *mut mjtNum,
    pub(crate) pair_solreffriction: *mut mjtNum,
    pub(crate) pair_solimp: *mut mjtNum,
    pub(crate) pair_margin: *mut mjtNum,
    pub(crate) pair_gap: *mut mjtNum,
    pub(crate) pair_adhesion: *mut mjtNum,
    pub(crate) pair_friction: *mut mjtNum,
    pub(crate) exclude_signature: *mut ::core::ffi::c_int,
    pub(crate) eq_type: *mut ::core::ffi::c_int,
    pub(crate) eq_obj1id: *mut ::core::ffi::c_int,
    pub(crate) eq_obj2id: *mut ::core::ffi::c_int,
    pub(crate) eq_objtype: *mut ::core::ffi::c_int,
    pub(crate) eq_active0: *mut mjtBool,
    pub(crate) eq_solref: *mut mjtNum,
    pub(crate) eq_solimp: *mut mjtNum,
    pub(crate) eq_data: *mut mjtNum,
    pub(crate) tendon_adr: *mut ::core::ffi::c_int,
    pub(crate) tendon_num: *mut ::core::ffi::c_int,
    pub(crate) tendon_matid: *mut ::core::ffi::c_int,
    pub(crate) tendon_actuatorid: *mut ::core::ffi::c_int,
    pub(crate) tendon_group: *mut ::core::ffi::c_int,
    pub(crate) tendon_treenum: *mut ::core::ffi::c_int,
    pub(crate) tendon_treeid: *mut ::core::ffi::c_int,
    pub(crate) ten_J_rownnz: *mut ::core::ffi::c_int,
    pub(crate) ten_J_rowadr: *mut ::core::ffi::c_int,
    pub(crate) ten_J_colind: *mut ::core::ffi::c_int,
    pub(crate) tendon_limited: *mut mjtBool,
    pub(crate) tendon_actfrclimited: *mut mjtBool,
    pub(crate) tendon_width: *mut mjtNum,
    pub(crate) tendon_solref_lim: *mut mjtNum,
    pub(crate) tendon_solimp_lim: *mut mjtNum,
    pub(crate) tendon_solref_fri: *mut mjtNum,
    pub(crate) tendon_solimp_fri: *mut mjtNum,
    pub(crate) tendon_range: *mut mjtNum,
    pub(crate) tendon_actfrcrange: *mut mjtNum,
    pub(crate) tendon_margin: *mut mjtNum,
    pub(crate) tendon_stiffness: *mut mjtNum,
    pub(crate) tendon_stiffnesspoly: *mut mjtNum,
    pub(crate) tendon_damping: *mut mjtNum,
    pub(crate) tendon_dampingpoly: *mut mjtNum,
    pub(crate) tendon_armature: *mut mjtNum,
    pub(crate) tendon_frictionloss: *mut mjtNum,
    pub(crate) tendon_lengthspring: *mut mjtNum,
    pub(crate) tendon_length0: *mut mjtNum,
    pub(crate) tendon_invweight0: *mut mjtNum,
    pub(crate) tendon_user: *mut mjtNum,
    pub(crate) tendon_rgba: *mut f32,
    pub(crate) wrap_type: *mut ::core::ffi::c_int,
    pub(crate) wrap_objid: *mut ::core::ffi::c_int,
    pub(crate) wrap_prm: *mut mjtNum,
    pub(crate) actuator_trntype: *mut ::core::ffi::c_int,
    pub(crate) actuator_dyntype: *mut ::core::ffi::c_int,
    pub(crate) actuator_gaintype: *mut ::core::ffi::c_int,
    pub(crate) actuator_biastype: *mut ::core::ffi::c_int,
    pub(crate) actuator_ctrladr: *mut ::core::ffi::c_int,
    pub(crate) actuator_ctrlnum: *mut ::core::ffi::c_int,
    pub(crate) actuator_ctrlspec: *mut ::core::ffi::c_int,
    pub(crate) actuator_outadr: *mut ::core::ffi::c_int,
    pub(crate) actuator_outnum: *mut ::core::ffi::c_int,
    pub(crate) actuator_actadr: *mut ::core::ffi::c_int,
    pub(crate) actuator_actnum: *mut ::core::ffi::c_int,
    pub(crate) actuator_trnid: *mut ::core::ffi::c_int,
    pub(crate) actuator_cranklength: *mut mjtNum,
    pub(crate) actuator_dynprm: *mut mjtNum,
    pub(crate) actuator_gainprm: *mut mjtNum,
    pub(crate) actuator_biasprm: *mut mjtNum,
    pub(crate) actuator_actlimited: *mut mjtBool,
    pub(crate) actuator_actrange: *mut mjtNum,
    pub(crate) actuator_actearly: *mut mjtBool,
    pub(crate) actuator_history: *mut ::core::ffi::c_int,
    pub(crate) actuator_historyadr: *mut ::core::ffi::c_int,
    pub(crate) actuator_delay: *mut mjtNum,
    pub(crate) actuator_damping: *mut mjtNum,
    pub(crate) actuator_dampingpoly: *mut mjtNum,
    pub(crate) actuator_armature: *mut mjtNum,
    pub(crate) actuator_group: *mut ::core::ffi::c_int,
    pub(crate) actuator_user: *mut mjtNum,
    pub(crate) actuator_plugin: *mut ::core::ffi::c_int,
    pub(crate) actuator_forcelimited: *mut mjtBool,
    pub(crate) actuator_forcerange: *mut mjtNum,
    pub(crate) actuator_ctrllimited: *mut mjtBool,
    pub(crate) actuator_ctrlrange: *mut mjtNum,
    pub(crate) actuator_gear: *mut mjtNum,
    pub(crate) actuator_acc0: *mut mjtNum,
    pub(crate) actuator_length0: *mut mjtNum,
    pub(crate) actuator_lengthrange: *mut mjtNum,
    pub(crate) sensor_type: *mut ::core::ffi::c_int,
    pub(crate) sensor_datatype: *mut ::core::ffi::c_int,
    pub(crate) sensor_needstage: *mut ::core::ffi::c_int,
    pub(crate) sensor_objtype: *mut ::core::ffi::c_int,
    pub(crate) sensor_objid: *mut ::core::ffi::c_int,
    pub(crate) sensor_reftype: *mut ::core::ffi::c_int,
    pub(crate) sensor_refid: *mut ::core::ffi::c_int,
    pub(crate) sensor_intprm: *mut ::core::ffi::c_int,
    pub(crate) sensor_dim: *mut ::core::ffi::c_int,
    pub(crate) sensor_adr: *mut ::core::ffi::c_int,
    pub(crate) sensor_cutoff: *mut mjtNum,
    pub(crate) sensor_noise: *mut mjtNum,
    pub(crate) sensor_history: *mut ::core::ffi::c_int,
    pub(crate) sensor_historyadr: *mut ::core::ffi::c_int,
    pub(crate) sensor_delay: *mut mjtNum,
    pub(crate) sensor_interval: *mut mjtNum,
    pub(crate) sensor_user: *mut mjtNum,
    pub(crate) sensor_plugin: *mut ::core::ffi::c_int,
    pub(crate) plugin: *mut ::core::ffi::c_int,
    pub(crate) plugin_stateadr: *mut ::core::ffi::c_int,
    pub(crate) plugin_statenum: *mut ::core::ffi::c_int,
    pub(crate) plugin_attr: *mut ::core::ffi::c_char,
    pub(crate) plugin_attradr: *mut ::core::ffi::c_int,
    pub(crate) numeric_adr: *mut ::core::ffi::c_int,
    pub(crate) numeric_size: *mut ::core::ffi::c_int,
    pub(crate) numeric_data: *mut mjtNum,
    pub(crate) text_adr: *mut ::core::ffi::c_int,
    pub(crate) text_size: *mut ::core::ffi::c_int,
    pub(crate) text_data: *mut ::core::ffi::c_char,
    pub(crate) tuple_adr: *mut ::core::ffi::c_int,
    pub(crate) tuple_size: *mut ::core::ffi::c_int,
    pub(crate) tuple_objtype: *mut ::core::ffi::c_int,
    pub(crate) tuple_objid: *mut ::core::ffi::c_int,
    pub(crate) tuple_objprm: *mut mjtNum,
    pub(crate) key_time: *mut mjtNum,
    pub(crate) key_qpos: *mut mjtNum,
    pub(crate) key_qvel: *mut mjtNum,
    pub(crate) key_act: *mut mjtNum,
    pub(crate) key_mpos: *mut mjtNum,
    pub(crate) key_mquat: *mut mjtNum,
    pub(crate) key_ctrl: *mut mjtNum,
    pub(crate) name_bodyadr: *mut ::core::ffi::c_int,
    pub(crate) name_jntadr: *mut ::core::ffi::c_int,
    pub(crate) name_geomadr: *mut ::core::ffi::c_int,
    pub(crate) name_siteadr: *mut ::core::ffi::c_int,
    pub(crate) name_camadr: *mut ::core::ffi::c_int,
    pub(crate) name_lightadr: *mut ::core::ffi::c_int,
    pub(crate) name_flexadr: *mut ::core::ffi::c_int,
    pub(crate) name_meshadr: *mut ::core::ffi::c_int,
    pub(crate) name_skinadr: *mut ::core::ffi::c_int,
    pub(crate) name_hfieldadr: *mut ::core::ffi::c_int,
    pub(crate) name_texadr: *mut ::core::ffi::c_int,
    pub(crate) name_matadr: *mut ::core::ffi::c_int,
    pub(crate) name_pairadr: *mut ::core::ffi::c_int,
    pub(crate) name_excludeadr: *mut ::core::ffi::c_int,
    pub(crate) name_eqadr: *mut ::core::ffi::c_int,
    pub(crate) name_tendonadr: *mut ::core::ffi::c_int,
    pub(crate) name_actuatoradr: *mut ::core::ffi::c_int,
    pub(crate) name_sensoradr: *mut ::core::ffi::c_int,
    pub(crate) name_numericadr: *mut ::core::ffi::c_int,
    pub(crate) name_textadr: *mut ::core::ffi::c_int,
    pub(crate) name_tupleadr: *mut ::core::ffi::c_int,
    pub(crate) name_keyadr: *mut ::core::ffi::c_int,
    pub(crate) name_pluginadr: *mut ::core::ffi::c_int,
    pub(crate) names: *mut ::core::ffi::c_char,
    pub(crate) names_map: *mut ::core::ffi::c_int,
    pub(crate) paths: *mut ::core::ffi::c_char,
    pub(crate) B_rownnz: *mut ::core::ffi::c_int,
    pub(crate) B_rowadr: *mut ::core::ffi::c_int,
    pub(crate) B_colind: *mut ::core::ffi::c_int,
    pub(crate) M_rownnz: *mut ::core::ffi::c_int,
    pub(crate) M_rowadr: *mut ::core::ffi::c_int,
    pub(crate) M_colind: *mut ::core::ffi::c_int,
    pub(crate) mapM2M: *mut ::core::ffi::c_int,
    pub(crate) D_rownnz: *mut ::core::ffi::c_int,
    pub(crate) D_rowadr: *mut ::core::ffi::c_int,
    pub(crate) D_diag: *mut ::core::ffi::c_int,
    pub(crate) D_colind: *mut ::core::ffi::c_int,
    pub(crate) mapM2D: *mut ::core::ffi::c_int,
    pub(crate) mapD2M: *mut ::core::ffi::c_int,
    pub signature: u64,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjPreContact {
    pub dist: mjtNum,
    pub pos: [mjtNum; 3usize],
    pub normal: [mjtNum; 3usize],
    pub tangent: [mjtNum; 3usize],
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjContact {
    pub dist: mjtNum,
    pub pos: [mjtNum; 3usize],
    pub frame: [mjtNum; 9usize],
    pub includemargin: mjtNum,
    pub friction: [mjtNum; 5usize],
    pub solref: [mjtNum; 2usize],
    pub solreffriction: [mjtNum; 2usize],
    pub solimp: [mjtNum; 5usize],
    pub adhesion: mjtNum,
    pub mu: mjtNum,
    pub H: [mjtNum; 36usize],
    pub dim: ::core::ffi::c_int,
    pub geom1: ::core::ffi::c_int,
    pub geom2: ::core::ffi::c_int,
    pub geom: [::core::ffi::c_int; 2usize],
    pub flex: [::core::ffi::c_int; 2usize],
    pub elem: [::core::ffi::c_int; 2usize],
    pub vert: [::core::ffi::c_int; 2usize],
    pub exclude: ::core::ffi::c_int,
    pub efc_address: ::core::ffi::c_int,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjWarningStat {
    pub lastinfo: ::core::ffi::c_int,
    pub number: ::core::ffi::c_int,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjTimerStat {
    pub duration: mjtNum,
    pub number: ::core::ffi::c_int,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjSolverStat {
    pub improvement: mjtNum,
    pub gradient: mjtNum,
    pub lineslope: mjtNum,
    pub nactive: ::core::ffi::c_int,
    pub nchange: ::core::ffi::c_int,
    pub neval: ::core::ffi::c_int,
    pub nupdate: ::core::ffi::c_int,
}
#[repr(C)]
#[derive(Debug)]
pub struct mjData {
    pub narena: mjtSize,
    pub nbuffer: mjtSize,
    pub nplugin: ::core::ffi::c_int,
    pub pstack: usize,
    pub pbase: usize,
    pub parena: usize,
    pub threadpool: usize,
    pub threadlock: mjtBool,
    pub maxuse_stack: mjtSize,
    pub maxuse_arena: mjtSize,
    pub maxuse_con: ::core::ffi::c_int,
    pub maxuse_efc: ::core::ffi::c_int,
    pub solver: [mjSolverStat; 4000usize],
    pub solver_niter: [::core::ffi::c_int; 20usize],
    pub solver_nnz: [::core::ffi::c_int; 20usize],
    pub solver_fwdinv: [mjtNum; 2usize],
    pub warning: [mjWarningStat; 7usize],
    pub timer: [mjTimerStat; 15usize],
    pub ncon: ::core::ffi::c_int,
    pub ne: ::core::ffi::c_int,
    pub nf: ::core::ffi::c_int,
    pub nl: ::core::ffi::c_int,
    pub nefc: ::core::ffi::c_int,
    pub nJ: ::core::ffi::c_int,
    pub efm_active: ::core::ffi::c_int,
    pub nefmK: ::core::ffi::c_int,
    pub nefmdof: ::core::ffi::c_int,
    pub nefmL: ::core::ffi::c_int,
    pub nY: ::core::ffi::c_int,
    pub nA: ::core::ffi::c_int,
    pub nisland: ::core::ffi::c_int,
    pub nidof: ::core::ffi::c_int,
    pub ntree_awake: ::core::ffi::c_int,
    pub nbody_awake: ::core::ffi::c_int,
    pub nparent_awake: ::core::ffi::c_int,
    pub nv_awake: ::core::ffi::c_int,
    pub flg_energypos: mjtBool,
    pub flg_energyvel: mjtBool,
    pub flg_subtreevel: mjtBool,
    pub flg_rnepost: mjtBool,
    pub time: mjtNum,
    pub energy: [mjtNum; 2usize],
    pub buffer: *mut ::core::ffi::c_void,
    pub arena: *mut ::core::ffi::c_void,
    pub(crate) qpos: *mut mjtNum,
    pub(crate) qvel: *mut mjtNum,
    pub(crate) act: *mut mjtNum,
    pub(crate) history: *mut mjtNum,
    pub(crate) qacc_warmstart: *mut mjtNum,
    pub(crate) plugin_state: *mut mjtNum,
    pub(crate) ctrl: *mut mjtNum,
    pub(crate) qfrc_applied: *mut mjtNum,
    pub(crate) xfrc_applied: *mut mjtNum,
    pub(crate) eq_active: *mut mjtBool,
    pub(crate) mocap_pos: *mut mjtNum,
    pub(crate) mocap_quat: *mut mjtNum,
    pub(crate) qacc: *mut mjtNum,
    pub(crate) act_dot: *mut mjtNum,
    pub(crate) userdata: *mut mjtNum,
    pub(crate) sensordata: *mut mjtNum,
    pub(crate) tree_asleep: *mut ::core::ffi::c_int,
    pub(crate) plugin: *mut ::core::ffi::c_int,
    pub(crate) plugin_data: *mut usize,
    pub(crate) xpos: *mut mjtNum,
    pub(crate) xquat: *mut mjtNum,
    pub(crate) xmat: *mut mjtNum,
    pub(crate) xipos: *mut mjtNum,
    pub(crate) ximat: *mut mjtNum,
    pub(crate) xanchor: *mut mjtNum,
    pub(crate) xaxis: *mut mjtNum,
    pub(crate) geom_xpos: *mut mjtNum,
    pub(crate) geom_xmat: *mut mjtNum,
    pub(crate) site_xpos: *mut mjtNum,
    pub(crate) site_xmat: *mut mjtNum,
    pub(crate) cam_xpos: *mut mjtNum,
    pub(crate) cam_xmat: *mut mjtNum,
    pub(crate) light_xpos: *mut mjtNum,
    pub(crate) light_xdir: *mut mjtNum,
    pub(crate) subtree_com: *mut mjtNum,
    pub(crate) cdof: *mut mjtNum,
    pub(crate) cinert: *mut mjtNum,
    pub(crate) flexvert_xpos: *mut mjtNum,
    pub(crate) flexelem_aabb: *mut mjtNum,
    pub(crate) flexelem_krot: *mut mjtNum,
    pub(crate) flexedge_J: *mut mjtNum,
    pub(crate) flexedge_length: *mut mjtNum,
    pub(crate) flexvert_J: *mut mjtNum,
    pub(crate) flexvert_length: *mut mjtNum,
    pub(crate) bvh_aabb_dyn: *mut mjtNum,
    pub(crate) ten_wrapadr: *mut ::core::ffi::c_int,
    pub(crate) ten_wrapnum: *mut ::core::ffi::c_int,
    pub(crate) ten_J: *mut mjtNum,
    pub(crate) ten_length: *mut mjtNum,
    pub(crate) wrap_obj: *mut ::core::ffi::c_int,
    pub(crate) wrap_xpos: *mut mjtNum,
    pub(crate) actuator_length: *mut mjtNum,
    pub(crate) moment_rownnz: *mut ::core::ffi::c_int,
    pub(crate) moment_rowadr: *mut ::core::ffi::c_int,
    pub(crate) moment_colind: *mut ::core::ffi::c_int,
    pub(crate) actuator_moment: *mut mjtNum,
    pub(crate) crb: *mut mjtNum,
    pub(crate) M: *mut mjtNum,
    pub(crate) qLD: *mut mjtNum,
    pub(crate) qLDiagInv: *mut mjtNum,
    pub(crate) bvh_active: *mut mjtBool,
    pub(crate) tree_awake: *mut ::core::ffi::c_int,
    pub(crate) body_awake: *mut ::core::ffi::c_int,
    pub(crate) body_awake_ind: *mut ::core::ffi::c_int,
    pub(crate) parent_awake_ind: *mut ::core::ffi::c_int,
    pub(crate) dof_awake_ind: *mut ::core::ffi::c_int,
    pub(crate) flexedge_velocity: *mut mjtNum,
    pub(crate) ten_velocity: *mut mjtNum,
    pub(crate) actuator_velocity: *mut mjtNum,
    pub(crate) cvel: *mut mjtNum,
    pub(crate) cdof_dot: *mut mjtNum,
    pub(crate) qfrc_bias: *mut mjtNum,
    pub(crate) qfrc_spring: *mut mjtNum,
    pub(crate) qfrc_damper: *mut mjtNum,
    pub(crate) qfrc_gravcomp: *mut mjtNum,
    pub(crate) qfrc_fluid: *mut mjtNum,
    pub(crate) qfrc_adhesion: *mut mjtNum,
    pub(crate) qfrc_passive: *mut mjtNum,
    pub(crate) subtree_linvel: *mut mjtNum,
    pub(crate) subtree_angmom: *mut mjtNum,
    pub(crate) qH: *mut mjtNum,
    pub(crate) qHDiagInv: *mut mjtNum,
    pub(crate) qDeriv: *mut mjtNum,
    pub(crate) qLU: *mut mjtNum,
    pub(crate) actuator_force: *mut mjtNum,
    pub(crate) qfrc_actuator: *mut mjtNum,
    pub(crate) qfrc_smooth: *mut mjtNum,
    pub(crate) qacc_smooth: *mut mjtNum,
    pub(crate) qfrc_constraint: *mut mjtNum,
    pub(crate) qfrc_inverse: *mut mjtNum,
    pub(crate) cacc: *mut mjtNum,
    pub(crate) cfrc_int: *mut mjtNum,
    pub(crate) cfrc_ext: *mut mjtNum,
    pub(crate) contact: *mut mjContact,
    pub(crate) efc_type: *mut ::core::ffi::c_int,
    pub(crate) efc_id: *mut ::core::ffi::c_int,
    pub(crate) efc_J_rownnz: *mut ::core::ffi::c_int,
    pub(crate) efc_J_rowadr: *mut ::core::ffi::c_int,
    pub(crate) efc_J_rowsuper: *mut ::core::ffi::c_int,
    pub(crate) efc_J_colind: *mut ::core::ffi::c_int,
    pub(crate) efc_J: *mut mjtNum,
    pub(crate) efc_pos: *mut mjtNum,
    pub(crate) efc_margin: *mut mjtNum,
    pub(crate) efc_frictionloss: *mut mjtNum,
    pub(crate) efc_diagA: *mut mjtNum,
    pub(crate) efc_KBIP: *mut mjtNum,
    pub(crate) efc_D: *mut mjtNum,
    pub(crate) efc_R: *mut mjtNum,
    pub(crate) tendon_efcadr: *mut ::core::ffi::c_int,
    pub(crate) tree_island: *mut ::core::ffi::c_int,
    pub(crate) island_ntree: *mut ::core::ffi::c_int,
    pub(crate) island_itreeadr: *mut ::core::ffi::c_int,
    pub(crate) map_itree2tree: *mut ::core::ffi::c_int,
    pub(crate) dof_island: *mut ::core::ffi::c_int,
    pub(crate) island_nv: *mut ::core::ffi::c_int,
    pub(crate) island_idofadr: *mut ::core::ffi::c_int,
    pub(crate) island_dofadr: *mut ::core::ffi::c_int,
    pub(crate) map_dof2idof: *mut ::core::ffi::c_int,
    pub(crate) map_idof2dof: *mut ::core::ffi::c_int,
    pub(crate) ifrc_smooth: *mut mjtNum,
    pub(crate) iacc_smooth: *mut mjtNum,
    pub(crate) iacc: *mut mjtNum,
    pub(crate) efc_island: *mut ::core::ffi::c_int,
    pub(crate) island_ne: *mut ::core::ffi::c_int,
    pub(crate) island_nf: *mut ::core::ffi::c_int,
    pub(crate) island_nefc: *mut ::core::ffi::c_int,
    pub(crate) island_iefcadr: *mut ::core::ffi::c_int,
    pub(crate) map_efc2iefc: *mut ::core::ffi::c_int,
    pub(crate) map_iefc2efc: *mut ::core::ffi::c_int,
    pub(crate) iefc_type: *mut ::core::ffi::c_int,
    pub(crate) iefc_id: *mut ::core::ffi::c_int,
    pub(crate) iefc_frictionloss: *mut mjtNum,
    pub(crate) iefc_D: *mut mjtNum,
    pub(crate) iefc_R: *mut mjtNum,
    pub(crate) efc_Y_rownnz: *mut ::core::ffi::c_int,
    pub(crate) efc_Y_rowadr: *mut ::core::ffi::c_int,
    pub(crate) efc_Y_colind: *mut ::core::ffi::c_int,
    pub(crate) efc_Y: *mut mjtNum,
    pub(crate) efc_AR_rownnz: *mut ::core::ffi::c_int,
    pub(crate) efc_AR_rowadr: *mut ::core::ffi::c_int,
    pub(crate) efc_AR_colind: *mut ::core::ffi::c_int,
    pub(crate) efc_AR: *mut mjtNum,
    pub(crate) efc_vel: *mut mjtNum,
    pub(crate) efc_aref: *mut mjtNum,
    pub(crate) efm_c: *mut mjtNum,
    pub(crate) efm_K_rownnz: *mut ::core::ffi::c_int,
    pub(crate) efm_K_rowadr: *mut ::core::ffi::c_int,
    pub(crate) efm_K_colind: *mut ::core::ffi::c_int,
    pub(crate) efm_K_val: *mut mjtNum,
    pub(crate) efm_dofid: *mut ::core::ffi::c_int,
    pub(crate) efm_L_rownnz: *mut ::core::ffi::c_int,
    pub(crate) efm_L_rowadr: *mut ::core::ffi::c_int,
    pub(crate) efm_L_colind: *mut ::core::ffi::c_int,
    pub(crate) efm_L: *mut mjtNum,
    pub(crate) efc_b: *mut mjtNum,
    pub(crate) iefc_aref: *mut mjtNum,
    pub(crate) iefc_state: *mut ::core::ffi::c_int,
    pub(crate) iefc_force: *mut mjtNum,
    pub(crate) efc_state: *mut ::core::ffi::c_int,
    pub(crate) efc_force: *mut mjtNum,
    pub(crate) ifrc_constraint: *mut mjtNum,
    pub signature: u64,
}
pub(crate) type mjfGeneric =
    ::core::option::Option<unsafe extern "C" fn(m: *const mjModel, d: *mut mjData)>;
pub(crate) type mjfConFilt = ::core::option::Option<
    unsafe extern "C" fn(
        m: *const mjModel,
        d: *mut mjData,
        geom1: ::core::ffi::c_int,
        geom2: ::core::ffi::c_int,
    ) -> ::core::ffi::c_int,
>;
pub(crate) type mjfSensor = ::core::option::Option<
    unsafe extern "C" fn(m: *const mjModel, d: *mut mjData, stage: ::core::ffi::c_int),
>;
pub(crate) type mjfTime = ::core::option::Option<unsafe extern "C" fn() -> mjtNum>;
pub(crate) type mjfAct = ::core::option::Option<
    unsafe extern "C" fn(m: *const mjModel, d: *const mjData, id: ::core::ffi::c_int) -> mjtNum,
>;
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
pub type mjString = ::core::ffi::c_void;
pub type mjStringVec = ::core::ffi::c_void;
pub type mjIntVec = ::core::ffi::c_void;
pub type mjIntVecVec = ::core::ffi::c_void;
pub type mjFloatVec = ::core::ffi::c_void;
pub type mjFloatVecVec = ::core::ffi::c_void;
pub type mjDoubleVec = ::core::ffi::c_void;
pub type mjByteVec = ::core::ffi::c_void;
impl mjtGeomInertia {
    pub const VOLUME: mjtGeomInertia = mjtGeomInertia(0);
    pub const SHELL: mjtGeomInertia = mjtGeomInertia(1);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtGeomInertia(::core::ffi::c_uint);
impl mjtMeshInertia {
    pub const CONVEX: mjtMeshInertia = mjtMeshInertia(0);
    pub const EXACT: mjtMeshInertia = mjtMeshInertia(1);
    pub const LEGACY: mjtMeshInertia = mjtMeshInertia(2);
    pub const SHELL: mjtMeshInertia = mjtMeshInertia(3);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtMeshInertia(::core::ffi::c_uint);
impl mjtMeshBuiltin {
    pub const NONE: mjtMeshBuiltin = mjtMeshBuiltin(0);
    pub const SPHERE: mjtMeshBuiltin = mjtMeshBuiltin(1);
    pub const HEMISPHERE: mjtMeshBuiltin = mjtMeshBuiltin(2);
    pub const CONE: mjtMeshBuiltin = mjtMeshBuiltin(3);
    pub const SUPERSPHERE: mjtMeshBuiltin = mjtMeshBuiltin(4);
    pub const SUPERTORUS: mjtMeshBuiltin = mjtMeshBuiltin(5);
    pub const WEDGE: mjtMeshBuiltin = mjtMeshBuiltin(6);
    pub const PLATE: mjtMeshBuiltin = mjtMeshBuiltin(7);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtMeshBuiltin(::core::ffi::c_uint);
impl mjtBuiltin {
    pub const NONE: mjtBuiltin = mjtBuiltin(0);
    pub const GRADIENT: mjtBuiltin = mjtBuiltin(1);
    pub const CHECKER: mjtBuiltin = mjtBuiltin(2);
    pub const FLAT: mjtBuiltin = mjtBuiltin(3);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtBuiltin(::core::ffi::c_uint);
impl mjtMark {
    pub const NONE: mjtMark = mjtMark(0);
    pub const EDGE: mjtMark = mjtMark(1);
    pub const CROSS: mjtMark = mjtMark(2);
    pub const RANDOM: mjtMark = mjtMark(3);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtMark(::core::ffi::c_uint);
impl mjtLimited {
    pub const FALSE: mjtLimited = mjtLimited(0);
    pub const TRUE: mjtLimited = mjtLimited(1);
    pub const AUTO: mjtLimited = mjtLimited(2);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtLimited(::core::ffi::c_uint);
impl mjtAlignFree {
    pub const FALSE: mjtAlignFree = mjtAlignFree(0);
    pub const TRUE: mjtAlignFree = mjtAlignFree(1);
    pub const AUTO: mjtAlignFree = mjtAlignFree(2);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtAlignFree(::core::ffi::c_uint);
impl mjtInertiaFromGeom {
    pub const FALSE: mjtInertiaFromGeom = mjtInertiaFromGeom(0);
    pub const TRUE: mjtInertiaFromGeom = mjtInertiaFromGeom(1);
    pub const AUTO: mjtInertiaFromGeom = mjtInertiaFromGeom(2);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtInertiaFromGeom(::core::ffi::c_uint);
impl mjtOrientation {
    pub const QUAT: mjtOrientation = mjtOrientation(0);
    pub const AXISANGLE: mjtOrientation = mjtOrientation(1);
    pub const XYAXES: mjtOrientation = mjtOrientation(2);
    pub const ZAXIS: mjtOrientation = mjtOrientation(3);
    pub const EULER: mjtOrientation = mjtOrientation(4);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtOrientation(::core::ffi::c_uint);
impl mjtConflict {
    pub const WARNING: mjtConflict = mjtConflict(0);
    pub const MERGE: mjtConflict = mjtConflict(1);
    pub const ERROR: mjtConflict = mjtConflict(2);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtConflict(::core::ffi::c_uint);
pub const mjNCTIMER: usize = mjtCTimer::mjNCTIMER.0 as usize;
impl mjtCTimer {
    pub const TOTAL: mjtCTimer = mjtCTimer(0);
    pub const ASSETS: mjtCTimer = mjtCTimer(1);
    pub const TEXTURE: mjtCTimer = mjtCTimer(2);
    pub const MESH_LOAD: mjtCTimer = mjtCTimer(3);
    pub const MESH_HULL: mjtCTimer = mjtCTimer(4);
    pub const MESH_POLYGON: mjtCTimer = mjtCTimer(5);
    pub const MESH_INERTIA: mjtCTimer = mjtCTimer(6);
    pub const MESH_BVH: mjtCTimer = mjtCTimer(7);
    pub const MESH_OCTREE: mjtCTimer = mjtCTimer(8);
    const mjNCTIMER: mjtCTimer = mjtCTimer(9);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtCTimer(::core::ffi::c_uint);
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsElement {
    pub elemtype: mjtObj,
    pub signature: u64,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsCompiler {
    pub autolimits: mjtBool,
    pub boundmass: f64,
    pub boundinertia: f64,
    pub settotalmass: f64,
    pub balanceinertia: mjtBool,
    pub fitaabb: mjtBool,
    pub degree: mjtBool,
    pub(crate) eulerseq: [::core::ffi::c_char; 3usize],
    pub discardvisual: mjtBool,
    pub usethread: mjtBool,
    pub fusestatic: mjtBool,
    pub inertiafromgeom: mjtInertiaFromGeom,
    pub inertiagrouprange: [::core::ffi::c_int; 2usize],
    pub saveinertial: mjtBool,
    pub alignfree: mjtBool,
    pub conflict: mjtConflict,
    pub LRopt: mjLROpt,
    pub(crate) meshdir: *mut mjString,
    pub(crate) texturedir: *mut mjString,
    pub authored: u64,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsAuthored {
    pub option: u64,
    pub disableflags: ::core::ffi::c_int,
    pub enableflags: ::core::ffi::c_int,
    pub disableactuator: ::core::ffi::c_int,
    pub visual_global: u64,
    pub visual_quality: u64,
    pub visual_headlight: u64,
    pub visual_map: u64,
    pub visual_scale: u64,
    pub visual_rgba: u64,
}
#[repr(C)]
#[derive(Debug)]
pub struct mjSpec {
    pub(crate) element: *mut mjsElement,
    pub(crate) modelname: *mut mjString,
    pub compiler: mjsCompiler,
    pub strippath: mjtBool,
    pub option: mjOption,
    pub visual: mjVisual,
    pub stat: mjStatistic,
    pub memory: mjtSize,
    pub nemax: ::core::ffi::c_int,
    pub nuserdata: ::core::ffi::c_int,
    pub nuser_body: ::core::ffi::c_int,
    pub nuser_jnt: ::core::ffi::c_int,
    pub nuser_geom: ::core::ffi::c_int,
    pub nuser_site: ::core::ffi::c_int,
    pub nuser_cam: ::core::ffi::c_int,
    pub nuser_tendon: ::core::ffi::c_int,
    pub nuser_actuator: ::core::ffi::c_int,
    pub nuser_sensor: ::core::ffi::c_int,
    pub nkey: ::core::ffi::c_int,
    pub njmax: ::core::ffi::c_int,
    pub nconmax: ::core::ffi::c_int,
    pub nstack: mjtSize,
    pub(crate) comment: *mut mjString,
    pub(crate) modelfiledir: *mut mjString,
    pub hasImplicitPluginElem: mjtBool,
    pub authored: mjsAuthored,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsOrientation {
    pub type_: mjtOrientation,
    pub axisangle: [f64; 4usize],
    pub xyaxes: [f64; 6usize],
    pub zaxis: [f64; 3usize],
    pub euler: [f64; 3usize],
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsPlugin {
    pub(crate) element: *mut mjsElement,
    pub(crate) name: *mut mjString,
    pub(crate) plugin_name: *mut mjString,
    pub active: mjtBool,
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsBody {
    pub(crate) element: *mut mjsElement,
    pub(crate) childclass: *mut mjString,
    pub pos: [f64; 3usize],
    pub quat: [f64; 4usize],
    pub alt: mjsOrientation,
    pub mass: f64,
    pub ipos: [f64; 3usize],
    pub iquat: [f64; 4usize],
    pub inertia: [f64; 3usize],
    pub ialt: mjsOrientation,
    pub fullinertia: [f64; 6usize],
    pub mocap: mjtBool,
    pub gravcomp: f64,
    pub sleep: mjtSleepPolicy,
    pub simple: mjtByte,
    pub(crate) userdata: *mut mjDoubleVec,
    pub explicitinertial: mjtBool,
    pub plugin: mjsPlugin,
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsFrame {
    pub(crate) element: *mut mjsElement,
    pub(crate) childclass: *mut mjString,
    pub pos: [f64; 3usize],
    pub quat: [f64; 4usize],
    pub alt: mjsOrientation,
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsJoint {
    pub(crate) element: *mut mjsElement,
    pub type_: mjtJoint,
    pub pos: [f64; 3usize],
    pub axis: [f64; 3usize],
    pub ref_: f64,
    pub align: mjtAlignFree,
    pub stiffness: [f64; 3usize],
    pub springref: f64,
    pub springdamper: [f64; 2usize],
    pub limited: mjtLimited,
    pub range: [f64; 2usize],
    pub margin: f64,
    pub solref_limit: [mjtNum; 2usize],
    pub solimp_limit: [mjtNum; 5usize],
    pub actfrclimited: mjtLimited,
    pub actfrcrange: [f64; 2usize],
    pub armature: f64,
    pub damping: [f64; 3usize],
    pub frictionloss: f64,
    pub solref_friction: [mjtNum; 2usize],
    pub solimp_friction: [mjtNum; 5usize],
    pub group: ::core::ffi::c_int,
    pub actgravcomp: mjtBool,
    pub(crate) userdata: *mut mjDoubleVec,
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsGeom {
    pub(crate) element: *mut mjsElement,
    pub type_: mjtGeom,
    pub pos: [f64; 3usize],
    pub quat: [f64; 4usize],
    pub alt: mjsOrientation,
    pub fromto: [f64; 6usize],
    pub size: [f64; 3usize],
    pub contype: ::core::ffi::c_int,
    pub conaffinity: ::core::ffi::c_int,
    pub condim: ::core::ffi::c_int,
    pub priority: ::core::ffi::c_int,
    pub friction: [f64; 3usize],
    pub solmix: f64,
    pub solref: [mjtNum; 2usize],
    pub solimp: [mjtNum; 5usize],
    pub margin: f64,
    pub gap: f64,
    pub surfacevel: [f64; 6usize],
    pub adhesion: f64,
    pub mass: f64,
    pub density: f64,
    pub typeinertia: mjtGeomInertia,
    pub(crate) fluid_ellipsoid: mjtNum,
    pub fluid_coefs: [mjtNum; 5usize],
    pub(crate) material: *mut mjString,
    pub rgba: [f32; 4usize],
    pub group: ::core::ffi::c_int,
    pub(crate) hfieldname: *mut mjString,
    pub(crate) meshname: *mut mjString,
    pub fitscale: f64,
    pub(crate) userdata: *mut mjDoubleVec,
    pub plugin: mjsPlugin,
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsSite {
    pub(crate) element: *mut mjsElement,
    pub pos: [f64; 3usize],
    pub quat: [f64; 4usize],
    pub alt: mjsOrientation,
    pub fromto: [f64; 6usize],
    pub size: [f64; 3usize],
    pub type_: mjtGeom,
    pub(crate) material: *mut mjString,
    pub group: ::core::ffi::c_int,
    pub rgba: [f32; 4usize],
    pub(crate) userdata: *mut mjDoubleVec,
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsCamera {
    pub(crate) element: *mut mjsElement,
    pub pos: [f64; 3usize],
    pub quat: [f64; 4usize],
    pub alt: mjsOrientation,
    pub mode: mjtCamLight,
    pub(crate) targetbody: *mut mjString,
    pub proj: mjtProjection,
    pub resolution: [::core::ffi::c_int; 2usize],
    pub output: ::core::ffi::c_int,
    pub fovy: f64,
    pub ipd: f64,
    pub intrinsic: [f32; 4usize],
    pub sensor_size: [f32; 2usize],
    pub focal_length: [f32; 2usize],
    pub focal_pixel: [f32; 2usize],
    pub principal_length: [f32; 2usize],
    pub principal_pixel: [f32; 2usize],
    pub(crate) userdata: *mut mjDoubleVec,
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsLight {
    pub(crate) element: *mut mjsElement,
    pub pos: [f64; 3usize],
    pub dir: [f64; 3usize],
    pub mode: mjtCamLight,
    pub(crate) targetbody: *mut mjString,
    pub active: mjtBool,
    pub type_: mjtLightType,
    pub(crate) texture: *mut mjString,
    pub castshadow: mjtBool,
    pub bulbradius: f32,
    pub intensity: f32,
    pub range: f32,
    pub attenuation: [f32; 3usize],
    pub cutoff: f32,
    pub exponent: f32,
    pub ambient: [f32; 3usize],
    pub diffuse: [f32; 3usize],
    pub specular: [f32; 3usize],
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsFlex {
    pub(crate) element: *mut mjsElement,
    pub contype: ::core::ffi::c_int,
    pub conaffinity: ::core::ffi::c_int,
    pub condim: ::core::ffi::c_int,
    pub priority: ::core::ffi::c_int,
    pub friction: [f64; 3usize],
    pub solmix: f64,
    pub solref: [mjtNum; 2usize],
    pub solimp: [mjtNum; 5usize],
    pub margin: f64,
    pub gap: f64,
    pub dim: ::core::ffi::c_int,
    pub radius: f64,
    pub size: [f64; 3usize],
    pub internal: mjtBool,
    pub flatskin: mjtBool,
    pub selfcollide: mjtFlexSelf,
    pub passive: ::core::ffi::c_int,
    pub activelayers: ::core::ffi::c_int,
    pub group: ::core::ffi::c_int,
    pub edgestiffness: f64,
    pub edgedamping: f64,
    pub rgba: [f32; 4usize],
    pub(crate) material: *mut mjString,
    pub young: f64,
    pub poisson: f64,
    pub damping: f64,
    pub thickness: f64,
    pub elastic2d: ::core::ffi::c_int,
    pub cellcount: [::core::ffi::c_int; 3usize],
    pub order: ::core::ffi::c_int,
    pub(crate) nodebody: *mut mjStringVec,
    pub(crate) vertbody: *mut mjStringVec,
    pub(crate) node: *mut mjDoubleVec,
    pub(crate) vert: *mut mjDoubleVec,
    pub(crate) elem: *mut mjIntVec,
    pub(crate) texcoord: *mut mjFloatVec,
    pub(crate) elemtexcoord: *mut mjIntVec,
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsMesh {
    pub(crate) element: *mut mjsElement,
    pub(crate) content_type: *mut mjString,
    pub(crate) file: *mut mjString,
    pub refpos: [f64; 3usize],
    pub refquat: [f64; 4usize],
    pub scale: [f64; 3usize],
    pub inertia: mjtMeshInertia,
    pub smoothnormal: mjtBool,
    pub needsdf: mjtBool,
    pub maxhullvert: ::core::ffi::c_int,
    pub(crate) uservert: *mut mjFloatVec,
    pub(crate) usernormal: *mut mjFloatVec,
    pub(crate) usertexcoord: *mut mjFloatVec,
    pub(crate) userface: *mut mjIntVec,
    pub(crate) userfacenormal: *mut mjIntVec,
    pub(crate) userfacetexcoord: *mut mjIntVec,
    pub plugin: mjsPlugin,
    pub(crate) material: *mut mjString,
    pub octree_maxdepth: ::core::ffi::c_int,
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsHField {
    pub(crate) element: *mut mjsElement,
    pub(crate) content_type: *mut mjString,
    pub(crate) file: *mut mjString,
    pub size: [f64; 4usize],
    pub nrow: ::core::ffi::c_int,
    pub ncol: ::core::ffi::c_int,
    pub(crate) userdata: *mut mjFloatVec,
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsSkin {
    pub(crate) element: *mut mjsElement,
    pub(crate) file: *mut mjString,
    pub(crate) material: *mut mjString,
    pub rgba: [f32; 4usize],
    pub inflate: f32,
    pub group: ::core::ffi::c_int,
    pub(crate) vert: *mut mjFloatVec,
    pub(crate) texcoord: *mut mjFloatVec,
    pub(crate) face: *mut mjIntVec,
    pub(crate) bodyname: *mut mjStringVec,
    pub(crate) bindpos: *mut mjFloatVec,
    pub(crate) bindquat: *mut mjFloatVec,
    pub(crate) vertid: *mut mjIntVecVec,
    pub(crate) vertweight: *mut mjFloatVecVec,
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsTexture {
    pub(crate) element: *mut mjsElement,
    pub type_: mjtTexture,
    pub colorspace: mjtColorSpace,
    pub builtin: mjtBuiltin,
    pub mark: mjtMark,
    pub rgb1: [f64; 3usize],
    pub rgb2: [f64; 3usize],
    pub markrgb: [f64; 3usize],
    pub random: f64,
    pub height: ::core::ffi::c_int,
    pub width: ::core::ffi::c_int,
    pub nchannel: ::core::ffi::c_int,
    pub(crate) content_type: *mut mjString,
    pub(crate) file: *mut mjString,
    pub gridsize: [::core::ffi::c_int; 2usize],
    pub(crate) gridlayout: [::core::ffi::c_char; 12usize],
    pub(crate) cubefiles: *mut mjStringVec,
    pub(crate) data: *mut mjByteVec,
    pub hflip: mjtBool,
    pub vflip: mjtBool,
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsMaterial {
    pub(crate) element: *mut mjsElement,
    pub(crate) textures: *mut mjStringVec,
    pub texuniform: mjtBool,
    pub texrepeat: [f32; 2usize],
    pub emission: f32,
    pub specular: f32,
    pub shininess: f32,
    pub reflectance: f32,
    pub metallic: f32,
    pub roughness: f32,
    pub rgba: [f32; 4usize],
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsPair {
    pub(crate) element: *mut mjsElement,
    pub(crate) geomname1: *mut mjString,
    pub(crate) geomname2: *mut mjString,
    pub condim: ::core::ffi::c_int,
    pub solref: [mjtNum; 2usize],
    pub solreffriction: [mjtNum; 2usize],
    pub solimp: [mjtNum; 5usize],
    pub margin: f64,
    pub gap: f64,
    pub adhesion: f64,
    pub friction: [f64; 5usize],
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsExclude {
    pub(crate) element: *mut mjsElement,
    pub(crate) bodyname1: *mut mjString,
    pub(crate) bodyname2: *mut mjString,
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsEquality {
    pub(crate) element: *mut mjsElement,
    pub type_: mjtEq,
    pub data: [f64; 11usize],
    pub active: mjtBool,
    pub(crate) name1: *mut mjString,
    pub(crate) name2: *mut mjString,
    pub objtype: mjtObj,
    pub solref: [mjtNum; 2usize],
    pub solimp: [mjtNum; 5usize],
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsTendon {
    pub(crate) element: *mut mjsElement,
    pub stiffness: [f64; 3usize],
    pub springlength: [f64; 2usize],
    pub damping: [f64; 3usize],
    pub frictionloss: f64,
    pub solref_friction: [mjtNum; 2usize],
    pub solimp_friction: [mjtNum; 5usize],
    pub armature: f64,
    pub limited: mjtLimited,
    pub actfrclimited: mjtLimited,
    pub range: [f64; 2usize],
    pub actfrcrange: [f64; 2usize],
    pub margin: f64,
    pub solref_limit: [mjtNum; 2usize],
    pub solimp_limit: [mjtNum; 5usize],
    pub(crate) material: *mut mjString,
    pub width: f64,
    pub rgba: [f32; 4usize],
    pub group: ::core::ffi::c_int,
    pub(crate) userdata: *mut mjDoubleVec,
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsWrap {
    pub(crate) element: *mut mjsElement,
    pub type_: mjtWrap,
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsActuator {
    pub(crate) element: *mut mjsElement,
    pub gaintype: mjtGain,
    pub gainprm: [f64; 10usize],
    pub biastype: mjtBias,
    pub biasprm: [f64; 10usize],
    pub dyntype: mjtDyn,
    pub dynprm: [f64; 10usize],
    pub actdim: ::core::ffi::c_int,
    pub ctrlspec: ::core::ffi::c_int,
    pub actearly: mjtBool,
    pub trntype: mjtTrn,
    pub gear: [f64; 6usize],
    pub(crate) target: *mut mjString,
    pub(crate) refsite: *mut mjString,
    pub(crate) slidersite: *mut mjString,
    pub cranklength: f64,
    pub lengthrange: [f64; 2usize],
    pub inheritrange: f64,
    pub damping: [f64; 3usize],
    pub armature: f64,
    pub ctrllimited: mjtLimited,
    pub ctrlrange: [f64; 2usize],
    pub forcelimited: mjtLimited,
    pub forcerange: [f64; 2usize],
    pub actlimited: mjtLimited,
    pub actrange: [f64; 2usize],
    pub group: ::core::ffi::c_int,
    pub nsample: ::core::ffi::c_int,
    pub interp: ::core::ffi::c_int,
    pub delay: f64,
    pub(crate) userdata: *mut mjDoubleVec,
    pub plugin: mjsPlugin,
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsSensor {
    pub(crate) element: *mut mjsElement,
    pub type_: mjtSensor,
    pub objtype: mjtObj,
    pub(crate) objname: *mut mjString,
    pub reftype: mjtObj,
    pub(crate) refname: *mut mjString,
    pub intprm: [::core::ffi::c_int; 3usize],
    pub datatype: mjtDataType,
    pub needstage: mjtStage,
    pub dim: ::core::ffi::c_int,
    pub cutoff: f64,
    pub noise: f64,
    pub nsample: ::core::ffi::c_int,
    pub interp: ::core::ffi::c_int,
    pub delay: f64,
    pub interval: [f64; 2usize],
    pub(crate) userdata: *mut mjDoubleVec,
    pub plugin: mjsPlugin,
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsNumeric {
    pub(crate) element: *mut mjsElement,
    pub(crate) data: *mut mjDoubleVec,
    pub size: ::core::ffi::c_int,
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsText {
    pub(crate) element: *mut mjsElement,
    pub(crate) data: *mut mjString,
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsTuple {
    pub(crate) element: *mut mjsElement,
    pub(crate) objtype: *mut mjIntVec,
    pub(crate) objname: *mut mjStringVec,
    pub(crate) objprm: *mut mjDoubleVec,
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsKey {
    pub(crate) element: *mut mjsElement,
    pub time: f64,
    pub(crate) qpos: *mut mjDoubleVec,
    pub(crate) qvel: *mut mjDoubleVec,
    pub(crate) act: *mut mjDoubleVec,
    pub(crate) mpos: *mut mjDoubleVec,
    pub(crate) mquat: *mut mjDoubleVec,
    pub(crate) ctrl: *mut mjDoubleVec,
    pub(crate) info: *mut mjString,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjsDefault {
    pub(crate) element: *mut mjsElement,
    pub(crate) joint: *mut mjsJoint,
    pub(crate) geom: *mut mjsGeom,
    pub(crate) site: *mut mjsSite,
    pub(crate) camera: *mut mjsCamera,
    pub(crate) light: *mut mjsLight,
    pub(crate) flex: *mut mjsFlex,
    pub(crate) mesh: *mut mjsMesh,
    pub(crate) material: *mut mjsMaterial,
    pub(crate) pair: *mut mjsPair,
    pub(crate) equality: *mut mjsEquality,
    pub(crate) tendon: *mut mjsTendon,
    pub(crate) actuator: *mut mjsActuator,
}
impl mjtCatBit {
    pub const STATIC: mjtCatBit = mjtCatBit(1);
    pub const DYNAMIC: mjtCatBit = mjtCatBit(2);
    pub const DECOR: mjtCatBit = mjtCatBit(4);
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
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtCatBit(::core::ffi::c_uint);
impl mjtMouse {
    pub const NONE: mjtMouse = mjtMouse(0);
    pub const ROTATE_V: mjtMouse = mjtMouse(1);
    pub const ROTATE_H: mjtMouse = mjtMouse(2);
    pub const MOVE_V: mjtMouse = mjtMouse(3);
    pub const MOVE_H: mjtMouse = mjtMouse(4);
    pub const ZOOM: mjtMouse = mjtMouse(5);
    pub const MOVE_V_REL: mjtMouse = mjtMouse(6);
    pub const MOVE_H_REL: mjtMouse = mjtMouse(7);
    pub const TURN_V: mjtMouse = mjtMouse(8);
    pub const TURN_H: mjtMouse = mjtMouse(9);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtMouse(::core::ffi::c_uint);
impl mjtPertBit {
    pub const TRANSLATE: mjtPertBit = mjtPertBit(1);
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
pub struct mjtPertBit(::core::ffi::c_uint);
impl mjtCamera {
    pub const FREE: mjtCamera = mjtCamera(0);
    pub const TRACKING: mjtCamera = mjtCamera(1);
    pub const FIXED: mjtCamera = mjtCamera(2);
    pub const USER: mjtCamera = mjtCamera(3);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtCamera(::core::ffi::c_uint);
pub const mjNLABEL: usize = mjtLabel::mjNLABEL.0 as usize;
impl mjtLabel {
    pub const NONE: mjtLabel = mjtLabel(0);
    pub const BODY: mjtLabel = mjtLabel(1);
    pub const JOINT: mjtLabel = mjtLabel(2);
    pub const GEOM: mjtLabel = mjtLabel(3);
    pub const SITE: mjtLabel = mjtLabel(4);
    pub const CAMERA: mjtLabel = mjtLabel(5);
    pub const LIGHT: mjtLabel = mjtLabel(6);
    pub const TENDON: mjtLabel = mjtLabel(7);
    pub const ACTUATOR: mjtLabel = mjtLabel(8);
    pub const CONSTRAINT: mjtLabel = mjtLabel(9);
    pub const FLEX: mjtLabel = mjtLabel(10);
    pub const SKIN: mjtLabel = mjtLabel(11);
    pub const SELECTION: mjtLabel = mjtLabel(12);
    pub const SELPNT: mjtLabel = mjtLabel(13);
    pub const CONTACTPOINT: mjtLabel = mjtLabel(14);
    pub const CONTACTFORCE: mjtLabel = mjtLabel(15);
    pub const ISLAND: mjtLabel = mjtLabel(16);
    const mjNLABEL: mjtLabel = mjtLabel(17);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtLabel(::core::ffi::c_uint);
pub const mjNFRAME: usize = mjtFrame::mjNFRAME.0 as usize;
impl mjtFrame {
    pub const NONE: mjtFrame = mjtFrame(0);
    pub const BODY: mjtFrame = mjtFrame(1);
    pub const GEOM: mjtFrame = mjtFrame(2);
    pub const SITE: mjtFrame = mjtFrame(3);
    pub const CAMERA: mjtFrame = mjtFrame(4);
    pub const LIGHT: mjtFrame = mjtFrame(5);
    pub const CONTACT: mjtFrame = mjtFrame(6);
    pub const WORLD: mjtFrame = mjtFrame(7);
    const mjNFRAME: mjtFrame = mjtFrame(8);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtFrame(::core::ffi::c_uint);
pub const mjNVISFLAG: usize = mjtVisFlag::mjNVISFLAG.0 as usize;
impl mjtVisFlag {
    pub const CONVEXHULL: mjtVisFlag = mjtVisFlag(0);
    pub const TEXTURE: mjtVisFlag = mjtVisFlag(1);
    pub const JOINT: mjtVisFlag = mjtVisFlag(2);
    pub const CAMERA: mjtVisFlag = mjtVisFlag(3);
    pub const ACTUATOR: mjtVisFlag = mjtVisFlag(4);
    pub const ACTIVATION: mjtVisFlag = mjtVisFlag(5);
    pub const LIGHT: mjtVisFlag = mjtVisFlag(6);
    pub const TENDON: mjtVisFlag = mjtVisFlag(7);
    pub const RANGEFINDER: mjtVisFlag = mjtVisFlag(8);
    pub const CONSTRAINT: mjtVisFlag = mjtVisFlag(9);
    pub const INERTIA: mjtVisFlag = mjtVisFlag(10);
    pub const SCLINERTIA: mjtVisFlag = mjtVisFlag(11);
    pub const PERTFORCE: mjtVisFlag = mjtVisFlag(12);
    pub const PERTOBJ: mjtVisFlag = mjtVisFlag(13);
    pub const CONTACTPOINT: mjtVisFlag = mjtVisFlag(14);
    pub const ISLAND: mjtVisFlag = mjtVisFlag(15);
    pub const CONTACTFORCE: mjtVisFlag = mjtVisFlag(16);
    pub const CONTACTSPLIT: mjtVisFlag = mjtVisFlag(17);
    pub const TRANSPARENT: mjtVisFlag = mjtVisFlag(18);
    pub const AUTOCONNECT: mjtVisFlag = mjtVisFlag(19);
    pub const COM: mjtVisFlag = mjtVisFlag(20);
    pub const SELECT: mjtVisFlag = mjtVisFlag(21);
    pub const STATIC: mjtVisFlag = mjtVisFlag(22);
    pub const SKIN: mjtVisFlag = mjtVisFlag(23);
    pub const FLEXVERT: mjtVisFlag = mjtVisFlag(24);
    pub const FLEXEDGE: mjtVisFlag = mjtVisFlag(25);
    pub const FLEXFACE: mjtVisFlag = mjtVisFlag(26);
    pub const FLEXSKIN: mjtVisFlag = mjtVisFlag(27);
    pub const BODYBVH: mjtVisFlag = mjtVisFlag(28);
    pub const MESHBVH: mjtVisFlag = mjtVisFlag(29);
    pub const SDFITER: mjtVisFlag = mjtVisFlag(30);
    const mjNVISFLAG: mjtVisFlag = mjtVisFlag(31);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtVisFlag(::core::ffi::c_uint);
pub const mjNRNDFLAG: usize = mjtRndFlag::mjNRNDFLAG.0 as usize;
impl mjtRndFlag {
    pub const SHADOW: mjtRndFlag = mjtRndFlag(0);
    pub const WIREFRAME: mjtRndFlag = mjtRndFlag(1);
    pub const REFLECTION: mjtRndFlag = mjtRndFlag(2);
    pub const ADDITIVE: mjtRndFlag = mjtRndFlag(3);
    pub const SKYBOX: mjtRndFlag = mjtRndFlag(4);
    pub const FOG: mjtRndFlag = mjtRndFlag(5);
    pub const HAZE: mjtRndFlag = mjtRndFlag(6);
    pub const DEPTH: mjtRndFlag = mjtRndFlag(7);
    pub const SEGMENT: mjtRndFlag = mjtRndFlag(8);
    pub const IDCOLOR: mjtRndFlag = mjtRndFlag(9);
    pub const CULL_FACE: mjtRndFlag = mjtRndFlag(10);
    const mjNRNDFLAG: mjtRndFlag = mjtRndFlag(11);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtRndFlag(::core::ffi::c_uint);
impl mjtStereo {
    pub const NONE: mjtStereo = mjtStereo(0);
    pub const QUADBUFFERED: mjtStereo = mjtStereo(1);
    pub const SIDEBYSIDE: mjtStereo = mjtStereo(2);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtStereo(::core::ffi::c_uint);
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjvPerturb {
    pub select: ::core::ffi::c_int,
    pub flexselect: ::core::ffi::c_int,
    pub skinselect: ::core::ffi::c_int,
    pub active: ::core::ffi::c_int,
    pub active2: ::core::ffi::c_int,
    pub refpos: [mjtNum; 3usize],
    pub refquat: [mjtNum; 4usize],
    pub refselpos: [mjtNum; 3usize],
    pub localpos: [mjtNum; 3usize],
    pub localmass: mjtNum,
    pub scale: mjtNum,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjvCamera {
    pub type_: ::core::ffi::c_int,
    pub(crate) fixedcamid: ::core::ffi::c_int,
    pub(crate) trackbodyid: ::core::ffi::c_int,
    pub lookat: [mjtNum; 3usize],
    pub distance: mjtNum,
    pub azimuth: mjtNum,
    pub elevation: mjtNum,
    pub orthographic: ::core::ffi::c_int,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjvGLCamera {
    pub pos: [f32; 3usize],
    pub forward: [f32; 3usize],
    pub up: [f32; 3usize],
    pub frustum_center: f32,
    pub frustum_width: f32,
    pub frustum_bottom: f32,
    pub frustum_top: f32,
    pub frustum_near: f32,
    pub frustum_far: f32,
    pub orthographic: ::core::ffi::c_int,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjvGeom {
    pub type_: ::core::ffi::c_int,
    pub(crate) dataid: ::core::ffi::c_int,
    pub objtype: ::core::ffi::c_int,
    pub(crate) objid: ::core::ffi::c_int,
    pub category: ::core::ffi::c_int,
    pub(crate) matid: ::core::ffi::c_int,
    pub(crate) texid: ::core::ffi::c_int,
    pub texuniform: ::core::ffi::c_int,
    pub texcoord: ::core::ffi::c_int,
    pub(crate) segid: ::core::ffi::c_int,
    pub size: [f32; 3usize],
    pub pos: [f32; 3usize],
    pub mat: [f32; 9usize],
    pub rgba: [f32; 4usize],
    pub emission: f32,
    pub specular: f32,
    pub shininess: f32,
    pub reflectance: f32,
    pub texrepeat: [f32; 2usize],
    pub(crate) label: [::core::ffi::c_char; 100usize],
    pub camdist: f32,
    pub modelrbound: f32,
    pub transparent: mjtByte,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjvLight {
    pub(crate) id: ::core::ffi::c_int,
    pub pos: [f32; 3usize],
    pub dir: [f32; 3usize],
    pub type_: ::core::ffi::c_int,
    pub(crate) texid: ::core::ffi::c_int,
    pub attenuation: [f32; 3usize],
    pub cutoff: f32,
    pub exponent: f32,
    pub ambient: [f32; 3usize],
    pub diffuse: [f32; 3usize],
    pub specular: [f32; 3usize],
    pub headlight: mjtByte,
    pub castshadow: mjtByte,
    pub bulbradius: f32,
    pub intensity: f32,
    pub range: f32,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjvOption {
    pub label: ::core::ffi::c_int,
    pub frame: ::core::ffi::c_int,
    pub geomgroup: [mjtByte; 6usize],
    pub sitegroup: [mjtByte; 6usize],
    pub jointgroup: [mjtByte; 6usize],
    pub tendongroup: [mjtByte; 6usize],
    pub actuatorgroup: [mjtByte; 6usize],
    pub flexgroup: [mjtByte; 6usize],
    pub skingroup: [mjtByte; 6usize],
    pub flags: [mjtByte; 31usize],
    pub bvh_depth: ::core::ffi::c_int,
    pub flex_layer: ::core::ffi::c_int,
}
#[repr(C)]
#[derive(Debug)]
pub struct mjvScene {
    pub maxgeom: ::core::ffi::c_int,
    pub ngeom: ::core::ffi::c_int,
    pub(crate) geoms: *mut mjvGeom,
    pub(crate) geomorder: *mut ::core::ffi::c_int,
    pub nflex: ::core::ffi::c_int,
    pub(crate) flexedgeadr: *mut ::core::ffi::c_int,
    pub(crate) flexedgenum: *mut ::core::ffi::c_int,
    pub(crate) flexvertadr: *mut ::core::ffi::c_int,
    pub(crate) flexvertnum: *mut ::core::ffi::c_int,
    pub(crate) flexfaceadr: *mut ::core::ffi::c_int,
    pub(crate) flexfacenum: *mut ::core::ffi::c_int,
    pub(crate) flexfaceused: *mut ::core::ffi::c_int,
    pub(crate) flexedge: *mut ::core::ffi::c_int,
    pub(crate) flexvert: *mut f32,
    pub(crate) flexface: *mut f32,
    pub(crate) flexnormal: *mut f32,
    pub(crate) flextexcoord: *mut f32,
    pub flexvertopt: mjtByte,
    pub flexedgeopt: mjtByte,
    pub flexfaceopt: mjtByte,
    pub flexskinopt: mjtByte,
    pub nskin: ::core::ffi::c_int,
    pub(crate) skinfacenum: *mut ::core::ffi::c_int,
    pub(crate) skinvertadr: *mut ::core::ffi::c_int,
    pub(crate) skinvertnum: *mut ::core::ffi::c_int,
    pub(crate) skinvert: *mut f32,
    pub(crate) skinnormal: *mut f32,
    pub nlight: ::core::ffi::c_int,
    pub lights: [mjvLight; 100usize],
    pub camera: [mjvGLCamera; 2usize],
    pub enabletransform: mjtByte,
    pub translate: [f32; 3usize],
    pub rotate: [f32; 4usize],
    pub scale: f32,
    pub stereo: ::core::ffi::c_int,
    pub flags: [mjtByte; 11usize],
    pub framewidth: ::core::ffi::c_int,
    pub framergb: [f32; 3usize],
    pub status: ::core::ffi::c_int,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjvFigure {
    pub flg_legend: ::core::ffi::c_int,
    pub flg_ticklabel: [::core::ffi::c_int; 2usize],
    pub flg_extend: ::core::ffi::c_int,
    pub flg_barplot: ::core::ffi::c_int,
    pub flg_selection: ::core::ffi::c_int,
    pub flg_symmetric: ::core::ffi::c_int,
    pub linewidth: f32,
    pub gridwidth: f32,
    pub gridsize: [::core::ffi::c_int; 2usize],
    pub gridrgb: [f32; 3usize],
    pub figurergba: [f32; 4usize],
    pub panergba: [f32; 4usize],
    pub legendrgba: [f32; 4usize],
    pub textrgb: [f32; 3usize],
    pub linergb: [[f32; 3usize]; 100usize],
    pub range: [[f32; 2usize]; 2usize],
    pub(crate) xformat: [::core::ffi::c_char; 20usize],
    pub(crate) yformat: [::core::ffi::c_char; 20usize],
    pub(crate) minwidth: [::core::ffi::c_char; 20usize],
    pub(crate) title: [::core::ffi::c_char; 1000usize],
    pub(crate) xlabel: [::core::ffi::c_char; 100usize],
    pub(crate) linename: [[::core::ffi::c_char; 100usize]; 100usize],
    pub legendoffset: ::core::ffi::c_int,
    pub subplot: ::core::ffi::c_int,
    pub highlight: [::core::ffi::c_int; 2usize],
    pub(crate) highlightid: ::core::ffi::c_int,
    pub selection: f32,
    pub linepnt: [::core::ffi::c_int; 100usize],
    pub linedata: [[f32; 2002usize]; 100usize],
    pub xaxispixel: [::core::ffi::c_int; 2usize],
    pub yaxispixel: [::core::ffi::c_int; 2usize],
    pub xaxisdata: [f32; 2usize],
    pub yaxisdata: [f32; 2usize],
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjResource {
    pub(crate) name: *mut ::core::ffi::c_char,
    pub data: *mut ::core::ffi::c_void,
    pub(crate) vfs: *mut mjVFS,
    pub(crate) timestamp: [::core::ffi::c_char; 512usize],
    pub(crate) provider: *const mjpResourceProvider,
}
pub(crate) type mjfOpenResource =
    ::core::option::Option<unsafe extern "C" fn(resource: *mut mjResource) -> ::core::ffi::c_int>;
pub(crate) type mjfReadResource = ::core::option::Option<
    unsafe extern "C" fn(
        resource: *mut mjResource,
        buffer: *mut *const ::core::ffi::c_void,
    ) -> ::core::ffi::c_int,
>;
pub(crate) type mjfCloseResource =
    ::core::option::Option<unsafe extern "C" fn(resource: *mut mjResource)>;
pub(crate) type mjfMountResource =
    ::core::option::Option<unsafe extern "C" fn(resource: *mut mjResource) -> ::core::ffi::c_int>;
pub(crate) type mjfUnmountResource =
    ::core::option::Option<unsafe extern "C" fn(resource: *mut mjResource) -> ::core::ffi::c_int>;
pub(crate) type mjfResourceModified = ::core::option::Option<
    unsafe extern "C" fn(
        resource: *const mjResource,
        timestamp: *const ::core::ffi::c_char,
    ) -> ::core::ffi::c_int,
>;
pub(crate) type mjfWriteResource = ::core::option::Option<
    unsafe extern "C" fn(
        resource: *mut mjResource,
        buffer: *const ::core::ffi::c_void,
        nbytes: mjtSize,
    ) -> mjtSize,
>;
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjpResourceProvider {
    pub(crate) prefix: *const ::core::ffi::c_char,
    pub open: mjfOpenResource,
    pub read: mjfReadResource,
    pub close: mjfCloseResource,
    pub mount: mjfMountResource,
    pub unmount: mjfUnmountResource,
    pub modified: mjfResourceModified,
    pub write: mjfWriteResource,
    pub data: *mut ::core::ffi::c_void,
}
pub(crate) type mjfDecode = ::core::option::Option<
    unsafe extern "C" fn(resource: *mut mjResource, vfs: *const mjVFS) -> *mut mjSpec,
>;
pub(crate) type mjfCanDecode =
    ::core::option::Option<unsafe extern "C" fn(resource: *const mjResource) -> ::core::ffi::c_int>;
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjpDecoder {
    pub(crate) content_type: *const ::core::ffi::c_char,
    pub(crate) extension: *const ::core::ffi::c_char,
    pub can_decode: mjfCanDecode,
    pub decode: mjfDecode,
}
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
    pub encode: mjfEncode,
    pub close_resource: mjfCloseResource,
}
impl mjtPluginCapabilityBit {
    pub const ACTUATOR: mjtPluginCapabilityBit = mjtPluginCapabilityBit(1);
    pub const SENSOR: mjtPluginCapabilityBit = mjtPluginCapabilityBit(2);
    pub const PASSIVE: mjtPluginCapabilityBit = mjtPluginCapabilityBit(4);
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
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtPluginCapabilityBit(::core::ffi::c_uint);
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjpPlugin {
    pub(crate) name: *const ::core::ffi::c_char,
    pub nattribute: ::core::ffi::c_int,
    pub(crate) attributes: *const *const ::core::ffi::c_char,
    pub capabilityflags: ::core::ffi::c_int,
    pub needstage: ::core::ffi::c_int,
    pub(crate) nstate: ::core::option::Option<
        unsafe extern "C" fn(m: *const mjModel, instance: ::core::ffi::c_int) -> ::core::ffi::c_int,
    >,
    pub(crate) nsensordata: ::core::option::Option<
        unsafe extern "C" fn(
            m: *const mjModel,
            instance: ::core::ffi::c_int,
            sensor_id: ::core::ffi::c_int,
        ) -> ::core::ffi::c_int,
    >,
    pub(crate) init: ::core::option::Option<
        unsafe extern "C" fn(
            m: *const mjModel,
            d: *mut mjData,
            instance: ::core::ffi::c_int,
        ) -> ::core::ffi::c_int,
    >,
    pub(crate) destroy:
        ::core::option::Option<unsafe extern "C" fn(d: *mut mjData, instance: ::core::ffi::c_int)>,
    pub(crate) copy: ::core::option::Option<
        unsafe extern "C" fn(
            dest: *mut mjData,
            m: *const mjModel,
            src: *const mjData,
            instance: ::core::ffi::c_int,
        ),
    >,
    pub(crate) reset: ::core::option::Option<
        unsafe extern "C" fn(
            m: *const mjModel,
            plugin_state: *mut mjtNum,
            plugin_data: *mut ::core::ffi::c_void,
            instance: ::core::ffi::c_int,
        ),
    >,
    pub(crate) compute: ::core::option::Option<
        unsafe extern "C" fn(
            m: *const mjModel,
            d: *mut mjData,
            instance: ::core::ffi::c_int,
            capability_bit: ::core::ffi::c_int,
        ),
    >,
    pub(crate) advance: ::core::option::Option<
        unsafe extern "C" fn(m: *const mjModel, d: *mut mjData, instance: ::core::ffi::c_int),
    >,
    pub(crate) visualize: ::core::option::Option<
        unsafe extern "C" fn(
            m: *const mjModel,
            d: *mut mjData,
            opt: *const mjvOption,
            scn: *mut mjvScene,
            instance: ::core::ffi::c_int,
        ),
    >,
    pub(crate) actuator_act_dot: ::core::option::Option<
        unsafe extern "C" fn(m: *const mjModel, d: *mut mjData, instance: ::core::ffi::c_int),
    >,
    pub(crate) sdf_distance: ::core::option::Option<
        unsafe extern "C" fn(
            point: *const [mjtNum; 3usize],
            d: *const mjData,
            instance: ::core::ffi::c_int,
        ) -> mjtNum,
    >,
    pub(crate) sdf_gradient: ::core::option::Option<
        unsafe extern "C" fn(
            gradient: *mut [mjtNum; 3usize],
            point: *const [mjtNum; 3usize],
            d: *const mjData,
            instance: ::core::ffi::c_int,
        ),
    >,
    pub(crate) sdf_staticdistance: ::core::option::Option<
        unsafe extern "C" fn(point: *const [mjtNum; 3usize], attributes: *const mjtNum) -> mjtNum,
    >,
    pub(crate) sdf_attribute: ::core::option::Option<
        unsafe extern "C" fn(
            attribute: *mut [mjtNum; 0usize],
            name: *mut [*const ::core::ffi::c_char; 0usize],
            value: *mut [*const ::core::ffi::c_char; 0usize],
        ),
    >,
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
pub(crate) type mjfPluginLibraryLoadCallback = ::core::option::Option<
    unsafe extern "C" fn(
        filename: *const ::core::ffi::c_char,
        first: ::core::ffi::c_int,
        count: ::core::ffi::c_int,
    ),
>;
impl mjtGridPos {
    pub const TOPLEFT: mjtGridPos = mjtGridPos(0);
    pub const TOPRIGHT: mjtGridPos = mjtGridPos(1);
    pub const BOTTOMLEFT: mjtGridPos = mjtGridPos(2);
    pub const BOTTOMRIGHT: mjtGridPos = mjtGridPos(3);
    pub const TOP: mjtGridPos = mjtGridPos(4);
    pub const BOTTOM: mjtGridPos = mjtGridPos(5);
    pub const LEFT: mjtGridPos = mjtGridPos(6);
    pub const RIGHT: mjtGridPos = mjtGridPos(7);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtGridPos(::core::ffi::c_uint);
impl mjtFramebuffer {
    pub const WINDOW: mjtFramebuffer = mjtFramebuffer(0);
    pub const OFFSCREEN: mjtFramebuffer = mjtFramebuffer(1);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtFramebuffer(::core::ffi::c_uint);
impl mjtDepthMap {
    pub const ZERONEAR: mjtDepthMap = mjtDepthMap(0);
    pub const ZEROFAR: mjtDepthMap = mjtDepthMap(1);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtDepthMap(::core::ffi::c_uint);
impl mjtFontScale {
    pub const X50: mjtFontScale = mjtFontScale(50);
    pub const X100: mjtFontScale = mjtFontScale(100);
    pub const X150: mjtFontScale = mjtFontScale(150);
    pub const X200: mjtFontScale = mjtFontScale(200);
    pub const X250: mjtFontScale = mjtFontScale(250);
    pub const X300: mjtFontScale = mjtFontScale(300);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtFontScale(::core::ffi::c_uint);
impl mjtFont {
    pub const NORMAL: mjtFont = mjtFont(0);
    pub const SHADOW: mjtFont = mjtFont(1);
    pub const BIG: mjtFont = mjtFont(2);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtFont(::core::ffi::c_uint);
impl mjrPixelFormat {
    pub const UNKNOWN: mjrPixelFormat = mjrPixelFormat(0);
    pub const R8: mjrPixelFormat = mjrPixelFormat(1);
    pub const RGB8: mjrPixelFormat = mjrPixelFormat(2);
    pub const RGBA8: mjrPixelFormat = mjrPixelFormat(3);
    pub const R32F: mjrPixelFormat = mjrPixelFormat(4);
    pub const DEPTH32F: mjrPixelFormat = mjrPixelFormat(5);
    pub const KTX: mjrPixelFormat = mjrPixelFormat(6);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjrPixelFormat(::core::ffi::c_uint);
impl mjrVertexAttributeUsage {
    pub const POSITION: mjrVertexAttributeUsage = mjrVertexAttributeUsage(0);
    pub const NORMAL: mjrVertexAttributeUsage = mjrVertexAttributeUsage(1);
    pub const TANGENTS: mjrVertexAttributeUsage = mjrVertexAttributeUsage(2);
    pub const UV: mjrVertexAttributeUsage = mjrVertexAttributeUsage(3);
    pub const COLOR: mjrVertexAttributeUsage = mjrVertexAttributeUsage(4);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjrVertexAttributeUsage(::core::ffi::c_uint);
impl mjrVertexAttributeType {
    pub const FLOAT2: mjrVertexAttributeType = mjrVertexAttributeType(0);
    pub const FLOAT3: mjrVertexAttributeType = mjrVertexAttributeType(1);
    pub const FLOAT4: mjrVertexAttributeType = mjrVertexAttributeType(2);
    pub const UBYTE4: mjrVertexAttributeType = mjrVertexAttributeType(3);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjrVertexAttributeType(::core::ffi::c_uint);
impl mjrIndexType {
    pub const U16: mjrIndexType = mjrIndexType(0);
    pub const U32: mjrIndexType = mjrIndexType(1);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjrIndexType(::core::ffi::c_uint);
impl mjrMeshPrimitiveType {
    pub const TRIANGLES: mjrMeshPrimitiveType = mjrMeshPrimitiveType(0);
    pub const LINES: mjrMeshPrimitiveType = mjrMeshPrimitiveType(1);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjrMeshPrimitiveType(::core::ffi::c_uint);
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjrRect {
    pub left: ::core::ffi::c_int,
    pub bottom: ::core::ffi::c_int,
    pub width: ::core::ffi::c_int,
    pub height: ::core::ffi::c_int,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjrRendererInfo {
    pub(crate) renderer: *const ::core::ffi::c_char,
    pub(crate) backend: *const ::core::ffi::c_char,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjrVertexAttribute {
    pub usage: ::core::ffi::c_int,
    pub type_: ::core::ffi::c_int,
}
pub use self::mjtColorSpace as mjrColorSpace;
pub use self::mjtLightType as mjrLightType;
pub use self::mjtTexture as mjrSamplerType;
pub type mjrCamera = mjvGLCamera;
#[repr(C)]
#[derive(Debug)]
pub struct mjrContext {
    pub lineWidth: f32,
    pub shadowClip: f32,
    pub shadowScale: f32,
    pub fogStart: f32,
    pub fogEnd: f32,
    pub fogRGBA: [f32; 4usize],
    pub shadowSize: ::core::ffi::c_int,
    pub offWidth: ::core::ffi::c_int,
    pub offHeight: ::core::ffi::c_int,
    pub offSamples: ::core::ffi::c_int,
    pub fontScale: ::core::ffi::c_int,
    pub auxWidth: [::core::ffi::c_int; 10usize],
    pub auxHeight: [::core::ffi::c_int; 10usize],
    pub auxSamples: [::core::ffi::c_int; 10usize],
    pub offFBO: ::core::ffi::c_uint,
    pub offFBO_r: ::core::ffi::c_uint,
    pub offColor: ::core::ffi::c_uint,
    pub offColor_r: ::core::ffi::c_uint,
    pub offDepthStencil: ::core::ffi::c_uint,
    pub offDepthStencil_r: ::core::ffi::c_uint,
    pub shadowFBO: ::core::ffi::c_uint,
    pub shadowTex: ::core::ffi::c_uint,
    pub auxFBO: [::core::ffi::c_uint; 10usize],
    pub auxFBO_r: [::core::ffi::c_uint; 10usize],
    pub auxColor: [::core::ffi::c_uint; 10usize],
    pub auxColor_r: [::core::ffi::c_uint; 10usize],
    pub(crate) mat_texid: [::core::ffi::c_int; 10000usize],
    pub mat_texuniform: [::core::ffi::c_int; 1000usize],
    pub mat_texrepeat: [f32; 2000usize],
    pub ntexture: ::core::ffi::c_int,
    pub textureType: [::core::ffi::c_int; 1000usize],
    pub texture: [::core::ffi::c_uint; 1000usize],
    pub basePlane: ::core::ffi::c_uint,
    pub baseMesh: ::core::ffi::c_uint,
    pub baseHField: ::core::ffi::c_uint,
    pub baseBuiltin: ::core::ffi::c_uint,
    pub baseFontNormal: ::core::ffi::c_uint,
    pub baseFontShadow: ::core::ffi::c_uint,
    pub baseFontBig: ::core::ffi::c_uint,
    pub rangePlane: ::core::ffi::c_int,
    pub rangeMesh: ::core::ffi::c_int,
    pub rangeHField: ::core::ffi::c_int,
    pub rangeBuiltin: ::core::ffi::c_int,
    pub rangeFont: ::core::ffi::c_int,
    pub nskin: ::core::ffi::c_int,
    pub(crate) skinvertVBO: *mut ::core::ffi::c_uint,
    pub(crate) skinnormalVBO: *mut ::core::ffi::c_uint,
    pub(crate) skintexcoordVBO: *mut ::core::ffi::c_uint,
    pub(crate) skinfaceVBO: *mut ::core::ffi::c_uint,
    pub charWidth: [::core::ffi::c_int; 127usize],
    pub charWidthBig: [::core::ffi::c_int; 127usize],
    pub charHeight: ::core::ffi::c_int,
    pub charHeightBig: ::core::ffi::c_int,
    pub glInitialized: ::core::ffi::c_int,
    pub windowAvailable: ::core::ffi::c_int,
    pub windowSamples: ::core::ffi::c_int,
    pub windowStereo: ::core::ffi::c_int,
    pub windowDoublebuffer: ::core::ffi::c_int,
    pub currentBuffer: ::core::ffi::c_int,
    pub readPixelFormat: ::core::ffi::c_int,
    pub readDepthMap: ::core::ffi::c_int,
}
impl mjtButton {
    pub const NONE: mjtButton = mjtButton(0);
    pub const LEFT: mjtButton = mjtButton(1);
    pub const RIGHT: mjtButton = mjtButton(2);
    pub const MIDDLE: mjtButton = mjtButton(3);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtButton(::core::ffi::c_uint);
impl mjtEvent {
    pub const NONE: mjtEvent = mjtEvent(0);
    pub const MOVE: mjtEvent = mjtEvent(1);
    pub const PRESS: mjtEvent = mjtEvent(2);
    pub const RELEASE: mjtEvent = mjtEvent(3);
    pub const SCROLL: mjtEvent = mjtEvent(4);
    pub const KEY: mjtEvent = mjtEvent(5);
    pub const RESIZE: mjtEvent = mjtEvent(6);
    pub const REDRAW: mjtEvent = mjtEvent(7);
    pub const FILESDROP: mjtEvent = mjtEvent(8);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtEvent(::core::ffi::c_uint);
pub const mjNITEM: usize = mjtItem::mjNITEM.0 as usize;
impl mjtItem {
    pub const END: mjtItem = mjtItem(-2);
    pub const SECTION: mjtItem = mjtItem(-1);
    pub const SEPARATOR: mjtItem = mjtItem(0);
    pub const STATIC: mjtItem = mjtItem(1);
    pub const BUTTON: mjtItem = mjtItem(2);
    pub const CHECKINT: mjtItem = mjtItem(3);
    pub const CHECKBYTE: mjtItem = mjtItem(4);
    pub const RADIO: mjtItem = mjtItem(5);
    pub const RADIOLINE: mjtItem = mjtItem(6);
    pub const SELECT: mjtItem = mjtItem(7);
    pub const SLIDERINT: mjtItem = mjtItem(8);
    pub const SLIDERNUM: mjtItem = mjtItem(9);
    pub const EDITINT: mjtItem = mjtItem(10);
    pub const EDITNUM: mjtItem = mjtItem(11);
    pub const EDITFLOAT: mjtItem = mjtItem(12);
    pub const EDITTXT: mjtItem = mjtItem(13);
    const mjNITEM: mjtItem = mjtItem(14);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtItem(::core::ffi::c_int);
impl mjtSection {
    pub const CLOSED: mjtSection = mjtSection(0);
    pub const OPEN: mjtSection = mjtSection(1);
    pub const FIXED: mjtSection = mjtSection(2);
}
#[repr(transparent)]
#[derive(Debug, Copy, Clone, Hash, PartialEq, Eq)]
pub struct mjtSection(::core::ffi::c_uint);
pub(crate) type mjfItemEnable = ::core::option::Option<
    unsafe extern "C" fn(
        category: ::core::ffi::c_int,
        data: *mut ::core::ffi::c_void,
    ) -> ::core::ffi::c_int,
>;
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjuiState {
    pub nrect: ::core::ffi::c_int,
    pub rect: [mjrRect; 25usize],
    pub userdata: *mut ::core::ffi::c_void,
    pub type_: ::core::ffi::c_int,
    pub left: ::core::ffi::c_int,
    pub right: ::core::ffi::c_int,
    pub middle: ::core::ffi::c_int,
    pub doubleclick: ::core::ffi::c_int,
    pub button: ::core::ffi::c_int,
    pub buttontime: f64,
    pub x: f64,
    pub y: f64,
    pub dx: f64,
    pub dy: f64,
    pub sx: f64,
    pub sy: f64,
    pub control: ::core::ffi::c_int,
    pub shift: ::core::ffi::c_int,
    pub alt: ::core::ffi::c_int,
    pub key: ::core::ffi::c_int,
    pub keytime: f64,
    pub mouserect: ::core::ffi::c_int,
    pub dragrect: ::core::ffi::c_int,
    pub dragbutton: ::core::ffi::c_int,
    pub dropcount: ::core::ffi::c_int,
    pub(crate) droppaths: *mut *const ::core::ffi::c_char,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjuiThemeSpacing {
    pub total: ::core::ffi::c_int,
    pub scroll: ::core::ffi::c_int,
    pub label: ::core::ffi::c_int,
    pub section: ::core::ffi::c_int,
    pub cornersect: ::core::ffi::c_int,
    pub cornersep: ::core::ffi::c_int,
    pub itemside: ::core::ffi::c_int,
    pub(crate) itemmid: ::core::ffi::c_int,
    pub itemver: ::core::ffi::c_int,
    pub texthor: ::core::ffi::c_int,
    pub textver: ::core::ffi::c_int,
    pub linescroll: ::core::ffi::c_int,
    pub samples: ::core::ffi::c_int,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjuiThemeColor {
    pub master: [f32; 3usize],
    pub thumb: [f32; 3usize],
    pub secttitle: [f32; 3usize],
    pub secttitle2: [f32; 3usize],
    pub secttitleuncheck: [f32; 3usize],
    pub secttitleuncheck2: [f32; 3usize],
    pub secttitlecheck: [f32; 3usize],
    pub secttitlecheck2: [f32; 3usize],
    pub sectfont: [f32; 3usize],
    pub sectsymbol: [f32; 3usize],
    pub sectpane: [f32; 3usize],
    pub separator: [f32; 3usize],
    pub separator2: [f32; 3usize],
    pub shortcut: [f32; 3usize],
    pub fontactive: [f32; 3usize],
    pub fontinactive: [f32; 3usize],
    pub decorinactive: [f32; 3usize],
    pub decorinactive2: [f32; 3usize],
    pub button: [f32; 3usize],
    pub check: [f32; 3usize],
    pub radio: [f32; 3usize],
    pub select: [f32; 3usize],
    pub select2: [f32; 3usize],
    pub slider: [f32; 3usize],
    pub slider2: [f32; 3usize],
    pub edit: [f32; 3usize],
    pub edit2: [f32; 3usize],
    pub cursor: [f32; 3usize],
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjuiItemSingle {
    pub modifier: ::core::ffi::c_int,
    pub shortcut: ::core::ffi::c_int,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjuiItemMulti {
    pub nelem: ::core::ffi::c_int,
    pub(crate) name: [[::core::ffi::c_char; 40usize]; 35usize],
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjuiItemSlider {
    pub range: [f64; 2usize],
    pub divisions: f64,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjuiItemEdit {
    pub nelem: ::core::ffi::c_int,
    pub range: [[f64; 2usize]; 7usize],
}
#[repr(C)]
#[derive(Copy, Clone)]
pub struct mjuiItem {
    pub type_: ::core::ffi::c_int,
    pub(crate) name: [::core::ffi::c_char; 40usize],
    pub state: ::core::ffi::c_int,
    pub pdata: *mut ::core::ffi::c_void,
    pub(crate) sectionid: ::core::ffi::c_int,
    pub(crate) itemid: ::core::ffi::c_int,
    pub(crate) userid: ::core::ffi::c_int,
    pub(crate) __bindgen_anon_1: mjuiItem___bindgen_ty_1,
    pub rect: mjrRect,
    pub skip: ::core::ffi::c_int,
}
#[repr(C)]
#[derive(Copy, Clone)]
pub(crate) union mjuiItem___bindgen_ty_1 {
    pub single: mjuiItemSingle,
    pub multi: mjuiItemMulti,
    pub slider: mjuiItemSlider,
    pub edit: mjuiItemEdit,
}
#[repr(C)]
#[derive(Copy, Clone)]
pub struct mjuiSection {
    pub(crate) name: [::core::ffi::c_char; 40usize],
    pub state: ::core::ffi::c_int,
    pub modifier: ::core::ffi::c_int,
    pub shortcut: ::core::ffi::c_int,
    pub checkbox: ::core::ffi::c_int,
    pub nitem: ::core::ffi::c_int,
    pub item: [mjuiItem; 200usize],
    pub rtitle: mjrRect,
    pub rcontent: mjrRect,
    pub lastclick: ::core::ffi::c_int,
}
#[repr(C)]
#[derive(Copy, Clone)]
pub struct mjUI {
    pub spacing: mjuiThemeSpacing,
    pub color: mjuiThemeColor,
    pub predicate: mjfItemEnable,
    pub userdata: *mut ::core::ffi::c_void,
    pub(crate) rectid: ::core::ffi::c_int,
    pub(crate) auxid: ::core::ffi::c_int,
    pub radiocol: ::core::ffi::c_int,
    pub width: ::core::ffi::c_int,
    pub height: ::core::ffi::c_int,
    pub maxheight: ::core::ffi::c_int,
    pub scroll: ::core::ffi::c_int,
    pub mousesect: ::core::ffi::c_int,
    pub mouseitem: ::core::ffi::c_int,
    pub mousehelp: ::core::ffi::c_int,
    pub mouseclicks: ::core::ffi::c_int,
    pub mousesectcheck: ::core::ffi::c_int,
    pub editsect: ::core::ffi::c_int,
    pub edititem: ::core::ffi::c_int,
    pub editcursor: ::core::ffi::c_int,
    pub editscroll: ::core::ffi::c_int,
    pub(crate) edittext: [::core::ffi::c_char; 300usize],
    pub(crate) editchanged: *mut mjuiItem,
    pub nsect: ::core::ffi::c_int,
    pub sect: [mjuiSection; 10usize],
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct mjuiDef {
    pub type_: ::core::ffi::c_int,
    pub(crate) name: [::core::ffi::c_char; 40usize],
    pub state: ::core::ffi::c_int,
    pub pdata: *mut ::core::ffi::c_void,
    pub(crate) other: [::core::ffi::c_char; 300usize],
    pub otherint: ::core::ffi::c_int,
}
unsafe extern "C" {
    pub(crate) static mut mju_user_malloc:
        ::core::option::Option<unsafe extern "C" fn(arg1: usize) -> *mut ::core::ffi::c_void>;
    pub(crate) static mut mju_user_free:
        ::core::option::Option<unsafe extern "C" fn(arg1: *mut ::core::ffi::c_void)>;
    pub(crate) static mut mju_user_error:
        ::core::option::Option<unsafe extern "C" fn(arg1: *const ::core::ffi::c_char)>;
    pub(crate) static mut mju_user_warning:
        ::core::option::Option<unsafe extern "C" fn(arg1: *const ::core::ffi::c_char)>;
    pub(crate) static mut mjcb_passive: mjfGeneric;
    pub(crate) static mut mjcb_control: mjfGeneric;
    pub(crate) static mut mjcb_contactfilter: mjfConFilt;
    pub(crate) static mut mjcb_sensor: mjfSensor;
    pub(crate) static mut mjcb_time: mjfTime;
    pub(crate) static mut mjcb_act_dyn: mjfAct;
    pub(crate) static mut mjcb_act_gain: mjfAct;
    pub(crate) static mut mjcb_act_bias: mjfAct;
    pub(crate) static mut mjCOLLISIONFUNC: [[mjfCollision; 9usize]; 9usize];
    pub(crate) static mut mjDISABLESTRING: [*const ::core::ffi::c_char; 20usize];
    pub(crate) static mut mjENABLESTRING: [*const ::core::ffi::c_char; 6usize];
    pub(crate) static mut mjTIMERSTRING: [*const ::core::ffi::c_char; 15usize];
    pub(crate) static mut mjLABELSTRING: [*const ::core::ffi::c_char; 17usize];
    pub(crate) static mut mjFRAMESTRING: [*const ::core::ffi::c_char; 8usize];
    pub(crate) static mut mjVISSTRING: [[*const ::core::ffi::c_char; 3usize]; 31usize];
    pub(crate) static mut mjRNDSTRING: [[*const ::core::ffi::c_char; 3usize]; 11usize];
    pub(crate) static mut mjTOPICSTRING: [*const ::core::ffi::c_char; 3usize];
    pub(crate) fn mj_defaultVFS(vfs: *mut mjVFS);
    pub(crate) fn mj_mountVFS(
        vfs: *mut mjVFS,
        filepath: *const ::core::ffi::c_char,
        provider: *const mjpResourceProvider,
    ) -> ::core::ffi::c_int;
    pub(crate) fn mj_unmountVFS(
        vfs: *mut mjVFS,
        filename: *const ::core::ffi::c_char,
    ) -> ::core::ffi::c_int;
    pub(crate) fn mj_addFileVFS(
        vfs: *mut mjVFS,
        directory: *const ::core::ffi::c_char,
        filename: *const ::core::ffi::c_char,
    ) -> ::core::ffi::c_int;
    pub(crate) fn mj_addBufferVFS(
        vfs: *mut mjVFS,
        name: *const ::core::ffi::c_char,
        buffer: *const ::core::ffi::c_void,
        nbuffer: ::core::ffi::c_int,
    ) -> ::core::ffi::c_int;
    pub(crate) fn mj_deleteFileVFS(
        vfs: *mut mjVFS,
        filename: *const ::core::ffi::c_char,
    ) -> ::core::ffi::c_int;
    pub(crate) fn mj_containsBufferVFS(
        vfs: *mut mjVFS,
        name: *const ::core::ffi::c_char,
    ) -> ::core::ffi::c_int;
    pub(crate) fn mj_containsFileVFS(
        vfs: *mut mjVFS,
        directory: *const ::core::ffi::c_char,
        filename: *const ::core::ffi::c_char,
    ) -> ::core::ffi::c_int;
    pub(crate) fn mj_deleteVFS(vfs: *mut mjVFS);
    pub(crate) fn mj_getCacheSize(cache: *const mjCache) -> usize;
    pub(crate) fn mj_getCacheCapacity(cache: *const mjCache) -> usize;
    pub(crate) fn mj_setCacheCapacity(cache: *mut mjCache, size: usize) -> usize;
    pub(crate) fn mj_getCache() -> *mut mjCache;
    pub(crate) fn mj_clearCache(cache: *mut mjCache);
    pub(crate) fn mj_loadXML(
        filename: *const ::core::ffi::c_char,
        vfs: *const mjVFS,
        error: *mut ::core::ffi::c_char,
        error_sz: ::core::ffi::c_int,
    ) -> *mut mjModel;
    pub(crate) fn mj_parseXML(
        filename: *const ::core::ffi::c_char,
        vfs: *const mjVFS,
        error: *mut ::core::ffi::c_char,
        error_sz: ::core::ffi::c_int,
    ) -> *mut mjSpec;
    pub(crate) fn mj_parseXMLString(
        xml: *const ::core::ffi::c_char,
        vfs: *const mjVFS,
        error: *mut ::core::ffi::c_char,
        error_sz: ::core::ffi::c_int,
    ) -> *mut mjSpec;
    pub(crate) fn mj_parse(
        filename: *const ::core::ffi::c_char,
        content_type: *const ::core::ffi::c_char,
        vfs: *const mjVFS,
        error: *mut ::core::ffi::c_char,
        error_sz: ::core::ffi::c_int,
    ) -> *mut mjSpec;
    pub(crate) fn mj_encode(
        s: *const mjSpec,
        m: *const mjModel,
        filename: *const ::core::ffi::c_char,
        content_type: *const ::core::ffi::c_char,
        vfs: *const mjVFS,
        error: *mut ::core::ffi::c_char,
        error_sz: ::core::ffi::c_int,
    ) -> mjtSize;
    pub(crate) fn mj_compile(s: *mut mjSpec, vfs: *const mjVFS) -> *mut mjModel;
    pub(crate) fn mj_copyBack(s: *mut mjSpec, m: *const mjModel) -> ::core::ffi::c_int;
    pub(crate) fn mj_recompile(
        s: *mut mjSpec,
        vfs: *const mjVFS,
        m: *mut mjModel,
        d: *mut mjData,
    ) -> ::core::ffi::c_int;
    pub(crate) fn mj_saveLastXML(
        filename: *const ::core::ffi::c_char,
        m: *const mjModel,
        error: *mut ::core::ffi::c_char,
        error_sz: ::core::ffi::c_int,
    ) -> ::core::ffi::c_int;
    pub(crate) fn mj_freeLastXML();
    pub(crate) fn mj_saveXMLString(
        s: *const mjSpec,
        xml: *mut ::core::ffi::c_char,
        xml_sz: ::core::ffi::c_int,
        error: *mut ::core::ffi::c_char,
        error_sz: ::core::ffi::c_int,
    ) -> ::core::ffi::c_int;
    pub(crate) fn mj_saveXML(
        s: *const mjSpec,
        filename: *const ::core::ffi::c_char,
        error: *mut ::core::ffi::c_char,
        error_sz: ::core::ffi::c_int,
    ) -> ::core::ffi::c_int;
    pub(crate) fn mju_getXMLDependencies(
        filename: *const ::core::ffi::c_char,
        dependencies: *mut mjStringVec,
    );
    pub(crate) fn mj_step(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_step1(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_step2(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_forward(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_inverse(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_forwardSkip(
        m: *const mjModel,
        d: *mut mjData,
        skipstage: ::core::ffi::c_int,
        skipsensor: ::core::ffi::c_int,
    );
    pub(crate) fn mj_inverseSkip(
        m: *const mjModel,
        d: *mut mjData,
        skipstage: ::core::ffi::c_int,
        skipsensor: ::core::ffi::c_int,
    );
    pub(crate) fn mj_defaultLROpt(opt: *mut mjLROpt);
    pub(crate) fn mj_defaultSolRefImp(solref: *mut mjtNum, solimp: *mut mjtNum);
    pub(crate) fn mj_defaultOption(opt: *mut mjOption);
    pub(crate) fn mj_defaultVisual(vis: *mut mjVisual);
    pub(crate) fn mj_copyModel(dest: *mut mjModel, src: *const mjModel) -> *mut mjModel;
    pub(crate) fn mj_saveModel(
        m: *const mjModel,
        filename: *const ::core::ffi::c_char,
        buffer: *mut ::core::ffi::c_void,
        buffer_sz: ::core::ffi::c_int,
    );
    pub(crate) fn mj_loadModel(
        filename: *const ::core::ffi::c_char,
        vfs: *const mjVFS,
    ) -> *mut mjModel;
    pub(crate) fn mj_loadModelBuffer(
        buffer: *const ::core::ffi::c_void,
        buffer_sz: ::core::ffi::c_int,
    ) -> *mut mjModel;
    pub(crate) fn mj_deleteModel(m: *mut mjModel);
    pub(crate) fn mj_sizeModel(m: *const mjModel) -> mjtSize;
    pub(crate) fn mj_makeData(m: *const mjModel) -> *mut mjData;
    pub(crate) fn mj_copyData(
        dest: *mut mjData,
        m: *const mjModel,
        src: *const mjData,
    ) -> *mut mjData;
    pub(crate) fn mjv_copyData(
        dest: *mut mjData,
        m: *const mjModel,
        src: *const mjData,
    ) -> *mut mjData;
    pub(crate) fn mj_resetCtrl(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_resetData(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_resetDataDebug(
        m: *const mjModel,
        d: *mut mjData,
        debug_value: ::core::ffi::c_uchar,
    );
    pub(crate) fn mj_resetDataKeyframe(m: *const mjModel, d: *mut mjData, key: ::core::ffi::c_int);
    pub(crate) fn mj_markStack(d: *mut mjData);
    pub(crate) fn mj_freeStack(d: *mut mjData);
    pub(crate) fn mj_stackAllocByte(
        d: *mut mjData,
        bytes: usize,
        alignment: usize,
    ) -> *mut ::core::ffi::c_void;
    pub(crate) fn mj_stackAllocNum(d: *mut mjData, size: usize) -> *mut mjtNum;
    pub(crate) fn mj_stackAllocInt(d: *mut mjData, size: usize) -> *mut ::core::ffi::c_int;
    pub(crate) fn mj_deleteData(d: *mut mjData);
    pub(crate) fn mj_resetCallbacks();
    pub(crate) fn mj_setConst(m: *mut mjModel, d: *mut mjData);
    pub(crate) fn mj_setLengthRange(
        m: *mut mjModel,
        d: *mut mjData,
        index: ::core::ffi::c_int,
        opt: *const mjLROpt,
        error: *mut ::core::ffi::c_char,
        error_sz: ::core::ffi::c_int,
    ) -> ::core::ffi::c_int;
    pub(crate) fn mj_makeSpec() -> *mut mjSpec;
    pub(crate) fn mj_copySpec(s: *const mjSpec) -> *mut mjSpec;
    pub(crate) fn mj_deleteSpec(s: *mut mjSpec);
    pub(crate) fn mjs_activatePlugin(
        s: *mut mjSpec,
        name: *const ::core::ffi::c_char,
    ) -> ::core::ffi::c_int;
    pub(crate) fn mjs_setDeepCopy(
        s: *mut mjSpec,
        deepcopy: ::core::ffi::c_int,
    ) -> ::core::ffi::c_int;
    pub(crate) fn mj_printFormattedModel(
        m: *const mjModel,
        filename: *const ::core::ffi::c_char,
        float_format: *const ::core::ffi::c_char,
    );
    pub(crate) fn mj_printModel(m: *const mjModel, filename: *const ::core::ffi::c_char);
    pub(crate) fn mj_printFormattedData(
        m: *const mjModel,
        d: *const mjData,
        filename: *const ::core::ffi::c_char,
        float_format: *const ::core::ffi::c_char,
    );
    pub(crate) fn mj_printData(
        m: *const mjModel,
        d: *const mjData,
        filename: *const ::core::ffi::c_char,
    );
    pub(crate) fn mju_printMat(mat: *const mjtNum, nr: ::core::ffi::c_int, nc: ::core::ffi::c_int);
    pub(crate) fn mju_printMatSparse(
        mat: *const mjtNum,
        nr: ::core::ffi::c_int,
        rownnz: *const ::core::ffi::c_int,
        rowadr: *const ::core::ffi::c_int,
        colind: *const ::core::ffi::c_int,
    );
    pub(crate) fn mj_printSchema(
        filename: *const ::core::ffi::c_char,
        buffer: *mut ::core::ffi::c_char,
        buffer_sz: ::core::ffi::c_int,
        flg_html: ::core::ffi::c_int,
        flg_pad: ::core::ffi::c_int,
    ) -> ::core::ffi::c_int;
    pub(crate) fn mj_printScene(s: *const mjvScene, filename: *const ::core::ffi::c_char);
    pub(crate) fn mj_printFormattedScene(
        s: *const mjvScene,
        filename: *const ::core::ffi::c_char,
        float_format: *const ::core::ffi::c_char,
    );
    pub(crate) fn mj_fwdKinematics(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_fwdPosition(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_fwdVelocity(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_fwdActuation(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_fwdAcceleration(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_fwdConstraint(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_Euler(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_RungeKutta(m: *const mjModel, d: *mut mjData, N: ::core::ffi::c_int);
    pub(crate) fn mj_implicit(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_invPosition(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_invVelocity(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_invConstraint(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_compareFwdInv(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_sensorPos(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_sensorVel(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_sensorAcc(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_energyPos(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_energyVel(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_checkPos(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_checkVel(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_checkAcc(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_kinematics(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_comPos(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_camlight(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_flex(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_tendon(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_transmission(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_crb(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_makeM(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_factorM(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_solveM(
        m: *const mjModel,
        d: *mut mjData,
        x: *mut mjtNum,
        y: *const mjtNum,
        n: ::core::ffi::c_int,
    );
    pub(crate) fn mj_solveM2(
        m: *const mjModel,
        d: *mut mjData,
        x: *mut mjtNum,
        y: *const mjtNum,
        sqrtInvD: *const mjtNum,
        n: ::core::ffi::c_int,
    );
    pub(crate) fn mj_comVel(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_passive(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_subtreeVel(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_rne(
        m: *const mjModel,
        d: *mut mjData,
        flg_acc: ::core::ffi::c_int,
        result: *mut mjtNum,
    );
    pub(crate) fn mj_rnePostConstraint(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_maxContact(
        m: *const mjModel,
        g1: ::core::ffi::c_int,
        g2: ::core::ffi::c_int,
        has_margin: ::core::ffi::c_int,
    ) -> ::core::ffi::c_int;
    pub(crate) fn mj_collision(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_makeConstraint(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_island(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_projectConstraint(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_referenceConstraint(m: *const mjModel, d: *mut mjData);
    pub(crate) fn mj_constraintUpdate(
        m: *const mjModel,
        d: *mut mjData,
        jar: *const mjtNum,
        cost: *mut [mjtNum; 1usize],
        flg_coneHessian: ::core::ffi::c_int,
    );
    pub(crate) fn mj_stateSize(m: *const mjModel, sig: ::core::ffi::c_int) -> ::core::ffi::c_int;
    pub(crate) fn mj_getState(
        m: *const mjModel,
        d: *const mjData,
        state: *mut mjtNum,
        sig: ::core::ffi::c_int,
    );
    pub(crate) fn mj_extractState(
        m: *const mjModel,
        src: *const mjtNum,
        srcsig: ::core::ffi::c_int,
        dst: *mut mjtNum,
        dstsig: ::core::ffi::c_int,
    );
    pub(crate) fn mj_setState(
        m: *const mjModel,
        d: *mut mjData,
        state: *const mjtNum,
        sig: ::core::ffi::c_int,
    );
    pub(crate) fn mj_copyState(
        m: *const mjModel,
        src: *const mjData,
        dst: *mut mjData,
        sig: ::core::ffi::c_int,
    );
    pub(crate) fn mj_readCtrl(
        m: *const mjModel,
        d: *const mjData,
        id: ::core::ffi::c_int,
        time: mjtNum,
        interp: ::core::ffi::c_int,
    ) -> mjtNum;
    pub(crate) fn mj_readSensor(
        m: *const mjModel,
        d: *const mjData,
        id: ::core::ffi::c_int,
        time: mjtNum,
        result: *mut mjtNum,
        interp: ::core::ffi::c_int,
    ) -> *const mjtNum;
    pub(crate) fn mj_initCtrlHistory(
        m: *const mjModel,
        d: *mut mjData,
        id: ::core::ffi::c_int,
        times: *const mjtNum,
        values: *const mjtNum,
    );
    pub(crate) fn mj_initSensorHistory(
        m: *const mjModel,
        d: *mut mjData,
        id: ::core::ffi::c_int,
        times: *const mjtNum,
        values: *const mjtNum,
        phase: mjtNum,
    );
    pub(crate) fn mj_setKeyframe(m: *mut mjModel, d: *const mjData, k: ::core::ffi::c_int);
    pub(crate) fn mj_addContact(
        m: *const mjModel,
        d: *mut mjData,
        con: *const mjContact,
    ) -> ::core::ffi::c_int;
    pub(crate) fn mj_isPyramidal(m: *const mjModel) -> ::core::ffi::c_int;
    pub(crate) fn mj_isSparse(m: *const mjModel) -> ::core::ffi::c_int;
    pub(crate) fn mj_isDual(m: *const mjModel) -> ::core::ffi::c_int;
    pub(crate) fn mj_mulJacVec(
        m: *const mjModel,
        d: *const mjData,
        res: *mut mjtNum,
        vec: *const mjtNum,
    );
    pub(crate) fn mj_mulJacTVec(
        m: *const mjModel,
        d: *const mjData,
        res: *mut mjtNum,
        vec: *const mjtNum,
    );
    pub(crate) fn mj_jac(
        m: *const mjModel,
        d: *const mjData,
        jacp: *mut mjtNum,
        jacr: *mut mjtNum,
        point: *const [mjtNum; 3usize],
        body: ::core::ffi::c_int,
    );
    pub(crate) fn mj_jacBody(
        m: *const mjModel,
        d: *const mjData,
        jacp: *mut mjtNum,
        jacr: *mut mjtNum,
        body: ::core::ffi::c_int,
    );
    pub(crate) fn mj_jacBodyCom(
        m: *const mjModel,
        d: *const mjData,
        jacp: *mut mjtNum,
        jacr: *mut mjtNum,
        body: ::core::ffi::c_int,
    );
    pub(crate) fn mj_jacSubtreeCom(
        m: *const mjModel,
        d: *mut mjData,
        jacp: *mut mjtNum,
        body: ::core::ffi::c_int,
    );
    pub(crate) fn mj_jacGeom(
        m: *const mjModel,
        d: *const mjData,
        jacp: *mut mjtNum,
        jacr: *mut mjtNum,
        geom: ::core::ffi::c_int,
    );
    pub(crate) fn mj_jacSite(
        m: *const mjModel,
        d: *const mjData,
        jacp: *mut mjtNum,
        jacr: *mut mjtNum,
        site: ::core::ffi::c_int,
    );
    pub(crate) fn mj_jacPointAxis(
        m: *const mjModel,
        d: *mut mjData,
        jacPoint: *mut mjtNum,
        jacAxis: *mut mjtNum,
        point: *const [mjtNum; 3usize],
        axis: *const [mjtNum; 3usize],
        body: ::core::ffi::c_int,
    );
    pub(crate) fn mj_jacDot(
        m: *const mjModel,
        d: *const mjData,
        jacp: *mut mjtNum,
        jacr: *mut mjtNum,
        point: *const [mjtNum; 3usize],
        body: ::core::ffi::c_int,
    );
    pub(crate) fn mj_angmomMat(
        m: *const mjModel,
        d: *mut mjData,
        mat: *mut mjtNum,
        body: ::core::ffi::c_int,
    );
    pub(crate) fn mj_name2id(
        m: *const mjModel,
        type_: ::core::ffi::c_int,
        name: *const ::core::ffi::c_char,
    ) -> ::core::ffi::c_int;
    pub(crate) fn mj_id2name(
        m: *const mjModel,
        type_: ::core::ffi::c_int,
        id: ::core::ffi::c_int,
    ) -> *const ::core::ffi::c_char;
    pub(crate) fn mj_actuatorInputName(
        m: *const mjModel,
        id: ::core::ffi::c_int,
        input: ::core::ffi::c_int,
    ) -> *const ::core::ffi::c_char;
    pub(crate) fn mj_fullM(m: *const mjModel, d: *const mjData, dst: *mut mjtNum);
    pub(crate) fn mj_mulM(
        m: *const mjModel,
        d: *const mjData,
        res: *mut mjtNum,
        vec: *const mjtNum,
    );
    pub(crate) fn mj_mulM2(
        m: *const mjModel,
        d: *const mjData,
        res: *mut mjtNum,
        vec: *const mjtNum,
    );
    pub(crate) fn mj_addM(
        m: *const mjModel,
        d: *mut mjData,
        dst: *mut mjtNum,
        rownnz: *mut ::core::ffi::c_int,
        rowadr: *mut ::core::ffi::c_int,
        colind: *mut ::core::ffi::c_int,
    );
    pub(crate) fn mj_applyFT(
        m: *const mjModel,
        d: *mut mjData,
        force: *const [mjtNum; 3usize],
        torque: *const [mjtNum; 3usize],
        point: *const [mjtNum; 3usize],
        body: ::core::ffi::c_int,
        qfrc_target: *mut mjtNum,
    );
    pub(crate) fn mj_objectVelocity(
        m: *const mjModel,
        d: *const mjData,
        objtype: ::core::ffi::c_int,
        objid: ::core::ffi::c_int,
        res: *mut [mjtNum; 6usize],
        flg_local: ::core::ffi::c_int,
    );
    pub(crate) fn mj_objectAcceleration(
        m: *const mjModel,
        d: *const mjData,
        objtype: ::core::ffi::c_int,
        objid: ::core::ffi::c_int,
        res: *mut [mjtNum; 6usize],
        flg_local: ::core::ffi::c_int,
    );
    pub(crate) fn mj_geomDistance(
        m: *const mjModel,
        d: *mut mjData,
        geom1: ::core::ffi::c_int,
        geom2: ::core::ffi::c_int,
        distmax: mjtNum,
        fromto: *mut [mjtNum; 6usize],
    ) -> mjtNum;
    pub(crate) fn mj_contactForce(
        m: *const mjModel,
        d: *const mjData,
        id: ::core::ffi::c_int,
        result: *mut [mjtNum; 6usize],
    );
    pub(crate) fn mj_differentiatePos(
        m: *const mjModel,
        qvel: *mut mjtNum,
        dt: mjtNum,
        qpos1: *const mjtNum,
        qpos2: *const mjtNum,
    );
    pub(crate) fn mj_integratePos(
        m: *const mjModel,
        qpos: *mut mjtNum,
        qvel: *const mjtNum,
        dt: mjtNum,
    );
    pub(crate) fn mj_normalizeQuat(m: *const mjModel, qpos: *mut mjtNum);
    pub(crate) fn mj_local2Global(
        d: *mut mjData,
        xpos: *mut [mjtNum; 3usize],
        xmat: *mut [mjtNum; 9usize],
        pos: *const [mjtNum; 3usize],
        quat: *const [mjtNum; 4usize],
        body: ::core::ffi::c_int,
        sameframe: mjtByte,
    );
    pub(crate) fn mj_getTotalmass(m: *const mjModel) -> mjtNum;
    pub(crate) fn mj_setTotalmass(m: *mut mjModel, newmass: mjtNum);
    pub(crate) fn mj_getPluginConfig(
        m: *const mjModel,
        plugin_id: ::core::ffi::c_int,
        attrib: *const ::core::ffi::c_char,
    ) -> *const ::core::ffi::c_char;
    pub(crate) fn mj_loadPluginLibrary(path: *const ::core::ffi::c_char);
    pub(crate) fn mj_loadAllPluginLibraries(
        directory: *const ::core::ffi::c_char,
        callback: mjfPluginLibraryLoadCallback,
    );
    pub(crate) fn mj_version() -> ::core::ffi::c_int;
    pub(crate) fn mj_versionString() -> *const ::core::ffi::c_char;
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
    pub(crate) fn mj_rayHfield(
        m: *const mjModel,
        d: *const mjData,
        geomid: ::core::ffi::c_int,
        pnt: *const [mjtNum; 3usize],
        vec: *const [mjtNum; 3usize],
        normal: *mut [mjtNum; 3usize],
    ) -> mjtNum;
    pub(crate) fn mj_rayMesh(
        m: *const mjModel,
        d: *const mjData,
        geomid: ::core::ffi::c_int,
        pnt: *const [mjtNum; 3usize],
        vec: *const [mjtNum; 3usize],
        normal: *mut [mjtNum; 3usize],
    ) -> mjtNum;
    pub(crate) fn mju_rayGeom(
        pos: *const [mjtNum; 3usize],
        mat: *const [mjtNum; 9usize],
        size: *const [mjtNum; 3usize],
        pnt: *const [mjtNum; 3usize],
        vec: *const [mjtNum; 3usize],
        geomtype: ::core::ffi::c_int,
        normal: *mut [mjtNum; 3usize],
    ) -> mjtNum;
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
    pub(crate) fn mju_raySkin(
        nface: ::core::ffi::c_int,
        nvert: ::core::ffi::c_int,
        face: *const ::core::ffi::c_int,
        vert: *const f32,
        pnt: *const [mjtNum; 3usize],
        vec: *const [mjtNum; 3usize],
        vertid: *mut [::core::ffi::c_int; 1usize],
    ) -> mjtNum;
    pub(crate) fn mjv_defaultCamera(cam: *mut mjvCamera);
    pub(crate) fn mjv_defaultFreeCamera(m: *const mjModel, cam: *mut mjvCamera);
    pub(crate) fn mjv_defaultPerturb(pert: *mut mjvPerturb);
    pub(crate) fn mjv_room2model(
        modelpos: *mut [mjtNum; 3usize],
        modelquat: *mut [mjtNum; 4usize],
        roompos: *const [mjtNum; 3usize],
        roomquat: *const [mjtNum; 4usize],
        scn: *const mjvScene,
    );
    pub(crate) fn mjv_model2room(
        roompos: *mut [mjtNum; 3usize],
        roomquat: *mut [mjtNum; 4usize],
        modelpos: *const [mjtNum; 3usize],
        modelquat: *const [mjtNum; 4usize],
        scn: *const mjvScene,
    );
    pub(crate) fn mjv_cameraInModel(
        headpos: *mut [mjtNum; 3usize],
        forward: *mut [mjtNum; 3usize],
        up: *mut [mjtNum; 3usize],
        scn: *const mjvScene,
    );
    pub(crate) fn mjv_cameraInRoom(
        headpos: *mut [mjtNum; 3usize],
        forward: *mut [mjtNum; 3usize],
        up: *mut [mjtNum; 3usize],
        scn: *const mjvScene,
    );
    pub(crate) fn mjv_frustumHeight(scn: *const mjvScene) -> mjtNum;
    pub(crate) fn mjv_alignToCamera(
        res: *mut [mjtNum; 3usize],
        vec: *const [mjtNum; 3usize],
        forward: *const [mjtNum; 3usize],
    );
    pub(crate) fn mjv_moveCamera(
        m: *const mjModel,
        action: ::core::ffi::c_int,
        reldx: mjtNum,
        reldy: mjtNum,
        cam: *mut mjvCamera,
    );
    pub(crate) fn mjv_movePerturb(
        m: *const mjModel,
        d: *const mjData,
        action: ::core::ffi::c_int,
        reldx: mjtNum,
        reldy: mjtNum,
        scn: *const mjvScene,
        pert: *mut mjvPerturb,
    );
    pub(crate) fn mjv_moveModel(
        m: *const mjModel,
        action: ::core::ffi::c_int,
        reldx: mjtNum,
        reldy: mjtNum,
        roomup: *const [mjtNum; 3usize],
        scn: *mut mjvScene,
    );
    pub(crate) fn mjv_initPerturb(
        m: *const mjModel,
        d: *mut mjData,
        scn: *const mjvScene,
        pert: *mut mjvPerturb,
    );
    pub(crate) fn mjv_applyPerturbPose(
        m: *const mjModel,
        d: *mut mjData,
        pert: *const mjvPerturb,
        flg_paused: ::core::ffi::c_int,
    );
    pub(crate) fn mjv_applyPerturbForce(m: *const mjModel, d: *mut mjData, pert: *const mjvPerturb);
    pub(crate) fn mjv_averageCamera(
        cam1: *const mjvGLCamera,
        cam2: *const mjvGLCamera,
    ) -> mjvGLCamera;
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
    pub(crate) fn mjv_defaultOption(opt: *mut mjvOption);
    pub(crate) fn mjv_defaultFigure(fig: *mut mjvFigure);
    pub(crate) fn mjv_initGeom(
        geom: *mut mjvGeom,
        type_: ::core::ffi::c_int,
        size: *const [mjtNum; 3usize],
        pos: *const [mjtNum; 3usize],
        mat: *const [mjtNum; 9usize],
        rgba: *const [f32; 4usize],
    );
    pub(crate) fn mjv_connector(
        geom: *mut mjvGeom,
        type_: ::core::ffi::c_int,
        width: mjtNum,
        from: *const [mjtNum; 3usize],
        to: *const [mjtNum; 3usize],
    );
    pub(crate) fn mjv_defaultScene(scn: *mut mjvScene);
    pub(crate) fn mjv_makeScene(m: *const mjModel, scn: *mut mjvScene, maxgeom: ::core::ffi::c_int);
    pub(crate) fn mjv_freeScene(scn: *mut mjvScene);
    pub(crate) fn mjv_updateScene(
        m: *const mjModel,
        d: *mut mjData,
        opt: *const mjvOption,
        pert: *const mjvPerturb,
        cam: *mut mjvCamera,
        catmask: ::core::ffi::c_int,
        scn: *mut mjvScene,
    );
    pub(crate) fn mjv_copyModel(dest: *mut mjModel, src: *const mjModel);
    pub(crate) fn mjv_addGeoms(
        m: *const mjModel,
        d: *mut mjData,
        opt: *const mjvOption,
        pert: *const mjvPerturb,
        catmask: ::core::ffi::c_int,
        scn: *mut mjvScene,
    );
    pub(crate) fn mjv_makeLights(m: *const mjModel, d: *const mjData, scn: *mut mjvScene);
    pub(crate) fn mjv_updateCamera(
        m: *const mjModel,
        d: *const mjData,
        cam: *mut mjvCamera,
        scn: *mut mjvScene,
    );
    pub(crate) fn mjv_updateSkin(m: *const mjModel, d: *const mjData, scn: *mut mjvScene);
    pub(crate) fn mjv_cameraFrame(
        headpos: *mut [mjtNum; 3usize],
        forward: *mut [mjtNum; 3usize],
        up: *mut [mjtNum; 3usize],
        right: *mut [mjtNum; 3usize],
        d: *const mjData,
        cam: *const mjvCamera,
    );
    pub(crate) fn mjv_cameraFrustum(
        zver: *mut [f32; 2usize],
        zhor: *mut [f32; 2usize],
        zclip: *mut [f32; 2usize],
        m: *const mjModel,
        cam: *const mjvCamera,
    );
    pub(crate) fn mjr_defaultContext(con: *mut mjrContext);
    pub(crate) fn mjr_defaultRendererInfo(info: *mut mjrRendererInfo);
    pub(crate) fn mjr_getRendererInfo(info: *mut mjrRendererInfo);
    pub(crate) fn mjr_makeContext(
        m: *const mjModel,
        con: *mut mjrContext,
        fontscale: ::core::ffi::c_int,
    );
    pub(crate) fn mjr_changeFont(fontscale: ::core::ffi::c_int, con: *mut mjrContext);
    pub(crate) fn mjr_addAux(
        index: ::core::ffi::c_int,
        width: ::core::ffi::c_int,
        height: ::core::ffi::c_int,
        samples: ::core::ffi::c_int,
        con: *mut mjrContext,
    );
    pub(crate) fn mjr_freeContext(con: *mut mjrContext);
    pub(crate) fn mjr_resizeOffscreen(
        width: ::core::ffi::c_int,
        height: ::core::ffi::c_int,
        con: *mut mjrContext,
    );
    pub(crate) fn mjr_uploadTexture(
        m: *const mjModel,
        con: *const mjrContext,
        texid: ::core::ffi::c_int,
    );
    pub(crate) fn mjr_uploadMesh(
        m: *const mjModel,
        con: *const mjrContext,
        meshid: ::core::ffi::c_int,
    );
    pub(crate) fn mjr_uploadHField(
        m: *const mjModel,
        con: *const mjrContext,
        hfieldid: ::core::ffi::c_int,
    );
    pub(crate) fn mjr_restoreBuffer(con: *const mjrContext);
    pub(crate) fn mjr_setBuffer(framebuffer: ::core::ffi::c_int, con: *mut mjrContext);
    pub(crate) fn mjr_readPixels(
        rgb: *mut ::core::ffi::c_uchar,
        depth: *mut f32,
        viewport: mjrRect,
        con: *const mjrContext,
    );
    pub(crate) fn mjr_drawPixels(
        rgb: *const ::core::ffi::c_uchar,
        depth: *const f32,
        viewport: mjrRect,
        con: *const mjrContext,
    );
    pub(crate) fn mjr_blitBuffer(
        src: mjrRect,
        dst: mjrRect,
        flg_color: ::core::ffi::c_int,
        flg_depth: ::core::ffi::c_int,
        con: *const mjrContext,
    );
    pub(crate) fn mjr_setAux(index: ::core::ffi::c_int, con: *const mjrContext);
    pub(crate) fn mjr_blitAux(
        index: ::core::ffi::c_int,
        src: mjrRect,
        left: ::core::ffi::c_int,
        bottom: ::core::ffi::c_int,
        con: *const mjrContext,
    );
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
    pub(crate) fn mjr_overlay(
        font: ::core::ffi::c_int,
        gridpos: ::core::ffi::c_int,
        viewport: mjrRect,
        overlay: *const ::core::ffi::c_char,
        overlay2: *const ::core::ffi::c_char,
        con: *const mjrContext,
    );
    pub(crate) fn mjr_maxViewport(con: *const mjrContext) -> mjrRect;
    pub(crate) fn mjr_rectangle(viewport: mjrRect, r: f32, g: f32, b: f32, a: f32);
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
    pub(crate) fn mjr_figure(viewport: mjrRect, fig: *mut mjvFigure, con: *const mjrContext);
    pub(crate) fn mjr_render(viewport: mjrRect, scn: *mut mjvScene, con: *const mjrContext);
    pub(crate) fn mjr_finish();
    pub(crate) fn mjr_getError() -> ::core::ffi::c_int;
    pub(crate) fn mjr_findRect(
        x: ::core::ffi::c_int,
        y: ::core::ffi::c_int,
        nrect: ::core::ffi::c_int,
        rect: *const mjrRect,
    ) -> ::core::ffi::c_int;
    pub(crate) fn mjui_themeSpacing(ind: ::core::ffi::c_int) -> mjuiThemeSpacing;
    pub(crate) fn mjui_themeColor(ind: ::core::ffi::c_int) -> mjuiThemeColor;
    pub(crate) fn mjui_add(ui: *mut mjUI, def: *const mjuiDef);
    pub(crate) fn mjui_addToSection(ui: *mut mjUI, sect: ::core::ffi::c_int, def: *const mjuiDef);
    pub(crate) fn mjui_resize(ui: *mut mjUI, con: *const mjrContext);
    pub(crate) fn mjui_update(
        section: ::core::ffi::c_int,
        item: ::core::ffi::c_int,
        ui: *const mjUI,
        state: *const mjuiState,
        con: *const mjrContext,
    );
    pub(crate) fn mjui_event(
        ui: *mut mjUI,
        state: *mut mjuiState,
        con: *const mjrContext,
    ) -> *mut mjuiItem;
    pub(crate) fn mjui_render(ui: *mut mjUI, state: *const mjuiState, con: *const mjrContext);
    pub(crate) fn mju_error(msg: *const ::core::ffi::c_char, ...);
    pub(crate) fn mju_warning(msg: *const ::core::ffi::c_char, ...);
    pub(crate) fn mju_clearHandlers();
    pub(crate) fn mju_setLogHandler(handler: mjfLogHandler) -> mjfLogHandler;
    pub(crate) fn mju_getLogConfig() -> mjLogConfig;
    pub(crate) fn mju_setLogConfig(config: mjLogConfig);
    pub(crate) fn mju_info(topic: ::core::ffi::c_int, msg: *const ::core::ffi::c_char, ...);
    pub(crate) fn mju_message(msg: *const mjLogMessage);
    pub(crate) fn mju_malloc(size: usize) -> *mut ::core::ffi::c_void;
    pub(crate) fn mju_free(ptr: *mut ::core::ffi::c_void);
    pub(crate) fn mj_warning(d: *mut mjData, warning: ::core::ffi::c_int, info: ::core::ffi::c_int);
    pub(crate) fn mju_writeLog(type_: *const ::core::ffi::c_char, msg: *const ::core::ffi::c_char);
    pub(crate) fn mjs_getError(s: *mut mjSpec) -> *const ::core::ffi::c_char;
    pub(crate) fn mjs_getTimer(s: *mut mjSpec) -> *const f64;
    pub(crate) fn mjs_isWarning(s: *mut mjSpec) -> ::core::ffi::c_int;
    pub(crate) fn mjs_numWarnings(spec: *const mjSpec) -> ::core::ffi::c_int;
    pub(crate) fn mjs_getWarning(
        spec: *const mjSpec,
        index: ::core::ffi::c_int,
    ) -> *const ::core::ffi::c_char;
    pub(crate) fn mju_zero3(res: *mut [mjtNum; 3usize]);
    pub(crate) fn mju_copy3(res: *mut [mjtNum; 3usize], data: *const [mjtNum; 3usize]);
    pub(crate) fn mju_scl3(res: *mut [mjtNum; 3usize], vec: *const [mjtNum; 3usize], scl: mjtNum);
    pub(crate) fn mju_add3(
        res: *mut [mjtNum; 3usize],
        vec1: *const [mjtNum; 3usize],
        vec2: *const [mjtNum; 3usize],
    );
    pub(crate) fn mju_sub3(
        res: *mut [mjtNum; 3usize],
        vec1: *const [mjtNum; 3usize],
        vec2: *const [mjtNum; 3usize],
    );
    pub(crate) fn mju_addTo3(res: *mut [mjtNum; 3usize], vec: *const [mjtNum; 3usize]);
    pub(crate) fn mju_subFrom3(res: *mut [mjtNum; 3usize], vec: *const [mjtNum; 3usize]);
    pub(crate) fn mju_addToScl3(
        res: *mut [mjtNum; 3usize],
        vec: *const [mjtNum; 3usize],
        scl: mjtNum,
    );
    pub(crate) fn mju_addScl3(
        res: *mut [mjtNum; 3usize],
        vec1: *const [mjtNum; 3usize],
        vec2: *const [mjtNum; 3usize],
        scl: mjtNum,
    );
    pub(crate) fn mju_normalize3(vec: *mut [mjtNum; 3usize]) -> mjtNum;
    pub(crate) fn mju_norm3(vec: *const [mjtNum; 3usize]) -> mjtNum;
    pub(crate) fn mju_dot3(vec1: *const [mjtNum; 3usize], vec2: *const [mjtNum; 3usize]) -> mjtNum;
    pub(crate) fn mju_dist3(pos1: *const [mjtNum; 3usize], pos2: *const [mjtNum; 3usize])
    -> mjtNum;
    pub(crate) fn mju_mulMatVec3(
        res: *mut [mjtNum; 3usize],
        mat: *const [mjtNum; 9usize],
        vec: *const [mjtNum; 3usize],
    );
    pub(crate) fn mju_mulMatTVec3(
        res: *mut [mjtNum; 3usize],
        mat: *const [mjtNum; 9usize],
        vec: *const [mjtNum; 3usize],
    );
    pub(crate) fn mju_cross(
        res: *mut [mjtNum; 3usize],
        a: *const [mjtNum; 3usize],
        b: *const [mjtNum; 3usize],
    );
    pub(crate) fn mju_zero4(res: *mut [mjtNum; 4usize]);
    pub(crate) fn mju_unit4(res: *mut [mjtNum; 4usize]);
    pub(crate) fn mju_copy4(res: *mut [mjtNum; 4usize], data: *const [mjtNum; 4usize]);
    pub(crate) fn mju_normalize4(vec: *mut [mjtNum; 4usize]) -> mjtNum;
    pub(crate) fn mju_zero(res: *mut mjtNum, n: ::core::ffi::c_int);
    pub(crate) fn mju_fill(res: *mut mjtNum, val: mjtNum, n: ::core::ffi::c_int);
    pub(crate) fn mju_copy(res: *mut mjtNum, vec: *const mjtNum, n: ::core::ffi::c_int);
    pub(crate) fn mju_sum(vec: *const mjtNum, n: ::core::ffi::c_int) -> mjtNum;
    pub(crate) fn mju_L1(vec: *const mjtNum, n: ::core::ffi::c_int) -> mjtNum;
    pub(crate) fn mju_scl(res: *mut mjtNum, vec: *const mjtNum, scl: mjtNum, n: ::core::ffi::c_int);
    pub(crate) fn mju_add(
        res: *mut mjtNum,
        vec1: *const mjtNum,
        vec2: *const mjtNum,
        n: ::core::ffi::c_int,
    );
    pub(crate) fn mju_sub(
        res: *mut mjtNum,
        vec1: *const mjtNum,
        vec2: *const mjtNum,
        n: ::core::ffi::c_int,
    );
    pub(crate) fn mju_addTo(res: *mut mjtNum, vec: *const mjtNum, n: ::core::ffi::c_int);
    pub(crate) fn mju_subFrom(res: *mut mjtNum, vec: *const mjtNum, n: ::core::ffi::c_int);
    pub(crate) fn mju_addToScl(
        res: *mut mjtNum,
        vec: *const mjtNum,
        scl: mjtNum,
        n: ::core::ffi::c_int,
    );
    pub(crate) fn mju_addScl(
        res: *mut mjtNum,
        vec1: *const mjtNum,
        vec2: *const mjtNum,
        scl: mjtNum,
        n: ::core::ffi::c_int,
    );
    pub(crate) fn mju_normalize(res: *mut mjtNum, n: ::core::ffi::c_int) -> mjtNum;
    pub(crate) fn mju_norm(res: *const mjtNum, n: ::core::ffi::c_int) -> mjtNum;
    pub(crate) fn mju_dot(
        vec1: *const mjtNum,
        vec2: *const mjtNum,
        n: ::core::ffi::c_int,
    ) -> mjtNum;
    pub(crate) fn mju_mulMatVec(
        res: *mut mjtNum,
        mat: *const mjtNum,
        vec: *const mjtNum,
        nr: ::core::ffi::c_int,
        nc: ::core::ffi::c_int,
    );
    pub(crate) fn mju_mulMatTVec(
        res: *mut mjtNum,
        mat: *const mjtNum,
        vec: *const mjtNum,
        nr: ::core::ffi::c_int,
        nc: ::core::ffi::c_int,
    );
    pub(crate) fn mju_mulVecMatVec(
        vec1: *const mjtNum,
        mat: *const mjtNum,
        vec2: *const mjtNum,
        n: ::core::ffi::c_int,
    ) -> mjtNum;
    pub(crate) fn mju_transpose(
        res: *mut mjtNum,
        mat: *const mjtNum,
        nr: ::core::ffi::c_int,
        nc: ::core::ffi::c_int,
    );
    pub(crate) fn mju_symmetrize(res: *mut mjtNum, mat: *const mjtNum, n: ::core::ffi::c_int);
    pub(crate) fn mju_eye(mat: *mut mjtNum, n: ::core::ffi::c_int);
    pub(crate) fn mju_mulMatMat(
        res: *mut mjtNum,
        mat1: *const mjtNum,
        mat2: *const mjtNum,
        r1: ::core::ffi::c_int,
        c1: ::core::ffi::c_int,
        c2: ::core::ffi::c_int,
    );
    pub(crate) fn mju_mulMatMatT(
        res: *mut mjtNum,
        mat1: *const mjtNum,
        mat2: *const mjtNum,
        r1: ::core::ffi::c_int,
        c1: ::core::ffi::c_int,
        r2: ::core::ffi::c_int,
    );
    pub(crate) fn mju_mulMatTMat(
        res: *mut mjtNum,
        mat1: *const mjtNum,
        mat2: *const mjtNum,
        r1: ::core::ffi::c_int,
        c1: ::core::ffi::c_int,
        c2: ::core::ffi::c_int,
    );
    pub(crate) fn mju_sqrMatTD(
        res: *mut mjtNum,
        mat: *const mjtNum,
        diag: *const mjtNum,
        nr: ::core::ffi::c_int,
        nc: ::core::ffi::c_int,
    );
    pub(crate) fn mju_transformSpatial(
        res: *mut [mjtNum; 6usize],
        vec: *const [mjtNum; 6usize],
        flg_force: ::core::ffi::c_int,
        newpos: *const [mjtNum; 3usize],
        oldpos: *const [mjtNum; 3usize],
        rotnew2old: *const [mjtNum; 9usize],
    );
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
    pub(crate) fn mju_sparse2dense(
        res: *mut mjtNum,
        mat: *const mjtNum,
        nr: ::core::ffi::c_int,
        nc: ::core::ffi::c_int,
        rownnz: *const ::core::ffi::c_int,
        rowadr: *const ::core::ffi::c_int,
        colind: *const ::core::ffi::c_int,
    );
    pub(crate) fn mju_sym2dense(
        res: *mut mjtNum,
        mat: *const mjtNum,
        n: ::core::ffi::c_int,
        rownnz: *const ::core::ffi::c_int,
        rowadr: *const ::core::ffi::c_int,
        colind: *const ::core::ffi::c_int,
    );
    pub(crate) fn mju_rotVecQuat(
        res: *mut [mjtNum; 3usize],
        vec: *const [mjtNum; 3usize],
        quat: *const [mjtNum; 4usize],
    );
    pub(crate) fn mju_negQuat(res: *mut [mjtNum; 4usize], quat: *const [mjtNum; 4usize]);
    pub(crate) fn mju_mulQuat(
        res: *mut [mjtNum; 4usize],
        quat1: *const [mjtNum; 4usize],
        quat2: *const [mjtNum; 4usize],
    );
    pub(crate) fn mju_mulQuatAxis(
        res: *mut [mjtNum; 4usize],
        quat: *const [mjtNum; 4usize],
        axis: *const [mjtNum; 3usize],
    );
    pub(crate) fn mju_axisAngle2Quat(
        res: *mut [mjtNum; 4usize],
        axis: *const [mjtNum; 3usize],
        angle: mjtNum,
    );
    pub(crate) fn mju_quat2Vel(
        res: *mut [mjtNum; 3usize],
        quat: *const [mjtNum; 4usize],
        dt: mjtNum,
    );
    pub(crate) fn mju_subQuat(
        res: *mut [mjtNum; 3usize],
        qa: *const [mjtNum; 4usize],
        qb: *const [mjtNum; 4usize],
    );
    pub(crate) fn mju_quat2Mat(res: *mut [mjtNum; 9usize], quat: *const [mjtNum; 4usize]);
    pub(crate) fn mju_mat2Quat(quat: *mut [mjtNum; 4usize], mat: *const [mjtNum; 9usize]);
    pub(crate) fn mju_derivQuat(
        res: *mut [mjtNum; 4usize],
        quat: *const [mjtNum; 4usize],
        vel: *const [mjtNum; 3usize],
    );
    pub(crate) fn mju_quatIntegrate(
        quat: *mut [mjtNum; 4usize],
        vel: *const [mjtNum; 3usize],
        scale: mjtNum,
    );
    pub(crate) fn mju_quatZ2Vec(quat: *mut [mjtNum; 4usize], vec: *const [mjtNum; 3usize]);
    pub(crate) fn mju_mat2Rot(
        quat: *mut [mjtNum; 4usize],
        mat: *const [mjtNum; 9usize],
    ) -> ::core::ffi::c_int;
    pub(crate) fn mju_euler2Quat(
        quat: *mut [mjtNum; 4usize],
        euler: *const [mjtNum; 3usize],
        seq: *const ::core::ffi::c_char,
    );
    pub(crate) fn mju_mulPose(
        posres: *mut [mjtNum; 3usize],
        quatres: *mut [mjtNum; 4usize],
        pos1: *const [mjtNum; 3usize],
        quat1: *const [mjtNum; 4usize],
        pos2: *const [mjtNum; 3usize],
        quat2: *const [mjtNum; 4usize],
    );
    pub(crate) fn mju_negPose(
        posres: *mut [mjtNum; 3usize],
        quatres: *mut [mjtNum; 4usize],
        pos: *const [mjtNum; 3usize],
        quat: *const [mjtNum; 4usize],
    );
    pub(crate) fn mju_trnVecPose(
        res: *mut [mjtNum; 3usize],
        pos: *const [mjtNum; 3usize],
        quat: *const [mjtNum; 4usize],
        vec: *const [mjtNum; 3usize],
    );
    pub(crate) fn mju_cholFactor(
        mat: *mut mjtNum,
        n: ::core::ffi::c_int,
        mindiag: mjtNum,
    ) -> ::core::ffi::c_int;
    pub(crate) fn mju_cholSolve(
        res: *mut mjtNum,
        mat: *const mjtNum,
        vec: *const mjtNum,
        n: ::core::ffi::c_int,
    );
    pub(crate) fn mju_cholUpdate(
        mat: *mut mjtNum,
        x: *mut mjtNum,
        n: ::core::ffi::c_int,
        flg_plus: ::core::ffi::c_int,
    ) -> ::core::ffi::c_int;
    pub(crate) fn mju_cholFactorBand(
        mat: *mut mjtNum,
        ntotal: ::core::ffi::c_int,
        nband: ::core::ffi::c_int,
        ndense: ::core::ffi::c_int,
        diagadd: mjtNum,
        diagmul: mjtNum,
    ) -> mjtNum;
    pub(crate) fn mju_cholSolveBand(
        res: *mut mjtNum,
        mat: *const mjtNum,
        vec: *const mjtNum,
        ntotal: ::core::ffi::c_int,
        nband: ::core::ffi::c_int,
        ndense: ::core::ffi::c_int,
    );
    pub(crate) fn mju_band2Dense(
        res: *mut mjtNum,
        mat: *const mjtNum,
        ntotal: ::core::ffi::c_int,
        nband: ::core::ffi::c_int,
        ndense: ::core::ffi::c_int,
        flg_sym: mjtBool,
    );
    pub(crate) fn mju_dense2Band(
        res: *mut mjtNum,
        mat: *const mjtNum,
        ntotal: ::core::ffi::c_int,
        nband: ::core::ffi::c_int,
        ndense: ::core::ffi::c_int,
    );
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
    pub(crate) fn mju_bandDiag(
        i: ::core::ffi::c_int,
        ntotal: ::core::ffi::c_int,
        nband: ::core::ffi::c_int,
        ndense: ::core::ffi::c_int,
    ) -> ::core::ffi::c_int;
    pub(crate) fn mju_eig3(
        eigval: *mut [mjtNum; 3usize],
        eigvec: *mut [mjtNum; 9usize],
        quat: *mut [mjtNum; 4usize],
        mat: *const [mjtNum; 9usize],
    ) -> ::core::ffi::c_int;
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
    pub(crate) fn mju_muscleGain(
        len: mjtNum,
        vel: mjtNum,
        lengthrange: *const [mjtNum; 2usize],
        acc0: mjtNum,
        prm: *const [mjtNum; 9usize],
    ) -> mjtNum;
    pub(crate) fn mju_muscleBias(
        len: mjtNum,
        lengthrange: *const [mjtNum; 2usize],
        acc0: mjtNum,
        prm: *const [mjtNum; 9usize],
    ) -> mjtNum;
    pub(crate) fn mju_muscleDynamics(
        ctrl: mjtNum,
        act: mjtNum,
        prm: *const [mjtNum; 3usize],
    ) -> mjtNum;
    pub(crate) fn mju_encodePyramid(
        pyramid: *mut mjtNum,
        force: *const mjtNum,
        mu: *const mjtNum,
        dim: ::core::ffi::c_int,
    );
    pub(crate) fn mju_decodePyramid(
        force: *mut mjtNum,
        pyramid: *const mjtNum,
        mu: *const mjtNum,
        dim: ::core::ffi::c_int,
    );
    pub(crate) fn mju_springDamper(
        pos0: mjtNum,
        vel0: mjtNum,
        Kp: mjtNum,
        Kv: mjtNum,
        dt: mjtNum,
    ) -> mjtNum;
    pub(crate) fn mju_min(a: mjtNum, b: mjtNum) -> mjtNum;
    pub(crate) fn mju_max(a: mjtNum, b: mjtNum) -> mjtNum;
    pub(crate) fn mju_clip(x: mjtNum, min: mjtNum, max: mjtNum) -> mjtNum;
    pub(crate) fn mju_sign(x: mjtNum) -> mjtNum;
    pub(crate) fn mju_round(x: mjtNum) -> ::core::ffi::c_int;
    pub(crate) fn mju_type2Str(type_: ::core::ffi::c_int) -> *const ::core::ffi::c_char;
    pub(crate) fn mju_str2Type(str_: *const ::core::ffi::c_char) -> ::core::ffi::c_int;
    pub(crate) fn mju_writeNumBytes(nbytes: usize) -> *const ::core::ffi::c_char;
    pub(crate) fn mju_warningText(
        warning: ::core::ffi::c_int,
        info: usize,
    ) -> *const ::core::ffi::c_char;
    pub(crate) fn mju_isBad(x: mjtNum) -> ::core::ffi::c_int;
    pub(crate) fn mju_isZero(vec: *const mjtNum, n: ::core::ffi::c_int) -> ::core::ffi::c_int;
    pub(crate) fn mju_standardNormal(num2: *mut mjtNum) -> mjtNum;
    pub(crate) fn mju_f2n(res: *mut mjtNum, vec: *const f32, n: ::core::ffi::c_int);
    pub(crate) fn mju_n2f(res: *mut f32, vec: *const mjtNum, n: ::core::ffi::c_int);
    pub(crate) fn mju_d2n(res: *mut mjtNum, vec: *const f64, n: ::core::ffi::c_int);
    pub(crate) fn mju_n2d(res: *mut f64, vec: *const mjtNum, n: ::core::ffi::c_int);
    pub(crate) fn mju_insertionSort(list: *mut mjtNum, n: ::core::ffi::c_int);
    pub(crate) fn mju_insertionSortInt(list: *mut ::core::ffi::c_int, n: ::core::ffi::c_int);
    pub(crate) fn mju_Halton(index: ::core::ffi::c_int, base: ::core::ffi::c_int) -> mjtNum;
    pub(crate) fn mju_strncpy(
        dst: *mut ::core::ffi::c_char,
        src: *const ::core::ffi::c_char,
        n: ::core::ffi::c_int,
    ) -> *mut ::core::ffi::c_char;
    pub(crate) fn mju_sigmoid(x: mjtNum) -> mjtNum;
    pub(crate) fn mjc_getSDF(m: *const mjModel, id: ::core::ffi::c_int) -> *const mjpPlugin;
    pub(crate) fn mjc_distance(
        m: *const mjModel,
        d: *const mjData,
        s: *const mjSDF,
        x: *const [mjtNum; 3usize],
    ) -> mjtNum;
    pub(crate) fn mjc_gradient(
        m: *const mjModel,
        d: *const mjData,
        s: *const mjSDF,
        gradient: *mut [mjtNum; 3usize],
        x: *const [mjtNum; 3usize],
    );
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
    pub(crate) fn mjd_subQuat(
        qa: *const [mjtNum; 4usize],
        qb: *const [mjtNum; 4usize],
        Da: *mut [mjtNum; 9usize],
        Db: *mut [mjtNum; 9usize],
    );
    pub(crate) fn mjd_quatIntegrate(
        vel: *const [mjtNum; 3usize],
        scale: mjtNum,
        Dquat: *mut [mjtNum; 9usize],
        Dvel: *mut [mjtNum; 9usize],
        Dscale: *mut [mjtNum; 3usize],
    );
    pub(crate) fn mjp_defaultPlugin(plugin: *mut mjpPlugin);
    pub(crate) fn mjp_registerPlugin(plugin: *const mjpPlugin) -> ::core::ffi::c_int;
    pub(crate) fn mjp_pluginCount() -> ::core::ffi::c_int;
    pub(crate) fn mjp_getPlugin(
        name: *const ::core::ffi::c_char,
        slot: *mut ::core::ffi::c_int,
    ) -> *const mjpPlugin;
    pub(crate) fn mjp_getPluginAtSlot(slot: ::core::ffi::c_int) -> *const mjpPlugin;
    pub(crate) fn mjp_defaultResourceProvider(provider: *mut mjpResourceProvider);
    pub(crate) fn mjp_registerResourceProvider(
        provider: *const mjpResourceProvider,
    ) -> ::core::ffi::c_int;
    pub(crate) fn mjp_resourceProviderCount() -> ::core::ffi::c_int;
    pub(crate) fn mjp_getResourceProvider(
        resource_name: *const ::core::ffi::c_char,
    ) -> *const mjpResourceProvider;
    pub(crate) fn mjp_getResourceProviderAtSlot(
        slot: ::core::ffi::c_int,
    ) -> *const mjpResourceProvider;
    pub(crate) fn mjp_registerDecoder(decoder: *const mjpDecoder);
    pub(crate) fn mjp_defaultDecoder(decoder: *mut mjpDecoder);
    pub(crate) fn mjp_findDecoder(
        resource: *const mjResource,
        content_type: *const ::core::ffi::c_char,
    ) -> *const mjpDecoder;
    pub(crate) fn mjp_registerEncoder(encoder: *const mjpEncoder);
    pub(crate) fn mjp_defaultEncoder(encoder: *mut mjpEncoder);
    pub(crate) fn mjp_findEncoder(
        filename: *const ::core::ffi::c_char,
        content_type: *const ::core::ffi::c_char,
    ) -> *const mjpEncoder;
    pub(crate) fn mju_openResource(
        dir: *const ::core::ffi::c_char,
        name: *const ::core::ffi::c_char,
        vfs: *const mjVFS,
        error: *mut ::core::ffi::c_char,
        nerror: usize,
    ) -> *mut mjResource;
    pub(crate) fn mju_closeResource(resource: *mut mjResource);
    pub(crate) fn mju_readResource(
        resource: *mut mjResource,
        buffer: *mut *const ::core::ffi::c_void,
    ) -> ::core::ffi::c_int;
    pub(crate) fn mju_writeResource(
        name: *const ::core::ffi::c_char,
        buffer: *const ::core::ffi::c_void,
        nbytes: mjtSize,
        vfs: *const mjVFS,
        error: *mut ::core::ffi::c_char,
        nerror: usize,
    ) -> mjtSize;
    pub(crate) fn mju_getResourceDir(
        resource: *mut mjResource,
        dir: *mut *const ::core::ffi::c_char,
        ndir: *mut ::core::ffi::c_int,
    );
    pub(crate) fn mju_isModifiedResource(
        resource: *const mjResource,
        timestamp: *const ::core::ffi::c_char,
    ) -> ::core::ffi::c_int;
    pub(crate) fn mju_decodeResource(
        resource: *mut mjResource,
        content_type: *const ::core::ffi::c_char,
        vfs: *const mjVFS,
    ) -> *mut mjSpec;
    pub(crate) fn mju_threadpool(d: *mut mjData, nthread: ::core::ffi::c_int);
    pub(crate) fn mjs_attach(
        parent: *mut mjsElement,
        child: *const mjsElement,
        prefix: *const ::core::ffi::c_char,
        suffix: *const ::core::ffi::c_char,
    ) -> *mut mjsElement;
    pub(crate) fn mjs_addBody(body: *mut mjsBody, def: *const mjsDefault) -> *mut mjsBody;
    pub(crate) fn mjs_addSite(body: *mut mjsBody, def: *const mjsDefault) -> *mut mjsSite;
    pub(crate) fn mjs_addJoint(body: *mut mjsBody, def: *const mjsDefault) -> *mut mjsJoint;
    pub(crate) fn mjs_addFreeJoint(body: *mut mjsBody) -> *mut mjsJoint;
    pub(crate) fn mjs_addGeom(body: *mut mjsBody, def: *const mjsDefault) -> *mut mjsGeom;
    pub(crate) fn mjs_addCamera(body: *mut mjsBody, def: *const mjsDefault) -> *mut mjsCamera;
    pub(crate) fn mjs_addLight(body: *mut mjsBody, def: *const mjsDefault) -> *mut mjsLight;
    pub(crate) fn mjs_addFrame(body: *mut mjsBody, parentframe: *mut mjsFrame) -> *mut mjsFrame;
    pub(crate) fn mjs_delete(spec: *mut mjSpec, element: *mut mjsElement) -> ::core::ffi::c_int;
    pub(crate) fn mjs_addActuator(s: *mut mjSpec, def: *const mjsDefault) -> *mut mjsActuator;
    pub(crate) fn mjs_addSensor(s: *mut mjSpec) -> *mut mjsSensor;
    pub(crate) fn mjs_addFlex(s: *mut mjSpec) -> *mut mjsFlex;
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
    pub(crate) fn mjs_addPair(s: *mut mjSpec, def: *const mjsDefault) -> *mut mjsPair;
    pub(crate) fn mjs_addExclude(s: *mut mjSpec) -> *mut mjsExclude;
    pub(crate) fn mjs_addEquality(s: *mut mjSpec, def: *const mjsDefault) -> *mut mjsEquality;
    pub(crate) fn mjs_addTendon(s: *mut mjSpec, def: *const mjsDefault) -> *mut mjsTendon;
    pub(crate) fn mjs_wrapSite(
        tendon: *mut mjsTendon,
        name: *const ::core::ffi::c_char,
    ) -> *mut mjsWrap;
    pub(crate) fn mjs_wrapGeom(
        tendon: *mut mjsTendon,
        name: *const ::core::ffi::c_char,
        sidesite: *const ::core::ffi::c_char,
    ) -> *mut mjsWrap;
    pub(crate) fn mjs_wrapJoint(
        tendon: *mut mjsTendon,
        name: *const ::core::ffi::c_char,
        coef: f64,
    ) -> *mut mjsWrap;
    pub(crate) fn mjs_wrapPulley(tendon: *mut mjsTendon, divisor: f64) -> *mut mjsWrap;
    pub(crate) fn mjs_addNumeric(s: *mut mjSpec) -> *mut mjsNumeric;
    pub(crate) fn mjs_addText(s: *mut mjSpec) -> *mut mjsText;
    pub(crate) fn mjs_addTuple(s: *mut mjSpec) -> *mut mjsTuple;
    pub(crate) fn mjs_addKey(s: *mut mjSpec) -> *mut mjsKey;
    pub(crate) fn mjs_addPlugin(s: *mut mjSpec) -> *mut mjsPlugin;
    pub(crate) fn mjs_addDefault(
        s: *mut mjSpec,
        classname: *const ::core::ffi::c_char,
        parent: *const mjsDefault,
    ) -> *mut mjsDefault;
    pub(crate) fn mjs_setToMotor(actuator: *mut mjsActuator) -> *const ::core::ffi::c_char;
    pub(crate) fn mjs_setToPosition(
        actuator: *mut mjsActuator,
        kp: f64,
        kv: *mut [f64; 1usize],
        dampratio: *mut [f64; 1usize],
        timeconst: *mut [f64; 1usize],
        inheritrange: f64,
    ) -> *const ::core::ffi::c_char;
    pub(crate) fn mjs_setToIntVelocity(
        actuator: *mut mjsActuator,
        kp: f64,
        kv: *mut [f64; 1usize],
        dampratio: *mut [f64; 1usize],
        timeconst: *mut [f64; 1usize],
        inheritrange: f64,
    ) -> *const ::core::ffi::c_char;
    pub(crate) fn mjs_setToVelocity(
        actuator: *mut mjsActuator,
        kv: f64,
    ) -> *const ::core::ffi::c_char;
    pub(crate) fn mjs_setToOrientation(
        actuator: *mut mjsActuator,
        kp: f64,
        kv: *mut [f64; 1usize],
        dampratio: *mut [f64; 1usize],
        ctrlspec: ::core::ffi::c_int,
    ) -> *const ::core::ffi::c_char;
    pub(crate) fn mjs_setToDamper(
        actuator: *mut mjsActuator,
        kv: f64,
    ) -> *const ::core::ffi::c_char;
    pub(crate) fn mjs_setToCylinder(
        actuator: *mut mjsActuator,
        timeconst: f64,
        bias: f64,
        area: f64,
        diameter: f64,
    ) -> *const ::core::ffi::c_char;
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
    pub(crate) fn mjs_setToAdhesion(
        actuator: *mut mjsActuator,
        gain: f64,
    ) -> *const ::core::ffi::c_char;
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
    pub(crate) fn mjs_addMesh(s: *mut mjSpec, def: *const mjsDefault) -> *mut mjsMesh;
    pub(crate) fn mjs_addHField(s: *mut mjSpec) -> *mut mjsHField;
    pub(crate) fn mjs_addSkin(s: *mut mjSpec) -> *mut mjsSkin;
    pub(crate) fn mjs_addTexture(s: *mut mjSpec) -> *mut mjsTexture;
    pub(crate) fn mjs_addMaterial(s: *mut mjSpec, def: *const mjsDefault) -> *mut mjsMaterial;
    pub(crate) fn mjs_makeMesh(
        mesh: *mut mjsMesh,
        builtin: mjtMeshBuiltin,
        params: *mut f64,
        nparams: ::core::ffi::c_int,
    ) -> ::core::ffi::c_int;
    pub(crate) fn mjs_getSpec(element: *const mjsElement) -> *mut mjSpec;
    pub(crate) fn mjs_getOriginSpec(element: *const mjsElement) -> *mut mjSpec;
    pub(crate) fn mjs_getCompiler(element: *const mjsElement) -> *mut mjsCompiler;
    pub(crate) fn mjs_findSpec(
        spec: *const mjSpec,
        name: *const ::core::ffi::c_char,
    ) -> *mut mjSpec;
    pub(crate) fn mjs_findBody(s: *const mjSpec, name: *const ::core::ffi::c_char) -> *mut mjsBody;
    pub(crate) fn mjs_findElement(
        s: *const mjSpec,
        type_: mjtObj,
        name: *const ::core::ffi::c_char,
    ) -> *mut mjsElement;
    pub(crate) fn mjs_findChild(
        body: *const mjsBody,
        name: *const ::core::ffi::c_char,
    ) -> *mut mjsBody;
    pub(crate) fn mjs_getParent(element: *const mjsElement) -> *mut mjsBody;
    pub(crate) fn mjs_getFrame(element: *const mjsElement) -> *mut mjsFrame;
    pub(crate) fn mjs_findFrame(
        s: *const mjSpec,
        name: *const ::core::ffi::c_char,
    ) -> *mut mjsFrame;
    pub(crate) fn mjs_getDefault(element: *const mjsElement) -> *mut mjsDefault;
    pub(crate) fn mjs_findDefault(
        s: *const mjSpec,
        classname: *const ::core::ffi::c_char,
    ) -> *mut mjsDefault;
    pub(crate) fn mjs_getSpecDefault(s: *const mjSpec) -> *mut mjsDefault;
    pub(crate) fn mjs_getId(element: *const mjsElement) -> ::core::ffi::c_int;
    pub(crate) fn mjs_firstChild(
        body: *const mjsBody,
        type_: mjtObj,
        recurse: ::core::ffi::c_int,
    ) -> *mut mjsElement;
    pub(crate) fn mjs_nextChild(
        body: *const mjsBody,
        child: *const mjsElement,
        recurse: ::core::ffi::c_int,
    ) -> *mut mjsElement;
    pub(crate) fn mjs_firstElement(s: *const mjSpec, type_: mjtObj) -> *mut mjsElement;
    pub(crate) fn mjs_nextElement(s: *const mjSpec, element: *const mjsElement) -> *mut mjsElement;
    pub(crate) fn mjs_getWrapTarget(wrap: *const mjsWrap) -> *mut mjsElement;
    pub(crate) fn mjs_getWrapSideSite(wrap: *const mjsWrap) -> *mut mjsSite;
    pub(crate) fn mjs_getWrapDivisor(wrap: *const mjsWrap) -> f64;
    pub(crate) fn mjs_getWrapCoef(wrap: *const mjsWrap) -> f64;
    pub(crate) fn mjs_setName(
        element: *mut mjsElement,
        name: *const ::core::ffi::c_char,
    ) -> ::core::ffi::c_int;
    pub(crate) fn mjs_setBuffer(
        dest: *mut mjByteVec,
        array: *const ::core::ffi::c_void,
        size: ::core::ffi::c_int,
    );
    pub(crate) fn mjs_setString(dest: *mut mjString, text: *const ::core::ffi::c_char);
    pub(crate) fn mjs_setStringVec(dest: *mut mjStringVec, text: *const ::core::ffi::c_char);
    pub(crate) fn mjs_setInStringVec(
        dest: *mut mjStringVec,
        i: ::core::ffi::c_int,
        text: *const ::core::ffi::c_char,
    ) -> mjtBool;
    pub(crate) fn mjs_appendString(dest: *mut mjStringVec, text: *const ::core::ffi::c_char);
    pub(crate) fn mjs_setInt(
        dest: *mut mjIntVec,
        array: *const ::core::ffi::c_int,
        size: ::core::ffi::c_int,
    );
    pub(crate) fn mjs_appendIntVec(
        dest: *mut mjIntVecVec,
        array: *const ::core::ffi::c_int,
        size: ::core::ffi::c_int,
    );
    pub(crate) fn mjs_setFloat(dest: *mut mjFloatVec, array: *const f32, size: ::core::ffi::c_int);
    pub(crate) fn mjs_appendFloatVec(
        dest: *mut mjFloatVecVec,
        array: *const f32,
        size: ::core::ffi::c_int,
    );
    pub(crate) fn mjs_setDouble(
        dest: *mut mjDoubleVec,
        array: *const f64,
        size: ::core::ffi::c_int,
    );
    pub(crate) fn mjs_setPluginAttributes(
        plugin: *mut mjsPlugin,
        attributes: *mut ::core::ffi::c_void,
    );
    pub(crate) fn mjs_getName(element: *mut mjsElement) -> *mut mjString;
    pub(crate) fn mjs_getString(source: *const mjString) -> *const ::core::ffi::c_char;
    pub(crate) fn mjs_getDouble(
        source: *const mjDoubleVec,
        size: *mut ::core::ffi::c_int,
    ) -> *const f64;
    pub(crate) fn mjs_getWrapNum(tendonspec: *const mjsTendon) -> ::core::ffi::c_int;
    pub(crate) fn mjs_getWrap(tendonspec: *const mjsTendon, i: ::core::ffi::c_int) -> *mut mjsWrap;
    pub(crate) fn mjs_getPluginAttributes(plugin: *const mjsPlugin) -> *const ::core::ffi::c_void;
    pub(crate) fn mjs_setDefault(element: *mut mjsElement, def: *const mjsDefault);
    pub(crate) fn mjs_setFrame(dest: *mut mjsElement, frame: *mut mjsFrame) -> ::core::ffi::c_int;
    pub(crate) fn mjs_resolveOrientation(
        quat: *mut [f64; 4usize],
        degree: mjtByte,
        sequence: *const ::core::ffi::c_char,
        orientation: *const mjsOrientation,
    ) -> *const ::core::ffi::c_char;
    pub(crate) fn mjs_bodyToFrame(body: *mut *mut mjsBody) -> *mut mjsFrame;
    pub(crate) fn mjs_setUserValue(
        element: *mut mjsElement,
        key: *const ::core::ffi::c_char,
        data: *const ::core::ffi::c_void,
    );
    pub(crate) fn mjs_setUserValueWithCleanup(
        element: *mut mjsElement,
        key: *const ::core::ffi::c_char,
        data: *const ::core::ffi::c_void,
        cleanup: ::core::option::Option<unsafe extern "C" fn(arg1: *const ::core::ffi::c_void)>,
    );
    pub(crate) fn mjs_getUserValue(
        element: *mut mjsElement,
        key: *const ::core::ffi::c_char,
    ) -> *const ::core::ffi::c_void;
    pub(crate) fn mjs_deleteUserValue(element: *mut mjsElement, key: *const ::core::ffi::c_char);
    pub(crate) fn mjs_sensorDim(sensor: *const mjsSensor) -> ::core::ffi::c_int;
    pub(crate) fn mjs_defaultSpec(spec: *mut mjSpec);
    pub(crate) fn mjs_defaultOrientation(orient: *mut mjsOrientation);
    pub(crate) fn mjs_defaultBody(body: *mut mjsBody);
    pub(crate) fn mjs_defaultFrame(frame: *mut mjsFrame);
    pub(crate) fn mjs_defaultJoint(joint: *mut mjsJoint);
    pub(crate) fn mjs_defaultGeom(geom: *mut mjsGeom);
    pub(crate) fn mjs_defaultSite(site: *mut mjsSite);
    pub(crate) fn mjs_defaultCamera(camera: *mut mjsCamera);
    pub(crate) fn mjs_defaultLight(light: *mut mjsLight);
    pub(crate) fn mjs_defaultFlex(flex: *mut mjsFlex);
    pub(crate) fn mjs_defaultMesh(mesh: *mut mjsMesh);
    pub(crate) fn mjs_defaultHField(hfield: *mut mjsHField);
    pub(crate) fn mjs_defaultSkin(skin: *mut mjsSkin);
    pub(crate) fn mjs_defaultTexture(texture: *mut mjsTexture);
    pub(crate) fn mjs_defaultMaterial(material: *mut mjsMaterial);
    pub(crate) fn mjs_defaultPair(pair: *mut mjsPair);
    pub(crate) fn mjs_defaultEquality(equality: *mut mjsEquality);
    pub(crate) fn mjs_defaultTendon(tendon: *mut mjsTendon);
    pub(crate) fn mjs_defaultActuator(actuator: *mut mjsActuator);
    pub(crate) fn mjs_defaultSensor(sensor: *mut mjsSensor);
    pub(crate) fn mjs_defaultNumeric(numeric: *mut mjsNumeric);
    pub(crate) fn mjs_defaultText(text: *mut mjsText);
    pub(crate) fn mjs_defaultTuple(tuple: *mut mjsTuple);
    pub(crate) fn mjs_defaultKey(key: *mut mjsKey);
    pub(crate) fn mjs_defaultPlugin(plugin: *mut mjsPlugin);
    pub(crate) fn mjs_asBody(element: *mut mjsElement) -> *mut mjsBody;
    pub(crate) fn mjs_asGeom(element: *mut mjsElement) -> *mut mjsGeom;
    pub(crate) fn mjs_asJoint(element: *mut mjsElement) -> *mut mjsJoint;
    pub(crate) fn mjs_asSite(element: *mut mjsElement) -> *mut mjsSite;
    pub(crate) fn mjs_asCamera(element: *mut mjsElement) -> *mut mjsCamera;
    pub(crate) fn mjs_asLight(element: *mut mjsElement) -> *mut mjsLight;
    pub(crate) fn mjs_asFrame(element: *mut mjsElement) -> *mut mjsFrame;
    pub(crate) fn mjs_asActuator(element: *mut mjsElement) -> *mut mjsActuator;
    pub(crate) fn mjs_asSensor(element: *mut mjsElement) -> *mut mjsSensor;
    pub(crate) fn mjs_asFlex(element: *mut mjsElement) -> *mut mjsFlex;
    pub(crate) fn mjs_asPair(element: *mut mjsElement) -> *mut mjsPair;
    pub(crate) fn mjs_asEquality(element: *mut mjsElement) -> *mut mjsEquality;
    pub(crate) fn mjs_asExclude(element: *mut mjsElement) -> *mut mjsExclude;
    pub(crate) fn mjs_asTendon(element: *mut mjsElement) -> *mut mjsTendon;
    pub(crate) fn mjs_asNumeric(element: *mut mjsElement) -> *mut mjsNumeric;
    pub(crate) fn mjs_asText(element: *mut mjsElement) -> *mut mjsText;
    pub(crate) fn mjs_asTuple(element: *mut mjsElement) -> *mut mjsTuple;
    pub(crate) fn mjs_asKey(element: *mut mjsElement) -> *mut mjsKey;
    pub(crate) fn mjs_asMesh(element: *mut mjsElement) -> *mut mjsMesh;
    pub(crate) fn mjs_asHField(element: *mut mjsElement) -> *mut mjsHField;
    pub(crate) fn mjs_asSkin(element: *mut mjsElement) -> *mut mjsSkin;
    pub(crate) fn mjs_asTexture(element: *mut mjsElement) -> *mut mjsTexture;
    pub(crate) fn mjs_asMaterial(element: *mut mjsElement) -> *mut mjsMaterial;
    pub(crate) fn mjs_asPlugin(element: *mut mjsElement) -> *mut mjsPlugin;
}
