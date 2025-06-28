//! # [Model Editing](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#model-editing)
//! 
//! The structs below are defined in mjspec.h and, with the exception of
//! the top level [`mjSpec`] struct, begin with the `mjs` prefix.
//! For more details, see the [Model Editing](https://mujoco.readthedocs.io/en/stable/programming/modeledit.html) chapter.

pub use crate::bindgen::{mjSpec, mjsElement, mjsCompiler};

use crate::bindgen::{mjOption, mjVisual, mjStatistic, mjString, mjLROpt};

derive_fields_mapping!(mjSpec {});

/*
typedef struct mjSpec_ {           // model specification
  mjsElement* element;             // element type
  mjString* modelname;             // model name

  // compiler data
  mjsCompiler compiler;            // compiler options
  mjtByte strippath;               // automatically strip paths from mesh files
  mjString* meshdir;               // mesh and hfield directory
  mjString* texturedir;            // texture directory

  // engine data
  mjOption option;                 // physics options
  mjVisual visual;                 // visual options
  mjStatistic stat;                // statistics override (if defined)

  // sizes
  size_t memory;                   // number of bytes in arena+stack memory
  int nemax;                       // max number of equality constraints
  int nuserdata;                   // number of mjtNums in userdata
  int nuser_body;                  // number of mjtNums in body_user
  int nuser_jnt;                   // number of mjtNums in jnt_user
  int nuser_geom;                  // number of mjtNums in geom_user
  int nuser_site;                  // number of mjtNums in site_user
  int nuser_cam;                   // number of mjtNums in cam_user
  int nuser_tendon;                // number of mjtNums in tendon_user
  int nuser_actuator;              // number of mjtNums in actuator_user
  int nuser_sensor;                // number of mjtNums in sensor_user
  int nkey;                        // number of keyframes
  int njmax;                       // (deprecated) max number of constraints
  int nconmax;                     // (deprecated) max number of detected contacts
  size_t nstack;                   // (deprecated) number of mjtNums in mjData stack

  // global data
  mjString* comment;               // comment at top of XML
  mjString* modelfiledir;          // path to model file

  // other
  mjtByte hasImplicitPluginElem;   // already encountered an implicit plugin sensor/actuator
} mjSpec;
*/
/*
typedef struct mjsElement_ {       // element type, do not modify
  mjtObj elemtype;                 // element type
  uint64_t signature;              // compilation signature
} mjsElement;
*/
impl MjsElement {
    pub fn elemtype(&self) -> crate::bindgen::mjtObj {
        self.0.elemtype
    }

    pub fn signature(&self) -> u64 {
        self.0.signature
    }
}

/*
typedef struct mjsCompiler_ {      // compiler options
  mjtByte autolimits;              // infer "limited" attribute based on range
  double boundmass;                // enforce minimum body mass
  double boundinertia;             // enforce minimum body diagonal inertia
  double settotalmass;             // rescale masses and inertias; <=0: ignore
  mjtByte balanceinertia;          // automatically impose A + B >= C rule
  mjtByte fitaabb;                 // meshfit to aabb instead of inertia box
  mjtByte degree;                  // angles in radians or degrees
  char eulerseq[3];                // sequence for euler rotations
  mjtByte discardvisual;           // discard visual geoms in parser
  mjtByte usethread;               // use multiple threads to speed up compiler
  mjtByte fusestatic;              // fuse static bodies with parent
  int inertiafromgeom;             // use geom inertias (mjtInertiaFromGeom)
  int inertiagrouprange[2];        // range of geom groups used to compute inertia
  mjtByte saveinertial;            // save explicit inertial clause for all bodies to XML
  int alignfree;                   // align free joints with inertial frame
  mjLROpt LRopt;                   // options for lengthrange computation
} mjsCompiler;
*/

// boolean (mjtByte fields and `int alignfree`)
macro_rules! booleans {
    ($($name:ident / $set_name:ident = $description:literal;)*) => {
        impl MjsCompiler {
            $(
                #[allow(non_snake_case)]
                #[doc = "if or not "]
                #[doc = $description]
                pub fn $name(&self) -> bool {
                    self.0.$name != 0
                }

                #[allow(non_snake_case)]
                #[doc = "set if or not "]
                #[doc = $description]
                pub fn $set_name(&mut self, yes: bool) {
                    self.0.$name = yes as _;
                }
            )*
        }
    };
}
booleans! {
    autolimits / set_autolimits         = "infer `limited` attribute based on range";
    balanceinertia / set_balanceinertia = "automatically impose A + B >= C rule";
    fitaabb / set_fitaabb               = "meshfit to aabb instead of inertia box";
    degree / set_degree                 = "angles in radians or degrees";
    discardvisual / set_discardvisual   = "discard visual geoms in parser";
    usethread / set_usethread           = "use multiple threads to speed up compiler";
    fusestatic / set_fusestatic         = "fuse static bodies with parent";
    saveinertial / set_saveinertial     = "save explicit inertial clause for all bodies to XML";
    alignfree / set_alignfree           = "align free joints with inertial frame (default: `false`)";
}

// double
/*
  double boundmass;                // enforce minimum body mass
  double boundinertia;             // enforce minimum body diagonal inertia
  double settotalmass;             // rescale masses and inertias; <=0: ignore
*/
impl MjsCompiler {
    /// enforce minimum body mass
    pub fn boundmass(&self) -> f64 {self.0.boundmass}
    /// enforce minimum body mass
    pub fn set_boundmass(&mut self, value: f64) {self.0.boundmass = value}

    /// enforce minimum body mass
    pub fn boundinertia(&self) -> f64 {self.0.boundinertia}
    /// enforce minimum body mass
    pub fn set_boundinertia(&mut self, value: f64) {self.0.boundinertia = value}

    /// enforce minimum body mass
    pub fn settotalmass(&self) -> f64 {self.0.settotalmass}
    /// enforce minimum body mass
    pub fn set_settotalmass(&mut self, value: f64) {self.0.settotalmass = value}
}

// char
/*
  char eulerseq[3];                // sequence for euler rotations
*/
impl MjsCompiler {
    /// sequence for euler rotations e.g. `['x', 'y', 'z']`
    pub fn eulerseq(&self) -> [char; 3] {
        self.0.eulerseq.map(|i8| char::from(u8::try_from(i8).expect("non-ASCII `eulerseq` of MjsCompiler")))
    }
    /// set sequence for euler rotations e.g. `['x', 'y', 'z']`
    /// 
    /// expected to be an array of ASCII charactor.
    pub fn set_eulerseq(&mut self, seq: [char; 3]) {
        self.0.eulerseq = seq.map(|char| char.is_ascii().then(|| char as i8).expect("non-ASCII `eulerseq` of MjsCompiler"))
    }
}

// enum / id
/*
  int inertiafromgeom;             // use geom inertias (mjtInertiaFromGeom)
  int inertiagrouprange[2];        // range of geom groups used to compute inertia
*/
impl MjsCompiler {
    /// use geom inertias by `mjtInertiaFromGeom`
    pub fn inertiafromgeom(&self) -> crate::bindgen::mjtInertiaFromGeom {
        crate::bindgen::mjtInertiaFromGeom(self.0.inertiafromgeom as u32)
    }
    /// set use geom inertias by `mjtInertiaFromGeom`
    pub fn set_inertiafromgeom(&mut self, inertia_from_geom: crate::bindgen::mjtInertiaFromGeom) {
        self.0.inertiafromgeom = inertia_from_geom.0 as i32;
    }

    /// range of geom group ids used to compute inertia (default: `0..(bindgen::mjNGROUP as usize)`).
    pub fn inertiagrouprange(&self) -> std::ops::Range<usize> {
        let [min, max] = self.0.inertiagrouprange;
        (min as usize)..(1 + max as usize)
    }
    /// set range of geom group ids used to compute inertia (default: `0..(bindgen::mjNGROUP as usize)`).
    pub fn set_inertiagrouprange(&mut self, range: std::ops::Range<usize>) {
        self.0.inertiagrouprange = [range.start as i32, range.end as i32 - 1];
    }
}
