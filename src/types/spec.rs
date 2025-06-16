use super::{MjOption, MjStatistic, MjVisual};
use crate::{MjString};

/// Model specification
pub struct MjSpec(pub(crate) crate::bindgen::mjSpec);

/// Special type corresponding to any element.
/// This struct is the first member of all other elements;
/// in the low-level C++ implementation, it is not included as a member
/// but via class inheritance. Inclusion via inheritance allows
/// the compiler to static_cast an mjsElement to the correct C++ object class.
/// Unlike all other attributes of the structs below, which are user-settable
/// by design, modifying the contents of an mjsElement is not allowed
/// and leads to undefined behavior.
pub struct MjsElement(pub(crate) crate::bindgen::mjsElement);

/// Compiler options.
pub struct MjsCompiler(pub(crate) crate::bindgen::mjsCompiler);

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

typedef struct mjsElement_ {       // element type, do not modify
  mjtObj elemtype;                 // element type
  uint64_t signature;              // compilation signature
} mjsElement;

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

impl MjSpec {
    /* SAFETY: each `transmute` is just between a **newtype** struct and its inner type. */

    pub fn element(&self) -> &MjsElement {
        let inner_ref = unsafe { &*self.0.element };
        unsafe { std::mem::transmute(inner_ref) }
    }
    pub fn element_mut(&mut self) -> &mut MjsElement {
        let inner_ref = unsafe { &mut *self.0.element };
        unsafe { std::mem::transmute(inner_ref) }
    }

    pub fn modelname(&self) -> &MjString {
        let inner_ref = unsafe { &*self.0.modelname };
        unsafe { std::mem::transmute(inner_ref) }
    }
    pub fn modelname_mut(&mut self) -> &mut MjString {
        let inner_ref = unsafe { &mut *self.0.modelname };
        unsafe { std::mem::transmute(inner_ref) }
    }
}

macro_rules! sizes {
    ($($name:ident / $set_name:ident = $description:literal;)*) => {
        impl MjSpec {
            $(
                #[allow(non_snake_case)]
                #[doc = $description]
                pub fn $name(&self) -> usize {
                    self.0.$name as _
                }

                #[allow(non_snake_case)]
                #[doc = "set "]
                #[doc = $description]
                pub fn $set_name(&mut self, value: usize) {
                    self.0.$name = value as _;
                }
            )*
        }
    };
}
sizes! {
  memory / set_memory                 = "number of bytes in arena+stack memory";
  nemax / set_nemax                   = "max number of equality constraints";
  nuserdata / set_nuserdata           = "number of mjtNums in userdata";
  nuser_body / set_nuser_body         = "number of mjtNums in body_user";
  nuser_jnt / set_nuser_jnt           = "number of mjtNums in jnt_user";
  nuser_geom / set_nuser_geom         = "number of mjtNums in geom_user";
  nuser_site / set_nuser_site         = "number of mjtNums in site_user";
  nuser_cam / set_nuser_cam           = "number of mjtNums in cam_user";
  nuser_tendon / set_nuser_tendon     = "number of mjtNums in tendon_user";
  nuser_actuator / set_nuser_actuator = "number of mjtNums in actuator_user";
  nuser_sensor / set_nuser_sensor     = "number of mjtNums in sensor_user";
  nkey / set_nkey                     = "number of keyframes";
}
