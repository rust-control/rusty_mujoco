use super::{MjOption, MjStatistic, MjVisual};
use crate::{MjString};

newtype! {
    /// Model specification
    MjSpec of crate::bindgen::mjSpec
}

newtype! {
    /// Special type corresponding to any element.
    /// This struct is the first member of all other elements;
    /// in the low-level C++ implementation, it is not included as a member
    /// but via class inheritance. Inclusion via inheritance allows
    /// the compiler to static_cast an mjsElement to the correct C++ object class.
    /// Unlike all other attributes of the structs below, which are user-settable
    /// by design, modifying the contents of an mjsElement is not allowed
    /// and leads to undefined behavior.
    MjsElement of crate::bindgen::mjsElement
}

newtype! {
    /// Compiler options.
    MjsCompiler of crate::bindgen::mjsCompiler
}

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
  mjsElement* element;             // element type
  mjString* modelname;             // model name
*/
impl MjSpec {
    /// element type, do not modify
    pub fn element(&self) -> &MjsElement {
        (unsafe { &*self.0.element }).into()
    }

    /// model name
    pub fn modelname(&self) -> &MjString {
        (unsafe { &*self.0.modelname }).into()
    }
    /// mutable model name
    pub fn modelname_mut(&mut self) -> &mut MjString {
        (unsafe { &mut *self.0.modelname }).into()
    }
}

// compiler data
/*
  mjsCompiler compiler;            // compiler options
  mjtByte strippath;               // automatically strip paths from mesh files
  mjString* meshdir;               // mesh and hfield directory
  mjString* texturedir;            // texture directory
*/
impl MjSpec {
    /// compiler options
    pub fn compiler(&self) -> &MjsCompiler {
        (&self.0.compiler).into()
    }
    /// mutable compiler options
    pub fn compiler_mut(&mut self) -> &mut MjsCompiler {
        (&mut self.0.compiler).into()
    }

    /// if or not automatically strip paths from mesh files
    pub fn strippath(&self) -> bool {
        self.0.strippath != 0
    }
    /// if or not automatically strip paths from mesh files
    pub fn set_strippath(&mut self, yes: bool) {
        self.0.strippath = yes as u8;
    }

    /// mesh and hfield directory
    pub fn meshdir(&self) -> &MjString {
        (unsafe { &*self.0.meshdir }).into()
    }
    /// mutable mesh and hfield directory
    pub fn meshdir_mut(&mut self) -> &mut MjString {
        (unsafe { &mut *self.0.meshdir }).into()
    }

    /// texture directory
    pub fn texturedir(&self) -> &MjString {
        (unsafe { &*self.0.texturedir }).into()
    }
    /// mutable texture directory
    pub fn texturedir_mut(&mut self) -> &mut MjString {
        (unsafe { &mut *self.0.texturedir }).into()
    }
}

// engine data
/*
  mjOption option;                 // physics options
  mjVisual visual;                 // visual options
  mjStatistic stat;                // statistics override (if defined)
*/
impl MjSpec {
    /// physics options
    pub fn option(&self) -> &MjOption {
        (&self.0.option).into()
    }
    /// mutable physics options
    pub fn option_mut(&mut self) -> &mut MjOption {
        (&mut self.0.option).into()
    }

    /// visual options
    pub fn visual(&self) -> &MjVisual {
        (&self.0.visual).into()
    }
    /// mutable visual options
    pub fn visual_mut(&mut self) -> &mut MjVisual {
        (&mut self.0.visual).into()
    }

    /// statistics override (if defined)
    pub fn stat(&self) -> &MjStatistic {
        (&self.0.stat).into()
    }
    /// mutable statistics override (if defined)
    pub fn stat_mut(&mut self) -> &mut MjStatistic {
        (&mut self.0.stat).into()
    }
}

// sizes
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

// global data
/*
  mjString* comment;               // comment at top of XML
  mjString* modelfiledir;          // path to model file
*/
impl MjSpec {
    /// comment at top of XML
    pub fn comment(&self) -> Option<&MjString> {
        if self.0.comment.is_null() {
            None
        } else {
            Some((unsafe { &*self.0.comment }).into())
        }
    }

    /// path to model file
    pub fn modelfiledir(&self) -> &MjString {
        (unsafe { &*self.0.modelfiledir }).into()
    }
    /// mutable path to model file
    pub fn modelfiledir_mut(&mut self) -> &mut MjString {
        (unsafe { &mut *self.0.modelfiledir }).into()
    }
}

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
        unsafe { std::mem::transmute(self.0.inertiafromgeom) }
    }
    /// set use geom inertias by `mjtInertiaFromGeom`
    pub fn set_inertiafromgeom(&mut self, inertia_from_geom: crate::bindgen::mjtInertiaFromGeom) {
        self.0.inertiafromgeom = inertia_from_geom as i32;
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
