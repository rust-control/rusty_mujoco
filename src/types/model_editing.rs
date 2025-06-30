//! # [Model Editing](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#model-editing)
//! 
//! The structs below are defined in mjspec.h and, with the exception of
//! the top level [`mjSpec`] struct, begin with the `mjs` prefix.
//! For more details, see the [Model Editing](https://mujoco.readthedocs.io/en/stable/programming/modeledit.html) chapter.

pub use crate::bindgen::{
    mjSpec, mjsElement, mjsCompiler,
    mjtInertiaFromGeom,
};

use crate::bindgen::{mjOption, mjVisual, mjStatistic, mjString, mjLROpt};

derive_fields_mapping!(mjSpec {
    boolean_flags {
        strippath / set_strippath = "automatically strip paths from mesh files";
        hasImplicitPluginElem = "already encountered an implicit plugin sensor/actuator";
    }
    scalars {
        memory / set_memory: usize = "number of bytes in arena+stack memory";
        nemax / set_nemax: usize = "max number of equality constraints";
        nuserdata / set_nuserdata: usize = "number of mjtNums in userdata";
        nuser_body / set_nuser_body: usize = "number of mjtNums in body_user";
        nuser_jnt / set_nuser_jnt: usize = "number of mjtNums in jnt_user";
        nuser_geom / set_nuser_geom: usize = "number of mjtNums in geom_user";
        nuser_site / set_nuser_site: usize = "number of mjtNums in site_user";
        nuser_cam / set_nuser_cam: usize = "number of mjtNums in cam_user";
        nuser_tendon / set_nuser_tendon: usize = "number of mjtNums in tendon_user";
        nuser_actuator / set_nuser_actuator: usize = "number of mjtNums in actuator_user";
        nuser_sensor / set_nuser_sensor: usize = "number of mjtNums in sensor_user";
        nkey / set_nkey: usize = "number of keyframes";
    }
    structs {
        compiler / compiler_mut: mjsCompiler = "compiler options";
        option / option_mut: mjOption = "physics options";
        visual / visual_mut: mjVisual = "visual options";
        stat / stat_mut: mjStatistic = "statistics override (if defined)";
    }
});
impl mjSpec {
    /// element type, do not modify
    pub fn element(&self) -> &mjsElement {unsafe { &*self.element }}

    /// model name
    pub fn modelname(&self) -> Option<&mjString> {
        if self.modelname.is_null() {None} else {Some(unsafe { &*self.modelname })}
    }
    /// mutable model name
    pub fn modelname_mut(&mut self) -> Option<&mut mjString> {
        if self.modelname.is_null() {None} else {Some(unsafe { &mut *self.modelname })}
    }

    /// comment at top of XML
    pub fn comment(&self) -> Option<&mjString> {
        if self.comment.is_null() {None} else {Some(unsafe { &*self.comment })}
    }
    /// mutable comment at top of XML
    pub fn comment_mut(&mut self) -> Option<&mut mjString> {
        if self.comment.is_null() {None} else {Some(unsafe { &mut *self.comment })}
    }

    /// path to model file
    pub fn modelfiledir(&self) -> Option<&mjString> {
        if self.modelfiledir.is_null() {None} else {Some(unsafe { &*self.modelfiledir })}
    }
    /// mutable path to model file
    pub fn modelfiledir_mut(&mut self) -> Option<&mut mjString> {
        if self.modelfiledir.is_null() {None} else {Some(unsafe { &mut *self.modelfiledir })}
    }

    /// mesh and hfield directory
    pub fn meshdir(&self) -> Option<&mjString> {
        if self.meshdir.is_null() {None} else {Some(unsafe { &*self.meshdir })}
    }
    /// mutable mesh and hfield directory
    pub fn meshdir_mut(&mut self) -> Option<&mut mjString> {
        if self.meshdir.is_null() {None} else {Some(unsafe { &mut *self.meshdir })}
    }

    /// texture directory
    pub fn texturedir(&self) -> Option<&mjString> {
        if self.texturedir.is_null() {None} else {Some(unsafe { &*self.texturedir })}
    }
    /// mutable texture directory
    pub fn texturedir_mut(&mut self) -> Option<&mut mjString> {
        if self.texturedir.is_null() {None} else {Some(unsafe { &mut *self.texturedir })}
    }
}

impl mjsElement {
    /// element type
    pub fn elemtype(&self) -> crate::mjtObj {
        self.elemtype
    }
    /// compilation signature
    pub fn signature(&self) -> u64 {
        self.signature
    }
}

derive_fields_mapping!(mjsCompiler {
    boolean_flags {
        autolimits / set_autolimits = "infer 'limited' attribute based on range";
        balanceinertia / set_balanceinertia = "automatically impose A + B >= C rule";
        fitaabb / set_fitaabb = "mesh fit to AABB instead of inertia box";
        degree / set_degree = "angles in radians or degrees";
        saveinertial / set_saveinertial = "save explicit inertial clause for all bodies to XML";
        discardvisual / set_discardvisual = "discard visual geoms in parser";
        fusestatic / set_fusestatic = "fuse static bodies with parent";
        usethread / set_usethread = "use multiple threads to speed up compiler";
    }
    scalars {
        boundmass / set_boundmass: f64 = "enforce minimum body mass";
        boundinertia / set_boundinertia: f64 = "enforce minimum body diagonal inertia";
        settotalmass / set_settotalmass: f64 = "rescale masses and inertias; <=0: ignore";
        inertiagrouprange / set_inertiagrouprange: [i32; 2] = "range of geom groups used to compute inertia";
        alignfree / set_alignfree: i32 = "align free joints with inertial frame";
    }
    enums {
        inertiafromgeom / set_inertiafromgeom: mjtInertiaFromGeom = "use geom inertias (mjtInertiaFromGeom)";
    }
    structs {
        LRopt / LRopt_mut: mjLROpt = "options for lengthrange computation";
    }
});
impl mjsCompiler {
    /// sequence for Euler rotations
    pub fn eulerseq(&self) -> [char; 3] {
        self.eulerseq.map(|i8| u8::try_from(i8).expect("unexpected char in `eulerseq`") as char)
    }
    /// set sequence for Euler rotations (**ASCII characters**)
    pub fn set_eulerseq(&mut self, seq: [char; 3]) -> &mut Self {
        assert!(seq.iter().all(char::is_ascii), "eulerseq must contain ASCII characters only");
        self.eulerseq = seq.map(|c| (c as u8).try_into().expect("unexpected char in `eulerseq`"));
        self
    }
}

/*
typedef struct mjsBody_ {          // body specification
  mjsElement* element;             // element type
  mjString* name;                  // name
  mjString* childclass;            // childclass name

  // body frame
  double pos[3];                   // frame position
  double quat[4];                  // frame orientation
  mjsOrientation alt;              // frame alternative orientation

  // inertial frame
  double mass;                     // mass
  double ipos[3];                  // inertial frame position
  double iquat[4];                 // inertial frame orientation
  double inertia[3];               // diagonal inertia (in i-frame)
  mjsOrientation ialt;             // inertial frame alternative orientation
  double fullinertia[6];           // non-axis-aligned inertia matrix

  // other
  mjtByte mocap;                   // is this a mocap body
  double gravcomp;                 // gravity compensation
  mjDoubleVec* userdata;           // user data
  mjtByte explicitinertial;        // whether to save the body with explicit inertial clause
  mjsPlugin plugin;                // passive force plugin
  mjString* info;                  // message appended to compiler errors
} mjsBody;
*/

/*
typedef struct mjsFrame_ {         // frame specification
  mjsElement* element;             // element type
  mjString* name;                  // name
  mjString* childclass;            // childclass name
  double pos[3];                   // position
  double quat[4];                  // orientation
  mjsOrientation alt;              // alternative orientation
  mjString* info;                  // message appended to compiler errors
} mjsFrame;
*/

/*
typedef struct mjsJoint_ {         // joint specification
  mjsElement* element;             // element type
  mjString* name;                  // name
  mjtJoint type;                   // joint type

  // kinematics
  double pos[3];                   // anchor position
  double axis[3];                  // joint axis
  double ref;                      // value at reference configuration: qpos0
  int align;                       // align free joint with body com (mjtAlignFree)

  // stiffness
  double stiffness;                // stiffness coefficient
  double springref;                // spring reference value: qpos_spring
  double springdamper[2];          // timeconst, dampratio

  // limits
  int limited;                     // does joint have limits (mjtLimited)
  double range[2];                 // joint limits
  double margin;                   // margin value for joint limit detection
  mjtNum solref_limit[mjNREF];     // solver reference: joint limits
  mjtNum solimp_limit[mjNIMP];     // solver impedance: joint limits
  int actfrclimited;               // are actuator forces on joint limited (mjtLimited)
  double actfrcrange[2];           // actuator force limits

  // dof properties
  double armature;                 // armature inertia (mass for slider)
  double damping;                  // damping coefficient
  double frictionloss;             // friction loss
  mjtNum solref_friction[mjNREF];  // solver reference: dof friction
  mjtNum solimp_friction[mjNIMP];  // solver impedance: dof friction

  // other
  int group;                       // group
  mjtByte actgravcomp;             // is gravcomp force applied via actuators
  mjDoubleVec* userdata;           // user data
  mjString* info;                  // message appended to compiler errors
} mjsJoint;
*/

/*
typedef struct mjsGeom_ {          // geom specification
  mjsElement* element;             // element type
  mjString* name;                  // name
  mjtGeom type;                    // geom type

  // frame, size
  double pos[3];                   // position
  double quat[4];                  // orientation
  mjsOrientation alt;              // alternative orientation
  double fromto[6];                // alternative for capsule, cylinder, box, ellipsoid
  double size[3];                  // type-specific size

  // contact related
  int contype;                     // contact type
  int conaffinity;                 // contact affinity
  int condim;                      // contact dimensionality
  int priority;                    // contact priority
  double friction[3];              // one-sided friction coefficients: slide, roll, spin
  double solmix;                   // solver mixing for contact pairs
  mjtNum solref[mjNREF];           // solver reference
  mjtNum solimp[mjNIMP];           // solver impedance
  double margin;                   // margin for contact detection
  double gap;                      // include in solver if dist < margin-gap

  // inertia inference
  double mass;                     // used to compute density
  double density;                  // used to compute mass and inertia from volume or surface
  mjtGeomInertia typeinertia;      // selects between surface and volume inertia

  // fluid forces
  mjtNum fluid_ellipsoid;          // whether ellipsoid-fluid model is active
  mjtNum fluid_coefs[5];           // ellipsoid-fluid interaction coefs

  // visual
  mjString* material;              // name of material
  float rgba[4];                   // rgba when material is omitted
  int group;                       // group

  // other
  mjString* hfieldname;            // heightfield attached to geom
  mjString* meshname;              // mesh attached to geom
  double fitscale;                 // scale mesh uniformly
  mjDoubleVec* userdata;           // user data
  mjsPlugin plugin;                // sdf plugin
  mjString* info;                  // message appended to compiler errors
} mjsGeom;
*/

/*
typedef struct mjsSite_ {          // site specification
  mjsElement* element;             // element type
  mjString* name;                  // name

  // frame, size
  double pos[3];                   // position
  double quat[4];                  // orientation
  mjsOrientation alt;              // alternative orientation
  double fromto[6];                // alternative for capsule, cylinder, box, ellipsoid
  double size[3];                  // geom size

  // visual
  mjtGeom type;                    // geom type
  mjString* material;              // name of material
  int group;                       // group
  float rgba[4];                   // rgba when material is omitted

  // other
  mjDoubleVec* userdata;           // user data
  mjString* info;                  // message appended to compiler errors
} mjsSite;
*/

/*
typedef struct mjsCamera_ {        // camera specification
  mjsElement* element;             // element type
  mjString* name;                  // name

  // extrinsics
  double pos[3];                   // position
  double quat[4];                  // orientation
  mjsOrientation alt;              // alternative orientation
  mjtCamLight mode;                // tracking mode
  mjString* targetbody;            // target body for tracking/targeting

  // intrinsics
  int orthographic;                // is camera orthographic
  double fovy;                     // y-field of view
  double ipd;                      // inter-pupilary distance
  float intrinsic[4];              // camera intrinsics (length)
  float sensor_size[2];            // sensor size (length)
  float resolution[2];             // resolution (pixel)
  float focal_length[2];           // focal length (length)
  float focal_pixel[2];            // focal length (pixel)
  float principal_length[2];       // principal point (length)
  float principal_pixel[2];        // principal point (pixel)

  // other
  mjDoubleVec* userdata;           // user data
  mjString* info;                  // message appended to compiler errors
} mjsCamera;
*/

/*
typedef struct mjsLight_ {         // light specification
  mjsElement* element;             // element type
  mjString* name;                  // name

  // frame
  double pos[3];                   // position
  double dir[3];                   // direction
  mjtCamLight mode;                // tracking mode
  mjString* targetbody;            // target body for targeting

  // intrinsics
  mjtByte active;                  // is light active
  mjtLightType type;               // type of light
  mjString* texture;               // texture name for image lights
  mjtByte castshadow;              // does light cast shadows
  float bulbradius;                // bulb radius, for soft shadows
  float intensity;                 // intensity, in candelas
  float range;                     // range of effectiveness
  float attenuation[3];            // OpenGL attenuation (quadratic model)
  float cutoff;                    // OpenGL cutoff
  float exponent;                  // OpenGL exponent
  float ambient[3];                // ambient color
  float diffuse[3];                // diffuse color
  float specular[3];               // specular color

  // other
  mjString* info;                  // message appended to compiler errorsx
} mjsLight;
*/

/*
typedef struct mjsFlex_ {          // flex specification
  mjsElement* element;             // element type
  mjString* name;                  // name

  // contact properties
  int contype;                     // contact type
  int conaffinity;                 // contact affinity
  int condim;                      // contact dimensionality
  int priority;                    // contact priority
  double friction[3];              // one-sided friction coefficients: slide, roll, spin
  double solmix;                   // solver mixing for contact pairs
  mjtNum solref[mjNREF];           // solver reference
  mjtNum solimp[mjNIMP];           // solver impedance
  double margin;                   // margin for contact detection
  double gap;                      // include in solver if dist<margin-gap

  // other properties
  int dim;                         // element dimensionality
  double radius;                   // radius around primitive element
  mjtByte internal;                // enable internal collisions
  mjtByte flatskin;                // render flex skin with flat shading
  int selfcollide;                 // mode for flex self collision
  int vertcollide;                 // mode for vertex collision
  int activelayers;                // number of active element layers in 3D
  int group;                       // group for visualizatioh
  double edgestiffness;            // edge stiffness
  double edgedamping;              // edge damping
  float rgba[4];                   // rgba when material is omitted
  mjString* material;              // name of material used for rendering
  double young;                    // Young's modulus
  double poisson;                  // Poisson's ratio
  double damping;                  // Rayleigh's damping
  double thickness;                // thickness (2D only)
  int elastic2d;                   // 2D passive forces; 0: none, 1: bending, 2: stretching, 3: both

  // mesh properties
  mjStringVec* nodebody;           // node body names
  mjStringVec* vertbody;           // vertex body names
  mjDoubleVec* node;               // node positions
  mjDoubleVec* vert;               // vertex positions
  mjIntVec* elem;                  // element vertex ids
  mjFloatVec* texcoord;            // vertex texture coordinates
  mjIntVec* elemtexcoord;          // element texture coordinates

  // other
  mjString* info;                  // message appended to compiler errors
} mjsFlex;
*/

/*
typedef struct mjsMesh_ {          // mesh specification
  mjsElement* element;             // element type
  mjString* name;                  // name
  mjString* content_type;          // content type of file
  mjString* file;                  // mesh file
  double refpos[3];                // reference position
  double refquat[4];               // reference orientation
  double scale[3];                 // rescale mesh
  mjtMeshInertia inertia;          // inertia type (convex, legacy, exact, shell)
  mjtByte smoothnormal;            // do not exclude large-angle faces from normals
  int maxhullvert;                 // maximum vertex count for the convex hull
  mjFloatVec* uservert;            // user vertex data
  mjFloatVec* usernormal;          // user normal data
  mjFloatVec* usertexcoord;        // user texcoord data
  mjIntVec* userface;              // user vertex indices
  mjIntVec* userfacetexcoord;      // user texcoord indices
  mjsPlugin plugin;                // sdf plugin
  mjString* info;                  // message appended to compiler errors
} mjsMesh;
*/

/*
typedef struct mjsHField_ {        // height field specification
  mjsElement* element;             // element type
  mjString* name;                  // name
  mjString* content_type;          // content type of file
  mjString* file;                  // file: (nrow, ncol, [elevation data])
  double size[4];                  // hfield size (ignore referencing geom size)
  int nrow;                        // number of rows
  int ncol;                        // number of columns
  mjFloatVec* userdata;            // user-provided elevation data
  mjString* info;                  // message appended to compiler errors
} mjsHField;
*/

/*
typedef struct mjsSkin_ {          // skin specification
  mjsElement* element;             // element type
  mjString* name;                  // name
  mjString* file;                  // skin file
  mjString* material;              // name of material used for rendering
  float rgba[4];                   // rgba when material is omitted
  float inflate;                   // inflate in normal direction
  int group;                       // group for visualization

  // mesh
  mjFloatVec* vert;                // vertex positions
  mjFloatVec* texcoord;            // texture coordinates
  mjIntVec* face;                  // faces

  // skin
  mjStringVec* bodyname;           // body names
  mjFloatVec* bindpos;             // bind pos
  mjFloatVec* bindquat;            // bind quat
  mjIntVecVec* vertid;             // vertex ids
  mjFloatVecVec* vertweight;       // vertex weights

  // other
  mjString* info;                  // message appended to compiler errors
} mjsSkin;
*/

/*
typedef struct mjsTexture_ {       // texture specification
  mjsElement* element;             // element type
  mjString* name;                  // name
  mjtTexture type;                 // texture type
  mjtColorSpace colorspace;        // colorspace

  // method 1: builtin
  int builtin;                     // builtin type (mjtBuiltin)
  int mark;                        // mark type (mjtMark)
  double rgb1[3];                  // first color for builtin
  double rgb2[3];                  // second color for builtin
  double markrgb[3];               // mark color
  double random;                   // probability of random dots
  int height;                      // height in pixels (square for cube and skybox)
  int width;                       // width in pixels
  int nchannel;                    // number of channels

  // method 2: single file
  mjString* content_type;          // content type of file
  mjString* file;                  // png file to load; use for all sides of cube
  int gridsize[2];                 // size of grid for composite file; (1,1)-repeat
  char gridlayout[13];             // row-major: L,R,F,B,U,D for faces; . for unused

  // method 3: separate files
  mjStringVec* cubefiles;          // different file for each side of the cube

  // method 4: from buffer read by user
  mjByteVec* data;                  // texture data

  // flip options
  mjtByte hflip;                   // horizontal flip
  mjtByte vflip;                   // vertical flip

  // other
  mjString* info;                  // message appended to compiler errors
} mjsTexture;
*/

/*
typedef struct mjsMaterial_ {      // material specification
  mjsElement* element;             // element type
  mjString* name;                  // name
  mjStringVec* textures;           // names of textures (empty: none)
  mjtByte texuniform;              // make texture cube uniform
  float texrepeat[2];              // texture repetition for 2D mapping
  float emission;                  // emission
  float specular;                  // specular
  float shininess;                 // shininess
  float reflectance;               // reflectance
  float metallic;                  // metallic
  float roughness;                 // roughness
  float rgba[4];                   // rgba
  mjString* info;                  // message appended to compiler errors
} mjsMaterial;
*/

/*
typedef struct mjsPair_ {          // pair specification
  mjsElement* element;             // element type
  mjString* name;                  // name
  mjString* geomname1;             // name of geom 1
  mjString* geomname2;             // name of geom 2

  // optional parameters: computed from geoms if not set by user
  int condim;                      // contact dimensionality
  mjtNum solref[mjNREF];           // solver reference, normal direction
  mjtNum solreffriction[mjNREF];   // solver reference, frictional directions
  mjtNum solimp[mjNIMP];           // solver impedance
  double margin;                   // margin for contact detection
  double gap;                      // include in solver if dist<margin-gap
  double friction[5];              // full contact friction
  mjString* info;                  // message appended to errors
} mjsPair;
*/

/*
typedef struct mjsExclude_ {       // exclude specification
  mjsElement* element;             // element type
  mjString* name;                  // name
  mjString* bodyname1;             // name of geom 1
  mjString* bodyname2;             // name of geom 2
  mjString* info;                  // message appended to errors
} mjsExclude;
*/

/*
typedef struct mjsEquality_ {      // equality specification
  mjsElement* element;             // element type
  mjString* name;                  // name
  mjtEq type;                      // constraint type
  double data[mjNEQDATA];          // type-dependent data
  mjtByte active;                  // is equality initially active
  mjString* name1;                 // name of object 1
  mjString* name2;                 // name of object 2
  mjtObj objtype;                  // type of both objects
  mjtNum solref[mjNREF];           // solver reference
  mjtNum solimp[mjNIMP];           // solver impedance
  mjString* info;                  // message appended to errors
} mjsEquality;
*/

/*
typedef struct mjsTendon_ {        // tendon specification
  mjsElement* element;             // element type
  mjString* name;                  // name

  // stiffness, damping, friction, armature
  double stiffness;                // stiffness coefficient
  double springlength[2];          // spring resting length; {-1, -1}: use qpos_spring
  double damping;                  // damping coefficient
  double frictionloss;             // friction loss
  mjtNum solref_friction[mjNREF];  // solver reference: tendon friction
  mjtNum solimp_friction[mjNIMP];  // solver impedance: tendon friction
  double armature;                 // inertia associated with tendon velocity

  // length range
  int limited;                     // does tendon have limits (mjtLimited)
  int actfrclimited;               // does tendon have actuator force limits
  double range[2];                 // length limits
  double actfrcrange[2];           // actuator force limits
  double margin;                   // margin value for tendon limit detection
  mjtNum solref_limit[mjNREF];     // solver reference: tendon limits
  mjtNum solimp_limit[mjNIMP];     // solver impedance: tendon limits

  // visual
  mjString* material;              // name of material for rendering
  double width;                    // width for rendering
  float rgba[4];                   // rgba when material is omitted
  int group;                       // group

  // other
  mjDoubleVec* userdata;           // user data
  mjString* info;                  // message appended to errors
} mjsTendon;
*/

/*
typedef struct mjsWrap_ {          // wrapping object specification
  mjsElement* element;             // element type
  mjString* info;                  // message appended to errors
} mjsWrap;
*/

/*
typedef struct mjsActuator_ {      // actuator specification
  mjsElement* element;             // element type
  mjString* name;                  // name

  // gain, bias
  mjtGain gaintype;                // gain type
  double gainprm[mjNGAIN];         // gain parameters
  mjtBias biastype;                // bias type
  double biasprm[mjNGAIN];         // bias parameters

  // activation state
  mjtDyn dyntype;                  // dynamics type
  double dynprm[mjNDYN];           // dynamics parameters
  int actdim;                      // number of activation variables
  mjtByte actearly;                // apply next activations to qfrc

  // transmission
  mjtTrn trntype;                  // transmission type
  double gear[6];                  // length and transmitted force scaling
  mjString* target;                // name of transmission target
  mjString* refsite;               // reference site, for site transmission
  mjString* slidersite;            // site defining cylinder, for slider-crank
  double cranklength;              // crank length, for slider-crank
  double lengthrange[2];           // transmission length range
  double inheritrange;             // automatic range setting for position and intvelocity

  // input/output clamping
  int ctrllimited;                 // are control limits defined (mjtLimited)
  double ctrlrange[2];             // control range
  int forcelimited;                // are force limits defined (mjtLimited)
  double forcerange[2];            // force range
  int actlimited;                  // are activation limits defined (mjtLimited)
  double actrange[2];              // activation range

  // other
  int group;                       // group
  mjDoubleVec* userdata;           // user data
  mjsPlugin plugin;                // actuator plugin
  mjString* info;                  // message appended to compiler errors
} mjsActuator;
*/

/*
typedef struct mjsSensor_ {        // sensor specification
  mjsElement* element;             // element type
  mjString* name;                  // name

  // sensor definition
  mjtSensor type;                  // type of sensor
  mjtObj objtype;                  // type of sensorized object
  mjString* objname;               // name of sensorized object
  mjtObj reftype;                  // type of referenced object
  mjString* refname;               // name of referenced object

  // user-defined sensors
  mjtDataType datatype;            // data type for sensor measurement
  mjtStage needstage;              // compute stage needed to simulate sensor
  int dim;                         // number of scalar outputs

  // output post-processing
  double cutoff;                   // cutoff for real and positive datatypes
  double noise;                    // noise stdev

  // other
  mjDoubleVec* userdata;           // user data
  mjsPlugin plugin;                // sensor plugin
  mjString* info;                  // message appended to compiler errors
} mjsSensor;
*/

/*
typedef struct mjsNumeric_ {       // custom numeric field specification
  mjsElement* element;             // element type
  mjString* name;                  // name
  mjDoubleVec* data;               // initialization data
  int size;                        // array size, can be bigger than data size
  mjString* info;                  // message appended to compiler errors
} mjsNumeric;
*/

/*
typedef struct mjsText_ {          // custom text specification
  mjsElement* element;             // element type
  mjString* name;                  // name
  mjString* data;                  // text string
  mjString* info;                  // message appended to compiler errors
} mjsText;
*/

/*
typedef struct mjsTuple_ {         // tuple specification
  mjsElement* element;             // element type
  mjString* name;                  // name
  mjIntVec* objtype;               // object types
  mjStringVec* objname;            // object names
  mjDoubleVec* objprm;             // object parameters
  mjString* info;                  // message appended to compiler errors
} mjsTuple;
*/

/*
typedef struct mjsKey_ {           // keyframe specification
  mjsElement* element;             // element type
  mjString* name;                  // name
  double time;                     // time
  mjDoubleVec* qpos;               // qpos
  mjDoubleVec* qvel;               // qvel
  mjDoubleVec* act;                // act
  mjDoubleVec* mpos;               // mocap pos
  mjDoubleVec* mquat;              // mocap quat
  mjDoubleVec* ctrl;               // ctrl
  mjString* info;                  // message appended to compiler errors
} mjsKey;
*/

/*
typedef struct mjsDefault_ {       // default specification
  mjsElement* element;             // element type
  mjString* name;                  // class name
  mjsJoint* joint;                 // joint defaults
  mjsGeom* geom;                   // geom defaults
  mjsSite* site;                   // site defaults
  mjsCamera* camera;               // camera defaults
  mjsLight* light;                 // light defaults
  mjsFlex* flex;                   // flex defaults
  mjsMesh* mesh;                   // mesh defaults
  mjsMaterial* material;           // material defaults
  mjsPair* pair;                   // pair defaults
  mjsEquality* equality;           // equality defaults
  mjsTendon* tendon;               // tendon defaults
  mjsActuator* actuator;           // actuator defaults
} mjsDefault;
*/

/*
typedef struct mjsPlugin_ {        // plugin specification
  mjsElement* element;             // element type
  mjString* name;                  // instance name
  mjString* plugin_name;           // plugin name
  mjtByte active;                  // is the plugin active
  mjString* info;                  // message appended to compiler errors
} mjsPlugin;
*/

/*
typedef struct mjsOrientation_ {   // alternative orientation specifiers
  mjtOrientation type;             // active orientation specifier
  double axisangle[4];             // axis and angle
  double xyaxes[6];                // x and y axes
  double zaxis[3];                 // z axis (minimal rotation)
  double euler[3];                 // Euler angles
} mjsOrientation;
*/
