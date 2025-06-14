use super::*;
use crate::bindgen::{mjNREF, mjNIMP};

pub struct MjModel(crate::bindgen::mjModel);

macro_rules! impl_size_getters {
    ($($n_name:ident = $description:literal;)*) => {
        impl MjModel {
            $(
                #[allow(non_snake_case)]
                #[doc = $description]
                pub fn $n_name(&self) -> usize {
                    self.0.$n_name as usize
                }
            )*  
        }
    };
}
impl_size_getters! {
    // sizes needed at mjModel construction
    nq                         = "number of generalized coordinates = dim(qpos)";
    nv                         = "number of degrees of freedom = dim(qvel)";
    nu                         = "number of actuators/controls = dim(ctrl)";
    na                         = "number of activation states = dim(act)";
    nbody                      = "number of bodies";
    nbvh                       = "number of total bounding volumes in all bodies";
    nbvhstatic                 = "number of static bounding volumes (aabb stored in mjModel)";
    nbvhdynamic                = "number of dynamic bounding volumes (aabb stored in mjData)";
    njnt                       = "number of joints";
    ngeom                      = "number of geoms";
    nsite                      = "number of sites";
    ncam                       = "number of cameras";
    nlight                     = "number of lights";
    nflex                      = "number of flexes";
    nflexnode                  = "number of dofs in all flexes";
    nflexvert                  = "number of vertices in all flexes";
    nflexedge                  = "number of edges in all flexes";
    nflexelem                  = "number of elements in all flexes";
    nflexelemdata              = "number of element vertex ids in all flexes";
    nflexelemedge              = "number of element edge ids in all flexes";
    nflexshelldata             = "number of shell fragment vertex ids in all flexes";
    nflexevpair                = "number of element-vertex pairs in all flexes";
    nflextexcoord              = "number of vertices with texture coordinates";
    nmesh                      = "number of meshes";
    nmeshvert                  = "number of vertices in all meshes";
    nmeshnormal                = "number of normals in all meshes";
    nmeshtexcoord              = "number of texcoords in all meshes";
    nmeshface                  = "number of triangular faces in all meshes";
    nmeshgraph                 = "number of ints in mesh auxiliary data";
    nmeshpoly                  = "number of polygons in all meshes";
    nmeshpolyvert              = "number of vertices in all polygons";
    nmeshpolymap               = "number of polygons in vertex map";
    nskin                      = "number of skins";
    nskinvert                  = "number of vertices in all skins";
    nskintexvert               = "number of vertiex with texcoords in all skins";
    nskinface                  = "number of triangular faces in all skins";
    nskinbone                  = "number of bones in all skins";
    nskinbonevert              = "number of vertices in all skin bones";
    nhfield                    = "number of heightfields";
    nhfielddata                = "number of data points in all heightfields";
    ntex                       = "number of textures";
    ntexdata                   = "number of bytes in texture rgb data";
    nmat                       = "number of materials";
    npair                      = "number of predefined geom pairs";
    nexclude                   = "number of excluded geom pairs";
    neq                        = "number of equality constraints";
    ntendon                    = "number of tendons";
    nwrap                      = "number of wrap objects in all tendon paths";
    nsensor                    = "number of sensors";
    nnumeric                   = "number of numeric custom fields";
    nnumericdata               = "number of mjtNums in all numeric fields";
    ntext                      = "number of text custom fields";
    ntextdata                  = "number of mjtBytes in all text fields";
    ntuple                     = "number of tuple custom fields";
    ntupledata                 = "number of objects in all tuple fields";
    nkey                       = "number of keyframes";
    nmocap                     = "number of mocap bodies";
    nplugin                    = "number of plugin instances";
    npluginattr                = "number of chars in all plugin config attributes";
    nuser_body                 = "number of mjtNums in body_user";
    nuser_jnt                  = "number of mjtNums in jnt_user";
    nuser_geom                 = "number of mjtNums in geom_user";
    nuser_site                 = "number of mjtNums in site_user";
    nuser_cam                  = "number of mjtNums in cam_user";
    nuser_tendon               = "number of mjtNums in tendon_user";
    nuser_actuator             = "number of mjtNums in actuator_user";
    nuser_sensor               = "number of mjtNums in sensor_user";
    nnames                     = "number of chars in all names";
    npaths                     = "number of chars in all paths";

    // sizes set after mjModel construction
    nnames_map                 = "number of slots in the names hash map";
    nM                         = "number of non-zeros in sparse inertia matrix";
    nB                         = "number of non-zeros in sparse body-dof matrix";
    nC                         = "number of non-zeros in sparse reduced dof-dof matrix";
    nD                         = "number of non-zeros in sparse dof-dof matrix";
    nJmom                      = "number of non-zeros in sparse actuator_moment matrix";
    ntree                      = "number of kinematic trees under world body";
    ngravcomp                  = "number of bodies with nonzero gravcomp";
    nemax                      = "number of potential equality-constraint rows";
    njmax                      = "number of available rows in constraint Jacobian (legacy)";
    nconmax                    = "number of potential contacts in contact list (legacy)";
    nuserdata                  = "number of mjtNums reserved for the user";
    nsensordata                = "number of mjtNums in sensor data vector";
    npluginstate               = "number of mjtNums in plugin state vector";

    narena                     = "number of bytes in the mjData arena (inclusive of stack)";
    nbuffer                    = "number of bytes in buffer";
}

impl MjModel {
    /// physics options
    pub fn opt(&self) -> MjOption {
        MjOption(self.0.opt)
    }
    /// update physics options
    pub fn update_opt(&mut self, f: impl FnOnce(&mut MjOption) -> &mut MjOption) -> &mut Self {
        let mut opt = self.opt();
        f(&mut opt);
        self.0.opt = opt.0;
        self
    }

    /// visualization options
    pub fn vis(&self) -> MjVisual {
        MjVisual(self.0.vis)
    }
    /// update visualization options
    pub fn update_visual(&mut self, f: impl FnOnce(&mut MjVisual) -> &mut MjVisual) -> &mut Self {
        let mut vis = self.vis();
        f(&mut vis);
        self.0.vis = vis.0;
        self
    }

    /// model statistics
    pub fn stat(&self) -> MjStatistic {
        MjStatistic(self.0.stat)
    }
    /// update model statistics
    pub fn update_stat(&mut self, f: impl FnOnce(&mut MjStatistic) -> &mut MjStatistic) -> &mut Self {
        let mut stat = self.stat();
        f(&mut stat);
        self.0.stat = stat.0;
        self
    }
}

macro_rules! impl_buffer_slices {
    ($($name:ident: [$T:ty; $size:ident $(* $mul_lit:literal)? $(* $mul_const:ident)? $(x $mul:ident)?] = $description:literal;)*) => {
        impl MjModel {
            $(
                #[allow(non_snake_case)]
                #[doc = $description]
                pub fn $name(&self) -> &[$T] {
                    unsafe { std::slice::from_raw_parts(self.0.$name, self.$size() $(* $mul_lit)? $(* $mul_const as usize)? $(* self.$mul())?) }
                }
            )*
        }
    };
}
impl_buffer_slices! {
    // default generalized coordinates
    qpos0: [f64; nq] = "qpos values at default pose";
    qpos_spring: [f64; nq] = "reference pose for springs";

    // bodies
    body_parentid: [i32; nbody]          = "id of body's parent";
    body_rootid: [i32; nbody]            = "id of root above body";
    body_weldid: [i32; nbody]            = "id of body that this body is welded to";
    body_mocapid: [i32; nbody]           = "id of mocap data; -1: none";
    body_jntnum: [i32; nbody]            = "number of joints for this body";
    body_jntadr: [i32; nbody]            = "start addr of joints; -1: no joints";
    body_dofnum: [i32; nbody]            = "number of motion degrees of freedom";
    body_dofadr: [i32; nbody]            = "start addr of dofs; -1: no dofs";
    body_treeid: [i32; nbody]            = "id of body's kinematic tree; -1: static";
    body_geomnum: [i32; nbody]           = "number of geoms";
    body_geomadr: [i32; nbody]           = "start addr of geoms; -1: no geoms";
    body_simple: [u8; nbody]             = "1: diag M; 2: diag M, sliders only";
    body_sameframe: [u8; nbody]          = "same frame as inertia (mjtSameframe)";
    body_pos: [f64; nbody * 3]           = "position offset rel. to parent body";
    body_quat: [f64; nbody * 4]          = "orientation offset rel. to parent body";
    body_ipos: [f64; nbody * 3]          = "local position of center of mass";
    body_iquat: [f64; nbody * 4]         = "local orientation of inertia ellipsoid";
    body_mass: [f64; nbody * 1]          = "mass";
    body_subtreemass: [f64; nbody * 1]   = "mass of subtree starting at this body";
    body_inertia: [f64; nbody * 3]       = "diagonal inertia in ipos/iquat frame";
    body_invweight0: [f64; nbody * 2]    = "mean inv inert in qpos0 (trn, rot)";
    body_gravcomp: [f64; nbody * 1]      = "antigravity force, units of body weight";
    body_margin: [f64; nbody * 1]        = "MAX over all geom margins";
    body_user: [f64; nbody x nuser_body] = "user data";
    body_plugin: [i32; nbody]            = "plugin instance id; -1: not in use";
    body_contype: [i32; nbody]           = "OR over all geom contypes";
    body_conaffinity: [i32; nbody]       = "OR over all geom conaffinities";
    body_bvhadr: [i32; nbody]            = "address of bvh root";
    body_bvhnum: [i32; nbody]            = "number of bounding volumes";

    // bounding volume hierarchy
    bvh_depth: [i32; nbvh * 1]      = "depth in the bounding volume hierarchy";
    bvh_child: [i32; nbvh * 2]      = "left and right children in tree";
    bvh_nodeid: [i32; nbvh * 1]     = "geom or elem id of node; -1: non-leaf";
    bvh_aabb: [f64; nbvhstatic * 6] = "local bounding box (center, size)";

    // joints
    jnt_type: [i32; njnt * 1]            = "type of joint (mjtJoint)";
    jnt_qposadr: [i32; njnt * 1]         = "start addr in 'qpos' for joint's data";
    jnt_dofadr: [i32; njnt * 1]          = "start addr in 'qvel' for joint's data";
    jnt_bodyid: [i32; njnt * 1]          = "id of joint's body";
    jnt_group: [i32; njnt * 1]           = "group for visibility";
    jnt_limited: [u8; njnt * 1]          = "does joint have limits";
    jnt_actfrclimited: [u8; njnt * 1]    = "does joint have actuator force limits";
    jnt_actgravcomp: [u8; njnt * 1]      = "is gravcomp force applied via actuators";
    jnt_solref: [f64; njnt * mjNREF]    = "constraint solver reference: limit";
    jnt_solimp: [f64; njnt * mjNIMP]    = "constraint solver impedance: limit";
    jnt_pos: [f64; njnt * 3]            = "local anchor position";
    jnt_axis: [f64; njnt * 3]           = "local joint axis";
    jnt_stiffness: [f64; njnt * 1]      = "stiffness coefficient";
    jnt_range: [f64; njnt * 2]          = "joint limits";
    jnt_actfrcrange: [f64; njnt * 2]    = "range of total actuator force";
    jnt_margin: [f64; njnt * 1]         = "min distance for limit detection";
    jnt_user: [f64; njnt x nuser_jnt]   = "user data";
}
