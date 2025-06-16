use super::*;
use crate::bindgen::{mjNREF, mjNIMP, mjNFLUID, mjNEQDATA, mjNGAIN, mjNBIAS, mjNDYN, mjtTextureRole::mjNTEXROLE};

/// This is the main data structure holding the MuJoCo model. It is treated as constant by the simulator.
pub struct MjModel(pub(crate) crate::bindgen::mjModel);

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
    pub fn opt(&self) -> &MjOption {
        (&self.0.opt).into()
    }

    /// visualization options
    pub fn vis(&self) -> &MjVisual {
        (&self.0.vis).into()
    }

    /// model statistics
    pub fn stat(&self) -> &MjStatistic {
        (&self.0.stat).into()
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

    // dofs
    dof_bodyid: [i32; nv * 1]           = "id of dof's body";
    dof_jntid: [i32; nv * 1]            = "id of dof's joint";
    dof_parentid: [i32; nv * 1]         = "id of dof's parent; -1: none";
    dof_treeid: [i32; nv * 1]           = "id of dof's kinematic tree";
    dof_Madr: [i32; nv * 1]             = "dof address in M-diagonal";
    dof_simplenum: [i32; nv * 1]        = "number of consecutive simple dofs";
    dof_solref: [f64; nv * mjNREF]      = "constraint solver reference:frictionloss";
    dof_solimp: [f64; nv * mjNIMP]      = "constraint solver impedance:frictionloss";
    dof_frictionloss: [f64; nv * 1]     = "dof friction loss";
    dof_armature: [f64; nv * 1]         = "dof armature inertia/mass";
    dof_damping: [f64; nv * 1]          = "damping coefficient";
    dof_invweight0: [f64; nv * 1]       = "diag. inverse inertia in qpos0";
    dof_M0: [f64; nv * 1]               = "diag. inertia in qpos0";

    // geoms
    geom_type: [i32; ngeom * 1]            = "geometric type (mjtGeom)";
    geom_contype: [i32; ngeom * 1]         = "geom contact type";
    geom_conaffinity: [i32; ngeom * 1]     = "geom contact affinity";
    geom_condim: [i32; ngeom * 1]          = "contact dimensionality (1, 3, 4, 6)";
    geom_bodyid: [i32; ngeom * 1]          = "id of geom's body";
    geom_dataid: [i32; ngeom * 1]          = "id of geom's mesh/hfield; -1: none";
    geom_matid: [i32; ngeom * 1]           = "material id for rendering; -1: none";
    geom_group: [i32; ngeom * 1]           = "group for visibility";
    geom_priority: [i32; ngeom * 1]        = "geom contact priority";
    geom_plugin: [i32; ngeom * 1]          = "plugin instance id; -1: not in use";
    geom_sameframe: [u8; ngeom * 1]        = "same frame as body (mjtSameframe)";
    geom_solmix: [f64; ngeom * 1]          = "mixing coef for solref/imp in geom pair";
    geom_solref: [f64; ngeom * mjNREF]     = "constraint solver reference: contact";
    geom_solimp: [f64; ngeom * mjNIMP]     = "constraint solver impedance: contact";
    geom_size: [f64; ngeom * 3]            = "geom-specific size parameters";
    geom_aabb: [f64; ngeom * 6]            = "bounding box, (center, size)";
    geom_rbound: [f64; ngeom * 1]          = "radius of bounding sphere";
    geom_pos: [f64; ngeom * 3]             = "local position offset rel. to body";
    geom_quat: [f64; ngeom * 4]            = "local orientation offset rel. to body";
    geom_friction: [f64; ngeom * 3]        = "friction for (slide, spin, roll)";
    geom_margin: [f64; ngeom * 1]          = "detect contact if dist<margin";
    geom_gap: [f64; ngeom * 1]             = "include in solver if dist<margin-gap";
    geom_fluid: [f64; ngeom * mjNFLUID]    = "fluid interaction parameters";
    geom_user: [f64; ngeom x nuser_geom]   = "user data";
    geom_rgba: [f32; ngeom * 4]            = "rgba when material is omitted";

    // sites
    site_type: [i32; nsite * 1]            = "geom type for rendering (mjtGeom)";
    site_bodyid: [i32; nsite * 1]          = "id of site's body";
    site_matid: [i32; nsite * 1]           = "material id for rendering; -1: none";
    site_group: [i32; nsite * 1]           = "group for visibility";
    site_sameframe: [u8; nsite * 1]        = "same frame as body (mjtSameframe)";
    site_size: [f64; nsite * 3]            = "geom size for rendering";
    site_pos: [f64; nsite * 3]             = "local position offset rel. to body";
    site_quat: [f64; nsite * 4]            = "local orientation offset rel. to body";
    site_user: [f64; nsite x nuser_site]   = "user data";
    site_rgba: [f32; nsite * 4]            = "rgba when material is omitted";

    // cameras
    cam_mode: [i32; ncam * 1]             = "camera tracking mode (mjtCamLight)";
    cam_bodyid: [i32; ncam * 1]           = "id of camera's body";
    cam_targetbodyid: [i32; ncam * 1]     = "id of targeted body; -1: none";
    cam_pos: [f64; ncam * 3]              = "position rel. to body frame";
    cam_quat: [f64; ncam * 4]             = "orientation rel. to body frame";
    cam_poscom0: [f64; ncam * 3]          = "global position rel. to sub-com in qpos0";
    cam_pos0: [f64; ncam * 3]             = "global position rel. to body in qpos0";
    cam_mat0: [f64; ncam * 9]             = "global orientation in qpos0";
    cam_orthographic: [i32; ncam * 1]     = "orthographic camera; 0: no, 1: yes";
    cam_fovy: [f64; ncam * 1]             = "y field-of-view (ortho ? len : deg)";
    cam_ipd: [f64; ncam * 1]              = "inter-pupilary distance";
    cam_resolution: [i32; ncam * 2]       = "resolution: pixels [width, height]";
    cam_sensorsize: [f32; ncam * 2]       = "sensor size: length [width, height]";
    cam_intrinsic: [f32; ncam * 4]        = "[focal length; principal point]";
    cam_user: [f64; ncam x nuser_cam]     = "user data";

    // lights
    light_mode: [i32; nlight * 1]           = "light tracking mode (mjtCamLight)";
    light_bodyid: [i32; nlight * 1]         = "id of light's body";
    light_targetbodyid: [i32; nlight * 1]   = "id of targeted body; -1: none";
    light_castshadow: [u8; nlight * 1]      = "does light cast shadows";
    light_bulbradius: [f32; nlight * 1]     = "light radius for soft shadows";
    light_active: [u8; nlight * 1]          = "is light on";
    light_pos: [f64; nlight * 3]            = "position rel. to body frame";
    light_dir: [f64; nlight * 3]            = "direction rel. to body frame";
    light_poscom0: [f64; nlight * 3]        = "global position rel. to sub-com in qpos0";
    light_pos0: [f64; nlight * 3]           = "global position rel. to body in qpos0";
    light_dir0: [f64; nlight * 3]           = "global direction in qpos0";
    light_attenuation: [f32; nlight * 3]    = "OpenGL attenuation (quadratic model)";
    light_cutoff: [f32; nlight * 1]         = "OpenGL cutoff";
    light_exponent: [f32; nlight * 1]       = "OpenGL exponent";
    light_ambient: [f32; nlight * 3]        = "ambient rgb (alpha=1)";
    light_diffuse: [f32; nlight * 3]        = "diffuse rgb (alpha=1)";
    light_specular: [f32; nlight * 3]       = "specular rgb (alpha=1)";

    // flexes: contact properties
    flex_contype: [i32; nflex * 1]         = "flex contact type";
    flex_conaffinity: [i32; nflex * 1]     = "flex contact affinity";
    flex_condim: [i32; nflex * 1]          = "contact dimensionality (1, 3, 4, 6)";
    flex_priority: [i32; nflex * 1]        = "flex contact priority";
    flex_solmix: [f64; nflex * 1]          = "mix coef for solref/imp in contact pair";
    flex_solref: [f64; nflex * mjNREF]          = "constraint solver reference: contact";
    flex_solimp: [f64; nflex * mjNIMP]          = "constraint solver impedance: contact";
    flex_friction: [f64; nflex * 3]        = "friction for (slide, spin, roll)";
    flex_margin: [f64; nflex * 1]          = "detect contact if dist<margin";
    flex_gap: [f64; nflex * 1]             = "include in solver if dist<margin-gap";
    flex_internal: [u8; nflex * 1]         = "internal flex collision enabled";
    flex_selfcollide: [i32; nflex * 1]     = "self collision mode (mjtFlexSelf)";
    flex_activelayers: [i32; nflex * 1]    = "number of active element layers, 3D only";

    // flexes: other properties
    flex_dim: [i32; nflex * 1]             = "1: lines, 2: triangles, 3: tetrahedra";
    flex_matid: [i32; nflex * 1]           = "material id for rendering";
    flex_group: [i32; nflex * 1]           = "group for visibility";
    flex_interp: [i32; nflex * 1]          = "interpolation (0: vertex, 1: nodes)";
    flex_nodeadr: [i32; nflex * 1]         = "first node address";
    flex_nodenum: [i32; nflex * 1]         = "number of nodes";
    flex_vertadr: [i32; nflex * 1]         = "first vertex address";
    flex_vertnum: [i32; nflex * 1]         = "number of vertices";
    flex_edgeadr: [i32; nflex * 1]         = "first edge address";
    flex_edgenum: [i32; nflex * 1]         = "number of edges";
    flex_elemadr: [i32; nflex * 1]         = "first element address";
    flex_elemnum: [i32; nflex * 1]         = "number of elements";
    flex_elemdataadr: [i32; nflex * 1]     = "first element vertex id address";
    flex_elemedgeadr: [i32; nflex * 1]     = "first element edge id address";
    flex_shellnum: [i32; nflex * 1]        = "number of shells";
    flex_shelldataadr: [i32; nflex * 1]    = "first shell data address";
    flex_evpairadr: [i32; nflex * 1]       = "first evpair address";
    flex_evpairnum: [i32; nflex * 1]       = "number of evpairs";
    flex_texcoordadr: [i32; nflex * 1]     = "address in flex_texcoord; -1: none";
    flex_nodebodyid: [i32; nflexnode * 1]      = "node body ids";
    flex_vertbodyid: [i32; nflexvert * 1]      = "vertex body ids";
    flex_edge: [i32; nflexedge * 2]            = "edge vertex ids (2 per edge)";
    flex_elem: [i32; nflexelemdata * 1]            = "element vertex ids (dim+1 per elem)";
    flex_elemtexcoord: [i32; nflexelemdata * 1]    = "element texture coordinates (dim+1)";
    flex_elemedge: [i32; nflexelemedge * 1]        = "element edge ids";
    flex_elemlayer: [i32; nflexelem * 1]       = "element distance from surface, 3D only";
    flex_shell: [i32; nflexshelldata * 1]           = "shell fragment vertex ids (dim per frag)";
    flex_evpair: [i32; nflexevpair * 2]          = "(element, vertex) collision pairs";
    flex_vert: [f64; nflexvert * 3]            = "vertex positions in local body frames";
    flex_vert0: [f64; nflexvert * 3]           = "vertex positions in qpos0 on [0, 1]^d";
    flex_node: [f64; nflexnode * 3]            = "node positions in local body frames";
    flex_node0: [f64; nflexnode * 3]           = "Cartesian node positions in qpos0";
    flexedge_length0: [f64; nflexedge * 1]     = "edge lengths in qpos0";
    flexedge_invweight0: [f64; nflexedge * 1]  = "edge inv. weight in qpos0";
    flex_radius: [f64; nflex * 1]          = "radius around primitive element";
    flex_stiffness: [f64; nflexelem * 21]       = "finite element stiffness matrix";
    flex_damping: [f64; nflex * 1]         = "Rayleigh's damping coefficient";
    flex_edgestiffness: [f64; nflex * 1]   = "edge stiffness";
    flex_edgedamping: [f64; nflex * 1]     = "edge damping";
    flex_edgeequality: [u8; nflex * 1]     = "is edge equality constraint defined";
    flex_rigid: [u8; nflex * 1]            = "are all verices in the same body";
    flexedge_rigid: [u8; nflexedge * 1]        = "are both edge vertices in same body";
    flex_centered: [u8; nflex * 1]         = "are all vertex coordinates (0,0,0)";
    flex_flatskin: [u8; nflex * 1]         = "render flex skin with flat shading";
    flex_bvhadr: [i32; nflex * 1]          = "address of bvh root; -1: no bvh";
    flex_bvhnum: [i32; nflex * 1]          = "number of bounding volumes";
    flex_rgba: [f32; nflex * 4]            = "rgba when material is omitted";
    flex_texcoord: [f32; nflextexcoord * 2]        = "vertex texture coordinates";

    // meshes
    mesh_vertadr: [i32; nmesh * 1]         = "first vertex address";
    mesh_vertnum: [i32; nmesh * 1]         = "number of vertices";
    mesh_faceadr: [i32; nmesh * 1]         = "first face address";
    mesh_facenum: [i32; nmesh * 1]         = "number of faces";
    mesh_bvhadr: [i32; nmesh * 1]          = "address of bvh root";
    mesh_bvhnum: [i32; nmesh * 1]          = "number of bvh";
    mesh_normaladr: [i32; nmesh * 1]       = "first normal address";
    mesh_normalnum: [i32; nmesh * 1]       = "number of normals";
    mesh_texcoordadr: [i32; nmesh * 1]     = "texcoord data address; -1: no texcoord";
    mesh_texcoordnum: [i32; nmesh * 1]     = "number of texcoord";
    mesh_graphadr: [i32; nmesh * 1]        = "graph data address; -1: no graph";
    mesh_vert: [f32; nmeshvert * 3]            = "vertex positions for all meshes";
    mesh_normal: [f32; nmeshnormal * 3]          = "normals for all meshes";
    mesh_texcoord: [f32; nmeshtexcoord * 2]        = "vertex texcoords for all meshes";
    mesh_face: [i32; nmeshface * 3]            = "vertex face data";
    mesh_facenormal: [i32; nmeshface * 3]      = "normal face data";
    mesh_facetexcoord: [i32; nmeshface * 3]    = "texture face data";
    mesh_graph: [i32; nmeshgraph * 1]           = "convex graph data";
    mesh_scale: [f64; nmesh * 3]           = "scaling applied to asset vertices";
    mesh_pos: [f64; nmesh * 3]             = "translation applied to asset vertices";
    mesh_quat: [f64; nmesh * 4]            = "rotation applied to asset vertices";
    mesh_pathadr: [i32; nmesh * 1]         = "address of asset path for mesh; -1: none";
    mesh_polynum: [i32; nmesh * 1]         = "number of polygons per mesh";
    mesh_polyadr: [i32; nmesh * 1]         = "first polygon address per mesh";
    mesh_polynormal: [f64; nmeshpoly * 3]      = "all polygon normals";
    mesh_polyvertadr: [i32; nmeshpoly * 1]     = "polygon vertex start address";
    mesh_polyvertnum: [i32; nmeshpoly * 1]     = "number of vertices per polygon";
    mesh_polyvert: [i32; nmeshpolyvert * 1]        = "all polygon vertices";
    mesh_polymapadr: [i32; nmeshvert * 1]      = "first polygon address per vertex";
    mesh_polymapnum: [i32; nmeshvert * 1]      = "number of polygons per vertex";
    mesh_polymap: [i32; nmeshpolymap * 1]         = "vertex to polygon map";

    // skins
    skin_matid: [i32; nskin * 1]           = "skin material id; -1: none";
    skin_group: [i32; nskin * 1]           = "group for visibility";
    skin_rgba: [f32; nskin * 4]            = "skin rgba";
    skin_inflate: [f32; nskin * 1]         = "inflate skin in normal direction";
    skin_vertadr: [i32; nskin * 1]         = "first vertex address";
    skin_vertnum: [i32; nskin * 1]         = "number of vertices";
    skin_texcoordadr: [i32; nskin * 1]     = "texcoord data address; -1: no texcoord";
    skin_faceadr: [i32; nskin * 1]         = "first face address";
    skin_facenum: [i32; nskin * 1]         = "number of faces";
    skin_boneadr: [i32; nskin * 1]         = "first bone in skin";
    skin_bonenum: [i32; nskin * 1]         = "number of bones in skin";
    skin_vert: [f32; nskinvert * 3]            = "vertex positions for all skin meshes";
    skin_texcoord: [f32; nskintexvert * 2]        = "vertex texcoords for all skin meshes";
    skin_face: [i32; nskinface * 3]            = "triangle faces for all skin meshes";
    skin_bonevertadr: [i32; nskinbone * 1]     = "first vertex in each bone";
    skin_bonevertnum: [i32; nskinbone * 1]     = "number of vertices in each bone";
    skin_bonebindpos: [f32; nskinbone * 3]     = "bind pos of each bone";
    skin_bonebindquat: [f32; nskinbone * 4]    = "bind quat of each bone";
    skin_bonebodyid: [i32; nskinbone * 1]      = "body id of each bone";
    skin_bonevertid: [i32; nskinbonevert * 1]      = "mesh ids of vertices in each bone";
    skin_bonevertweight: [f32; nskinbonevert * 1]  = "weights of vertices in each bone";
    skin_pathadr: [i32; nskin * 1]         = "address of asset path for skin; -1: none";

    // height fields
    hfield_size: [f64; nhfield * 4]          = "x, y, z_top, z_bottom)";
    hfield_nrow: [i32; nhfield * 1]          = "number of rows in grid";
    hfield_ncol: [i32; nhfield * 1]          = "number of columns in grid";
    hfield_adr: [i32; nhfield * 1]           = "address in hfield_data";
    hfield_data: [f32; nhfielddata * 1]          = "elevation data";
    hfield_pathadr: [i32; nhfield * 1]       = "address of hfield asset path; -1: none";

    // textures
    tex_type: [i32; ntex * 1]             = "texture type (mjtTexture)";
    tex_height: [i32; ntex * 1]           = "number of rows in texture image";
    tex_width: [i32; ntex * 1]            = "number of columns in texture image";
    tex_nchannel: [i32; ntex * 1]         = "number of channels in texture image";
    tex_adr: [i32; ntex * 1]              = "start address in tex_data";
    tex_data: [u8; ntexdata * 1]              = "pixel values";
    tex_pathadr: [i32; ntex * 1]          = "address of texture asset path; -1: none";

    // materials
    mat_texid: [i32; nmat * mjNTEXROLE]            = "indices of textures; -1: none";
    mat_texuniform: [u8; nmat * 1]        = "make texture cube uniform";
    mat_texrepeat: [f32; nmat * 2]        = "texture repetition for 2d mapping";
    mat_emission: [f32; nmat * 1]         = "emission (x rgb)";
    mat_specular: [f32; nmat * 1]         = "specular (x white)";
    mat_shininess: [f32; nmat * 1]        = "shininess coef";
    mat_reflectance: [f32; nmat * 1]      = "reflectance (0: disable)";
    mat_metallic: [f32; nmat * 1]         = "metallic coef";
    mat_roughness: [f32; nmat * 1]        = "roughness coef";
    mat_rgba: [f32; nmat * 4]             = "rgba";

    // predefined geom pairs for collision detection; has precedence over exclude
    pair_dim: [i32; npair * 1]             = "contact dimensionality";
    pair_geom1: [i32; npair * 1]           = "id of geom1";
    pair_geom2: [i32; npair * 1]           = "id of geom2";
    pair_signature: [i32; npair * 1]       = "body1 << 16 + body2";
    pair_solref: [f64; npair * mjNREF]          = "solver reference: contact normal";
    pair_solreffriction: [f64; npair * mjNREF]  = "solver reference: contact friction";
    pair_solimp: [f64; npair * mjNIMP]          = "solver impedance: contact";
    pair_margin: [f64; npair * 1]          = "detect contact if dist<margin";
    pair_gap: [f64; npair * 1]             = "include in solver if dist<margin-gap";
    pair_friction: [f64; npair * 5]        = "tangent1, 2, spin, roll1, 2";

    // excluded body pairs for collision detection
    exclude_signature: [i32; nexclude * 1]    = "body1 << 16 + body2";

    // equality constraints
    eq_type: [i32; neq * 1]              = "constraint type (mjtEq)";
    eq_obj1id: [i32; neq * 1]            = "id of object 1";
    eq_obj2id: [i32; neq * 1]            = "id of object 2";
    eq_objtype: [i32; neq * 1]           = "type of both objects (mjtObj)";
    eq_active0: [u8; neq * 1]            = "initial enable/disable constraint state";
    eq_solref: [f64; neq * mjNREF]            = "constraint solver reference";
    eq_solimp: [f64; neq * mjNIMP]            = "constraint solver impedance";
    eq_data: [f64; neq * mjNEQDATA]              = "numeric data for constraint";

    // tendons
    tendon_adr: [i32; ntendon * 1]           = "address of first object in tendon's path";
    tendon_num: [i32; ntendon * 1]           = "number of objects in tendon's path";
    tendon_matid: [i32; ntendon * 1]         = "material id for rendering";
    tendon_group: [i32; ntendon * 1]         = "group for visibility";
    tendon_limited: [u8; ntendon * 1]        = "does tendon have length limits";
    tendon_actfrclimited: [u8; ntendon * 1]  = "does tendon have actuator force limits";
    tendon_width: [f64; ntendon * 1]         = "width for rendering";
    tendon_solref_lim: [f64; ntendon * mjNREF]    = "constraint solver reference: limit";
    tendon_solimp_lim: [f64; ntendon * mjNIMP]    = "constraint solver impedance: limit";
    tendon_solref_fri: [f64; ntendon * mjNREF]    = "constraint solver reference: friction";
    tendon_solimp_fri: [f64; ntendon * mjNIMP]    = "constraint solver impedance: friction";
    tendon_range: [f64; ntendon * 2]         = "tendon length limits";
    tendon_actfrcrange: [f64; ntendon * 2]   = "range of total actuator force";
    tendon_margin: [f64; ntendon * 1]        = "min distance for limit detection";
    tendon_stiffness: [f64; ntendon * 1]     = "stiffness coefficient";
    tendon_damping: [f64; ntendon * 1]       = "damping coefficient";
    tendon_armature: [f64; ntendon * 1]      = "inertia associated with tendon velocity";
    tendon_frictionloss: [f64; ntendon * 1]  = "loss due to friction";
    tendon_lengthspring: [f64; ntendon * 2]  = "spring resting length range";
    tendon_length0: [f64; ntendon * 1]       = "tendon length in qpos0";
    tendon_invweight0: [f64; ntendon * 1]    = "inv. weight in qpos0";
    tendon_user: [f64; ntendon x nuser_tendon]          = "user data";
    tendon_rgba: [f32; ntendon * 4]          = "rgba when material is omitted";

    // list of all wrap objects in tendon paths
    wrap_type: [i32; nwrap * 1]            = "wrap object type (mjtWrap)";
    wrap_objid: [i32; nwrap * 1]           = "object id: geom, site, joint";
    wrap_prm: [f64; nwrap * 1]             = "divisor, joint coef, or site id";

    // actuators
    actuator_trntype: [i32; nu * 1]     = "transmission type (mjtTrn)";
    actuator_dyntype: [i32; nu * 1]     = "dynamics type (mjtDyn)";
    actuator_gaintype: [i32; nu * 1]    = "gain type (mjtGain)";
    actuator_biastype: [i32; nu * 1]    = "bias type (mjtBias)";
    actuator_trnid: [i32; nu * 2]       = "transmission id: joint, tendon, site";
    actuator_actadr: [i32; nu * 1]      = "first activation address; -1: stateless";
    actuator_actnum: [i32; nu * 1]      = "number of activation variables";
    actuator_group: [i32; nu * 1]       = "group for visibility";
    actuator_ctrllimited: [u8; nu * 1]  = "is control limited";
    actuator_forcelimited: [u8; nu * 1] = "force limited";
    actuator_actlimited: [u8; nu * 1]   = "is activation limited";
    actuator_dynprm: [f64; nu * mjNDYN]      = "dynamics parameters";
    actuator_gainprm: [f64; nu * mjNGAIN]     = "gain parameters";
    actuator_biasprm: [f64; nu * mjNBIAS]     = "bias parameters";
    actuator_actearly: [u8; nu * 1]     = "step activation before force";
    actuator_ctrlrange: [f64; nu * 2]   = "range of controls";
    actuator_forcerange: [f64; nu * 2]  = "range of forces";
    actuator_actrange: [f64; nu * 2]    = "range of activations";
    actuator_gear: [f64; nu * 6]        = "scale length and transmitted force";
    actuator_cranklength: [f64; nu * 1] = "crank length for slider-crank";
    actuator_acc0: [f64; nu * 1]        = "acceleration from unit force in qpos0";
    actuator_length0: [f64; nu * 1]     = "actuator length in qpos0";
    actuator_lengthrange: [f64; nu * 2] = "feasible actuator length range";
    actuator_user: [f64; nu x nuser_actuator]        = "user data";
    actuator_plugin: [i32; nu * 1]      = "plugin instance id; -1: not a plugin";

    // sensors
    sensor_type: [i32; nsensor * 1]          = "sensor type (mjtSensor)";
    sensor_datatype: [i32; nsensor * 1]      = "numeric data type (mjtDataType)";
    sensor_needstage: [i32; nsensor * 1]     = "required compute stage (mjtStage)";
    sensor_objtype: [i32; nsensor * 1]       = "type of sensorized object (mjtObj)";
    sensor_objid: [i32; nsensor * 1]         = "id of sensorized object";
    sensor_reftype: [i32; nsensor * 1]       = "type of reference frame (mjtObj)";
    sensor_refid: [i32; nsensor * 1]         = "id of reference frame; -1: global frame";
    sensor_dim: [i32; nsensor * 1]           = "number of scalar outputs";
    sensor_adr: [i32; nsensor * 1]           = "address in sensor array";
    sensor_cutoff: [f64; nsensor * 1]        = "cutoff for real and positive; 0: ignore";
    sensor_noise: [f64; nsensor * 1]         = "noise standard deviation";
    sensor_user: [f64; nsensor x nuser_sensor]          = "user data";
    sensor_plugin: [i32; nsensor * 1]        = "plugin instance id; -1: not a plugin";

    // plugin instances
    plugin: [i32; nplugin * 1]               = "globally registered plugin slot number";
    plugin_stateadr: [i32; nplugin * 1]      = "address in the plugin state array";
    plugin_statenum: [i32; nplugin * 1]      = "number of states in the plugin instance";
    plugin_attr: [i8; npluginattr * 1]          = "config attributes of plugin instances";
    plugin_attradr: [i32; nplugin * 1]       = "address to each instance's config attrib";

    // custom numeric fields
    numeric_adr: [i32; nnumeric * 1]          = "address of field in numeric_data";
    numeric_size: [i32; nnumeric * 1]         = "size of numeric field";
    numeric_data: [f64; nnumericdata * 1]         = "array of all numeric fields";

    // custom text fields
    text_adr: [i32; ntext * 1]             = "address of text in text_data";
    text_size: [i32; ntext * 1]            = "size of text field (strlen+1)";
    text_data: [i8; ntextdata * 1]            = "array of all text fields (0-terminated)";

    // custom tuple fields
    tuple_adr: [i32; ntuple * 1]            = "address of text in text_data";
    tuple_size: [i32; ntuple * 1]           = "number of objects in tuple";
    tuple_objtype: [i32; ntupledata * 1]        = "array of object types in all tuples";
    tuple_objid: [i32; ntupledata * 1]          = "array of object ids in all tuples";
    tuple_objprm: [f64; ntupledata * 1]         = "array of object params in all tuples";

    // keyframes
    key_time: [f64; nkey * 1]             = "key time";
    key_qpos: [f64; nkey x nq]             = "key position";
    key_qvel: [f64; nkey x nv]             = "key velocity";
    key_act: [f64; nkey x na]              = "key activation";
    key_mpos: [f64; nmocap*3]             = "key mocap position                       (nkey ";
    key_mquat: [f64; nmocap*4]            = "key mocap quaternion                     (nkey ";
    key_ctrl: [f64; nkey x nu]             = "key control";

    // names
    name_bodyadr: [i32; nbody * 1]         = "body name pointers";
    name_jntadr: [i32; njnt * 1]          = "joint name pointers";
    name_geomadr: [i32; ngeom * 1]         = "geom name pointers";
    name_siteadr: [i32; nsite * 1]         = "site name pointers";
    name_camadr: [i32; ncam * 1]          = "camera name pointers";
    name_lightadr: [i32; nlight * 1]        = "light name pointers";
    name_flexadr: [i32; nflex * 1]         = "flex name pointers";
    name_meshadr: [i32; nmesh * 1]         = "mesh name pointers";
    name_skinadr: [i32; nskin * 1]         = "skin name pointers";
    name_hfieldadr: [i32; nhfield * 1]       = "hfield name pointers";
    name_texadr: [i32; ntex * 1]          = "texture name pointers";
    name_matadr: [i32; nmat * 1]          = "material name pointers";
    name_pairadr: [i32; npair * 1]         = "geom pair name pointers";
    name_excludeadr: [i32; nexclude * 1]      = "exclude name pointers";
    name_eqadr: [i32; neq * 1]           = "equality constraint name pointers";
    name_tendonadr: [i32; ntendon * 1]       = "tendon name pointers";
    name_actuatoradr: [i32; nu * 1]     = "actuator name pointers";
    name_sensoradr: [i32; nsensor * 1]       = "sensor name pointers";
    name_numericadr: [i32; nnumeric * 1]      = "numeric name pointers";
    name_textadr: [i32; ntext * 1]         = "text name pointers";
    name_tupleadr: [i32; ntuple * 1]        = "tuple name pointers";
    name_keyadr: [i32; nkey * 1]          = "keyframe name pointers";
    name_pluginadr: [i32; nplugin * 1]       = "plugin instance name pointers";
    names: [i8; nnames * 1]                = "names of all objects, 0-terminated";
    names_map: [i32; nnames_map * 1]            = "internal hash map of names";

    // paths
    paths: [i8; npaths * 1]                = "paths to assets, 0-terminated";
}

impl MjModel {
    pub fn signature(&self) -> u64 {
        self.0.signature
    }
}
