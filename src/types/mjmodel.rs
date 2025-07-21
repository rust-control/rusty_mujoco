//! # [mjModel](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#mjmodel)
//! 
//! This is the main data structure holding the MuJoCo model. It is treated as constant by the simulator.

pub use crate::bindgen::{mjModel, mjOption, mjVisual, mjStatistic};

derive_fields_mapping!(mjModel {
    scalars {
        // sizes needed at mjModel construction
        nq: usize = "number of generalized coordinates = dim(qpos)";
        nv: usize = "number of degrees of freedom = dim(qvel)";
        nu: usize = "number of actuators/controls = dim(ctrl)";
        na: usize = "number of activation states = dim(act)";
        nbody: usize = "number of bodies";
        nbvh: usize = "number of total bounding volumes in all bodies";
        nbvhstatic: usize = "number of static bounding volumes (aabb stored in mjModel)";
        nbvhdynamic: usize = "number of dynamic bounding volumes (aabb stored in mjData)";
        njnt: usize = "number of joints";
        ngeom: usize = "number of geoms";
        nsite: usize = "number of sites";
        ncam: usize = "number of cameras";
        nlight: usize = "number of lights";
        nflex: usize = "number of flexes";
        nflexnode: usize = "number of dofs in all flexes";
        nflexvert: usize = "number of vertices in all flexes";
        nflexedge: usize = "number of edges in all flexes";
        nflexelem: usize = "number of elements in all flexes";
        nflexelemdata: usize = "number of element vertex ids in all flexes";
        nflexelemedge: usize = "number of element edge ids in all flexes";
        nflexshelldata: usize = "number of shell fragment vertex ids in all flexes";
        nflexevpair: usize = "number of element-vertex pairs in all flexes";
        nflextexcoord: usize = "number of vertices with texture coordinates";
        nmesh: usize = "number of meshes";
        nmeshvert: usize = "number of vertices in all meshes";
        nmeshnormal: usize = "number of normals in all meshes";
        nmeshtexcoord: usize = "number of texcoords in all meshes";
        nmeshface: usize = "number of triangular faces in all meshes";
        nmeshgraph: usize = "number of ints in mesh auxiliary data";
        nmeshpoly: usize = "number of polygons in all meshes";
        nmeshpolyvert: usize = "number of vertices in all polygons";
        nmeshpolymap: usize = "number of polygons in vertex map";
        nskin: usize = "number of skins";
        nskinvert: usize = "number of vertices in all skins";
        nskintexvert: usize = "number of vertiex with texcoords in all skins";
        nskinface: usize = "number of triangular faces in all skins";
        nskinbone: usize = "number of bones in all skins";
        nskinbonevert: usize = "number of vertices in all skin bones";
        nhfield: usize = "number of heightfields";
        nhfielddata: usize = "number of data points in all heightfields";
        ntex: usize = "number of textures";
        ntexdata: usize = "number of bytes in texture rgb data";
        nmat: usize = "number of materials";
        npair: usize = "number of predefined geom pairs";
        nexclude: usize = "number of excluded geom pairs";
        neq: usize = "number of equality constraints";
        ntendon: usize = "number of tendons";
        nwrap: usize = "number of wrap objects in all tendon paths";
        nsensor: usize = "number of sensors";
        nnumeric: usize = "number of numeric custom fields";
        nnumericdata: usize = "number of mjtNums in all numeric fields";
        ntext: usize = "number of text custom fields";
        ntextdata: usize = "number of mjtBytes in all text fields";
        ntuple: usize = "number of tuple custom fields";
        ntupledata: usize = "number of objects in all tuple fields";
        nkey: usize = "number of keyframes";
        nmocap: usize = "number of mocap bodies";
        nplugin: usize = "number of plugin instances";
        npluginattr: usize = "number of chars in all plugin config attributes";
        nuser_body: usize = "number of mjtNums in body_user";
        nuser_jnt: usize = "number of mjtNums in jnt_user";
        nuser_geom: usize = "number of mjtNums in geom_user";
        nuser_site: usize = "number of mjtNums in site_user";
        nuser_cam: usize = "number of mjtNums in cam_user";
        nuser_tendon: usize = "number of mjtNums in tendon_user";
        nuser_actuator: usize = "number of mjtNums in actuator_user";
        nuser_sensor: usize = "number of mjtNums in sensor_user";
        nnames: usize = "number of chars in all names";
        npaths: usize = "number of chars in all paths";

        // sizes set after mjModel construction
        nnames_map: usize = "number of slots in the names hash map";
        nM: usize = "number of non-zeros in sparse inertia matrix";
        nB: usize = "number of non-zeros in sparse body-dof matrix";
        nC: usize = "number of non-zeros in sparse reduced dof-dof matrix";
        nD: usize = "number of non-zeros in sparse dof-dof matrix";
        nJmom: usize = "number of non-zeros in sparse actuator_moment matrix";
        ntree: usize = "number of kinematic trees under world body";
        ngravcomp: usize = "number of bodies with nonzero gravcomp";
        nemax: usize = "number of potential equality-constraint rows";
        njmax: usize = "number of available rows in constraint Jacobian (legacy)";
        nconmax: usize = "number of potential contacts in contact list (legacy)";
        nuserdata: usize = "number of mjtNums reserved for the user";
        nsensordata: usize = "number of mjtNums in sensor data vector";
        npluginstate: usize = "number of mjtNums in plugin state vector";

        narena: usize = "number of bytes in the mjData arena (inclusive of stack)";
        nbuffer: usize = "number of bytes in buffer";

        signature: u64 = "model signature, used to detect changes in model";
    }
    structs {
        opt: mjOption = "physics options\n\n**note**: _read-only_. To dynamically mutate this, consider building `MjModel` via `MjSpec`.";
        vis: mjVisual = "visualization options\n\n**note**: _read-only_. To dynamically mutate this, consider building `MjModel` via `MjSpec`.";
        stat: mjStatistic = "model statistics\n\n**note**: _read-only_. To dynamically mutate this, consider building `MjModel` via `MjSpec`.";
    }
});

pub use crate::bindgen::{
    mjtSameFrame, mjtJoint, mjtGeom, mjtCamLight, mjtFlexSelf, mjtTexture, mjtEq, mjtObj, mjtWrap, mjtTrn, mjtDyn, mjtGain, mjtBias, mjtSensor, mjtDataType, mjtStage,
    mjNREF, mjNIMP, mjNFLUID, mjNTEXROLE, mjNEQDATA, mjNDYN, mjNGAIN, mjNBIAS,
    mjMAXVAL, mjMINVAL
};
use crate::{
    helper::Rgba,
    ObjectId, Obj, obj, JointObjectId, Joint, NodeId, VertexId, BoneId, ElementId, ElementDataId, EdgeId, EdgeDataId, ShellDataId, EvPairId, TexcoordId, FaceId, NormalId, TexDataId, HFieldDataId, BoneVertexId,
};
use std::{
    slice::from_raw_parts as slice,
    array::from_fn as array,
};

impl mjModel {
    pub fn object_id<O: Obj>(&self, name: &str) -> Option<ObjectId<O>> {
        crate::mj_name2id::<O>(self, name)
    }
    
    pub fn joint_object_id<J: Joint>(&self, name: &str) -> Option<JointObjectId<J>> {
        let object_id = self.object_id::<obj::Joint>(name)?;
        object_id.as_joint::<J>(self)
    }
}

#[allow(non_snake_case)]
impl mjModel {
    /// qpos values at default pose
    pub fn qpos0<J: Joint>(&self, joint_id: JointObjectId<J>) -> J::Qpos {
        unsafe {
            let ptr = self.qpos0.add(self.jnt_qposadr.add(joint_id.index()).read() as usize);
            J::Qpos::try_from(slice(ptr, J::QPOS_SIZE)).ok().unwrap()
        }
    }    
    /// reference pose for springs
    pub fn qpos_spring<J: Joint>(&self, joint_id: JointObjectId<J>) -> J::Qpos {
        unsafe {
            let ptr = self.qpos_spring.add(self.jnt_qposadr.add(joint_id.index()).read() as usize);
            J::Qpos::try_from(slice(ptr, J::QPOS_SIZE)).ok().unwrap()
        }
    }
    
    /// id of body's parent
    pub fn body_parentid(&self, id: ObjectId<obj::Body>) -> Option<ObjectId<obj::Body>> {
        let index = (unsafe { self.body_parentid.add(id.index()).read() }).try_into().ok()?;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// id of root above body
    pub fn body_rootid(&self, id: ObjectId<obj::Body>) -> ObjectId<obj::Body> {
        let index = (unsafe { self.body_rootid.add(id.index()).read() }) as usize;
        unsafe { ObjectId::new_unchecked(index) }
    }
    /// id of body that this body is welded to
    pub fn body_weldid(&self, id: ObjectId<obj::Body>) -> ObjectId<obj::Body> {
        let index = (unsafe { self.body_weldid.add(id.index()).read() }) as usize;
        unsafe { ObjectId::new_unchecked(index) }
    }
    /// id of mocap data
    pub fn body_mocapid(&self, id: ObjectId<obj::Body>) -> Option<ObjectId<obj::Body>> {
        let index = (unsafe { self.body_mocapid.add(id.index()).read() }).try_into().ok()?;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// number of joints for this body
    pub fn body_jntnum(&self, id: ObjectId<obj::Body>) -> usize {
        (unsafe { self.body_jntnum.add(id.index()).read() }) as usize
    }
    /// start addr of joints
    pub fn body_jntadr(&self, id: ObjectId<obj::Body>) -> Option<usize> {
        (unsafe { self.body_jntadr.add(id.index()).read() }).try_into().ok()
    }
    /// number of motion degrees of freedom
    pub fn body_dofnum(&self, id: ObjectId<obj::Body>) -> usize {
        (unsafe { self.body_dofnum.add(id.index()).read() }) as usize
    }
    /// start addr of dofs
    pub fn body_dofadr(&self, id: ObjectId<obj::Body>) -> Option<usize> {
        (unsafe { self.body_dofadr.add(id.index()).read() }).try_into().ok()
    }
    /// id of body's kinematic tree
    pub fn body_treeid(&self, id: ObjectId<obj::Body>) -> Option<ObjectId<obj::Body>> {
        let index = (unsafe { self.body_treeid.add(id.index()).read() }).try_into().ok()?;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// number of geoms
    pub fn body_geomnum(&self, id: ObjectId<obj::Body>) -> usize {
        (unsafe { self.body_geomnum.add(id.index()).read() }) as usize
    }
    /// start addr of geoms
    pub fn body_geomadr(&self, id: ObjectId<obj::Body>) -> Option<usize> {
        (unsafe { self.body_geomadr.add(id.index()).read() }).try_into().ok()
    }
    /// body_simple: 1: diag M; 2: diag M, sliders only
    pub fn body_simple(&self, id: ObjectId<obj::Body>) -> u8 {
        unsafe { self.body_simple.add(id.index()).read() }
    }
    /// same frame as inertia
    pub fn body_sameframe(&self, id: ObjectId<obj::Body>) -> mjtSameFrame {
        mjtSameFrame((unsafe { self.body_sameframe.add(id.index()).read() }) as u32)
    }
    /// position offset relative to parent body
    pub fn body_pos(&self, id: ObjectId<obj::Body>) -> [f64; 3] {
        unsafe {
            let ptr = self.body_pos.add(id.index() * 3);
            array(|i| ptr.add(i).read())
        }
    }
    /// orientation offset relative to parent body
    pub fn body_quat(&self, id: ObjectId<obj::Body>) -> [f64; 4] {
        unsafe {
            let ptr = self.body_quat.add(id.index() * 4);
            array(|i| ptr.add(i).read())
        }
    }
    /// local position of center of mass
    pub fn body_ipos(&self, id: ObjectId<obj::Body>) -> [f64; 3] {
        unsafe {
            let ptr = self.body_ipos.add(id.index() * 3);
            array(|i| ptr.add(i).read())
        }
    }
    /// local orientation of inertia ellipsoid
    pub fn body_iquat(&self, id: ObjectId<obj::Body>) -> [f64; 4] {
        unsafe {
            let ptr = self.body_iquat.add(id.index() * 4);
            array(|i| ptr.add(i).read())            
        }
    }
    /// mass
    pub fn body_mass(&self, id: ObjectId<obj::Body>) -> f64 {
        unsafe { self.body_mass.add(id.index()).read() }
    }
    /// mass of subtree starting at this body
    pub fn body_subtreemass(&self, id: ObjectId<obj::Body>) -> f64 {
        unsafe { self.body_subtreemass.add(id.index()).read() }
    }
    /// diagonal inertia in ipos/iquat frame
    pub fn body_inertia(&self, id: ObjectId<obj::Body>) -> [f64; 3] {
        unsafe {
            let ptr = self.body_inertia.add(id.index() * 3);
            array(|i| ptr.add(i).read())
        }
    }
    /// mean inverse inertia in qpos0 (translation, rotation)
    pub fn body_invweight0(&self, id: ObjectId<obj::Body>) -> (f64, f64) {
        unsafe {
            let ptr = self.body_invweight0.add(id.index() * 2);
            (ptr.read(), ptr.add(1).read())
        }
    }
    /// antigravity force, units of body weight
    pub fn body_gravcomp(&self, id: ObjectId<obj::Body>) -> f64 {
        unsafe { self.body_gravcomp.add(id.index()).read() }
    }
    /// maximum over all geom margins
    pub fn body_margin(&self, id: ObjectId<obj::Body>) -> f64 {
        unsafe { self.body_margin.add(id.index()).read() }
    }
    /// plugin instance id
    pub fn body_plugin(&self, id: ObjectId<obj::Body>) -> Option<ObjectId<obj::Plugin>> {
        let index = (unsafe { self.body_plugin.add(id.index()).read() }).try_into().ok()?;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// bit-wise OR over all geom contypes
    pub fn body_contype(&self, id: ObjectId<obj::Body>) -> i32 {
        unsafe { self.body_contype.add(id.index()).read() }
    }
    /// bit-wise OR over all geom conaffinities
    pub fn body_conaffinity(&self, id: ObjectId<obj::Body>) -> i32 {
        unsafe { self.body_conaffinity.add(id.index()).read() }
    }
    /// address of bounding volume hierarchy root
    pub fn body_bvhadr(&self, id: ObjectId<obj::Body>) -> Option<usize> {
        (unsafe { self.body_bvhadr.add(id.index()).read() }).try_into().ok()
    }
    /// number of bounding volumes
    pub fn body_bvhnum(&self, id: ObjectId<obj::Body>) -> Option<usize> {
        (unsafe { self.body_bvhnum.add(id.index()).read() }).try_into().ok()
    }
    
    /// depth in the bounding volume hierarchy
    pub fn bvh_depth(&self, id: ObjectId<obj::Geom>) -> usize {
        (unsafe { self.bvh_depth.add(id.index()).read() }) as usize
    }
    /// left and right children in tree
    pub fn bvh_child(&self, id: ObjectId<obj::Geom>) -> (Option<usize>, Option<usize>) {
        unsafe {
            let ptr = self.bvh_child.add(id.index() * 2);
            (
                ptr.read().try_into().ok(),
                ptr.add(1).read().try_into().ok()
            )
        }
    }
    /// geom or elem id of node
    pub fn bvh_nodeid(&self, id: ObjectId<obj::Geom>) -> Option<ObjectId<obj::Geom>> {
        let index = (unsafe { self.bvh_nodeid.add(id.index()).read() }).try_into().ok()?;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// local bounding box (center, size)
    pub fn bvh_aabb(&self, id: ObjectId<obj::Geom>) -> ([f64; 3], [f64; 3]) {
        unsafe {
            let ptr = self.bvh_aabb.add(id.index() * 6);
            (array(|i| ptr.add(i).read()), array(|i| ptr.add(i + 3).read()))
        }
    }

    /// type of joint
    pub fn jnt_type(&self, id: ObjectId<obj::Joint>) -> mjtJoint {
        mjtJoint((unsafe { self.jnt_type.add(id.index()).read() }) as u32)
    }
    /// start addr in 'qpos' for joint's data
    pub fn jnt_qposadr(&self, id: ObjectId<obj::Joint>) -> usize {
        (unsafe { self.jnt_qposadr.add(id.index()).read() }) as usize
    }
    /// start addr in 'qvel' for joint's data
    pub fn jnt_dofadr(&self, id: ObjectId<obj::Joint>) -> Option<usize> {
        (unsafe { self.jnt_dofadr.add(id.index()).read() }).try_into().ok()
    }
    /// id of joint's body
    pub fn jnt_bodyid(&self, id: ObjectId<obj::Joint>) -> ObjectId<obj::Body> {
        let index = (unsafe { self.jnt_bodyid.add(id.index()).read() }) as usize;
        unsafe { ObjectId::new_unchecked(index) }
    }
    /// group for visibility
    pub fn jnt_group(&self, id: ObjectId<obj::Joint>) -> i32 {
        unsafe { self.jnt_group.add(id.index()).read() }
    }
    /// does joint have limits
    pub fn jnt_limited(&self, id: ObjectId<obj::Joint>) -> bool {
        (unsafe { self.jnt_limited.add(id.index()).read() }) != 0
    }
    /// does joint have actuator force limits
    pub fn jnt_actfrclimited(&self, id: ObjectId<obj::Joint>) -> bool {
        (unsafe { self.jnt_actfrclimited.add(id.index()).read() }) != 0
    }
    /// is gravcomp force applied via actuators
    pub fn jnt_actgravcomp(&self, id: ObjectId<obj::Joint>) -> bool {
        (unsafe { self.jnt_actgravcomp.add(id.index()).read() }) != 0
    }
    /// constraint solver reference: limit
    pub fn jnt_solref(&self, id: ObjectId<obj::Joint>) -> [f64; mjNREF] {
        unsafe {
            let ptr = self.jnt_solref.add(id.index() * mjNREF);
            array(|i| ptr.add(i).read())
        }
    }
    /// constraint solver impedance: limit
    pub fn jnt_solimp(&self, id: ObjectId<obj::Joint>) -> [f64; mjNIMP] {
        unsafe {
            let ptr = self.jnt_solimp.add(id.index() * mjNIMP);
            array(|i| ptr.add(i).read())
        }
    }
    /// local anchor position
    pub fn jnt_pos(&self, id: ObjectId<obj::Joint>) -> [f64; 3] {
        unsafe {
            let ptr = self.jnt_pos.add(id.index() * 3);
            array(|i| ptr.add(i).read())
        }
    }
    /// local joint axis
    pub fn jnt_axis(&self, id: ObjectId<obj::Joint>) -> [f64; 3] {
        unsafe {
            let ptr = self.jnt_axis.add(id.index() * 3);
            array(|i| ptr.add(i).read())
        }
    }
    /// stiffness coefficient
    pub fn jnt_stiffness(&self, id: ObjectId<obj::Joint>) -> f64 {
        unsafe { self.jnt_stiffness.add(id.index()).read() }
    }
    /// joint limits
    pub fn jnt_range(&self, id: ObjectId<obj::Joint>) -> std::ops::Range<f64> {
        unsafe {
            let ptr = self.jnt_range.add(id.index() * 2);
            ptr.read()..ptr.add(1).read()
        }
    }
    /// range of total actuator force
    pub fn jnt_actfrcrange(&self, id: ObjectId<obj::Joint>) -> std::ops::Range<f64> {
        unsafe {
            let ptr = self.jnt_actfrcrange.add(id.index() * 2);
            ptr.read()..ptr.add(1).read()
        }
    }
    /// min distance for limit detection
    pub fn jnt_margin(&self, id: ObjectId<obj::Joint>) -> f64 {
        unsafe { self.jnt_margin.add(id.index()).read() }
    }

    /// id of dof's body
    pub fn dof_bodyid(&self, id: ObjectId<obj::Dof>) -> ObjectId<obj::Body> {
        let index = (unsafe { self.dof_bodyid.add(id.index()).read() }) as usize;
        unsafe { ObjectId::new_unchecked(index) }
    }
    /// id of dof's joint
    pub fn dof_jntid(&self, id: ObjectId<obj::Dof>) -> ObjectId<obj::Joint> {
        let index = (unsafe { self.dof_jntid.add(id.index()).read() }) as usize;
        unsafe { ObjectId::new_unchecked(index) }
    }
    /// id of dof's parent
    pub fn dof_parentid(&self, id: ObjectId<obj::Dof>) -> Option<ObjectId<obj::Dof>> {
        let index = (unsafe { self.dof_parentid.add(id.index()).read() }).try_into().ok()?;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// id of dof's kinematic tree
    pub fn dof_treeid(&self, id: ObjectId<obj::Dof>) -> usize {
        (unsafe { self.dof_treeid.add(id.index()).read() }) as usize
    }
    /// dof address in M-diagonal
    pub fn dof_Madr(&self, id: ObjectId<obj::Dof>) -> usize {
        (unsafe { self.dof_Madr.add(id.index()).read() }) as usize
    }
    /// number of consecutive simple dofs
    pub fn dof_simplenum(&self, id: ObjectId<obj::Dof>) -> usize {
        (unsafe { self.dof_simplenum.add(id.index()).read() }) as usize
    }
    /// constraint solver reference: friction loss
    pub fn dof_solref(&self, id: ObjectId<obj::Dof>) -> [f64; mjNREF] {
        unsafe {
            let ptr = self.dof_solref.add(id.index() * mjNREF);
            array(|i| ptr.add(i).read())
        }
    }
    /// constraint solver impedance: friction loss
    pub fn dof_solimp(&self, id: ObjectId<obj::Dof>) -> [f64; mjNIMP] {
        unsafe {
            let ptr = self.dof_solimp.add(id.index() * mjNIMP);
            array(|i| ptr.add(i).read())
        }
    }
    /// dof friction loss
    pub fn dof_frictionloss(&self, id: ObjectId<obj::Dof>) -> f64 {
        unsafe { self.dof_frictionloss.add(id.index()).read() }
    }
    /// dof armature inertia/mass
    pub fn dof_armature(&self, id: ObjectId<obj::Dof>) -> f64 {
        unsafe { self.dof_armature.add(id.index()).read() }
    }
    /// damping coefficient
    pub fn dof_damping(&self, id: ObjectId<obj::Dof>) -> f64 {
        unsafe { self.dof_damping.add(id.index()).read() }
    }
    /// diagonal inverse inertia in qpos0
    pub fn dof_invweight0(&self, id: ObjectId<obj::Dof>) -> f64 {
        unsafe { self.dof_invweight0.add(id.index()).read() }
    }
    /// diagonal inertia in qpos0
    pub fn dof_M0(&self, id: ObjectId<obj::Dof>) -> f64 {
        unsafe { self.dof_M0.add(id.index()).read() }
    }

    /// geometric type (mjtGeom)
    pub fn geom_type(&self, id: ObjectId<obj::Geom>) -> mjtGeom {
        mjtGeom((unsafe { self.geom_type.add(id.index()).read() }) as u32)
    }
    /// geom contact type
    pub fn geom_contype(&self, id: ObjectId<obj::Geom>) -> i32 {
        unsafe { self.geom_contype.add(id.index()).read() }
    }
    /// geom contact affinity
    pub fn geom_conaffinity(&self, id: ObjectId<obj::Geom>) -> i32 {
        unsafe { self.geom_conaffinity.add(id.index()).read() }
    }
    /// contact dimensionality (1, 3, 4, 6)
    pub fn geom_condim(&self, id: ObjectId<obj::Geom>) -> usize {
        (unsafe { self.geom_condim.add(id.index()).read() }) as usize
    }
    /// id of geom's body
    pub fn geom_bodyid(&self, id: ObjectId<obj::Geom>) -> ObjectId<obj::Body> {
        let index = (unsafe { self.geom_bodyid.add(id.index()).read() }) as usize;
        unsafe { ObjectId::new_unchecked(index) }
    }
    /// id of geom's mesh
    pub fn geom_dataid_mesh(&self, id: ObjectId<obj::Geom>) -> Option<ObjectId<obj::Mesh>> {
        if self.geom_type(id) != mjtGeom::MESH {return None}
        let index = (unsafe { self.geom_dataid.add(id.index()).read() }).try_into().ok()?;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// id of geom's hfield
    pub fn geom_dataid_hfield(&self, id: ObjectId<obj::Geom>) -> Option<ObjectId<obj::HField>> {
        if self.geom_type(id) != mjtGeom::HFIELD {return None}
        let index = (unsafe { self.geom_dataid.add(id.index()).read() }).try_into().ok()?;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// material id for rendering
    pub fn geom_matid(&self, id: ObjectId<obj::Geom>) -> Option<ObjectId<obj::Material>> {
        let index = (unsafe { self.geom_matid.add(id.index()).read() }).try_into().ok()?;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// group for visibility
    pub fn geom_group(&self, id: ObjectId<obj::Geom>) -> i32 {
        unsafe { self.geom_group.add(id.index()).read() }
    }
    /// geom contact priority
    pub fn geom_priority(&self, id: ObjectId<obj::Geom>) -> i32 {
        unsafe { self.geom_priority.add(id.index()).read() }
    }
    /// plugin instance id
    pub fn geom_plugin(&self, id: ObjectId<obj::Geom>) -> Option<ObjectId<obj::Plugin>> {
        let index = (unsafe { self.geom_plugin.add(id.index()).read() }).try_into().ok()?;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// same frame as body (mjtSameframe)
    pub fn geom_sameframe(&self, id: ObjectId<obj::Geom>) -> mjtSameFrame {
        mjtSameFrame((unsafe { self.geom_sameframe.add(id.index()).read() }) as u32)
    }
    /// mixing coef for solref/imp in geom pair
    pub fn geom_solmix(&self, id: ObjectId<obj::Geom>) -> f64 {
        unsafe { self.geom_solmix.add(id.index()).read() }
    }
    /// constraint solver reference: contact
    pub fn geom_solref(&self, id: ObjectId<obj::Geom>) -> [f64; mjNREF] {
        unsafe {
            let ptr = self.geom_solref.add(id.index() * mjNREF);
            array(|i| ptr.add(i).read())
        }
    }
    /// constraint solver impedance: contact
    pub fn geom_solimp(&self, id: ObjectId<obj::Geom>) -> [f64; mjNIMP] {
        unsafe {
            let ptr = self.geom_solimp.add(id.index() * mjNIMP);
            array(|i| ptr.add(i).read())
        }
    }
    /// geom-specific size parameters
    pub fn geom_size(&self, id: ObjectId<obj::Geom>) -> [f64; 3] {
        unsafe {
            let ptr = self.geom_size.add(id.index() * 3);
            array(|i| ptr.add(i).read())
        }
    }
    /// bounding box, (center, size)
    pub fn geom_aabb(&self, id: ObjectId<obj::Geom>) -> ([f64; 3], [f64; 3]) {
        unsafe {
            let ptr = self.geom_aabb.add(id.index() * 6);
            (array(|i| ptr.add(i).read()), array(|i| ptr.add(i + 3).read()))
        }
    }
    /// radius of bounding sphere
    pub fn geom_rbound(&self, id: ObjectId<obj::Geom>) -> f64 {
        unsafe { self.geom_rbound.add(id.index()).read() }
    }
    /// local position offset relative to body
    pub fn geom_pos(&self, id: ObjectId<obj::Geom>) -> [f64; 3] {
        unsafe {
            let ptr = self.geom_pos.add(id.index() * 3);
            array(|i| ptr.add(i).read())
        }
    }
    /// local orientation offset relative to body
    pub fn geom_quat(&self, id: ObjectId<obj::Geom>) -> [f64; 4] {
        unsafe {
            let ptr = self.geom_quat.add(id.index() * 4);
            array(|i| ptr.add(i).read())
        }
    }
    /// friction for (slide, spin, roll)
    pub fn geom_friction(&self, id: ObjectId<obj::Geom>) -> [f64; 3] {
        unsafe {
            let ptr = self.geom_friction.add(id.index() * 3);
            array(|i| ptr.add(i).read())
        }
    }
    /// detect contact if dist < margin
    pub fn geom_margin(&self, id: ObjectId<obj::Geom>) -> f64 {
        unsafe { self.geom_margin.add(id.index()).read() }
    }
    /// include in solver if dist < margin - gap
    pub fn geom_gap(&self, id: ObjectId<obj::Geom>) -> f64 {
        unsafe { self.geom_gap.add(id.index()).read() }
    }
    /// fluid interaction parameters
    pub fn geom_fluid(&self, id: ObjectId<obj::Geom>) -> [f64; mjNFLUID] {
        unsafe {
            let ptr = self.geom_fluid.add(id.index() * mjNFLUID);
            array(|i| ptr.add(i).read())
        }
    }
    /// rgba when material is omitted
    pub fn geom_rgba(&self, id: ObjectId<obj::Geom>) -> Rgba {
        unsafe {
            let ptr = self.geom_rgba.add(id.index() * 4);
            Rgba { r: ptr.read(), g: ptr.add(1).read(), b: ptr.add(2).read(), a: ptr.add(3).read() }
        }
    }

    /// geom type for rendering
    pub fn site_type(&self, id: ObjectId<obj::Site>) -> mjtGeom {
        mjtGeom((unsafe { self.site_type.add(id.index()).read() }) as u32)
    }
    /// id of site's body
    pub fn site_bodyid(&self, id: ObjectId<obj::Site>) -> ObjectId<obj::Body> {
        let index = (unsafe { self.site_bodyid.add(id.index()).read() }) as usize;
        unsafe { ObjectId::new_unchecked(index) }
    }
    /// material id for rendering
    pub fn site_matid(&self, id: ObjectId<obj::Site>) -> Option<ObjectId<obj::Material>> {
        let index = (unsafe { self.site_matid.add(id.index()).read() }).try_into().ok()?;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// group for visibility
    pub fn site_group(&self, id: ObjectId<obj::Site>) -> i32 {
        unsafe { self.site_group.add(id.index()).read() }
    }
    /// same frame as body
    pub fn site_sameframe(&self, id: ObjectId<obj::Site>) -> mjtSameFrame {
        mjtSameFrame((unsafe { self.site_sameframe.add(id.index()).read() }) as u32)
    }
    /// geom size for rendering
    pub fn site_size(&self, id: ObjectId<obj::Site>) -> [f64; 3] {
        unsafe {
            let ptr = self.site_size.add(id.index() * 3);
            array(|i| ptr.add(i).read())
        }
    }
    /// local position offset rel. to body
    pub fn site_pos(&self, id: ObjectId<obj::Site>) -> [f64; 3] {
        unsafe {
            let ptr = self.site_pos.add(id.index() * 3);
            array(|i| ptr.add(i).read())
        }
    }
    /// local orientation offset rel. to body
    pub fn site_quat(&self, id: ObjectId<obj::Site>) -> [f64; 4] {
        unsafe {
            let ptr = self.site_quat.add(id.index() * 4);
            array(|i| ptr.add(i).read())
        }
    }
    /// rgba when material is omitted
    pub fn site_rgba(&self, id: ObjectId<obj::Site>) -> Rgba {
        unsafe {
            let ptr = self.site_rgba.add(id.index() * 4);
            Rgba { r: ptr.read(), g: ptr.add(1).read(), b: ptr.add(2).read(), a: ptr.add(3).read() }
        }
    }

    /// camera tracking mode (mjtCamLight)
    pub fn cam_mode(&self, id: ObjectId<obj::Camera>) -> mjtCamLight {
        mjtCamLight((unsafe { self.cam_mode.add(id.index()).read() } as u32))
    }
    /// id of camera's body
    pub fn cam_bodyid(&self, id: ObjectId<obj::Camera>) -> ObjectId<obj::Body> {
        let index = (unsafe { self.cam_bodyid.add(id.index()).read() }) as usize;
        unsafe { ObjectId::new_unchecked(index) }
    }
    /// id of targeted body
    pub fn cam_targetbodyid(&self, id: ObjectId<obj::Camera>) -> Option<ObjectId<obj::Body>> {
        let index = (unsafe { self.cam_targetbodyid.add(id.index()).read() }).try_into().ok()?;
        unsafe { Some(ObjectId::new_unchecked(index)) }
    }
    /// does camera cast shadows
    pub fn cam_pos(&self, id: ObjectId<obj::Camera>) -> [f64; 3] {
        unsafe {
            let ptr = self.cam_pos.add(id.index() * 3);
            array(|i| ptr.add(i).read())
        }
    }
    /// orientation relative to body frame
    pub fn cam_quat(&self, id: ObjectId<obj::Camera>) -> [f64; 4] {
        unsafe {
            let ptr = self.cam_quat.add(id.index() * 4);
            array(|i| ptr.add(i).read())
        }
    }
    /// global position relative to sub-com in qpos0
    pub fn cam_poscom0(&self, id: ObjectId<obj::Camera>) -> [f64; 3] {
        unsafe {
            let ptr = self.cam_poscom0.add(id.index() * 3);
            array(|i| ptr.add(i).read())
        }
    }
    /// global position relative to body in qpos0
    pub fn cam_pos0(&self, id: ObjectId<obj::Camera>) -> [f64; 3] {
        unsafe {
            let ptr = self.cam_pos0.add(id.index() * 3);
            array(|i| ptr.add(i).read())
        }
    }
    /// global orientation in qpos0
    pub fn cam_mat0(&self, id: ObjectId<obj::Camera>) -> [[f64; 3]; 3] {
        unsafe {
            let ptr = self.cam_mat0.add(id.index() * 9);
            [
                [ptr.read(), ptr.add(1).read(), ptr.add(2).read()],
                [ptr.add(3).read(), ptr.add(4).read(), ptr.add(5).read()],
                [ptr.add(6).read(), ptr.add(7).read(), ptr.add(8).read()]
            ]
        }
    }
    /// orthographic camera
    pub fn cam_orthographic(&self, id: ObjectId<obj::Camera>) -> bool {
        (unsafe { self.cam_orthographic.add(id.index()).read() }) != 0
    }
    /// y field-of-view (ortho ? len : deg)
    pub fn cam_fovy(&self, id: ObjectId<obj::Camera>) -> f64 {
        unsafe { self.cam_fovy.add(id.index()).read() }
    }
    /// inter-pupilary distance
    pub fn cam_ipd(&self, id: ObjectId<obj::Camera>) -> f64 {
        unsafe { self.cam_ipd.add(id.index()).read() }
    }
    /// resolution: pixels (width, height)
    pub fn cam_resolution(&self, id: ObjectId<obj::Camera>) -> (usize, usize) {
        unsafe {
            let ptr = self.cam_resolution.add(id.index() * 2);
            (ptr.read() as usize, ptr.add(1).read() as usize)
        }
    }
    /// sensor size: length (width, height)
    pub fn cam_sensorsize(&self, id: ObjectId<obj::Camera>) -> (f32, f32) {
        unsafe {
            let ptr = self.cam_sensorsize.add(id.index() * 2);
            (ptr.read(), ptr.add(1).read())            
        }
    }
    /// [focal length; principal point]
    pub fn cam_intrinsic(&self, id: ObjectId<obj::Camera>) -> [f32; 4] {
        unsafe {
            let ptr = self.cam_intrinsic.add(id.index() * 4);
            array(|i| ptr.add(i).read())
        }
    }

    /// light tracking mode (mjtCamLight)
    pub fn light_mode(&self, id: ObjectId<obj::Light>) -> mjtCamLight {
        mjtCamLight((unsafe { self.light_mode.add(id.index()).read() } as u32))
    }
    /// id of light's body
    pub fn light_bodyid(&self, id: ObjectId<obj::Light>) -> ObjectId<obj::Body> {
        let index = (unsafe { self.light_bodyid.add(id.index()).read() }) as usize;
        unsafe { ObjectId::new_unchecked(index) }
    }
    /// id of targeted body
    pub fn light_targetbodyid(&self, id: ObjectId<obj::Light>) -> Option<ObjectId<obj::Body>> {
        let index = (unsafe { self.light_targetbodyid.add(id.index()).read() }).try_into().ok()?;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// does light cast shadows
    pub fn light_castshadow(&self, id: ObjectId<obj::Light>) -> bool {
        (unsafe { self.light_castshadow.add(id.index()).read() }) != 0
    }
    /// light radius for soft shadows
    pub fn light_bulbradius(&self, id: ObjectId<obj::Light>) -> f32 {
        unsafe { self.light_bulbradius.add(id.index()).read() }
    }
    /// is light on
    pub fn light_active(&self, id: ObjectId<obj::Light>) -> bool {
        (unsafe { self.light_active.add(id.index()).read() }) != 0
    }
    /// position relative to body frame
    pub fn light_pos(&self, id: ObjectId<obj::Light>) -> [f64; 3] {
        unsafe {
            let ptr = self.light_pos.add(id.index() * 3);
            array(|i| ptr.add(i).read())
        }
    }
    /// direction relative to body frame
    pub fn light_dir(&self, id: ObjectId<obj::Light>) -> [f64; 3] {
        unsafe {
            let ptr = self.light_dir.add(id.index() * 3);
            array(|i| ptr.add(i).read())
        }
    }
    /// global position relative to sub-com in qpos0
    pub fn light_poscom0(&self, id: ObjectId<obj::Light>) -> [f64; 3] {
        unsafe {
            let ptr = self.light_poscom0.add(id.index() * 3);
            array(|i| ptr.add(i).read())
        }
    }
    /// global position relative to body in qpos0
    pub fn light_pos0(&self, id: ObjectId<obj::Light>) -> [f64; 3] {
        unsafe {
            let ptr = self.light_pos0.add(id.index() * 3);
            array(|i| ptr.add(i).read())
        }
    }
    /// global direction in qpos0
    pub fn light_dir0(&self, id: ObjectId<obj::Light>) -> [f64; 3] {
        unsafe {
            let ptr = self.light_dir0.add(id.index() * 3);
            array(|i| ptr.add(i).read())
        }
    }
    /// OpenGL attenuation (quadratic model)
    pub fn light_attenuation(&self, id: ObjectId<obj::Light>) -> [f32; 3] {
        unsafe {
            let ptr = self.light_attenuation.add(id.index() * 3);
            array(|i| ptr.add(i).read())
        }
    }
    /// OpenGL cutoff
    pub fn light_cutoff(&self, id: ObjectId<obj::Light>) -> f32 {
        unsafe { self.light_cutoff.add(id.index()).read() }
    }
    /// OpenGL exponent
    pub fn light_exponent(&self, id: ObjectId<obj::Light>) -> f32 {
        unsafe { self.light_exponent.add(id.index()).read() }
    }
    /// ambient rgb (alpha=1)
    pub fn light_ambient(&self, id: ObjectId<obj::Light>) -> Rgba {
        unsafe {
            let ptr = self.light_ambient.add(id.index() * 3);
            Rgba { r: ptr.read(), g: ptr.add(1).read(), b: ptr.add(2).read(), a: 1.0 }
        }
    }
    /// diffuse rgb (alpha=1)
    pub fn light_diffuse(&self, id: ObjectId<obj::Light>) -> Rgba {
        unsafe {
            let ptr = self.light_diffuse.add(id.index() * 4);
            Rgba { r: ptr.read(), g: ptr.add(1).read(), b: ptr.add(2).read(), a: ptr.add(3).read() }
        }
    }
    /// specular rgb (alpha=1)
    pub fn light_specular(&self, id: ObjectId<obj::Light>) -> Rgba {
        unsafe {
            let ptr = self.light_specular.add(id.index() * 4);
            Rgba { r: ptr.read(), g: ptr.add(1).read(), b: ptr.add(2).read(), a: ptr.add(3).read() }
        }
    }

    /// flex contact type
    pub fn flex_contype(&self, id: ObjectId<obj::Flex>) -> i32 {
        unsafe { self.flex_contype.add(id.index()).read() }
    }
    /// flex contact affinity
    pub fn flex_conaffinity(&self, id: ObjectId<obj::Flex>) -> i32 {
        unsafe { self.flex_conaffinity.add(id.index()).read() }
    }
    /// contact dimensionality (1, 3, 4, 6)
    pub fn flex_condim(&self, id: ObjectId<obj::Flex>) -> usize {
        (unsafe { self.flex_condim.add(id.index()).read() }) as usize
    }
    /// flex contact priority
    pub fn flex_priority(&self, id: ObjectId<obj::Flex>) -> i32 {
        unsafe { self.flex_priority.add(id.index()).read() }
    }
    /// mix coef for solref/imp in contact pair
    pub fn flex_solmix(&self, id: ObjectId<obj::Flex>) -> f64 {
        unsafe { self.flex_solmix.add(id.index()).read() }
    }
    /// constraint solver reference: contact
    pub fn flex_solref(&self, id: ObjectId<obj::Flex>) -> [f64; mjNREF] {
        unsafe {
            let ptr = self.flex_solref.add(id.index() * mjNREF);
            array(|i| ptr.add(i).read())
        }
    }
    /// constraint solver impedance: contact
    pub fn flex_solimp(&self, id: ObjectId<obj::Flex>) -> [f64; mjNIMP] {
        unsafe {
            let ptr = self.flex_solimp.add(id.index() * mjNIMP);
            array(|i| ptr.add(i).read())
        }
    }
    /// friction for (slide, spin, roll)
    pub fn flex_friction(&self, id: ObjectId<obj::Flex>) -> [f64; 3] {
        unsafe {
            let ptr = self.flex_friction.add(id.index() * 3);
            array(|i| ptr.add(i).read())
        }
    }
    /// detect contact if dist < margin
    pub fn flex_margin(&self, id: ObjectId<obj::Flex>) -> f64 {
        unsafe { self.flex_margin.add(id.index()).read() }
    }
    /// include in solver if dist < margin - gap
    pub fn flex_gap(&self, id: ObjectId<obj::Flex>) -> f64 {
        unsafe { self.flex_gap.add(id.index()).read() }
    }
    /// internal flex collision enabled
    pub fn flex_internal(&self, id: ObjectId<obj::Flex>) -> bool {
        (unsafe { self.flex_internal.add(id.index()).read() }) != 0
    }
    /// self collision mode (mjtFlexSelf)
    pub fn flex_selfcollide(&self, id: ObjectId<obj::Flex>) -> mjtFlexSelf {
        mjtFlexSelf((unsafe { self.flex_selfcollide.add(id.index()).read() }) as u32)
    }
    /// number of active element layers, 3D only
    pub fn flex_activelayers(&self, id: ObjectId<obj::Flex>) -> usize {
        (unsafe { self.flex_activelayers.add(id.index()).read() }) as usize
    }

    /// 1: lines, 2: triangles, 3: tetrahedra
    pub fn flex_dim(&self, id: ObjectId<obj::Flex>) -> usize {
        (unsafe { self.flex_dim.add(id.index()).read() }) as usize
    }
    /// material id for rendering
    pub fn flex_matid(&self, id: ObjectId<obj::Flex>) -> Option<ObjectId<obj::Material>> {
        let index = (unsafe { self.flex_matid.add(id.index()).read() }).try_into().ok()?;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// group for visibility
    pub fn flex_group(&self, id: ObjectId<obj::Flex>) -> i32 {
        unsafe { self.flex_group.add(id.index()).read() }
    }
    /// interpolation (0: vertex, 1: nodes)
    pub fn flex_interp(&self, id: ObjectId<obj::Flex>) -> i32 {
        unsafe { self.flex_interp.add(id.index()).read() }
    }
    /// first node address
    pub fn flex_nodeadr(&self, id: ObjectId<obj::Flex>) -> Option<usize> {
        (unsafe { self.flex_nodeadr.add(id.index()).read() }).try_into().ok()
    }
    /// number of nodes
    pub fn flex_nodenum(&self, id: ObjectId<obj::Flex>) -> usize {
        (unsafe { self.flex_nodenum.add(id.index()).read() }) as usize
    }
    /// first vertex address
    pub fn flex_vertadr(&self, id: ObjectId<obj::Flex>) -> Option<usize> {
        (unsafe { self.flex_vertadr.add(id.index()).read() }).try_into().ok()
    }
    /// number of vertices
    pub fn flex_vertnum(&self, id: ObjectId<obj::Flex>) -> usize {
        (unsafe { self.flex_vertnum.add(id.index()).read() }) as usize
    }
    /// first edge address
    pub fn flex_edgeadr(&self, id: ObjectId<obj::Flex>) -> Option<usize> {
        (unsafe { self.flex_edgeadr.add(id.index()).read() }).try_into().ok()
    }
    /// number of edges
    pub fn flex_edgenum(&self, id: ObjectId<obj::Flex>) -> usize {
        (unsafe { self.flex_edgenum.add(id.index()).read() }) as usize
    }
    /// first element address
    pub fn flex_elemadr(&self, id: ObjectId<obj::Flex>) -> Option<usize> {
        (unsafe { self.flex_elemadr.add(id.index()).read() }).try_into().ok()
    }
    /// number of elements
    pub fn flex_elemnum(&self, id: ObjectId<obj::Flex>) -> usize {
        (unsafe { self.flex_elemnum.add(id.index()).read() }) as usize
    }
    /// first element vertex id address
    pub fn flex_elemdataadr(&self, id: ObjectId<obj::Flex>) -> Option<usize> {
        (unsafe { self.flex_elemdataadr.add(id.index()).read() }).try_into().ok()
    }
    /// first element edge id address
    pub fn flex_elemedgeadr(&self, id: ObjectId<obj::Flex>) -> Option<usize> {
        (unsafe { self.flex_elemedgeadr.add(id.index()).read() }).try_into().ok()
    }
    /// number of shells
    pub fn flex_shellnum(&self, id: ObjectId<obj::Flex>) -> usize {
        (unsafe { self.flex_shellnum.add(id.index()).read() }) as usize
    }
    /// first shell data address
    pub fn flex_shelldataadr(&self, id: ObjectId<obj::Flex>) -> Option<usize> {
        (unsafe { self.flex_shelldataadr.add(id.index()).read() }).try_into().ok()
    }
    /// first evpair address
    pub fn flex_evpairadr(&self, id: ObjectId<obj::Flex>) -> Option<usize> {
        (unsafe { self.flex_evpairadr.add(id.index()).read() }).try_into().ok()
    }
    /// number of evpairs
    pub fn flex_evpairnum(&self, id: ObjectId<obj::Flex>) -> usize {
        (unsafe { self.flex_evpairnum.add(id.index()).read() }) as usize
    }
    /// node body ids
    pub fn flex_nodebodyid(&self, id: NodeId<obj::Flex>) -> ObjectId<obj::Body> {
        let index = (unsafe { self.flex_nodebodyid.add(id.index()).read() }) as usize;
        unsafe { ObjectId::new_unchecked(index) }
    }
    /// vertex body ids
    pub fn flex_vertbodyid(&self, id: VertexId<obj::Flex>) -> ObjectId<obj::Body> {
        let index = (unsafe { self.flex_vertbodyid.add(id.index()).read() }) as usize;
        unsafe { ObjectId::new_unchecked(index) }
    }
    /// edge vertex ids (2 per edge)
    pub fn flex_edge(&self, id: EdgeId<obj::Flex>) -> (VertexId<obj::Flex>, VertexId<obj::Flex>) {
        unsafe {
            let ptr = self.flex_edge.add(id.index() * 2);
            (
                VertexId::new_unchecked(ptr.read() as usize),
                VertexId::new_unchecked(ptr.add(1).read() as usize)
            )
        }
    }
    /// element vertex ids (dim+1 per elem)
    pub fn flex_elem(&self, id: ElementDataId) -> VertexId<obj::Flex> {
        let index = (unsafe { self.flex_elem.add(id.index()).read() }) as usize;
        unsafe { VertexId::new_unchecked(index) }
    }
    /// element texture coordinates (dim+1)
    pub fn flex_elemtexcoord(&self, id: ElementDataId) -> usize {
        (unsafe { self.flex_elemtexcoord.add(id.index()).read() }) as usize
    }
    /// element edge ids
    pub fn flex_elemedge(&self, id: EdgeDataId) -> EdgeId<obj::Flex> {
        let index = (unsafe { self.flex_elemedge.add(id.index()).read() }) as usize;
        unsafe { EdgeId::new_unchecked(index) }
    }
    /// element distance from surface, 3D only
    pub fn flex_elemlayer(&self, id: ElementId<obj::Flex>) -> i32 {
        unsafe { self.flex_elemlayer.add(id.index()).read() }
    }
    /// shell fragment vertex ids (dim per frag)
    pub fn flex_shell(&self, id: ShellDataId) -> VertexId<obj::Flex> {
        let index = (unsafe { self.flex_shell.add(id.index()).read() }) as usize;
        unsafe { VertexId::new_unchecked(index) }
    }
    /// (element, vertex) collision pairs
    pub fn flex_evpair(&self, id: EvPairId) -> (ElementId<obj::Flex>, VertexId<obj::Flex>) {
        unsafe {
            let ptr = self.flex_evpair.add(id.index() * 2);
            (ElementId::new_unchecked(ptr.read() as usize), VertexId::new_unchecked(ptr.add(1).read() as usize))
        }
    }
    /// vertex positions in local body frames
    pub fn flex_vert(&self, id: VertexId<obj::Flex>) -> [f64; 3] {
        unsafe {
            let ptr = self.flex_vert.add(id.index() * 3);
            array(|i| ptr.add(i).read())
        }
    }
    /// vertex positions in qpos0 on [0, 1]^d
    pub fn flex_vert0(&self, id: VertexId<obj::Flex>) -> [f64; 3] {
        unsafe {
            let ptr = self.flex_vert0.add(id.index() * 3);
            array(|i| ptr.add(i).read())
        }
    }
    /// node positions in local body frames
    pub fn flex_node(&self, id: NodeId<obj::Flex>) -> [f64; 3] {
        unsafe {
            let ptr = self.flex_node.add(id.index() * 3);
            array(|i| ptr.add(i).read())
        }
    }
    /// Cartesian node positions in qpos0
    pub fn flex_node0(&self, id: NodeId<obj::Flex>) -> [f64; 3] {
        unsafe {
            let ptr = self.flex_node0.add(id.index() * 3);
            array(|i| ptr.add(i).read())
        }
    }
    /// edge lengths in qpos0
    pub fn flexedge_length0(&self, id: EdgeId<obj::Flex>) -> f64 {
        unsafe { self.flexedge_length0.add(id.index()).read() }
    }
    /// edge inv. weight in qpos0
    pub fn flexedge_invweight0(&self, id: EdgeId<obj::Flex>) -> f64 {
        unsafe { self.flexedge_invweight0.add(id.index()).read() }
    }
    /// radius around primitive element
    pub fn flex_radius(&self, id: ObjectId<obj::Flex>) -> f64 {
        unsafe { self.flex_radius.add(id.index()).read() }
    }
    /// finite element stiffness matrix
    pub fn flex_stiffness(&self, id: ElementId<obj::Flex>) -> [f64; 21] {
        unsafe {
            let ptr = self.flex_stiffness.add(id.index() * 21);
            array(|i| ptr.add(i).read())
        }
    }
    /// Rayleigh's damping coefficient
    pub fn flex_damping(&self, id: ObjectId<obj::Flex>) -> f64 {
        unsafe { self.flex_damping.add(id.index()).read() }
    }
    /// edge stiffness
    pub fn flex_edgestiffness(&self, id: ObjectId<obj::Flex>) -> f64 {
        unsafe { self.flex_edgestiffness.add(id.index()).read() }
    }
    /// edge damping
    pub fn flex_edgedamping(&self, id: ObjectId<obj::Flex>) -> f64 {
        unsafe { self.flex_edgedamping.add(id.index()).read() }
    }
    /// is edge equality constraint defined
    pub fn flex_edgeequality(&self, id: EdgeId<obj::Flex>) -> bool {
        (unsafe { self.flex_edgeequality.add(id.index()).read() }) != 0
    }
    /// are all verices in the same body
    pub fn flex_rigid(&self, id: ObjectId<obj::Flex>) -> bool {
        (unsafe { self.flex_rigid.add(id.index()).read() }) != 0
    }
    /// are both edge vertices in same body
    pub fn flexedge_rigid(&self, id: EdgeId<obj::Flex>) -> bool {
        (unsafe { self.flexedge_rigid.add(id.index()).read() }) != 0
    }
    /// are all vertex coordinates (0,0,0)
    pub fn flex_centered(&self, id: ObjectId<obj::Flex>) -> bool {
        (unsafe { self.flex_centered.add(id.index()).read() }) != 0
    }
    /// render flex skin with flat shading
    pub fn flex_flatskin(&self, id: ObjectId<obj::Flex>) -> bool {
        (unsafe { self.flex_flatskin.add(id.index()).read() }) != 0
    }
    /// address of bvh root
    pub fn flex_bvhadr(&self, id: ObjectId<obj::Flex>) -> Option<usize> {
        (unsafe { self.flex_bvhadr.add(id.index()).read() }).try_into().ok()
    }
    /// number of bounding volumes
    pub fn flex_bvhnum(&self, id: ObjectId<obj::Flex>) -> usize {
        (unsafe { self.flex_bvhnum.add(id.index()).read() }) as usize
    }
    /// rgba when material is omitted
    pub fn flex_rgba(&self, id: ObjectId<obj::Flex>) -> Rgba {
        unsafe {
            let ptr = self.flex_rgba.add(id.index() * 4);
            Rgba { r: ptr.read(), g: ptr.add(1).read(), b: ptr.add(2).read(), a: ptr.add(3).read() }
        }
    }
    /// vertex texture coordinates
    pub fn flex_texcoord(&self, id: TexcoordId<obj::Flex>) -> [f32; 2] {
        unsafe {
            let ptr = self.flex_texcoord.add(id.index() * 2);
            array(|i| ptr.add(i).read())
        }
    }

    /// first vertex address
    pub fn mesh_vertadr(&self, id: ObjectId<obj::Mesh>) -> Option<usize> {
        (unsafe { self.mesh_vertadr.add(id.index()).read() }).try_into().ok()
    }
    /// number of vertices
    pub fn mesh_vertnum(&self, id: ObjectId<obj::Mesh>) -> usize {
        (unsafe { self.mesh_vertnum.add(id.index()).read() }) as usize
    }
    /// first face address
    pub fn mesh_faceadr(&self, id: ObjectId<obj::Mesh>) -> Option<usize> {
        (unsafe { self.mesh_faceadr.add(id.index()).read() }).try_into().ok()
    }
    /// number of faces
    pub fn mesh_facenum(&self, id: ObjectId<obj::Mesh>) -> usize {
        (unsafe { self.mesh_facenum.add(id.index()).read() }) as usize
    }
    /// address of bvh root
    pub fn mesh_bvhadr(&self, id: ObjectId<obj::Mesh>) -> Option<usize> {
        (unsafe { self.mesh_bvhadr.add(id.index()).read() }).try_into().ok()
    }
    /// number of bounding volumes
    pub fn mesh_bvhnum(&self, id: ObjectId<obj::Mesh>) -> usize {
        (unsafe { self.mesh_bvhnum.add(id.index()).read() }) as usize
    }
    /// first normal address
    pub fn mesh_normaladr(&self, id: ObjectId<obj::Mesh>) -> Option<usize> {
        (unsafe { self.mesh_normaladr.add(id.index()).read() }).try_into().ok()
    }
    /// number of normals
    pub fn mesh_normalnum(&self, id: ObjectId<obj::Mesh>) -> usize {
        (unsafe { self.mesh_normalnum.add(id.index()).read() }) as usize
    }
    /// texcoord data address
    pub fn mesh_texcoordadr(&self, id: ObjectId<obj::Mesh>) -> Option<usize> {
        (unsafe { self.mesh_texcoordadr.add(id.index()).read() }).try_into().ok()
    }
    /// number of texcoord
    pub fn mesh_texcoordnum(&self, id: ObjectId<obj::Mesh>) -> usize {
        (unsafe { self.mesh_texcoordnum.add(id.index()).read() }) as usize
    }
    /// vertex positions for all meshes
    pub fn mesh_vert(&self, id: VertexId<obj::Mesh>) -> [f32; 3] {
        unsafe {
            let ptr = self.mesh_vert.add(id.index() * 3);
            array(|i| ptr.add(i).read())
        }
    }
    /// normals for all meshes
    pub fn mesh_normal(&self, id: NormalId) -> [f32; 3] {
        unsafe {
            let ptr = self.mesh_normal.add(id.index() * 3);
            array(|i| ptr.add(i).read())
        }
    }
    /// vertex texcoords for all meshes
    pub fn mesh_texcoord(&self, id: TexcoordId<obj::Mesh>) -> [f32; 2] {
        unsafe {
            let ptr = self.mesh_texcoord.add(id.index() * 2);
            array(|i| ptr.add(i).read())
        }
    }
    /// vertex face data
    pub fn mesh_face(&self, id: FaceId<obj::Mesh>) -> [VertexId<obj::Mesh>; 3] {
        let ptr = unsafe { self.mesh_face.add(id.index() * 3) };
        array(|i| unsafe { VertexId::new_unchecked(ptr.add(i).read() as usize) })
    }
    /// normal face data
    pub fn mesh_facenormal(&self, id: FaceId<obj::Mesh>) -> [NormalId; 3] {
        let ptr = unsafe { self.mesh_facenormal.add(id.index() * 3) };
        array(|i| unsafe { NormalId::new_unchecked(ptr.add(i).read() as usize) })
    }
    /// texture face data
    pub fn mesh_facetexcoord(&self, id: FaceId<obj::Mesh>) -> [TexcoordId<obj::Mesh>; 3] {
        let ptr = unsafe { self.mesh_facetexcoord.add(id.index() * 3) };
        array(|i| unsafe { TexcoordId::new_unchecked(ptr.add(i).read() as usize) })
    }
    /// scaling applied to asset vertices
    pub fn mesh_scale(&self, id: ObjectId<obj::Mesh>) -> [f64; 3] {
        unsafe {
            let ptr = self.mesh_scale.add(id.index() * 3);
            array(|i| ptr.add(i).read())
        }
    }
    /// translation applied to asset vertices
    pub fn mesh_pos(&self, id: ObjectId<obj::Mesh>) -> [f64; 3] {
        unsafe {
            let ptr = self.mesh_pos.add(id.index() * 3);
            array(|i| ptr.add(i).read())
        }
    }
    /// rotation applied to asset vertices
    pub fn mesh_quat(&self, id: ObjectId<obj::Mesh>) -> [f64; 4] {
        unsafe {
            let ptr = self.mesh_quat.add(id.index() * 4);
            array(|i| ptr.add(i).read())
        }
    }
    /// address of asset path for mesh
    pub fn mesh_pathadr(&self, id: ObjectId<obj::Mesh>) -> Option<usize> {
        (unsafe { self.mesh_pathadr.add(id.index()).read() }).try_into().ok()
    }
    // TODO `mesh_poly*`

    /// skin material id
    pub fn skin_matid(&self, id: ObjectId<obj::Skin>) -> Option<ObjectId<obj::Material>> {
        let index = (unsafe { self.skin_matid.add(id.index()).read() }).try_into().ok()?;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// group for visibility
    pub fn skin_group(&self, id: ObjectId<obj::Skin>) -> i32 {
        unsafe { self.skin_group.add(id.index()).read() }
    }
    /// rgba when material is omitted
    pub fn skin_rgba(&self, id: ObjectId<obj::Skin>) -> Rgba {
        unsafe {
            let ptr = self.skin_rgba.add(id.index() * 4);
            Rgba { r: ptr.read(), g: ptr.add(1).read(), b: ptr.add(2).read(), a: ptr.add(3).read() }
        }
    }
    /// inflate skin in normal direction
    pub fn skin_inflate(&self, id: ObjectId<obj::Skin>) -> f32 {
        unsafe { self.skin_inflate.add(id.index()).read() }
    }
    /// first vertex address
    pub fn skin_vertadr(&self, id: ObjectId<obj::Skin>) -> Option<usize> {
        (unsafe { self.skin_vertadr.add(id.index()).read() }).try_into().ok()
    }
    /// number of vertices
    pub fn skin_vertnum(&self, id: ObjectId<obj::Skin>) -> usize {
        (unsafe { self.skin_vertnum.add(id.index()).read() }) as usize
    }
    /// texcoord data address
    pub fn skin_texcoordadr(&self, id: ObjectId<obj::Skin>) -> Option<usize> {
        (unsafe { self.skin_texcoordadr.add(id.index()).read() }).try_into().ok()
    }
    /// first face address
    pub fn skin_faceadr(&self, id: ObjectId<obj::Skin>) -> Option<usize> {
        (unsafe { self.skin_faceadr.add(id.index()).read() }).try_into().ok()
    }
    /// number of faces
    pub fn skin_facenum(&self, id: ObjectId<obj::Skin>) -> usize {
        (unsafe { self.skin_facenum.add(id.index()).read() }) as usize
    }
    /// first bone in skin
    pub fn skin_boneadr(&self, id: ObjectId<obj::Skin>) -> Option<usize> {
        (unsafe { self.skin_boneadr.add(id.index()).read() }).try_into().ok()
    }
    /// number of bones in skin
    pub fn skin_bonenum(&self, id: ObjectId<obj::Skin>) -> usize {
        (unsafe { self.skin_bonenum.add(id.index()).read() }) as usize
    }
    /// vertex positions for all skin meshes
    pub fn skin_vert(&self, id: VertexId<obj::Skin>) -> [f32; 3] {
        unsafe {
            let ptr = self.skin_vert.add(id.index() * 3);
            array(|i| ptr.add(i).read())
        }
    }
    /// vertex texcoords for all skin meshes
    pub fn skin_texcoord(&self, id: TexcoordId<obj::Skin>) -> [f32; 2] {
        unsafe {
            let ptr = self.skin_texcoord.add(id.index() * 2);
            array(|i| ptr.add(i).read())
        }
    }
    /// triangle faces for all skin meshes
    pub fn skin_face(&self, id: FaceId<obj::Skin>) -> [VertexId<obj::Skin>; 3] {
        unsafe {
            let ptr = self.skin_face.add(id.index() * 3);
            array(|i| VertexId::new_unchecked(ptr.add(i).read() as usize))
        }
    }
    /// first vertex in each bone
    pub fn skin_bonevertadr(&self, id: BoneId) -> Option<usize> {
        (unsafe { self.skin_bonevertadr.add(id.index()).read() }).try_into().ok()
    }
    /// number of vertices in each bone
    pub fn skin_bonevertnum(&self, id: BoneId) -> usize {
        (unsafe { self.skin_bonevertnum.add(id.index()).read() }) as usize
    }
    /// bind pos of each bone
    pub fn skin_bonebindpos(&self, id: BoneId) -> [f32; 3] {
        unsafe {
            let ptr = self.skin_bonebindpos.add(id.index() * 3);
            array(|i| ptr.add(i).read())
        }
    }
    /// bind quat of each bone
    pub fn skin_bonebindquat(&self, id: BoneId) -> [f32; 4] {
        unsafe {
            let ptr = self.skin_bonebindquat.add(id.index() * 4);
            array(|i| ptr.add(i).read())
        }
    }
    /// body id of each bone
    pub fn skin_bonebodyid(&self, id: BoneId) -> ObjectId<obj::Body> {
        let index = (unsafe { self.skin_bonebodyid.add(id.index()).read() }) as usize;
        unsafe { ObjectId::new_unchecked(index) }
    }
    /// mesh ids of vertices in each bone
    pub fn skin_bonevertid(&self, id: BoneVertexId) -> VertexId<obj::Skin> {
        let index = (unsafe { self.skin_bonevertid.add(id.index()).read() }) as usize;
        unsafe { VertexId::new_unchecked(index) }
    }
    /// weights of vertices in each bone
    pub fn skin_bonevertweight(&self, id: BoneVertexId) -> f32 {
        unsafe { self.skin_bonevertweight.add(id.index()).read() }
    }
    /// address of asset path for skin
    pub fn skin_pathadr(&self, id: ObjectId<obj::Skin>) -> Option<usize> {
        (unsafe { self.skin_pathadr.add(id.index()).read() }).try_into().ok()
    }

    /// (x, y, z_top, z_bottom)
    pub fn hfield_size(&self, id: ObjectId<obj::HField>) -> [f64; 4] {
        unsafe {
            let ptr = self.hfield_size.add(id.index() * 4);
            array(|i| ptr.add(i).read())
        }
    }
    /// number of rows in grid
    pub fn hfield_nrow(&self, id: ObjectId<obj::HField>) -> usize {
        (unsafe { self.hfield_nrow.add(id.index()).read() }) as usize
    }
    /// number of columns in grid
    pub fn hfield_ncol(&self, id: ObjectId<obj::HField>) -> usize {
        (unsafe { self.hfield_ncol.add(id.index()).read() }) as usize
    }
    /// address in hfield_data
    pub fn hfield_adr(&self, id: ObjectId<obj::HField>) -> usize {
        (unsafe { self.hfield_adr.add(id.index()).read() }) as usize
    }
    /// elevation data
    pub fn hfield_data(&self, id: HFieldDataId) -> f32 {
        unsafe { self.hfield_data.add(id.index()).read() }
    }
    /// address of hfield asset path
    pub fn hfield_pathadr(&self, id: ObjectId<obj::HField>) -> Option<usize> {
        (unsafe { self.hfield_pathadr.add(id.index()).read() }).try_into().ok()
    }

    /// texture type (mjtTexture)
    pub fn tex_type(&self, id: ObjectId<obj::Texture>) -> mjtTexture {
        mjtTexture((unsafe { self.tex_type.add(id.index()).read() }) as u32)
    }
    /// number of rows in texture image [px]
    pub fn tex_height(&self, id: ObjectId<obj::Texture>) -> usize {
        (unsafe { self.tex_height.add(id.index()).read() }) as usize
    }
    /// number of columns in texture image [px]
    pub fn tex_width(&self, id: ObjectId<obj::Texture>) -> usize {
        (unsafe { self.tex_width.add(id.index()).read() }) as usize
    }
    /// number of channels in texture image
    pub fn tex_nchannel(&self, id: ObjectId<obj::Texture>) -> usize {
        (unsafe { self.tex_nchannel.add(id.index()).read() }) as usize
    }
    /// start address in tex_data
    pub fn tex_adr(&self, id: ObjectId<obj::Texture>) -> usize {
        (unsafe { self.tex_adr.add(id.index()).read() }) as usize
    }
    /// pixel values
    pub fn tex_data(&self, id: TexDataId) -> u8 {
        unsafe { self.tex_data.add(id.index()).read() }
    }
    /// address of texture asset path
    pub fn tex_pathadr(&self, id: ObjectId<obj::Texture>) -> Option<usize> {
        (unsafe { self.tex_pathadr.add(id.index()).read() }).try_into().ok()
    }

    /// indices of textures
    pub fn mat_texid(&self, id: ObjectId<obj::Material>) -> [Option<ObjectId<obj::Texture>>; mjNTEXROLE] {
        unsafe {
            let ptr = self.mat_texid.add(id.index() * mjNTEXROLE);
            array(|i| Some(ObjectId::new_unchecked(ptr.add(i).read().try_into().ok()?)))
        }
    }
    /// make texture cube uniform
    pub fn mat_texuniform(&self, id: ObjectId<obj::Material>) -> bool {
        (unsafe { self.mat_texuniform.add(id.index()).read() }) != 0
    }
    /// texture repetition for 2d mapping
    pub fn mat_texrepeat(&self, id: ObjectId<obj::Material>) -> [f32; 2] {
        unsafe {
            let ptr = self.mat_texrepeat.add(id.index() * 2);
            array(|i| ptr.add(i).read())
        }
    }
    /// emission (x rgb)
    pub fn mat_emission(&self, id: ObjectId<obj::Material>) -> f32 {
        unsafe { self.mat_emission.add(id.index()).read() }
    }
    /// specular (x white)
    pub fn mat_specular(&self, id: ObjectId<obj::Material>) -> f32 {
        unsafe { self.mat_specular.add(id.index()).read() }
    }
    /// shininess coef
    pub fn mat_shininess(&self, id: ObjectId<obj::Material>) -> f32 {
        unsafe { self.mat_shininess.add(id.index()).read() }
    }
    /// reflectance (0: disable)
    pub fn mat_reflectance(&self, id: ObjectId<obj::Material>) -> f32 {
        unsafe { self.mat_reflectance.add(id.index()).read() }
    }
    /// metallic coef
    pub fn mat_metallic(&self, id: ObjectId<obj::Material>) -> f32 {
        unsafe { self.mat_metallic.add(id.index()).read() }
    }
    /// roughness coef
    pub fn mat_roughness(&self, id: ObjectId<obj::Material>) -> f32 {
        unsafe { self.mat_roughness.add(id.index()).read() }
    }
    /// rgba
    pub fn mat_rgba(&self, id: ObjectId<obj::Material>) -> Rgba {
        unsafe {
            let ptr = self.mat_rgba.add(id.index() * 4);
            Rgba { r: ptr.read(), g: ptr.add(1).read(), b: ptr.add(2).read(), a: ptr.add(3).read() }
        }
    }

    /// contact dimensionality (1, 3, 4, 6)
    pub fn pair_dim(&self, id: ObjectId<obj::Pair>) -> usize {
        (unsafe { self.pair_dim.add(id.index()).read() }) as usize
    }
    /// id of geom1
    pub fn pair_geom1(&self, id: ObjectId<obj::Pair>) -> ObjectId<obj::Geom> {
        let index = (unsafe { self.pair_geom1.add(id.index()).read() }) as usize;
        unsafe { ObjectId::new_unchecked(index) }
    }
    /// id of geom2
    pub fn pair_geom2(&self, id: ObjectId<obj::Pair>) -> ObjectId<obj::Geom> {
        let index = (unsafe { self.pair_geom2.add(id.index()).read() }) as usize;
        unsafe { ObjectId::new_unchecked(index) }
    }
    /// body1 << 16 + body2
    pub fn pair_signature(&self, id: ObjectId<obj::Pair>) -> u32 {
        (unsafe { self.pair_signature.add(id.index()).read() }) as u32
    }
    /// solver reference: contact normal
    pub fn pair_solref(&self, id: ObjectId<obj::Pair>) -> [f64; mjNREF] {
        unsafe {
            let ptr = self.pair_solref.add(id.index() * mjNREF);
            array(|i| ptr.add(i).read())
        }
    }
    /// solver reference: contact friction
    pub fn pair_solreffriction(&self, id: ObjectId<obj::Pair>) -> [f64; mjNREF] {
        unsafe {
            let ptr = self.pair_solreffriction.add(id.index() * mjNREF);
            array(|i| ptr.add(i).read())
        }
    }
    /// solver impedance: contact
    pub fn pair_solimp(&self, id: ObjectId<obj::Pair>) -> [f64; mjNIMP] {
        unsafe {
            let ptr = self.pair_solimp.add(id.index() * mjNIMP);
            array(|i| ptr.add(i).read())
        }
    }
    /// detect contact if dist < margin
    pub fn pair_margin(&self, id: ObjectId<obj::Pair>) -> f64 {
        unsafe { self.pair_margin.add(id.index()).read() }
    }
    /// include in solver if dist < margin - gap
    pub fn pair_gap(&self, id: ObjectId<obj::Pair>) -> f64 {
        unsafe { self.pair_gap.add(id.index()).read() }
    }
    /// friction for (tangent1, tangent2, spin, roll1, roll2)
    pub fn pair_friction(&self, id: ObjectId<obj::Pair>) -> (f64, f64, f64, f64, f64) {
        unsafe {
            let ptr = self.pair_friction.add(id.index() * 5);
            (ptr.read(), ptr.add(1).read(), ptr.add(2).read(), ptr.add(3).read(), ptr.add(4).read())
        }
    }

    /// body1 << 16 + body2
    pub fn exclude_signature(&self, id: ObjectId<obj::Exclude>) -> u32 {
        (unsafe { self.exclude_signature.add(id.index()).read() }) as u32
    }

    /// constraint type (mjtEq)
    pub fn eq_type(&self, id: ObjectId<obj::Equality>) -> mjtEq {
        mjtEq((unsafe { self.eq_type.add(id.index()).read() }) as u32)
    }
    /// get object ids as `ObjectId<O>`; `None` if object is not of type `O`.
    /// 
    /// the object type can be got (as value) with `.eq_objtype()`.
    pub fn eq_objid<O: Obj>(&self, id: ObjectId<obj::Equality>) -> Option<(ObjectId<O>, ObjectId<O>)> {
        if self.eq_objtype(id) != O::TYPE {
            return None;
        }
        unsafe {
            Some((
                ObjectId::new_unchecked(self.eq_obj1id.add(id.index()).read() as usize),
                ObjectId::new_unchecked(self.eq_obj2id.add(id.index()).read() as usize)
            ))
        }
    }
    /// type of both objects (mjtObj)
    pub fn eq_objtype(&self, id: ObjectId<obj::Equality>) -> mjtObj {
        mjtObj((unsafe { self.eq_objtype.add(id.index()).read() }) as u32)
    }
    /// initial enable/disable constraint state
    pub fn eq_active0(&self, id: ObjectId<obj::Equality>) -> bool {
        (unsafe { self.eq_active0.add(id.index()).read() }) != 0
    }
    /// constraint solver reference
    pub fn eq_solref(&self, id: ObjectId<obj::Equality>) -> [f64; mjNREF] {
        unsafe {
            let ptr = self.eq_solref.add(id.index() * mjNREF);
            array(|i| ptr.add(i).read())
        }
    }
    /// constraint solver impedance
    pub fn eq_solimp(&self, id: ObjectId<obj::Equality>) -> [f64; mjNIMP] {
        unsafe {
            let ptr = self.eq_solimp.add(id.index() * mjNIMP);
            array(|i| ptr.add(i).read())
        }
    }
    /// numeric data for constraint
    pub fn eq_data(&self, id: ObjectId<obj::Equality>) -> [f64; mjNEQDATA] {
        unsafe {
            let ptr = self.eq_data.add(id.index() * mjNEQDATA);
            array(|i| ptr.add(i).read())
        }
    }

    /// first address of tendon path
    pub fn tendon_adr(&self, id: ObjectId<obj::Tendon>) -> usize {
        (unsafe { self.tendon_adr.add(id.index()).read() }) as usize
    }
    /// number of objects in tendon path
    pub fn tendon_num(&self, id: ObjectId<obj::Tendon>) -> usize {
        (unsafe { self.tendon_num.add(id.index()).read() }) as usize
    }
    /// material id for rendering
    pub fn tendon_matid(&self, id: ObjectId<obj::Tendon>) -> Option<ObjectId<obj::Material>> {
        let index = (unsafe { self.tendon_matid.add(id.index()).read() }) as usize;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// group for visibility
    pub fn tendon_group(&self, id: ObjectId<obj::Tendon>) -> i32 {
        unsafe { self.tendon_group.add(id.index()).read() }
    }
    /// does tendon have length limits
    pub fn tendon_limited(&self, id: ObjectId<obj::Tendon>) -> bool {
        (unsafe { self.tendon_limited.add(id.index()).read() }) != 0
    }
    /// does tendon have actuator force limits
    pub fn tendon_actfrclimited(&self, id: ObjectId<obj::Tendon>) -> bool {
        (unsafe { self.tendon_actfrclimited.add(id.index()).read() }) != 0
    }
    /// width for rendering
    pub fn tendon_width(&self, id: ObjectId<obj::Tendon>) -> f64 {
        unsafe { self.tendon_width.add(id.index()).read() }
    }
    /// constraint solver reference: limit
    pub fn tendon_solref_lim(&self, id: ObjectId<obj::Tendon>) -> [f64; mjNREF] {
        unsafe {
            let ptr = self.tendon_solref_lim.add(id.index() * mjNREF);
            array(|i| ptr.add(i).read())
        }
    }
    /// constraint solver impedance: limit
    pub fn tendon_solimp_lim(&self, id: ObjectId<obj::Tendon>) -> [f64; mjNIMP] {
        unsafe {
            let ptr = self.tendon_solimp_lim.add(id.index() * mjNIMP);
            array(|i| ptr.add(i).read())
        }
    }
    /// constraint solver reference: friction
    pub fn tendon_solref_fri(&self, id: ObjectId<obj::Tendon>) -> [f64; mjNREF] {
        unsafe {
            let ptr = self.tendon_solref_fri.add(id.index() * mjNREF);
            array(|i| ptr.add(i).read())
        }
    }
    /// constraint solver impedance: friction
    pub fn tendon_solimp_fri(&self, id: ObjectId<obj::Tendon>) -> [f64; mjNIMP] {
        unsafe {
            let ptr = self.tendon_solimp_fri.add(id.index() * mjNIMP);
            array(|i| ptr.add(i).read())
        }
    }
    /// tendon length limits
    pub fn tendon_range(&self, id: ObjectId<obj::Tendon>) -> std::ops::Range<f64> {
        unsafe {
            let ptr = self.tendon_range.add(id.index() * 2);
            ptr.read()..ptr.add(1).read()
        }
    }
    /// range of total actuator force
    pub fn tendon_actfrcrange(&self, id: ObjectId<obj::Tendon>) -> std::ops::Range<f64> {
        unsafe {
            let ptr = self.tendon_actfrcrange.add(id.index() * 2);
            ptr.read()..ptr.add(1).read()
        }
    }
    /// min distance for limit detection
    pub fn tendon_margin(&self, id: ObjectId<obj::Tendon>) -> f64 {
        unsafe { self.tendon_margin.add(id.index()).read() }
    }
    /// stiffness coefficient
    pub fn tendon_stiffness(&self, id: ObjectId<obj::Tendon>) -> f64 {
        unsafe { self.tendon_stiffness.add(id.index()).read() }
    }
    /// damping coefficient
    pub fn tendon_damping(&self, id: ObjectId<obj::Tendon>) -> f64 {
        unsafe { self.tendon_damping.add(id.index()).read() }
    }
    /// inertia associated with tendon velocity
    pub fn tendon_armature(&self, id: ObjectId<obj::Tendon>) -> f64 {
        unsafe { self.tendon_armature.add(id.index()).read() }
    }
    /// loss due to friction
    pub fn tendon_frictionloss(&self, id: ObjectId<obj::Tendon>) -> f64 {
        unsafe { self.tendon_frictionloss.add(id.index()).read() }
    }
    /// spring resting length range
    pub fn tendon_lengthspring(&self, id: ObjectId<obj::Tendon>) -> std::ops::Range<f64> {
        unsafe {
            let ptr = self.tendon_lengthspring.add(id.index() * 2);
            ptr.read()..ptr.add(1).read()
        }
    }
    /// tendon length in qpos0
    pub fn tendon_length0(&self, id: ObjectId<obj::Tendon>) -> f64 {
        unsafe { self.tendon_length0.add(id.index()).read() }
    }
    /// edge inv. weight in qpos0
    pub fn tendon_invweight0(&self, id: ObjectId<obj::Tendon>) -> f64 {
        unsafe { self.tendon_invweight0.add(id.index()).read() }
    }
    /// rgba when material is omitted
    pub fn tendon_rgba(&self, id: ObjectId<obj::Tendon>) -> Rgba {
        unsafe {
            let ptr = self.tendon_rgba.add(id.index() * 4);
            Rgba { r: ptr.read(), g: ptr.add(1).read(), b: ptr.add(2).read(), a: ptr.add(3).read() }
        }
    }

    /// wrap object type (mjtWrap)
    pub fn wrap_type(&self, index: usize) -> mjtWrap {
        mjtWrap((unsafe { self.wrap_type.add(index).read() }) as u32)
    }
    /// object id: geom, site, joint
    /// 
    /// Returns `None` if specified object type `O` does not match the wrap object type.
    /// The wrap object type can be got (as value) with `.wrap_type(index)`.
    pub fn wrap_objid<O: Obj>(&self, index: usize) -> Option<ObjectId<O>> {
        if self.wrap_type(index).0 != O::TYPE.0 {
            return None;
        }
        let index = (unsafe { self.wrap_objid.add(index).read() }) as usize;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// divisor, joint coef, or site id
    pub fn wrap_prm(&self, index: usize) -> f64 {
        unsafe { self.wrap_prm.add(index).read() }
    }
    
    /// transmission type (mjtTrn)
    pub fn actuator_trntype(&self, id: ObjectId<obj::Actuator>) -> mjtTrn {
        mjtTrn((unsafe { self.actuator_trntype.add(id.index()).read() }) as u32)
    }
    /// dynamics type (mjtDyn)
    pub fn actuator_dyntype(&self, id: ObjectId<obj::Actuator>) -> mjtDyn {
        mjtDyn((unsafe { self.actuator_dyntype.add(id.index()).read() }) as u32)
    }
    /// gain type (mjtGain)
    pub fn actuator_gaintype(&self, id: ObjectId<obj::Actuator>) -> mjtGain {
        mjtGain((unsafe { self.actuator_gaintype.add(id.index()).read() }) as u32)
    }
    /// bias type (mjtBias)
    pub fn actuator_biastype(&self, id: ObjectId<obj::Actuator>) -> mjtBias {
        mjtBias((unsafe { self.actuator_biastype.add(id.index()).read() }) as u32)
    }
    /// transmission id: joint, tendon, site
    ///
    /// Returns `None` if the transmission id does not match the object type `O`.
    pub fn actuator_trnid<O: Obj>(&self, id: ObjectId<obj::Actuator>) -> (Option<ObjectId<O>>, Option<ObjectId<O>>) {
        if (unsafe { self.actuator_trnid.add(id.index() * 2).read() }) != O::TYPE.0 as i32 {
            return (None, None);
        }
        unsafe {
            let ptr = self.actuator_trnid.add(id.index() * 2);
            (
                Some(ObjectId::new_unchecked(ptr.read() as usize)),
                Some(ObjectId::new_unchecked(ptr.add(1).read() as usize))
            )
        }
    }
    /// first activation address; None: stateless
    pub fn actuator_actadr(&self, id: ObjectId<obj::Actuator>) -> Option<usize> {
        (unsafe { self.actuator_actadr.add(id.index()).read() }).try_into().ok()
    }
    /// number of activation variables
    pub fn actuator_actnum(&self, id: ObjectId<obj::Actuator>) -> usize {
        (unsafe { self.actuator_actnum.add(id.index()).read() }) as usize
    }
    /// group for visibility
    pub fn actuator_group(&self, id: ObjectId<obj::Actuator>) -> i32 {
        unsafe { self.actuator_group.add(id.index()).read() }
    }
    /// is control limited
    pub fn actuator_ctrllimited(&self, id: ObjectId<obj::Actuator>) -> bool {
        (unsafe { self.actuator_ctrllimited.add(id.index()).read() }) != 0
    }
    /// is force limited
    pub fn actuator_forcelimited(&self, id: ObjectId<obj::Actuator>) -> bool {
        (unsafe { self.actuator_forcelimited.add(id.index()).read() }) != 0
    }
    /// is activation limited
    pub fn actuator_actlimited(&self, id: ObjectId<obj::Actuator>) -> bool {
        (unsafe { self.actuator_actlimited.add(id.index()).read() }) != 0
    }
    /// dynamics parameters
    pub fn actuator_dynprm(&self, id: ObjectId<obj::Actuator>) -> [f64; mjNDYN] {
        unsafe {
            let ptr = self.actuator_dynprm.add(id.index() * mjNDYN);
            array(|i| ptr.add(i).read())
        }
    }
    /// gain parameters
    pub fn actuator_gainprm(&self, id: ObjectId<obj::Actuator>) -> [f64; mjNGAIN] {
        unsafe {
            let ptr = self.actuator_gainprm.add(id.index() * mjNGAIN);
            array(|i| ptr.add(i).read())
        }
    }
    /// bias parameters
    pub fn actuator_biasprm(&self, id: ObjectId<obj::Actuator>) -> [f64; mjNBIAS] {
        unsafe {
            let ptr = self.actuator_biasprm.add(id.index() * mjNBIAS);
            array(|i| ptr.add(i).read())
        }
    }
    /// step activation before force
    pub fn actuator_actearly(&self, id: ObjectId<obj::Actuator>) -> bool {
        (unsafe { self.actuator_actearly.add(id.index()).read() }) != 0
    }
    /// range of controls
    pub fn actuator_ctrlrange(&self, id: ObjectId<obj::Actuator>) -> std::ops::Range<f64> {
        unsafe {
            let ptr = self.actuator_ctrlrange.add(id.index() * 2);
            ptr.read().max(mjMINVAL)..ptr.add(1).read().min(mjMAXVAL)
        }
    }
    /// range of forces
    pub fn actuator_forcerange(&self, id: ObjectId<obj::Actuator>) -> std::ops::Range<f64> {
        unsafe {
            let ptr = self.actuator_forcerange.add(id.index() * 2);
            ptr.read().max(mjMINVAL)..ptr.add(1).read().min(mjMAXVAL)
        }
    }
    /// range of activations
    pub fn actuator_actrange(&self, id: ObjectId<obj::Actuator>) -> std::ops::Range<f64> {
        unsafe {
            let ptr = self.actuator_actrange.add(id.index() * 2);
            ptr.read().max(mjMINVAL)..ptr.add(1).read().min(mjMAXVAL)
        }
    }
    /// scale length and transmitted force
    pub fn actuator_gear(&self, id: ObjectId<obj::Actuator>) -> [f64; 6] {
        unsafe {
            let ptr = self.actuator_gear.add(id.index() * 6);
            array(|i| ptr.add(i).read())
        }
    }
    /// crank length for slider-crank
    pub fn actuator_cranklength(&self, id: ObjectId<obj::Actuator>) -> f64 {
        unsafe { self.actuator_cranklength.add(id.index()).read() }
    }
    /// acceleration from unit force in qpos0
    pub fn actuator_acc0(&self, id: ObjectId<obj::Actuator>) -> f64 {
        unsafe { self.actuator_acc0.add(id.index()).read() }
    }
    /// actuator length in qpos0
    pub fn actuator_length0(&self, id: ObjectId<obj::Actuator>) -> f64 {
        unsafe { self.actuator_length0.add(id.index()).read() }
    }
    /// feasible actuator length range
    pub fn actuator_lengthrange(&self, id: ObjectId<obj::Actuator>) -> std::ops::Range<f64> {
        unsafe {
            let ptr = self.actuator_lengthrange.add(id.index() * 2);
            ptr.read().max(mjMINVAL)..ptr.add(1).read().min(mjMAXVAL)
        }
    }
    /// plugin instance id; None: not a plugin
    pub fn actuator_plugin(&self, id: ObjectId<obj::Actuator>) -> Option<ObjectId<obj::Plugin>> {
        let index = (unsafe { self.actuator_plugin.add(id.index()).read() }) as usize;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }

    /// sendor type (mjtSensor)
    pub fn sensor_type(&self, id: ObjectId<obj::Sensor>) -> mjtSensor {
        mjtSensor((unsafe { self.sensor_type.add(id.index()).read() }) as u32)
    }
    /// numeric data type (mjtDataType)
    pub fn sensor_datatype(&self, id: ObjectId<obj::Sensor>) -> mjtDataType {
        mjtDataType((unsafe { self.sensor_datatype.add(id.index()).read() }) as u32)
    }
    /// required compute stage (mjtStage)
    pub fn sensor_needstage(&self, id: ObjectId<obj::Sensor>) -> mjtStage {
        mjtStage((unsafe { self.sensor_needstage.add(id.index()).read() }) as u32)
    }
    /// type of sensorized object (mjtObj)
    pub fn sensor_objtype(&self, id: ObjectId<obj::Sensor>) -> mjtObj {
        mjtObj((unsafe { self.sensor_objtype.add(id.index()).read() }) as u32)
    }
    /// id of sensorized object
    ///
    /// Returns `None` if the object type does not match the sensor object type.
    /// The sensor object type can be got (as value) with `.sensor_objtype(id)`.
    pub fn sensor_objid<O: Obj>(&self, id: ObjectId<obj::Sensor>) -> Option<ObjectId<O>> {
        if self.sensor_objtype(id) != O::TYPE {
            return None;
        }
        let index = (unsafe { self.sensor_objid.add(id.index()).read() }) as usize;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// type of reference frame (mjtObj)
    pub fn sensor_reftype(&self, id: ObjectId<obj::Sensor>) -> mjtObj {
        mjtObj((unsafe { self.sensor_reftype.add(id.index()).read() }) as u32)
    }
    /// id of reference frame;
    /// 
    /// Returns `None` when it's global frame or, if the reference frame type does not match the sensor reference type.
    /// The sensor reference type can be got (as value) with `.sensor_reftype(id)`.
    pub fn sensor_refid<O: Obj>(&self, id: ObjectId<obj::Sensor>) -> Option<ObjectId<O>> {
        if (unsafe { self.sensor_refid.add(id.index()).read() }) == -1 {
            return None; // global frame
        }
        if self.sensor_reftype(id) != O::TYPE {
            return None;
        }
        let index = (unsafe { self.sensor_refid.add(id.index()).read() }) as usize;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }
    /// number of scalar outputs
    pub fn sensor_dim(&self, id: ObjectId<obj::Sensor>) -> usize {
        (unsafe { self.sensor_dim.add(id.index()).read() }) as usize
    }
    /// address in sensor array
    pub fn sensor_adr(&self, id: ObjectId<obj::Sensor>) -> usize {
        (unsafe { self.sensor_adr.add(id.index()).read() }) as usize
    }
    /// cutoff for real and positive; 0: ignore
    pub fn sensor_cutoff(&self, id: ObjectId<obj::Sensor>) -> f64 {
        unsafe { self.sensor_cutoff.add(id.index()).read() }
    }
    /// noise standard deviation
    pub fn sensor_noise(&self, id: ObjectId<obj::Sensor>) -> f64 {
        unsafe { self.sensor_noise.add(id.index()).read() }
    }
    /// plugin instance id; None: not a plugin
    pub fn sensor_plugin(&self, id: ObjectId<obj::Sensor>) -> Option<ObjectId<obj::Plugin>> {
        let index = (unsafe { self.sensor_plugin.add(id.index()).read() }) as usize;
        Some(unsafe { ObjectId::new_unchecked(index) })
    }

    /// globally registered plugin slot number
    pub fn plugin(&self, id: ObjectId<obj::Plugin>) -> usize {
        (unsafe { self.plugin.add(id.index()).read() }) as usize
    }
    /// address in the plugin state array
    pub fn plugin_stateadr(&self, id: ObjectId<obj::Plugin>) -> Option<usize> {
        (unsafe { self.plugin_stateadr.add(id.index()).read() }).try_into().ok()
    }
    /// number of states in the plugin instance
    pub fn plugin_statenum(&self, id: ObjectId<obj::Plugin>) -> usize {
        (unsafe { self.plugin_statenum.add(id.index()).read() }) as usize
    }
    /// config attributes of plugin instances
    pub fn plugin_attr(&self, id: ObjectId<obj::Plugin>) -> Option<&str> {
        let index = self.plugin_attradr(id)?;
        let ptr = unsafe { self.plugin_attr.add(index) };
        if ptr.is_null() {
            None
        } else {
            Some(unsafe { std::ffi::CStr::from_ptr(ptr).to_str().unwrap() })
        }
    }
    /// address to each instance's config attribute
    pub fn plugin_attradr(&self, id: ObjectId<obj::Plugin>) -> Option<usize> {
        (unsafe { self.plugin_attradr.add(id.index()).read() }).try_into().ok()
    }

    /// keyframe time
    pub fn key_time(&self, id: ObjectId<obj::Key>) -> f64 {
        unsafe { self.key_time.add(id.index()).read() }
    }
    /// keyframe position: f64 * `nq`
    pub fn key_qpos(&self, id: ObjectId<obj::Key>) -> &[f64] {
        unsafe {
            let ptr = self.key_qpos.add(id.index() * self.nq());
            slice(ptr, self.nq())
        }
    }
    /// keyframe velocity: f64 * `nv`
    pub fn key_qvel(&self, id: ObjectId<obj::Key>) -> &[f64] {
        unsafe {
            let ptr = self.key_qvel.add(id.index() * self.nv());
            slice(ptr, self.nv())
        }
    }
    /// keyframe activation: f64 * `na`
    pub fn key_act(&self, id: ObjectId<obj::Key>) -> &[f64] {
        unsafe {
            let ptr = self.key_act.add(id.index() * self.na());
            slice(ptr, self.na())
        }
    }
    /// keyframe mocap position: f64 * `nmocap * 3`
    pub fn key_mpos(&self, id: ObjectId<obj::Key>) -> &[f64] {
        unsafe {
            let ptr = self.key_mpos.add(id.index() * self.nmocap() * 3);
            slice(ptr, self.nmocap() * 3)
        }
    }
    /// keyframe mocap quaternion: f64 * `nmocap * 4`
    pub fn key_mquat(&self, id: ObjectId<obj::Key>) -> &[f64] {
        unsafe {
            let ptr = self.key_mquat.add(id.index() * self.nmocap() * 4);
            slice(ptr, self.nmocap() * 4)
        }
    }
    /// keyframe control: f64 * `nu`
    pub fn key_ctrl(&self, id: ObjectId<obj::Key>) -> &[f64] {
        unsafe {
            let ptr = self.key_ctrl.add(id.index() * self.nu());
            slice(ptr, self.nu())
        }
    }
    
    /* TODO: *name* */
}
