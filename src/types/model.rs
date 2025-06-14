pub struct MjModel(crate::bindgen::mjModel);

macro_rules! impl_size_getters {
    ($($n_name:ident: $description:literal;)*) => {
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
    nq:                         "number of generalized coordinates = dim(qpos)";
    nv:                         "number of degrees of freedom = dim(qvel)";
    nu:                         "number of actuators/controls = dim(ctrl)";
    na:                         "number of activation states = dim(act)";
    nbody:                      "number of bodies";
    nbvh:                       "number of total bounding volumes in all bodies";
    nbvhstatic:                 "number of static bounding volumes (aabb stored in mjModel)";
    nbvhdynamic:                "number of dynamic bounding volumes (aabb stored in mjData)";
    njnt:                       "number of joints";
    ngeom:                      "number of geoms";
    nsite:                      "number of sites";
    ncam:                       "number of cameras";
    nlight:                     "number of lights";
    nflex:                      "number of flexes";
    nflexnode:                  "number of dofs in all flexes";
    nflexvert:                  "number of vertices in all flexes";
    nflexedge:                  "number of edges in all flexes";
    nflexelem:                  "number of elements in all flexes";
    nflexelemdata:              "number of element vertex ids in all flexes";
    nflexelemedge:              "number of element edge ids in all flexes";
    nflexshelldata:             "number of shell fragment vertex ids in all flexes";
    nflexevpair:                "number of element-vertex pairs in all flexes";
    nflextexcoord:              "number of vertices with texture coordinates";
    nmesh:                      "number of meshes";
    nmeshvert:                  "number of vertices in all meshes";
    nmeshnormal:                "number of normals in all meshes";
    nmeshtexcoord:              "number of texcoords in all meshes";
    nmeshface:                  "number of triangular faces in all meshes";
    nmeshgraph:                 "number of ints in mesh auxiliary data";
    nmeshpoly:                  "number of polygons in all meshes";
    nmeshpolyvert:              "number of vertices in all polygons";
    nmeshpolymap:               "number of polygons in vertex map";
    nskin:                      "number of skins";
    nskinvert:                  "number of vertices in all skins";
    nskintexvert:               "number of vertiex with texcoords in all skins";
    nskinface:                  "number of triangular faces in all skins";
    nskinbone:                  "number of bones in all skins";
    nskinbonevert:              "number of vertices in all skin bones";
    nhfield:                    "number of heightfields";
    nhfielddata:                "number of data points in all heightfields";
    ntex:                       "number of textures";
    ntexdata:                   "number of bytes in texture rgb data";
    nmat:                       "number of materials";
    npair:                      "number of predefined geom pairs";
    nexclude:                   "number of excluded geom pairs";
    neq:                        "number of equality constraints";
    ntendon:                    "number of tendons";
    nwrap:                      "number of wrap objects in all tendon paths";
    nsensor:                    "number of sensors";
    nnumeric:                   "number of numeric custom fields";
    nnumericdata:               "number of mjtNums in all numeric fields";
    ntext:                      "number of text custom fields";
    ntextdata:                  "number of mjtBytes in all text fields";
    ntuple:                     "number of tuple custom fields";
    ntupledata:                 "number of objects in all tuple fields";
    nkey:                       "number of keyframes";
    nmocap:                     "number of mocap bodies";
    nplugin:                    "number of plugin instances";
    npluginattr:                "number of chars in all plugin config attributes";
    nuser_body:                 "number of mjtNums in body_user";
    nuser_jnt:                  "number of mjtNums in jnt_user";
    nuser_geom:                 "number of mjtNums in geom_user";
    nuser_site:                 "number of mjtNums in site_user";
    nuser_cam:                  "number of mjtNums in cam_user";
    nuser_tendon:               "number of mjtNums in tendon_user";
    nuser_actuator:             "number of mjtNums in actuator_user";
    nuser_sensor:               "number of mjtNums in sensor_user";
    nnames:                     "number of chars in all names";
    npaths:                     "number of chars in all paths";

    // sizes set after mjModel construction
    nnames_map:                 "number of slots in the names hash map";
    nM:                         "number of non-zeros in sparse inertia matrix";
    nB:                         "number of non-zeros in sparse body-dof matrix";
    nC:                         "number of non-zeros in sparse reduced dof-dof matrix";
    nD:                         "number of non-zeros in sparse dof-dof matrix";
    nJmom:                      "number of non-zeros in sparse actuator_moment matrix";
    ntree:                      "number of kinematic trees under world body";
    ngravcomp:                  "number of bodies with nonzero gravcomp";
    nemax:                      "number of potential equality-constraint rows";
    njmax:                      "number of available rows in constraint Jacobian (legacy)";
    nconmax:                    "number of potential contacts in contact list (legacy)";
    nuserdata:                  "number of mjtNums reserved for the user";
    nsensordata:                "number of mjtNums in sensor data vector";
    npluginstate:               "number of mjtNums in plugin state vector";

    narena:                     "number of bytes in the mjData arena (inclusive of stack)";
    nbuffer:                    "number of bytes in buffer";
}

impl MjModel {
}
