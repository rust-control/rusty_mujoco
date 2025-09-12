//! # [Rendering](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#tyrenderstructure)
//! 
//! The names of these struct types are prefixed with mjr.

pub use crate::bindgen::{
    mjtTexture, mjtTextureRole, mjtFramebuffer, mjtDepthMap,
    mjNAUX, mjMAXMATERIAL, mjNTEXROLE, mjMAXTEXTURE,
};

pub use crate::bindgen::mjrRect;
fields_mapping!(mjrRect {
    scalars {
        left / set_left: i32 = "left edge of rectangle [px]";
        bottom / set_bottom: i32 = "bottom edge of rectangle [px]";
        width / set_width: i32 = "width of rectangle [px]";
        height / set_height: i32 = "height of rectangle [px]";
    }
});
impl mjrRect {
    pub fn new(left: u32, bottom: u32, width: u32, height: u32) -> Self {
        Self {
            left: left as i32,
            bottom: bottom as i32,
            width: width as i32,
            height: height as i32,
        }
    }
}

resource_wrapper!(
    MjrContext for crate::bindgen::mjrContext;
    drop = crate::mjr_freeContext;
);
impl Default for MjrContext {
    /// Internally calls [`mjr_defaultContext`](crate::mjr_defaultContext).
    /// 
    /// **note**: Be sure to call [`mjr_makeContext`](crate::mjr_makeContext) for the returned `MjrContext` to allocate resources
    ///           before using it in rendering.
    fn default() -> Self {
        crate::mjr_defaultContext()
    }
}
impl MjrContext {
    /// Create a new `MjrContext` with given font scale.
    /// 
    /// This internally calls:
    /// 
    /// 1. [`mjr_defaultContext`](crate::mjr_defaultContext) to set default values for the scene.
    /// 2. [`mjr_makeContext`](crate::mjr_makeContext) to allocate resources in the scene.
    pub fn new(m: &crate::MjModel, fontscale: crate::mjtFontScale) -> Self {
        let mut con = Self::default();
        crate::mjr_makeContext(m, &mut con, fontscale);
        con
    }
}
fields_mapping!(MjrContext {
    boolean_flags {
        glInitialized = "is OpenGL initialized";
        windowAvailable = "is default/window framebuffer available";
        windowStereo = "is stereo available for default/window framebuffer";
        windowDoublebuffer = "is default/window framebuffer double buffered";
    }
    scalars {
        windowSamples: usize = "number of samples for default/window framebuffer";
        lineWidth: f32 = "line width for wireframe rendering";
        shadowClip: f32 = "clipping radius for directional lights";
        shadowScale: f32 = "fraction of light cutoff for spot lights";
        fogStart: f32 = "fog start = stat.extent * vis.map.fogstart";
        fogEnd: f32 = "fog end = stat.extent * vis.map.fogend";
        fogRGBA: [f32; 4] = "fog rgba";
        shadowSize: i32 = "size of shadow map texture";
        offWidth: i32 = "width of offscreen buffer";
        offHeight: i32 = "height of offscreen buffer";
        offSamples: usize = "number of offscreen buffer multisamples";
        fontScale: i32 = "font scale";
        auxWidth: [i32; mjNAUX] = "auxiliary buffer width";
        auxHeight: [i32; mjNAUX] = "auxiliary buffer height";
        auxSamples: [i32; mjNAUX] = "auxiliary buffer multisamples";
        offFBO: u32 = "offscreen framebuffer object";
        offFBO_r: u32 = "offscreen framebuffer for resolving multisamples";
        offColor: u32 = "offscreen color buffer";
        offColor_r: u32 = "offscreen color buffer for resolving multisamples";
        offDepthStencil: u32 = "offscreen depth and stencil buffer";
        offDepthStencil_r: u32 = "offscreen depth and stencil buffer for multisamples";
        shadowFBO: u32 = "shadow map framebuffer object";
        shadowTex: u32 = "shadow map texture";
        auxFBO: [u32; mjNAUX] = "auxiliary framebuffer object";
        auxFBO_r: [u32; mjNAUX] = "auxiliary framebuffer object for resolving";
        auxColor: [u32; mjNAUX] = "auxiliary color buffer";
        auxColor_r: [u32; mjNAUX] = "auxiliary color buffer for resolving";
        mat_texid: [i32; mjMAXMATERIAL * mjNTEXROLE] = "material texture ids (-1: no texture)";
        mat_texuniform: [i32; mjMAXMATERIAL] = "uniform cube mapping";
        mat_texrepeat: [f32; mjMAXMATERIAL * 2] = "texture repetition for 2d mapping";
        ntexture: usize = "number of allocated textures";
        texture: [u32; mjMAXTEXTURE] = "texture names";
        basePlane: u32 = "all planes from model";
        baseMesh: u32 = "all meshes from model";
        baseHField: u32 = "all height fields from model";
        baseBuiltin: u32 = "all builtin geoms, with quality from model";
        baseFontNormal: u32 = "normal font";
        baseFontShadow: u32 = "shadow font";
        baseFontBig: u32 = "big font";
        rangePlane: i32 = "all planes from model";
        rangeMesh: i32 = "all meshes from model";
        rangeHField: i32 = "all hfields from model";
        rangeBuiltin: i32 = "all builtin geoms, with quality from model";
        rangeFont: i32 = "all characters in font";
        charWidth: [i32; 127] = "character widths: normal and shadow [px]";
        charWidthBig: [i32; 127] = "character widths: big [px]";
        charHeight: i32 = "character heights: normal and shadow [px]";
        charHeightBig: i32 = "character heights: big [px]";
        nskin: usize = "number of skins";
    }
});
#[allow(non_snake_case)]
impl MjrContext {
    /// default color pixel format for `mjr_readPixels`
    pub fn readPixelFormat(&self) -> i32 {
        self.readPixelFormat
    }
    /// set default color pixel format for `mjr_readPixels`
    pub fn set_readPixelFormat(&mut self, value: i32) -> &mut Self {
        self.readPixelFormat = value; self
    }

    /// depth mapping: `mjDEPTH_ZERONEAR` or `mjDEPTH_ZEROFAR`
    pub fn readDepthMap(&self) -> mjtDepthMap {
        mjtDepthMap(self.readDepthMap as u32)
    }
    /// set depth mapping: `mjDEPTH_ZERONEAR` or `mjDEPTH_ZEROFAR`
    pub fn set_readDepthMap(&mut self, value: mjtDepthMap ) -> &mut Self {
        self.readDepthMap = value.0 as i32; self
    }

    /// type of texture (ntexture)
    pub fn textureType(&self) -> [mjtTexture; mjMAXTEXTURE] {
        self.textureType.map(|t| mjtTexture(t as u32))
    }

    /// currently active framebuffer: `mjFB_WINDOW` or `mjFB_OFFSCREEN`
    pub fn currentBuffer(&self) -> mjtFramebuffer {
        mjtFramebuffer(self.currentBuffer as u32)
    }

    /// skin vertex position VBOs (nskin)
    pub fn skinvertVBO(&self) -> &[u32] {
        unsafe { std::slice::from_raw_parts(self.skinvertVBO, self.nskin()) }
    }
    /// skin vertex normal VBOs (nskin)
    pub fn skinnormalVBO(&self) -> &[u32] {
        unsafe { std::slice::from_raw_parts(self.skinnormalVBO, self.nskin()) }
    }
    /// skin vertex texture coordinate VBOs (nskin)
    pub fn skintexcoordVBO(&self) -> &[u32] {
        unsafe { std::slice::from_raw_parts(self.skintexcoordVBO, self.nskin()) }
    }
    /// skin face index VBOs (nskin)
    pub fn skinfaceVBO(&self) -> &[u32] {
        unsafe { std::slice::from_raw_parts(self.skinfaceVBO, self.nskin()) }
    }
}
