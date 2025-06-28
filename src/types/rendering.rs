//! # [Rendering](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#tyrenderstructure)
//! 
//! The names of these struct types are prefixed with mjr.

pub use crate::bindgen::{mjrRect, mjrContext};

derive_fields_mapping!(mjrRect {
    scalars {
        left / set_left: i32 = "left edge of rectangle [px]";
        bottom / set_bottom: i32 = "bottom edge of rectangle [px]";
        width / set_width: i32 = "width of rectangle [px]";
        height / set_height: i32 = "height of rectangle [px]";
    }
});

/*
struct mjrContext_ {                // custom OpenGL context
  // parameters copied from mjVisual
  float lineWidth;                  // line width for wireframe rendering
  float shadowClip;                 // clipping radius for directional lights
  float shadowScale;                // fraction of light cutoff for spot lights
  float fogStart;                   // fog start = stat.extent * vis.map.fogstart
  float fogEnd;                     // fog end = stat.extent * vis.map.fogend
  float fogRGBA[4];                 // fog rgba
  int shadowSize;                   // size of shadow map texture
  int offWidth;                     // width of offscreen buffer
  int offHeight;                    // height of offscreen buffer
  int offSamples;                   // number of offscreen buffer multisamples

  // parameters specified at creation
  int fontScale;                    // font scale
  int auxWidth[mjNAUX];             // auxiliary buffer width
  int auxHeight[mjNAUX];            // auxiliary buffer height
  int auxSamples[mjNAUX];           // auxiliary buffer multisamples

  // offscreen rendering objects
  unsigned int offFBO;              // offscreen framebuffer object
  unsigned int offFBO_r;            // offscreen framebuffer for resolving multisamples
  unsigned int offColor;            // offscreen color buffer
  unsigned int offColor_r;          // offscreen color buffer for resolving multisamples
  unsigned int offDepthStencil;     // offscreen depth and stencil buffer
  unsigned int offDepthStencil_r;   // offscreen depth and stencil buffer for multisamples

  // shadow rendering objects
  unsigned int shadowFBO;           // shadow map framebuffer object
  unsigned int shadowTex;           // shadow map texture

  // auxiliary buffers
  unsigned int auxFBO[mjNAUX];      // auxiliary framebuffer object
  unsigned int auxFBO_r[mjNAUX];    // auxiliary framebuffer object for resolving
  unsigned int auxColor[mjNAUX];    // auxiliary color buffer
  unsigned int auxColor_r[mjNAUX];  // auxiliary color buffer for resolving

  // materials with textures
  int mat_texid[mjMAXMATERIAL*mjNTEXROLE]; // material texture ids (-1: no texture)
  int mat_texuniform[mjMAXMATERIAL];       // uniform cube mapping
  float mat_texrepeat[mjMAXMATERIAL*2];    // texture repetition for 2d mapping

  // texture objects and info
  int ntexture;                            // number of allocated textures
  int textureType[mjMAXTEXTURE];           // type of texture (mjtTexture) (ntexture)
  unsigned int texture[mjMAXTEXTURE];      // texture names

  // displaylist starting positions
  unsigned int basePlane;           // all planes from model
  unsigned int baseMesh;            // all meshes from model
  unsigned int baseHField;          // all height fields from model
  unsigned int baseBuiltin;         // all builtin geoms, with quality from model
  unsigned int baseFontNormal;      // normal font
  unsigned int baseFontShadow;      // shadow font
  unsigned int baseFontBig;         // big font

  // displaylist ranges
  int rangePlane;                   // all planes from model
  int rangeMesh;                    // all meshes from model
  int rangeHField;                  // all hfields from model
  int rangeBuiltin;                 // all builtin geoms, with quality from model
  int rangeFont;                    // all characters in font

  // skin VBOs
  int nskin;                        // number of skins
  unsigned int* skinvertVBO;        // skin vertex position VBOs (nskin)
  unsigned int* skinnormalVBO;      // skin vertex normal VBOs (nskin)
  unsigned int* skintexcoordVBO;    // skin vertex texture coordinate VBOs (nskin)
  unsigned int* skinfaceVBO;        // skin face index VBOs (nskin)

  // character info
  int charWidth[127];               // character widths: normal and shadow
  int charWidthBig[127];            // chacarter widths: big
  int charHeight;                   // character heights: normal and shadow
  int charHeightBig;                // character heights: big

  // capabilities
  int glInitialized;                // is OpenGL initialized
  int windowAvailable;              // is default/window framebuffer available
  int windowSamples;                // number of samples for default/window framebuffer
  int windowStereo;                 // is stereo available for default/window framebuffer
  int windowDoublebuffer;           // is default/window framebuffer double buffered

  // framebuffer
  int currentBuffer;                // currently active framebuffer: mjFB_WINDOW or mjFB_OFFSCREEN

  // pixel output format
  int readPixelFormat;              // default color pixel format for mjr_readPixels

  // depth output format
  int readDepthMap;                 // depth mapping: mjDEPTH_ZERONEAR or mjDEPTH_ZEROFAR
};
typedef struct mjrContext_ mjrContext;
*/
