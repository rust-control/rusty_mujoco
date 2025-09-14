//! # [OpenGL rendering](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#opengl-rendering)
//! 
//! These functions expose the OpenGL renderer. See
//! [simulate](https://mujoco.readthedocs.io/en/stable/programming/samples.html#sasimulate)
//! for an illustration of how to use these functions.

pub use crate::bindgen::{
    mjtFontScale, mjtFont, mjtGridPos,
};

use crate::helper::{
    Rgb, Rgba,
};
use crate::{
    ObjectId, obj,
    mjModel, mjrContext, mjvScene, mjrRect, mjvFigure,
    mjtFramebuffer,
};

/// Set default mjrContext.
/// 
/// **note**: [`mjrContext`] calls this function in its `Default` implementation.
/* void mjr_defaultContext(mjrContext* con); */
pub fn mjr_defaultContext() -> mjrContext {
    let mut c = Box::<crate::bindgen::mjrContext>::new_uninit();
    unsafe { crate::bindgen::mjr_defaultContext(c.as_mut_ptr()); }
    mjrContext::from_raw(Box::into_raw(unsafe { c.assume_init() }))
}

/// Allocate resources in custom OpenGL context; fontscale is [`mjtFontScale`].
/// 
/// **note**: [`mjrContext`] calls this function in its `new` implementation.
/* void mjr_makeContext(const mjModel* m, mjrContext* con, int fontscale); */
pub fn mjr_makeContext(m: &mjModel, con: &mut mjrContext, fontscale: mjtFontScale) {
    unsafe {
        crate::bindgen::mjr_makeContext(m.as_ptr(), con.as_mut_ptr(), fontscale.0 as i32);
    }
}

/// Change font of existing context.
/* void mjr_changeFont(int fontscale, mjrContext* con); */
pub fn mjr_changeFont(fontscale: mjtFontScale, con: &mut mjrContext) {
    unsafe {
        crate::bindgen::mjr_changeFont(fontscale.0 as i32, con.as_mut_ptr());
    }
}

/// Add Aux buffer with given index to context; free previous Aux buffer.
/* void mjr_addAux(int index, int width, int height, int samples, mjrContext* con); */
pub fn mjr_addAux(
    index: usize,
    width: i32,
    height: i32,
    samples: i32,
    con: &mut mjrContext,
) {
    unsafe {
        crate::bindgen::mjr_addAux(index as i32, width, height, samples, con.as_mut_ptr());
    }
}

/// Free resources in custom OpenGL context, set to default.
/// 
/// **note**: [`mjrContext`] calls this function in its `Drop` implementation.
/* void mjr_freeContext(mjrContext* con); */
pub fn mjr_freeContext(con: &mut mjrContext) {
    unsafe { crate::bindgen::mjr_freeContext(con.as_mut_ptr()); }
    drop(unsafe { Box::from_raw(con.as_mut_ptr()) });
}

/// Resize offscreen buffers.
/* void mjr_resizeOffscreen(int width, int height, mjrContext* con); */
pub fn mjr_resizeOffscreen(width: i32, height: i32, con: &mut mjrContext) {
    unsafe {
        crate::bindgen::mjr_resizeOffscreen(width, height, con.as_mut_ptr());
    }
}

/// Upload texture to GPU, overwriting previous upload if any.
/* void mjr_uploadTexture(const mjModel* m, const mjrContext* con, int texid); */
pub fn mjr_uploadTexture(m: &mjModel, con: &mjrContext, texid: ObjectId<obj::Texture>) {
    unsafe {
        crate::bindgen::mjr_uploadTexture(m.as_ptr(), con.as_ptr(), texid.index() as i32);
    }
}

/// Upload mesh to GPU, overwriting previous upload if any.
/* void mjr_uploadMesh(const mjModel* m, const mjrContext* con, int meshid); */
pub fn mjr_uploadMesh(m: &mjModel, con: &mjrContext, meshid: ObjectId<obj::Mesh>) {
    unsafe {
        crate::bindgen::mjr_uploadMesh(m.as_ptr(), con.as_ptr(), meshid.index() as i32);
    }
}

/// Upload height field to GPU, overwriting previous upload if any.
/* void mjr_uploadHField(const mjModel* m, const mjrContext* con, int hfieldid); */
pub fn mjr_uploadHField(m: &mjModel, con: &mjrContext, hfieldid: ObjectId<obj::HField>) {
    unsafe {
        crate::bindgen::mjr_uploadHField(m.as_ptr(), con.as_ptr(), hfieldid.index() as i32);
    }
}

/// Make con->currentBuffer current again.
/* void mjr_restoreBuffer(const mjrContext* con); */
pub fn mjr_restoreBuffer(con: &mjrContext) {
    unsafe {
        crate::bindgen::mjr_restoreBuffer(con.as_ptr());
    }
}

/// Set OpenGL framebuffer for rendering: mjFB_WINDOW or mjFB_OFFSCREEN.
/// If only one buffer is available, set that buffer and ignore framebuffer argument.
/* void mjr_setBuffer(int framebuffer, mjrContext* con); */
pub fn mjr_setBuffer(framebuffer: mjtFramebuffer, con: &mut mjrContext) {
    unsafe {
        crate::bindgen::mjr_setBuffer(framebuffer.0 as i32, con.as_mut_ptr());
    }
}

/// Read pixels from current OpenGL framebuffer to client buffer.
/// Viewport is in OpenGL framebuffer; client buffer starts at (0,0).
/* void mjr_readPixels(unsigned char* rgb, float* depth,
                    mjrRect viewport, const mjrContext* con); */
pub fn mjr_readPixels(
    rgb: &mut [u8],
    depth: &mut [f32],
    viewport: mjrRect,
    con: &mjrContext,
) {
    assert_eq!(rgb.len(), (viewport.width * viewport.height * 3) as usize);
    assert_eq!(depth.len(), viewport.width as usize * viewport.height as usize);
    unsafe {
        crate::bindgen::mjr_readPixels(
            rgb.as_mut_ptr(),
            depth.as_mut_ptr(),
            viewport,
            con.as_ptr(),
        );
    }
}

/// Draw pixels from client buffer to current OpenGL framebuffer.
/// Viewport is in OpenGL framebuffer; client buffer starts at (0,0).
/* void mjr_drawPixels(const unsigned char* rgb, const float* depth,
                    mjrRect viewport, const mjrContext* con); */
pub fn mjr_drawPixels(
    rgb: &[u8],
    depth: &[f32],
    viewport: mjrRect,
    con: &mjrContext,
) {
    assert_eq!(rgb.len(), (viewport.width * viewport.height * 3) as usize);
    assert_eq!(depth.len(), viewport.width as usize * viewport.height as usize);
    unsafe {
        crate::bindgen::mjr_drawPixels(
            rgb.as_ptr(),
            depth.as_ptr(),
            viewport,
            con.as_ptr(),
        );
    }
}

/// Blit from src viewport in current framebuffer to dst viewport in other framebuffer.
/// If src, dst have different size and flg_depth==0, color is interpolated with GL_LINEAR.
/* void mjr_blitBuffer(mjrRect src, mjrRect dst,
                    int flg_color, int flg_depth, const mjrContext* con); */
pub fn mjr_blitBuffer(
    src: mjrRect,
    dst: mjrRect,
    flg_color: bool,
    flg_depth: bool,
    con: &mjrContext,
) {
    unsafe {
        crate::bindgen::mjr_blitBuffer(
            src,
            dst,
            if flg_color { 1 } else { 0 },
            if flg_depth { 1 } else { 0 },
            con.as_ptr(),
        );
    }
}

/// Set Aux buffer for custom OpenGL rendering (call restoreBuffer when done).
/* void mjr_setAux(int index, const mjrContext* con); */
pub fn mjr_setAux(index: i32, con: &mjrContext) {
    unsafe {
        crate::bindgen::mjr_setAux(index, con.as_ptr());
    }
}

/// Blit from Aux buffer to con->currentBuffer.
/* void mjr_blitAux(int index, mjrRect src, int left, int bottom, const mjrContext* con); */
pub fn mjr_blitAux(
    index: usize,
    src: mjrRect,
    left: i32,
    bottom: i32,
    con: &mjrContext,
) {
    unsafe {
        crate::bindgen::mjr_blitAux(index as i32, src, left, bottom, con.as_ptr());
    }
}

/// Draw text at (x,y) in relative coordinates; font is mjtFont.
/* void mjr_text(int font, const char* txt, const mjrContext* con,
              float x, float y, float r, float g, float b); */
pub fn mjr_text(
    font: mjtFontScale,
    txt: &str,
    con: &mjrContext,
    x: f32,
    y: f32,
    Rgb { r, g, b }: Rgb,
) {
    let c_txt = std::ffi::CString::new(txt).expect("`txt` has invalid UTF-8");
    unsafe {
        crate::bindgen::mjr_text(
            font.0 as i32,
            c_txt.as_ptr(),
            con.as_ptr(),
            x,
            y,
            r,
            g,
            b,
        );
    }
}

/// Draw text overlay; font is mjtFont; gridpos is mjtGridPos.
/* void mjr_overlay(int font, int gridpos, mjrRect viewport,
                 const char* overlay, const char* overlay2, const mjrContext* con); */
pub fn mjr_overlay(
    font: mjtFont,
    gridpos: mjtGridPos,
    viewport: mjrRect,
    overlay: &str,
    overlay2: &str,
    con: &mjrContext,
) {
    let c_overlay = std::ffi::CString::new(overlay).expect("`overlay` has invalid UTF-8");
    let c_overlay2 = std::ffi::CString::new(overlay2).expect("`overlay2` has invalid UTF-8");
    unsafe {
        crate::bindgen::mjr_overlay(
            font.0 as i32,
            gridpos.0 as i32,
            viewport,
            c_overlay.as_ptr(),
            c_overlay2.as_ptr(),
            con.as_ptr(),
        );
    }
}

/// Get maximum viewport for active buffer.
/* mjrRect mjr_maxViewport(const mjrContext* con); */
pub fn mjr_maxViewport(con: &mjrContext) -> mjrRect {
    unsafe { crate::bindgen::mjr_maxViewport(con.as_ptr()) }
}

/// Draw rectangle.
/* void mjr_rectangle(mjrRect viewport, float r, float g, float b, float a); */
pub fn mjr_rectangle(viewport: mjrRect, Rgba { r, g, b, a }: Rgba) {
    unsafe {
        crate::bindgen::mjr_rectangle(viewport, r, g, b, a);
    }
}

/// Draw rectangle with centered text.
/* void mjr_label(mjrRect viewport, int font, const char* txt,
               float r, float g, float b, float a, float rt, float gt, float bt,
               const mjrContext* con); */
pub fn mjr_label(
    viewport: mjrRect,
    font: mjtFont,
    txt: &str,
    text_color: Rgba,
    background_color: Rgb,
    con: &mjrContext,
) {
    let c_txt = std::ffi::CString::new(txt).expect("`txt` has invalid UTF-8");
    unsafe {
        crate::bindgen::mjr_label(
            viewport,
            font.0 as i32,
            c_txt.as_ptr(),
            text_color.r, text_color.g, text_color.b, text_color.a,
            background_color.r, background_color.g, background_color.b,
            con.as_ptr(),
        );
    }
}

/// Draw 2D figure.
/* void mjr_figure(mjrRect viewport, mjvFigure* fig, const mjrContext* con); */
pub fn mjr_figure(viewport: mjrRect, fig: &mut mjvFigure, con: &mjrContext) {
    unsafe {
        crate::bindgen::mjr_figure(viewport, fig, con.as_ptr());
    }
}

/// Render 3D scene.
/* void mjr_render(mjrRect viewport, mjvScene* scn, const mjrContext* con); */
pub fn mjr_render(viewport: mjrRect, scn: &mut mjvScene, con: &mjrContext) {
    unsafe {
        crate::bindgen::mjr_render(viewport, scn.as_mut_ptr(), con.as_ptr());
    }
}

/// Call glFinish.
/* void mjr_finish(void); */
pub fn mjr_finish() {
    unsafe {
        crate::bindgen::mjr_finish();
    }
}

/// Call glGetError and return result.
/* int mjr_getError(void); */
pub fn mjr_getError() -> i32 {
    unsafe { crate::bindgen::mjr_getError() }
}

/// Find first rectangle containing mouse, returning the index; `None` for not found.
/* int mjr_findRect(int x, int y, int nrect, const mjrRect* rect); */
pub fn mjr_findRect(
    x: i32,
    y: i32,
    rect: &[mjrRect],
) -> Option<usize> {
    let index = unsafe {
        crate::bindgen::mjr_findRect(x, y, rect.len() as i32, rect.as_ptr())
    };
    if index < 0 {None} else {Some(index as usize)}
}
