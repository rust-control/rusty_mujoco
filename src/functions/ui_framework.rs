//! # [UI framework](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#ui-framework)
//! 
//! For a high-level description of the UI framework, see [User Interface](https://mujoco.readthedocs.io/en/stable/programming/ui.html#ui).

use crate::{mjUI, mjrContext, mjuiDef, mjuiItem, mjuiThemeColor, mjuiThemeSpacing};

/// Get builtin UI theme spacing (ind: 0-1).
/* mjuiThemeSpacing mjui_themeSpacing(int ind); */
pub fn mjui_themeSpacing(ind: i32) -> mjuiThemeSpacing {
    unsafe { crate::bindgen::mjui_themeSpacing(ind) }
}

/// Get builtin UI theme color (ind: 0-3).
/* mjuiThemeColor mjui_themeColor(int ind); */
pub fn mjui_themeColor(ind: i32) -> mjuiThemeColor {
    unsafe { crate::bindgen::mjui_themeColor(ind) }
}

/// This is the helper function used to construct a UI.
/// 
/// The second argument points to an array of [`mjuiDef`] structs,
/// each corresponding to one item. The last (unused) item has its
/// type set to -1, to mark termination.
/// 
/// The items are added after the end of the last used section.
/// There is also another version of this function ([`mjui_addToSection`])
/// which adds items to a specified section instead of adding them
/// at the end of the UI. Keep in mind that there is a maximum preallocated
/// number of sections and items per section, given by
/// [`mjMAXUISECT`](crate::mjMAXUISECT) and [`mjMAXUIITEM`](crate::mjMAXUIITEM).
/// Exceeding these maxima results in low-level errors.
/* void mjui_add(mjUI* ui, const mjuiDef* def); */
pub fn mjui_add(ui: &mut mjUI, def: impl AsRef<[mjuiDef]>) {
    let def: &[mjuiDef] = def.as_ref();
    if def.last().unwrap().type_() == -1 {
        unsafe { crate::bindgen::mjui_add(ui, def.as_ptr()) }
    } else {
        #[cold]
        #[inline(never)]
        fn with_dynamically_addiing_terminator(ui: &mut mjUI, def: &[mjuiDef]) {
            let mut def = def.to_vec();
            def.push(mjuiDef { type_: -1, name: [0; crate::mjMAXUINAME], state: 0, pdata: std::ptr::null_mut(), other: [0; crate::mjMAXUITEXT], otherint: 0 });
            unsafe { crate::bindgen::mjui_add(ui, def.as_ptr()); }
        }
        with_dynamically_addiing_terminator(ui, def);
    }
}

/// Add definitions to UI section. See [`mjui_add`] for details.
/* void mjui_addToSection(mjUI* ui, int sect, const mjuiDef* def); */
pub fn mjui_addToSection(
    ui: &mut mjUI,
    sect: i32,
    def: impl AsRef<[mjuiDef]>,
) {
    let def: &[mjuiDef] = def.as_ref();
    if def.last().unwrap().type_() == -1 {
        unsafe { crate::bindgen::mjui_addToSection(ui, sect, def.as_ptr()) }
    } else {
        #[cold]
        #[inline(never)]
        fn with_dynamically_addiing_terminator(
            ui: &mut mjUI,
            sect: i32,
            def: &[mjuiDef],
        ) {
            let mut def = def.to_vec();
            def.push(mjuiDef { type_: -1, name: [0; crate::mjMAXUINAME], state: 0, pdata: std::ptr::null_mut(), other: [0; crate::mjMAXUITEXT], otherint: 0 });
            unsafe { crate::bindgen::mjui_addToSection(ui, sect, def.as_ptr()); }
        }
        with_dynamically_addiing_terminator(ui, sect, def);
    }
}

/// Compute UI sizes.
/* void mjui_resize(mjUI* ui, const mjrContext* con); */
pub fn mjui_resize(ui: &mut mjUI, con: &mjrContext) {
    unsafe { crate::bindgen::mjui_resize(ui, con) }
}

/// This is the main UI update function. It needs to be called whenever
/// the user data (pointed to by the item data pointers) changes, or when
/// the UI state itself changes. It is normally called by a higher-level function
/// implemented by the user (`UiModify` in [simulate.cc](https://mujoco.readthedocs.io/en/stable/programming/samples.html#sasimulate))
/// which also recomputes the layout of all rectangles and associated
/// auxiliary buffers. The function updates the pixels in the offscreen
/// OpenGL buffer. To perform minimal updates, the user specifies the section
/// and the item that was modified. A value of `None` means all items and/or sections
/// need to be updated (which is needed following major changes.)
/* void mjui_update(int section, int item, const mjUI* ui,
                 const mjuiState* state, const mjrContext* con); */
pub fn mjui_update(
    section: Option<u32>,
    item: Option<u32>,
    ui: &mjUI,
    state: &crate::mjuiState,
    con: &mjrContext,
) {
    unsafe {
        crate::bindgen::mjui_update(
            section.map_or(-1, |s| s as i32),
            item.map_or(-1, |s| s as i32),
            ui,
            state,
            con,
        )
    }
}

/// This function is the low-level event handler. It makes
/// the necessary changes in the UI and returns a pointer to the item
/// that received the event (or `None` if no valid event was recorded).
/// 
/// This is normally called within the event handler implemented by the user
/// (`UiEvent` in [simulate.cc](https://mujoco.readthedocs.io/en/stable/programming/samples.html#sasimulate)),
/// and then some action is taken by user code depending on which UI item was
/// modified and what the state of that item is after the event is handled.
/* mjuiItem* mjui_event(mjUI* ui, mjuiState* state, const mjrContext* con); */
pub fn mjui_event<'ui>(
    ui: &'ui mut mjUI,
    state: &mut crate::mjuiState,
    con: &mjrContext,
) -> Option<&'ui mut mjuiItem> {
    let item = unsafe { crate::bindgen::mjui_event(ui, state, con) };
    if item.is_null() {None} else {Some(unsafe { &mut *item })}
}

/// This function is called in the screen refresh loop. It copies
/// the offscreen OpenGL buffer to the window framebuffer.
/// 
/// If there are multiple UIs in the application, it should be called
/// once for each UI. Thus `mjui_render` is called all the time,
/// while `mjui_update` is called only when changes in the UI take place.
/// dsffsdg
/* void mjui_render(mjUI* ui, const mjuiState* state, const mjrContext* con); */
pub fn mjui_render(ui: &mut mjUI, state: &crate::mjuiState, con: &mjrContext) {
    unsafe { crate::bindgen::mjui_render(ui, state, con) }
}
