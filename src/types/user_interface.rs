//! # [User Interface](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html#tyuistructure)
//! 
//! For a high-level description of the UI framework,
//! see [User Interface](https://mujoco.readthedocs.io/en/stable/programming/ui.html#ui).
//! The names of these struct types are prefixed with `mjui`, except for
//! the main [`mjUI`] struct itself.

pub use crate::bindgen::{
    mjuiState, mjuiThemeSpacing, mjuiThemeColor,
    mjuiItem, mjuiItemSingle, mjuiItemMulti, mjuiItemSlider, mjuiItemEdit,
    mjuiSection, mjuiDef, mjUI,
    mjtButton, mjtEvent, mjtItem, mjtSection,
    mjfItemEnable,
    mjMAXUIRECT, mjMAXUIMULTI, mjMAXUINAME, mjMAXUIEDIT, mjMAXUIITEM, mjMAXUITEXT, mjMAXUISECT,
};
use crate::bindgen::{
    mjrRect,
};
use crate::helper::{
    Rgb, copy_str_to_c_chararray,
};

derive_fields_mapping!(mjuiState {
    boolean_flags {
        left / set_left = "is left button down";
        right / set_right = "is right button down";
        middle / set_middle = "is middle button down";
        doubleclick / set_doubleclick = "is last press a double click";
        control / set_control = "is control down";
        shift / set_shift = "is shift down";
        alt / set_alt = "is alt down";
    }
    scalars {
        mouserect: usize = "index: which rectangle contains mouse";
        dragrect: usize = "index: which rectangle is dragged with mouse";
        nrect / set_nrect: usize = "number of rectangles used";
        dropcount / set_dropcount: usize = "number of files dropped";
        key / set_key: i32 = "which key was pressed";
        buttontime / set_buttontime: f64 = "time of last button press";
        x / set_x: f64 = "x position";
        y / set_y: f64 = "y position";
        dx / set_dx: f64 = "x displacement";
        dy / set_dy: f64 = "y displacement";
        sx / set_sx: f64 = "x scroll";
        sy / set_sy: f64 = "y scroll";
        keytime / set_keytime: f64 = "time of last key press";
    }
    enums {
        dragbutton: mjtButton = "which button started drag (mjtButton)";
        type_ / set_type: mjtEvent = "(type mjtEvent)";
        button / set_button: mjtButton = "which button was pressed (mjtButton)";
    }
    structs {
        rect / rect_mut: [mjrRect; mjMAXUIRECT] = "rectangles (index 0: entire window)";
    }
});
impl mjuiState {
    pub fn userdata(&self) -> *mut std::ffi::c_void {
        self.userdata
    }

    /// Returns the paths to files dropped.
    /// 
    /// ## Panics
    /// 
    /// - if not `index < self.dropcount()`
    pub fn droppath(&self, index: usize) -> &str {
        assert!(index < self.dropcount(), "mjuiState::droppath({index}): index out of bounds; must be < {}", self.dropcount());
        let c_str_ptr = unsafe { self.droppaths.add(index).read() };
        let c_str = unsafe { std::ffi::CStr::from_ptr(c_str_ptr) };
        c_str.to_str().expect("`droppath` returned non-UTF-8 bytes")
    }
}

derive_fields_mapping!(mjuiThemeSpacing {
    scalars {
        total / set_total: u32 = "total width";
        scroll / set_scroll: u32 = "scrollbar width";
        label / set_label: u32 = "label width";
        section / set_section: u32 = "section gap";
        cornersect / set_cornersect: u32 = "corner radius for section";
        cornersep / set_cornersep: u32 = "corner radius for separator";
        itemside / set_itemside: u32 = "item side gap";
        itemmid / set_itemmid: u32 = "item middle gap";
        itemver / set_itemver: u32 = "item vertical gap";
        texthor / set_texthor: u32 = "text horizontal gap";
        textver / set_textver: u32 = "text vertical gap";
        linescroll / set_linescroll: u32 = "number of pixels to scroll";
        samples / set_samples: u32 = "number of multisamples";
    }
});

macro_rules! mjuiThemeColor_derive_rgb_mapping {
    ($($name:ident / $set_name:ident = $description:literal;)*) => {
        impl mjuiThemeColor {
            $(
                #[doc = $description]
                pub fn $name(&self) -> Rgb {
                    let [r, g, b] = self.$name;
                    Rgb { r, g, b }
                }
                #[doc = "set "]
                #[doc = $description]
                pub fn $set_name(&mut self, Rgb { r, g, b }: Rgb) -> &mut Self {
                    self.$name = [r, g, b];
                    self
                }
            )*
        }
    };
}
mjuiThemeColor_derive_rgb_mapping! {
    master / set_master = "master background";
    thumb / set_thumb = "scrollbar thumb";
    secttitle / set_secttitle = "section title";
    secttitle2 / set_secttitle2 = "section title: bottom color";
    secttitleuncheck / set_secttitleuncheck = "section title with unchecked box";
    secttitleuncheck2 / set_secttitleuncheck2 = "section title with unchecked box: bottom color";
    secttitlecheck / set_secttitlecheck = "section title with checked box";
    secttitlecheck2 / set_secttitlecheck2 = "section title with checked box: bottom color";
    sectfont / set_sectfont = "section font";
    sectsymbol / set_sectsymbol = "section symbol";
    sectpane / set_sectpane = "section pane";
    separator / set_separator = "separator title";
    separator2 / set_separator2 = "separator title: bottom color";
    shortcut / set_shortcut = "shortcut background";
    fontactive / set_fontactive = "font active";
    fontinactive / set_fontinactive = "font inactive";
    decorinactive / set_decorinactive = "decor inactive";
    decorinactive2 / set_decorinactive2 = "inactive slider color 2";
    button / set_button = "button";
    check / set_check = "check";
    radio / set_radio = "radio";
    select / set_select = "select";
    select2 / set_select2 = "select pane";
    slider / set_slider = "slider";
    slider2 / set_slider2 = "slider color 2";
    edit / set_edit = "edit";
    edit2 / set_edit2 = "edit invalid";
    cursor / set_cursor = "edit cursor";
}

// derive_fields_mapping!(mjuiItemSingle {
//     scalars {
//         modifier / set_modifier: u32 = "modifier key (0: none, 1: control, 2: shift, 4: alt)";
//         shortcut / set_shortcut: u32 = "shortcut key (0: undefined)";
//     }
// });
pub enum Modifier {
    Control = 1,
    Shift = 2,
    Alt = 4,
}
impl mjuiItemSingle {
    /// modifier key
    pub fn modifier(&self) -> Option<Modifier> {
        match self.modifier {
            0 => None,
            1 => Some(Modifier::Control),
            2 => Some(Modifier::Shift),
            4 => Some(Modifier::Alt),
            _ => panic!("mjuiItemSingle::modifier: unexpected modifier value {}", self.modifier),
        }
    }
    /// Set the modifier key
    pub fn set_modifier(&mut self, modifier: Option<Modifier>) -> &mut Self {
        self.modifier = match modifier {
            None => 0,
            Some(Modifier::Control) => 1,
            Some(Modifier::Shift) => 2,
            Some(Modifier::Alt) => 4,
        };
        self
    }

    /// shortcut key
    pub fn shortcut(&self) -> Option<u32> {
        if self.shortcut == 0 {None} else {Some(self.shortcut as u32)}
    }
    /// Set the shortcut key
    pub fn set_shortcut(&mut self, shortcut: Option<u32>) -> &mut Self {
        self.shortcut = shortcut.unwrap_or(0) as i32;
        self
    }
}

derive_fields_mapping!(mjuiItemMulti {
    scalars {
        nelem: usize = "number of elements in group (**must be** < mjMAXUIMULTI)";
    }
});
impl mjuiItemMulti {
    /// Returns the name of the element at `index`.
    /// 
    /// ## Panics
    /// 
    /// - if not `index < self.nelem()`
    pub fn name(&self, index: usize) -> &str {
        let c_str = unsafe { std::ffi::CStr::from_ptr(self.name[index].as_ptr()) };
        c_str.to_str().expect("`mjuiItemMulti::name` returned non-UTF-8 bytes")
    }
}

derive_fields_mapping!(mjuiItemSlider {
    scalars {
        divisions / set_divisions: f64 = "number of range divisions";
    }
});
impl mjuiItemSlider {
    /// slider range
    pub fn range(&self) -> std::ops::Range<f64> {
        self.range[0]..self.range[1]
    }
    /// Set slider range
    pub fn set_range(&mut self, range: std::ops::Range<f64>) -> &mut Self {
        self.range = [range.start, range.end];
        self
    }
}

derive_fields_mapping!(mjuiItemEdit {
    scalars {
        nelem: usize = "number of elements in list (**must be** < mjMAXUIEDIT)";
    }
});
impl mjuiItemEdit {
    /// Returns the range of the element at `index`, `None` for ignore.
    /// 
    /// ## Panics
    /// 
    /// - if not `index < self.nelem()`
    pub fn range(&self, index: usize) -> Option<std::ops::Range<f64>> {
        let [min, max] = self.range[index];
        if min >= max {None} else {Some(min..max)}
    }
    /// Set the range of the element at `index`, `None` for ignore.
    /// 
    /// ## Panics
    /// 
    /// - if not `index < self.nelem()`
    pub fn set_range(&mut self, index: usize, range: Option<std::ops::Range<f64>>) -> &mut Self {
        match range {
            None => self.range[index] = [f64::INFINITY, f64::NEG_INFINITY],
            Some(r) => self.range[index] = [r.start, r.end],
        }
        self
    }
}

derive_fields_mapping!(mjuiItem {
    scalars {
        state / set_state: i32 = "state (0: disable, 1: enable, 2+: use predicate)";
        sectionid / set_sectionid: i32 = "id of section containing item";
        itemid / set_itemid: i32 = "id of item within section";
        userid / set_userid: i32 = "user-supplied id (for event handling)";
        skip: i32 = "item skipped due to closed separator";
    }
    structs {
        rect: mjrRect = "rectangle occupied by item";
    }
});
impl mjuiItem {
    /// data pointer (type-specific)
    pub fn pdata(&self) -> *mut std::ffi::c_void {
        self.pdata
    }

    /// type of the item
    pub fn type_(&self) -> mjtItem {
        mjtItem(self.type_)
    }
    /// Set the type of the item
    pub fn set_type(&mut self, type_: mjtItem) -> &mut Self {
        self.type_ = type_.0;
        self
    }

    /// name
    pub fn name(&self) -> &str {
        let c_str = unsafe { std::ffi::CStr::from_ptr(self.name.as_ptr()) };
        c_str.to_str().expect("`mjuiItem::name` returned non-UTF-8 bytes")
    }
    /* not provide for union safety
    /// Set the name of the item
    pub fn set_name(&mut self, name: &str) -> &mut Self {
        copy_str_to_c_chararray(name, &mut self.name);
        self
    }
    */
    /* mjtItem::{END, SECTION, SEPARATOR, STATIC} do not have any union properties. */
    /// check and button properties for: `mjtItem::{BUTTON, CHECKINT, CHECKBYTE}`
    pub fn single(&self) -> Option<&mjuiItemSingle> {
        match self.type_() {
            mjtItem::BUTTON | mjtItem::CHECKINT | mjtItem::CHECKBYTE => Some(unsafe { &self.__bindgen_anon_1.single }),
            _ => None,
        }
    }
    /// static, radio and select properties for: `mjtItem::{RADIO, RADIOLINE, SELECT}`
    pub fn multi(&self) -> Option<&mjuiItemMulti> {
        match self.type_() {
            mjtItem::RADIO | mjtItem::RADIOLINE | mjtItem::SELECT => Some(unsafe { &self.__bindgen_anon_1.multi }),
            _ => None,
        }
    }
    /// slider properties for: `mjtItem::{SLIDERINT, SLIDERNUM}`
    pub fn slider(&self) -> Option<&mjuiItemSlider> {
        match self.type_() {
            mjtItem::SLIDERINT | mjtItem::SLIDERNUM => Some(unsafe { &self.__bindgen_anon_1.slider }),
            _ => None,
        }
    }
    /// edit properties for: `mjtItem::{EDITINT, EDITNUM, EDITFLOAT, EDITTXT}`
    pub fn edit(&self) -> Option<&mjuiItemEdit> {
        match self.type_() {
            mjtItem::EDITINT | mjtItem::EDITNUM | mjtItem::EDITFLOAT | mjtItem::EDITTXT => Some(unsafe { &self.__bindgen_anon_1.edit }),
            _ => None,
        }
    }
}

derive_fields_mapping!(mjuiSection {
    scalars {
        lastclick: i32 = "last mouse click over this section";
        modifier / set_modifier: u32 = "modifier key (0: none, 1: control, 2: shift, 4: alt)";
        shortcut / set_shortcut: u32 = "shortcut key (0: undefined)";
        checkbox / set_checkbox: i32 = "0: none, 1: unchecked, 2: checked";
        nitem / set_nitem: usize = "number of items in use";
    }
    enums {
        state / set_state: mjtSection = "section state (mjtSection)";
    }
    structs {
        rtitle: mjrRect = "rectangle occupied by title";
        rcontent: mjrRect = "rectangle occupied by content";
    }
});
impl mjuiSection {
    /// name
    pub fn name(&self) -> &str {
        let c_str = unsafe { std::ffi::CStr::from_ptr(self.name.as_ptr()) };
        c_str.to_str().expect("`mjuiSection::name` returned non-UTF-8 bytes")
    }
    /// Set the name of the section
    pub fn set_name(&mut self, name: &str) -> &mut Self {
        copy_str_to_c_chararray(name, &mut self.name);
        self
    }
}

derive_fields_mapping!(mjuiDef {
    scalars {
        type_ / set_type: i32 = "type (mjtItem), -1: section";
        state / set_state: i32 = "state";
        otherint / set_otherint: i32 = "int with type-specific properties";
    }
});
impl mjuiDef {
    /// data pointer (type-specific)
    pub fn pdata(&self) -> *mut std::ffi::c_void {
        self.pdata
    }

    /// string with type-specific properties
    pub fn other(&self) -> &str {
        let c_str = unsafe { std::ffi::CStr::from_ptr(self.other.as_ptr()) };
        c_str.to_str().expect("`mjuiDef::other` returned non-UTF-8 bytes")
    }
    /// Set the string with type-specific properties
    pub fn set_other(&mut self, other: &str) -> &mut Self {
        copy_str_to_c_chararray(other, &mut self.other);
        self
    }

    /// name
    pub fn name(&self) -> &str {
        let c_str = unsafe { std::ffi::CStr::from_ptr(self.name.as_ptr()) };
        c_str.to_str().expect("`mjuiDef::name` returned non-UTF-8 bytes")
    }
    /// Set the name of the definition
    pub fn set_name(&mut self, name: &str) -> &mut Self {
        copy_str_to_c_chararray(name, &mut self.name);
        self
    }
}

/*
mjUI
// scalars; read&write
  int rectid;                     // index of this ui rectangle in mjuiState
  int auxid;                      // aux buffer index of this ui
  int radiocol;                   // number of radio columns (0 defaults to 2)
// scalars; read-only
  int width;                      // width
  int height;                     // current height
  int maxheight;                  // height when all sections open
  int scroll;                     // scroll from top of UI
  int mousesect;                  // 0: none, -1: scroll, otherwise 1+section
  int mouseitem;                  // item within section
  int mousehelp;                  // help button down: print shortcuts
  int mouseclicks;                // number of mouse clicks over UI
  int mousesectcheck;             // 0: none, otherwise 1+section
  int editsect;                   // 0: none, otherwise 1+section
  int edititem;                   // item within section
  int editcursor;                 // cursor position
  int editscroll;                 // horizontal scroll
  int nsect;                      // number of sections in use

// structs; read&write
  mjuiThemeSpacing spacing;       // UI theme spacing
  mjuiThemeColor color;           // UI theme color
// structs; read-only
  mjuiSection sect[mjMAXUISECT];  // preallocated array of sections
  
// by hand
  void* userdata;                 // pointer to user data (passed to predicate)
  char edittext[mjMAXUITEXT];     // current text
  mjuiItem* editchanged;          // pointer to changed edit in last mjui_event
  mjfItemEnable predicate;        // callback to set item state programmatically
*/
derive_fields_mapping!(mjUI {
    scalars {
        rectid / set_rectid: usize = "index of this ui rectangle in mjuiState";
        auxid / set_auxid: usize = "aux buffer index of this ui";
        radiocol / set_radiocol: usize = "number of radio columns (0 defaults to 2)";
        width: usize = "width";
        height: usize = "current height";
        maxheight: usize = "height when all sections open";
        scroll: f64 = "scroll from top of UI";
        mousesect: i32 = "0: none, -1: scroll, otherwise 1+section";
        mouseitem: i32 = "item within section";
        mousehelp: i32 = "help button down: print shortcuts";
        mouseclicks: usize = "number of mouse clicks over UI";
        mousesectcheck: i32 = "0: none, otherwise 1+section";
        editsect: i32 = "0: none, otherwise 1+section";
        edititem: i32 = "item within section";
        editcursor: usize = "cursor position";
        editscroll: f64 = "horizontal scroll";
        nsect: usize = "number of sections in use";
    }
    structs {
        spacing / set_spacing: mjuiThemeSpacing = "UI theme spacing";
        color / set_color: mjuiThemeColor = "UI theme color";
        sect: [mjuiSection; mjMAXUISECT] = "preallocated array of sections";
    }
});
impl mjUI {
    /// data pointer (type-specific)
    pub fn userdata(&self) -> *mut std::ffi::c_void {
        self.userdata
    }

    /// current text in the edit field
    pub fn edittext(&self) -> &str {
        let c_str = unsafe { std::ffi::CStr::from_ptr(self.edittext.as_ptr()) };
        c_str.to_str().expect("`mjUI::edittext` returned non-UTF-8 bytes")
    }
    /// Set the current text in the edit field
    pub fn set_edittext(&mut self, text: &str) -> &mut Self {
        copy_str_to_c_chararray(text, &mut self.edittext);
        self
    }

    /// Pointer to changed edit in last `mjui_event`
    pub fn editchanged(&self) -> Option<&mjuiItem> {
        if self.editchanged.is_null() {None} else {Some(unsafe {&*self.editchanged})}
    }

    /// Predicate callback to set item state programmatically
    pub fn predicate(&self) -> &mjfItemEnable {
        &self.predicate
    }
    /// Set the predicate callback to set item state programmatically
    pub fn set_predicate(&mut self, predicate: mjfItemEnable) -> &mut Self {
        self.predicate = predicate;
        self
    }
}
