use crate::{MjsElement, MjString};

wrapper! {
    /// Plugin specification.
    MjsPlugin of crate::bindgen::mjsPlugin
}

/*
typedef struct mjsPlugin_ {        // plugin specification
  mjsElement* element;             // element type
  mjString* name;                  // instance name
  mjString* plugin_name;           // plugin name
  mjtByte active;                  // is the plugin active
  mjString* info;                  // message appended to compiler errors
} mjsPlugin;
*/

impl MjsPlugin {
    /// element type
    pub fn element(&self) -> &MjsElement {
        (unsafe { &*self.0.element }).into()
    }

    ///  instance name
    pub fn name(&self) -> &MjString {
        (unsafe { &*self.0.name }).into()
    }

    /// plugin name
    pub fn plugin_name(&self) -> &MjString {
        (unsafe { &*self.0.plugin_name }).into()
    }

    /// is the plugin active
    pub fn active(&self) -> bool {
        self.0.active != 0
    }

    /// message appended to compiler errors
    pub fn info(&self) -> &MjString {
        (unsafe { &*self.0.info }).into()
    }
}
