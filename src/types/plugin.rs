use crate::bindgen::{mjsPlugin, mjsElement, mjString};

/*
typedef struct mjsPlugin_ {        // plugin specification
  mjsElement* element;             // element type
  mjString* name;                  // instance name
  mjString* plugin_name;           // plugin name
  mjtByte active;                  // is the plugin active
  mjString* info;                  // message appended to compiler errors
} mjsPlugin;
*/

impl mjsPlugin {
    /// element type
    pub fn element(&self) -> &mjsElement {
        unsafe { &*self.element }
    }

    ///  instance name
    pub fn name(&self) -> &mjString {
        unsafe { &*self.name }
    }

    /// plugin name
    pub fn plugin_name(&self) -> &mjString {
        unsafe { &*self.plugin_name }
    }

    /// is the plugin active
    pub fn active(&self) -> bool {
        self.active != 0
    }

    /// message appended to compiler errors
    pub fn info(&self) -> &mjString {
        unsafe { &*self.info }
    }
}
