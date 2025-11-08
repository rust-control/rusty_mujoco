<div align="center">
    <h1>Rusty MuJoCo Binding</h1>
    <p>Rust bindings for the <a href="https://mujoco.org">MuJoCo</a> physics simulator</p>
    <p>MuJoCo Version: <a href="https://github.com/google-deepmind/mujoco/releases/tag/3.3.2">3.3.2</a></p>
</div>

<br>

- Offers safe, low-level Rust wrappers for the entire MuJoCo API.
- Provides getters for all struct fields and setters for user-modifiable fields,
  instead of allowing direct field access.
- Implements automatic resource management via Rust's RAII pattern.
  For example, `mjModel` automatically calls `mj_deleteModel` in its `Drop` implementation, preventing memory leaks.
- Emphasizes type-safety and efficiency within a lean binding interface.
  For instance, the `ObjectId<T>` type system replaces raw integer IDs to:
    - Ensure compile-time safety for all object-related operations.
    - Improve performance by eliminating the need for repeated `mj_name2id` lookups.

<div align="right">
    <a href="https://github.com/rust-control/rusty_mujoco/blob/main/LICENSE">
        <img alt="license" src="https://img.shields.io/crates/l/rusty_mujoco.svg"/>
    </a>
    <a href="https://github.com/rust-control/rusty_mujoco/actions">
        <img alt="CI" src="https://github.com/rust-control/rusty_mujoco/actions/workflows/CI.yml/badge.svg?branch=main"/>
    </a>
    <a href="https://crates.io/crates/rusty_mujoco">
        <img alt="crates.io" src="https://img.shields.io/crates/v/rusty_mujoco.svg"/>
    </a>
</div>

<br>

## Requirements

- [MuJoCo 3.3.2](https://github.com/google-deepmind/mujoco/releases/tag/3.3.2) downloaded
  and expanded **as it is** (don't move or rename the files within it)
- `MUJOCO_LIB` environment variable set to the path of a directory containing
  `libmujoco.so` or `mujoco.lib` of MuJoCo 3.3.2
  (e.g. `$HOME/.mujoco/mujoco-3.3.2/lib` when you placed the official release above in `~/.mujoco`)

### Note / Tips

- For example on x86_64 Linux, run:
  ```sh
  wget https://github.com/google-deepmind/mujoco/releases/download/3.3.2/mujoco-3.3.2-linux-x86_64.tar.gz
  tar -xzf mujoco-3.3.2-linux-x86_64.tar.gz
  ```
  to download & expand MuJoCo 3.3.2.\
  On other platforms, do the same with the appropriate archive file for your system.
  
- One way to setup is to install MuJoCo to _a default standard path_ like `/usr/local/lib/`
  (or a folder in _PATH_ on Windows), then if needed create symlink to `mujoco-3.3.2/lib/libmujoco.so` there,
  and insert to your shell config file:
  ```sh
  # example on Linux with /usr/local/lib/
  export MUJOCO_LIB="/usr/local/lib/mujoco-3.3.2/lib"
  ```
  Or if you'd like to avoid to install MuJoCo to such a system directory:
  ```sh
  # example on Linux with $HOME/.mujoco/
  export MUJOCO_LIB="$HOME/.mujoco/mujoco-3.3.2/lib"
  export LD_LIBRARY_PATH="$MUJOCO_LIB:$LD_LIBRARY_PATH"
  ```
  
- Depending on your setting, be sure to specify `$MUJOCO_LIB` as shared library path
  when executing your app (for example `LD_LIBRARY_PATH=$MUJOCO_LIB cargo run` on Linux)

## Example

```toml
[dependencies]
rusty_mujoco = "0.2"
glfw = "0.60"
```

```rust,no_run
use rusty_mujoco::{mj_loadXML, mj_makeData, mj_name2id, mj_step, mjr_render, mjv_updateScene};
use rusty_mujoco::{mjrContext, mjrRect, mjvScene, mjvCamera, mjvOption, mjtCatBit, mjtFontScale};

let xml_path: String = todo!();
let camera_name: Option<String> = todo!();

let model = mj_loadXML(xml_path).expect("Failed to load XML file");
let mut data = mj_makeData(&model);

let mut glfw = glfw::init(glfw::fail_on_errors).expect("Failed to initialize GLFW");
let (mut window, _) = glfw
    .create_window(1200, 900, "Acrobot Simulation", glfw::WindowMode::Windowed)
    .expect("Failed to create GLFW window");
window.set_size_polling(true);
window.set_close_polling(true);
glfw::Context::make_current(&mut *window);

let con = mjrContext::new(&model, mjtFontScale::X150);
let opt = mjvOption::default();
let mut scn = mjvScene::new(&model, 2000);
let mut cam = mjvCamera::default();
camera_name.map(|name| cam.set_fixedcamid(mj_name2id(&model, &name).expect("No camera of such name in the model")));

while !window.should_close() {
    while data.time() < glfw.get_time() {
        mj_step(&model, &mut data);
    }
    
    let viewport = {
        let (width, height) = window.get_framebuffer_size();
        mjrRect::new(0, 0, width as u32, height as u32)
    };
    
    mjv_updateScene(
        &model,
        &mut data,
        &opt,
        None, /* No perturbation */
        &mut cam,
        mjtCatBit::ALL,
        &mut scn,
    );
    mjr_render(viewport, &mut scn, &con);
    
    glfw::Context::swap_buffers(&mut *window);
    glfw.poll_events();
}
```

See [examples/visualize_left_object.rs](./examples/visualize_left_object.rs) for full example
and [examples/README.md](./examples/README.md) for the description.

## Previous Works

- [TheButlah/mujoco-rs](https://github.com/TheButlah/mujoco-rs)
  : archived by the owner on Sep 19, 2021.
- [MuJoCo-Rust/MuJoCo-Rust](https://github.com/MuJoCo-Rust/MuJoCo-Rust)
  : seems not maintained anymore (last commit on 2 years ago) and not compatible with MuJoCo 3+.

## License

rusty_mujoco is licensed under [Apache Lisense, Version 2.0)](https://github.com/rust-control/rusty_mujoco/blob/main/LICENSE).
