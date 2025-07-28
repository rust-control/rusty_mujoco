## `visualize_left_object.rs`

This example leaves a single object in the scene and visualizes it.

### Requirements

- Of course [`rusty_mujoco` Requirements](https://github.com/rust-control/rusty_mujoco?tab=readme-ov-file#requirements)
- Additionally [`glfw` Prerequisites](https://github.com/PistonDevelopers/glfw-rs?tab=readme-ov-file#prerequisites)

### Usage

Pass the path to a MuJoCo model XML file as an argument. For example,
you can use the `humanoid.xml` model provided by MuJoCo:

```sh
cargo run --example visualize_left_object -- $MUJOCO_DIR/model/humanoid/humanoid.xml
```

Options:

- `--camera <camera name>`: Specify the name of camera to use for visualization (optional). 
  If not provided, the default camera will be used.
  - example: `--camera side` for the humanoid model

Depending on your system, you may need to give:

- `MUJOCO_DIR` environment variable, to the MuJoCo directory path (e.g. `$HOME/.mujoco/mujoco-3.3.2`)
- `LD_LIBRARY_PATH` (Linux), `DYLD_LIBRARY_PATH` (macOS), or `PATH` (Windows) configuration
  for searching the MuJoCo library path

like `LD_LIBRARY_PATH="$MUJOCO_DIR/lib" cargo run --example visualize_left_object -- $MUJOCO_DIR/model/humanoid.xml`
