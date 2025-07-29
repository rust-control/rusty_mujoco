<div align="center">
    <h1>Rusty MuJoCo Binding</h1>
    <p>Rust bindings for the <a href="https://mujoco.org">MuJoCo</a> physics simulator</p>
</div>

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

## MuJoCo Version

[**3.3.2**](https://github.com/google-deepmind/mujoco/releases/tag/3.3.2)

## Requirements

- [MuJoCo 3.3.2](https://github.com/google-deepmind/mujoco/releases/tag/3.3.2) downloaded
  and expanded **as it is** (don't move or rename the files within it)
- `MUJOCO_DIR` environment variable set to the path of the MuJoCo directory (e.g. `$HOME/.mujoco/mujoco-3.3.2`)

### Note & Tips

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
  export MUJOCO_DIR="/usr/local/lib/mujoco-3.3.2"
  ```
  Or if you'd like to avoid to install MuJoCo to such a system directory:
  ```sh
  # example on Linux with $HOME/.mujoco/
  export MUJOCO_DIR="$HOME/.mujoco/mujoco-3.3.2"
  export LD_LIBRARY_PATH="$MUJOCO_DIR/lib:$LD_LIBRARY_PATH"
  ```
  
- Depending on your setting, be sure to specify `$MUJOCO_DIR/lib` as shared library path
  when executing your app (for example `LD_LIBRARY_PATH=$MUJOCO_DIR/lib cargo run` on Linux)

## Usage

*Cargo.toml*
```toml
[dependencies]
rusty_mujoco = "0.1.0"
```

*src/main.rs*
```rust
fn main() {
    // TODO DOCUMENT...
}
```

See the [examples](./examples) directory for working examples.

## Previous Works

- [ThButlah/mujoco-rs](https://github.com/TheButlah/mujoco-rs/)
  : archived by the owner on Sep 19, 2021.
- [MuJoCo-Rust/MuJoCo-Rust](https://github.com/MuJoCo-Rust/MuJoCo-Rust)
  : seems not maintained anymore (last commit on 2 years ago) and not compatible with MuJoCo 3+.

## License

rusty_mujoco is licensed under [MIT LICENSE](https://github.com/rust-control/rusty_mujoco/blob/main/LICENSE).
