<div align="center">
    <h1>Rusty MuJoCo Binding</h1>
    <p>Rust bindings for the <a href="https://mujoco.org">MuJoCo</a> physics simulator</p>
</div>

<div align="right">
    <a href="https://github.com/rust-control/rusty_mujoco/blob/main/LICENSE">
        <img alt="license" src="https://img.shields.io/crates/l/rusty_mujoco.svg"/>
    </a>
    <a href="https://github.com/rust-control/rusty_mujoco/actions">
        <img alt="CI" src="https://github.com/rust-control/rusty_mujoco/workflows/CI/badge.svg?branch=main"/>
    </a>
    <a href="https://crates.io/crates/rusty_mujoco">
        <img alt="crates.io" src="https://img.shields.io/crates/v/rusty_mujoco.svg"/>
    </a>
</div>

## MuJoCo Version

_**3.3.2**_

## Requirements

- [MuJoCo 3.3.2](https://github.com/google-deepmind/mujoco/releases) downloaded and expanded
- `MUJOCO_LIB` environment variable containing the MuJoCo's **`lib` directory** path

## Note & Tips

- Generally, install MuJoCo to _a default standard path_ (or in _PATH_ on Windows) and set up your shell config like:
  ```sh
  # Linux example
  export MUJOCO_LIB="/usr/lib/mujoco-3.3.2/lib"
  ```
  Or if you'd like to avoid to install MuJoCo to a system directory:
  ```sh
  # Linux example
  export MUJOCO_LIB="$HOME/.mujoco/mujoco-3.3.2/lib"
  export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$MUJOCO_LIB"
  ```
- Depending on your setting, be sure to specify `MUJOCO_LIB` when executing your app. (for example `LD_LIBRARY_PATH=$MUJOCO_LIB cargo run` on Linux)

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

## License

rusty_mujoco is licensed under [MIT LICENSE](https://github.com/rust-control/rusty_mujoco/blob/main/LICENSE).
