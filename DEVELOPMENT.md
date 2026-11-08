# Development Rules

## Setup

Please introduce Nix and  then`nix develop` would be enough for most development.

(This is recommended because Nix-workflow can cover both Nix users and non-Nix users while other ways can't cover Nix users.)

## Following Upstream's Releases

0. Create & switch to a new branch.

### MuJoCo upstream

1. Read the changelog of the version to update (and write what are changed to a temporary file if you need). As of MuJoCo 3.11.0, `https://github.com/google-deepmind/mujoco/releases/tag/${version}` serves the changelog of each release. (e.g. MuJoCo 3.11.0 changelog is available at https://github.com/google-deepmind/mujoco/releases/tag/3.11.0)

2. Download an official release from https://github.com/google-deepmind/mujoco/releases/download/${version}/mujoco-${version}-linux-x86_64.tar.gz somewhere, expand it, and replace `./vendor/include` directory with new `include` directory in the expanded release. (The header files in `include` will not be different between platforms)

### Nix

3. Search https://github.com/NixOS/nixpkgs/commits/master/pkgs/by-name/mu/mujoco commit history and find specific commit that bumps `pkgs.mujoco` to the version to update. (e.g. MuJoCo 3.11.0 is bumped by [`cc551d3bc5defdc1419e169016899bb50db52905`](https://github.com/NixOS/nixpkgs/commit/cc551d3bc5defdc1419e169016899bb50db52905))

4. Update `flake.nix`'s pinning nixpkgs into one corresponded 2., e.g. from:

    ```nix
    nixpkgs-pin-mujoco-3_10_0.url = "github:nixos/nixpkgs/775bdd0467f3de2fb5017830d00fa1f573b2ecda";
    ```

    to:

    ```nix
    nixpkgs-pin-mujoco-3_11_0.url = "github:nixos/nixpkgs/cc551d3bc5defdc1419e169016899bb50db52905";
    ```

    (with renaming all `nixpkgs-pin-mujoco-3_10_0` to `nixpkgs-pin-mujoco-3_11_0`)

5. Run `nix develop`.

### Rust

6. Bump the crate version in `Cargo.toml`. Versioning rule: basically sync with upstream version, and add patch suffix for binding-specific updates.

7. Run `cargo build --features bindgen` to generate raw binding by the build script. Then understand all compile errors and setup a series of reasonable plans for minimal fix following existing codebase and the binding policy:
    - Offers safe, low-level Rust wrappers for the entire MuJoCo API.
    - Provides getters for all struct fields and setters for user-modifiable fields,
      instead of allowing direct field access.
    - Implements automatic resource management via Rust's RAII pattern.
      For example, `mjModel` automatically calls `mj_deleteModel` in its `Drop` implementation, preventing memory leaks.
    - Emphasizes type-safety and efficiency within a lean binding interface.
      For instance, the `ObjectId<T>` type system replaces raw integer IDs to:
        - Ensure compile-time safety for all object-related operations.
        - Improve performance by eliminating the need for repeated `mj_name2id` lookups.

8. For each reason of compile errors (7.), fix it, and then, if the change is listed in the change log (2.) mark it as "resolved".

9. For remaining changes in changelog (2.), implement the new feature following the binding policy.

10. Check buildability and test success referring `.github/workflows/CI.yml`.
