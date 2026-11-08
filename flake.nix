{
  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
    nixpkgs-pin-mujoco-3_11_0.url = "github:nixos/nixpkgs/cc551d3bc5defdc1419e169016899bb50db52905";
    flake-parts = {
      url = "github:hercules-ci/flake-parts";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    systems = {
      url = "github:nix-systems/default";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = inputs@{
    flake-parts,
    systems,
    ...
  }: flake-parts.lib.mkFlake { inherit inputs; } {
    systems = import systems;
    perSystem = {
      system,
      pkgs,
      pkgs-pin-mujoco-3_11_0,
      ...
    }: {
      _module.args = {
        pkgs = import inputs.nixpkgs {
          inherit system;
          overlays = [ (inputs.rust-overlay.overlays.default) ];
        };
        pkgs-pin-mujoco-3_11_0 = import inputs.nixpkgs-pin-mujoco-3_11_0 {
          inherit system;
        };
      };
      devShells.default =
        let
          rustToolchain = pkgs.rust-bin.stable.latest.default.override {
            extensions = [ "rust-src" ];
          };
          mujoco-3_11_0 = pkgs-pin-mujoco-3_11_0.mujoco;
        in
        pkgs.mkShell {
          RUST_SRC_PATH = "${rustToolchain}/lib/rustlib/src/rust/library";
          RUSTFLAGS = pkgs.lib.optionalString pkgs.stdenv.isLinux "-C link-arg=-fuse-ld=bfd";
          MUJOCO_LIB = "${mujoco-3_11_0}/lib";
          packages = [
            rustToolchain
          ] ++ (with pkgs-pin-mujoco-3_11_0; [
            pkg-config
            glfw3
          ]) ++ pkgs.lib.optionals pkgs.stdenv.isLinux (with pkgs-pin-mujoco-3_11_0; [
            libx11
            wayland
          ]);
        };
    };
  };
}
