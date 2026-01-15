{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs";
    flake-utils.url = "github:numtide/flake-utils";
    crane.url = "github:ipetkov/crane";
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
      crane,
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
        };
        craneLib = crane.mkLib pkgs;
        src = craneLib.cleanCargoSource ./.;
        libs = [
          pkgs.wayland # For wayland support
          pkgs.vulkan-loader # Graphics backend
        ];
        commonArgs = {
          inherit src;
          strictDeps = true;

          buildInputs = libs;

          # Additional environment variables can be set directly
          # MY_CUSTOM_VAR = "some value";
        };

        cargoArtifacts = craneLib.buildDepsOnly commonArgs;
        crate = craneLib.buildPackage (
          commonArgs
          // {
            inherit cargoArtifacts;
          }
        );
      in
      {
        packages = {
          default = crate;
        };

        devShell = craneLib.devShell {
          packages = [
            pkgs.rust-analyzer
            pkgs.just
          ];

          env = {
            # Required by rust-analyzer
            RUST_SRC_PATH = "${pkgs.rust.packages.stable.rustPlatform.rustLibSrc}";
            # Required to make `cargo run` work.
            LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath libs;
          };
        };

        formatter = pkgs.nixfmt-tree;
      }
    )
    // {
      inherit self;
    };
}
