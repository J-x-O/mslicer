{
  description = "mslicer dev environment";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs =
    { nixpkgs, ... }:
    let
      systems = [
        "x86_64-linux"
        "aarch64-linux"
      ];
      forAllSystems = f: nixpkgs.lib.genAttrs systems (system: f nixpkgs.legacyPackages.${system});

      # Libraries winit / wgpu load at runtime via dlopen.
      runtimeLibs =
        pkgs: with pkgs; [
          libxkbcommon
          wayland
          vulkan-loader
          libGL
          xorg.libX11
          xorg.libXcursor
          xorg.libXi
          xorg.libXrandr
        ];
    in
    {
      devShells = forAllSystems (pkgs: {
        default = pkgs.mkShell {
          LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath (runtimeLibs pkgs);
        };
      });

      # `nix run ./nix [-- args]` builds and launches mslicer from the repo root
      # using the cargo already on your PATH.
      apps = forAllSystems (
        pkgs:
        let
          run = pkgs.writeShellApplication {
            name = "run-mslicer";
            runtimeInputs = [ pkgs.git ];
            text = ''
              export LD_LIBRARY_PATH="${pkgs.lib.makeLibraryPath (runtimeLibs pkgs)}''${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
              cd "$(git rev-parse --show-toplevel)"
              exec cargo run -p mslicer --release -- "$@"
            '';
          };
        in
        {
          default = {
            type = "app";
            program = "${run}/bin/run-mslicer";
          };
        }
      );
    };
}
