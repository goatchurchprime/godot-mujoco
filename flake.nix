{
  description = "Generic MuJoCo 3.x GDExtension for Godot 4";
  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  outputs = { self, nixpkgs }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs { inherit system; };
      # Godot's stock Nix build embeds static libstdc++; MuJoCo's C++ XML
      # implementation must share the dynamic runtime with the host to avoid
      # symbol interposition crashes.
      godot-compatible = pkgs.godot_4_6.overrideAttrs (previous: {
        sconsFlags = builtins.filter (flag:
          !(pkgs.lib.hasPrefix "production=" flag
            || pkgs.lib.hasPrefix "debug_symbols=" flag
            || pkgs.lib.hasPrefix "lto=" flag
            || pkgs.lib.hasPrefix "use_static_cpp=" flag)
        ) previous.sconsFlags ++ [
          "production=false"
          "debug_symbols=false"
          "lto=none"
          "use_static_cpp=false"
        ];
      });
    in {
      devShells.${system}.default = pkgs.mkShell {
        nativeBuildInputs = with pkgs; [ pkg-config python3 scons patchelf ];
        buildInputs = [ godot-compatible pkgs.mujoco ];
        MUJOCO_ROOT = "${pkgs.mujoco}";
        shellHook = ''
          export GODOT_BIN=${godot-compatible}/bin/godot4
        '';
      };
    };
}
