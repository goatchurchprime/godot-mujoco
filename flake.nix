{
  description = "Generic MuJoCo 3.5 GDExtension for Godot 4";
  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.11";
  outputs = { self, nixpkgs }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs { inherit system; };
    in {
      devShells.${system}.default = pkgs.mkShell {
        nativeBuildInputs = with pkgs; [ pkg-config python3 scons patchelf ];
        buildInputs = with pkgs; [ godot_4 mujoco ];
        MUJOCO_ROOT = "${pkgs.mujoco}";
        shellHook = ''
          export GODOT_BIN=${pkgs.godot_4}/bin/godot4
        '';
      };
    };
}
