{
  description = "CREATE 99L Communication Board development shell";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { nixpkgs, ... }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs { inherit system; };
    in
    {
      devShells.${system}.default = pkgs.mkShell {
        packages = [
          pkgs.rustup
          pkgs.espup
          pkgs.espflash
          pkgs.pkg-config
          pkgs.git
        ];

        shellHook = ''
          export CARGO_TARGET_DIR="''${XDG_CACHE_HOME:-$HOME/.cache}/99l-comboard-target"

          if [ -f "$HOME/export-esp.sh" ]; then
            source "$HOME/export-esp.sh"
          else
            echo "ESP Rust toolchain is not installed."
            echo "Run: espup install"
          fi

          echo "CARGO_TARGET_DIR=$CARGO_TARGET_DIR"
        '';
      };
    };
}
