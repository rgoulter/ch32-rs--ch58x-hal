{ pkgs, lib, config, inputs, ... }:

let ch32 = inputs.rgoulter-ch32.packages.${pkgs.stdenv.system}; in
{
  devcontainer = {
    enable = true;
    settings.updateContentCommand = "";
  };

  languages = {
    rust = {
      channel = "nightly";
      components = [ "rustc" "cargo" "clippy" "llvm-tools-preview" "rustfmt" "rust-analyzer" ];
      enable = true;
      targets = ["riscv32imc-unknown-none-elf" "riscv32imac-unknown-none-elf"];
    };
    shell.enable = true;
  };

  packages = [
    pkgs.cargo-binutils
    pkgs.cargo-deny
    pkgs.cargo-nextest
    pkgs.just
    pkgs.lldb

    ch32.wchisp
  ];
}
