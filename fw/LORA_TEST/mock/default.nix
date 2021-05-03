let
 pkgs = import (builtins.fetchTarball {
                name = "nixpkgs-20.03";
                url = "https://github.com/NixOS/nixpkgs/archive/20.03.tar.gz";
                # Hash obtained using `nix-prefetch-url --unpack <url>`
                sha256 = "0182ys095dfx02vl2a20j1hz92dx3mfgz2a6fhn31bqlp1wa8hlq";
        }) {};
in
{ stdenv ? pkgs.stdenv }:
stdenv.mkDerivation {
    name = "secondary_lora-test";
    src = [ ./. ];
    buildInputs = [ pkgs.nodejs ];
    buildPhase = ''
        cp ${../SECONDARY_LORA.ino} beacon.cpp
        g++ -D ARDUINO=101 -D MOCK beacon.cpp TinyGPSPlus/src/TinyGPS++.cpp -o test -I TinyGPSPlus/src/ -I .
        ./test | xargs -n1 node ${../lora_decode.js}
    '';
    doCheck = false;
    installPhase = ''
        touch $out
    '';
}
