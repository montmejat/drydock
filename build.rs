//! Finds MuJoCo (a release from github.com/google-deepmind/mujoco, unpacked:
//! `MUJOCO_DIR`, or where `scripts/setup.sh` puts it), compiles the C shim
//! against its headers with the system's `cc`, and links both.

use std::env;
use std::path::PathBuf;
use std::process::Command;

/// The release the shim is written against.
const VERSION: &str = "3.16.0";

fn main() {
    println!("cargo:rerun-if-env-changed=MUJOCO_DIR");
    println!("cargo:rerun-if-changed=shim/world.c");
    let dir = match env::var_os("MUJOCO_DIR") {
        Some(dir) => PathBuf::from(dir),
        None => PathBuf::from(env::var_os("HOME").unwrap_or_default()).join(format!(".cache/drydock/mujoco-{VERSION}")),
    };
    let (include, lib) = (dir.join("include"), dir.join("lib"));
    if !include.join("mujoco/mujoco.h").exists() || !lib.join("libmujoco.so").exists() {
        panic!("no MuJoCo {VERSION} in {}: scripts/setup.sh fetches it, or set MUJOCO_DIR", dir.display());
    }

    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let object = out.join("world.o");
    let cc = env::var("CC").unwrap_or_else(|_| "cc".into());
    let status = Command::new(&cc)
        .args(["-c", "-O2", "-fPIC", "-std=c11", "-Wall", "-Wextra"])
        .arg("-I")
        .arg(&include)
        .arg("shim/world.c")
        .arg("-o")
        .arg(&object)
        .status()
        .unwrap_or_else(|e| panic!("can't run the C compiler `{cc}`: {e}"));
    assert!(status.success(), "the C shim doesn't compile");
    let archive = out.join("libdrydock_world.a");
    let _ = std::fs::remove_file(&archive);
    let status = Command::new("ar").arg("crs").arg(&archive).arg(&object).status().expect("can't run `ar`");
    assert!(status.success(), "`ar` failed");

    println!("cargo:rustc-link-search=native={}", out.display());
    println!("cargo:rustc-link-lib=static=drydock_world");
    println!("cargo:rustc-link-search=native={}", lib.display());
    println!("cargo:rustc-link-lib=dylib=mujoco");
    // Found at run time where it was at build time.
    println!("cargo:rustc-link-arg=-Wl,-rpath,{}", lib.display());
}
