"""Builds a portable RoboSBSE download for the current operating system: the desktop app, the engine jar, a
bundled Java runtime (jlink) and the examples, so that users start the app by double-clicking it.

Layouts:
  Windows  RoboSBSE-<version>-windows-x64.zip   robosbse.exe, engine/, runtime/, examples/
  Linux    RoboSBSE-<version>-linux-x64.tar.gz  robosbse, engine/, runtime/, examples/
  macOS    RoboSBSE-<version>-macos-<arch>.zip  RoboSBSE.app (Contents/MacOS/robosbse, Contents/Resources/...)

Requirements: a JDK 21 (JAVA_HOME or --jdk) with jlink, and Rust (cargo). Run from anywhere:
    python scripts/package/package.py [--jdk PATH] [--version 1.0.0] [--skip-build] [--out dist]
"""
import argparse
import os
import platform
import shutil
import subprocess
import sys
import tarfile
import zipfile

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ENGINE_DIR = os.path.join(ROOT, "backend", "java-engine")
UI_DIR = os.path.join(ROOT, "frontend", "rust-ui")
JAR = os.path.join(ENGINE_DIR, "build", "libs", "robosbse-engine.jar")
WINDOWS = sys.platform.startswith("win")
MACOS = sys.platform == "darwin"

START_HERE = {
    "windows": """RoboSBSE - Search-Based Robotics and Software Engineering Laboratory

Start: double-click robosbse.exe. Nothing to install: Java is bundled in the runtime folder.

First start: Windows may show "Windows protected your PC", because the program is not code-signed.
Click "More info", then "Run anyway".

Results are saved in the "experiments" folder next to robosbse.exe (or in %USERPROFILE%\\RoboSBSE when
this folder is read-only). Example experiments and maps are in "examples".
Documentation: README.md, and the docs folder of the source repository.
""",
    "linux": """RoboSBSE - Search-Based Robotics and Software Engineering Laboratory

Start: run ./robosbse in this folder (or double-click it in your file manager, if it allows running programs).
Nothing to install: Java is bundled in the runtime folder. The app needs OpenGL (Mesa) and X11 or Wayland.

Results are saved in the "experiments" folder next to the program (or in ~/RoboSBSE when this folder is
read-only). Example experiments and maps are in "examples".
""",
    "macos": """RoboSBSE - Search-Based Robotics and Software Engineering Laboratory

Start: double-click RoboSBSE.app (optionally move it to Applications first). Java is bundled inside the app.

First start: the app is not signed by an identified developer. Right-click RoboSBSE.app, choose "Open",
then "Open" again. If macOS reports that the app "is damaged", run this once in Terminal:
    xattr -dr com.apple.quarantine /path/to/RoboSBSE.app

Results are saved in ~/RoboSBSE/experiments (the app bundle itself is read-only).
""",
}

INFO_PLIST = """<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>CFBundleName</key><string>RoboSBSE</string>
  <key>CFBundleDisplayName</key><string>RoboSBSE</string>
  <key>CFBundleIdentifier</key><string>org.robosbse.lab</string>
  <key>CFBundleExecutable</key><string>robosbse</string>
  <key>CFBundlePackageType</key><string>APPL</string>
  <key>CFBundleShortVersionString</key><string>{version}</string>
  <key>CFBundleVersion</key><string>{version}</string>
  <key>LSMinimumSystemVersion</key><string>11.0</string>
  <key>NSHighResolutionCapable</key><true/>
</dict>
</plist>
"""


def run(cmd, cwd=None):
    print("+", " ".join(cmd), flush=True)
    subprocess.run(cmd, cwd=cwd, check=True)


def os_name():
    return "windows" if WINDOWS else "macos" if MACOS else "linux"


def arch():
    machine = platform.machine().lower()
    return "arm64" if machine in ("arm64", "aarch64") else "x64"


def find_jdk(arg):
    jdk = arg or os.environ.get("JAVA_HOME")
    if not jdk:
        sys.exit("no JDK: pass --jdk or set JAVA_HOME (a JDK 21 with jlink)")
    tool = os.path.join(jdk, "bin", "jlink" + (".exe" if WINDOWS else ""))
    if not os.path.isfile(tool):
        sys.exit(f"{tool} not found: --jdk must point at a JDK 21, not a JRE")
    return jdk


def build(jdk):
    env_java = dict(os.environ, JAVA_HOME=jdk)
    gradle = os.path.join(ENGINE_DIR, "gradlew.bat" if WINDOWS else "gradlew")
    print("+ gradlew engineJar", flush=True)
    subprocess.run([gradle, "engineJar", "--no-daemon", "--console=plain"], cwd=ENGINE_DIR, env=env_java, check=True)
    run(["cargo", "build", "--release"], cwd=UI_DIR)


def modules(jdk):
    out = subprocess.run([os.path.join(jdk, "bin", "jdeps"), "--print-module-deps", "--ignore-missing-deps",
                          "--multi-release", "21", JAR], capture_output=True, text=True, check=True).stdout
    mods = set(out.strip().splitlines()[-1].split(","))
    # Repair and improvement compile candidate patches at run time (javax.tools needs the compiler itself).
    mods |= {"jdk.compiler"}
    return ",".join(sorted(mods))


def stage(version, jdk, out):
    name = f"RoboSBSE-{version}-{os_name()}-{arch()}"
    top = os.path.join(out, name)
    shutil.rmtree(top, ignore_errors=True)
    if MACOS:
        app = os.path.join(top, "RoboSBSE.app", "Contents")
        bin_dir, res = os.path.join(app, "MacOS"), os.path.join(app, "Resources")
        os.makedirs(bin_dir)
        os.makedirs(res)
        with open(os.path.join(app, "Info.plist"), "w", encoding="utf-8") as f:
            f.write(INFO_PLIST.format(version=version))
    else:
        bin_dir = res = top
        os.makedirs(top)
    exe = "robosbse.exe" if WINDOWS else "robosbse"
    shutil.copy2(os.path.join(UI_DIR, "target", "release", exe), os.path.join(bin_dir, exe))
    os.makedirs(os.path.join(res, "engine"))
    shutil.copy2(JAR, os.path.join(res, "engine", "robosbse-engine.jar"))
    run([os.path.join(jdk, "bin", "jlink"), "--add-modules", modules(jdk), "--strip-debug", "--no-header-files",
         "--no-man-pages", "--compress=zip-6", "--output", os.path.join(res, "runtime")])
    shutil.copytree(os.path.join(ROOT, "examples"), os.path.join(res, "examples"))
    for doc in ("README.md", "LICENSE"):
        shutil.copy2(os.path.join(ROOT, doc), os.path.join(res, doc))
    with open(os.path.join(top, "START-HERE.txt"), "w", encoding="utf-8", newline="\r\n" if WINDOWS else "\n") as f:
        f.write(START_HERE[os_name()])
    return top, name


def smoke_test(res):
    """The bundled runtime runs the engine and can compile patches (repair and improvement need jdk.compiler)."""
    java = os.path.join(res, "runtime", "bin", "java" + (".exe" if WINDOWS else ""))
    jar = os.path.join(res, "engine", "robosbse-engine.jar")
    listed = subprocess.run([java, "--list-modules"], capture_output=True, text=True, check=True).stdout
    if "jdk.compiler" not in listed:
        sys.exit("the bundled runtime lacks jdk.compiler")
    run([java, "-jar", jar, "version"])
    run([java, "-jar", jar, "reproduce",
         os.path.join(res, "examples", "faulty-controllers", "regressions", "bug1-dropout-collision.json")])


def archive(top, name, out):
    if os_name() == "linux":
        path = os.path.join(out, name + ".tar.gz")
        with tarfile.open(path, "w:gz") as tar:
            tar.add(top, arcname=name)
        return path
    path = os.path.join(out, name + ".zip")
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as z:
        for folder, _, files in os.walk(top):
            for file in files:
                full = os.path.join(folder, file)
                z.write(full, os.path.join(name, os.path.relpath(full, top)))  # keeps Unix permission bits
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--jdk", help="JDK 21 home (default: JAVA_HOME)")
    ap.add_argument("--version", default="1.0.0")
    ap.add_argument("--skip-build", action="store_true", help="package the existing engine jar and release UI")
    ap.add_argument("--out", default=os.path.join(ROOT, "dist"))
    args = ap.parse_args()
    jdk = find_jdk(args.jdk)
    os.makedirs(args.out, exist_ok=True)
    if not args.skip_build:
        build(jdk)
    top, name = stage(args.version, jdk, os.path.abspath(args.out))
    smoke_test(os.path.join(top, "RoboSBSE.app", "Contents", "Resources") if MACOS else top)
    path = archive(top, name, os.path.abspath(args.out))
    print(f"{path}: {os.path.getsize(path) / 1e6:.1f} MB")


if __name__ == "__main__":
    main()
