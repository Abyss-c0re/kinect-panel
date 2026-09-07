# Kinect Panel

C + GTK3 control panel for an Xbox 360 Kinect: RGB / IR webcam, IR vision grade, tilt, live depth cloud, and ROCm frame-gen.

**C only.** There is no Python in this tree.

**Kinect is OFF unless a session is live.** GUI starts dark. `--headless` is the only auto-start.
Pipeline, ports, and power law: [`docs/PIPELINE.md`](docs/PIPELINE.md).

## Build (CMake)

Needs CMake ≥ 3.21, `libfreenect`, GTK 3, and a C11 compiler. HIP uses ROCm Clang (`gfx1201`) when present.

```bash
cmake --preset default
cmake --build build
./build/bin/kinect_panel
# or
cmake --install build --prefix ~/.local
```

`make` is a thin wrapper around the same CMake configure/build.

```bash
./build/bin/fg_probe    # {"backend":"hip", ... "collect":false}
```

## Use

- **Camera** tab: preview, RGB / IR / Cloud, tilt. Camera ON starts the session.
- **Image** tab: IR grade, Cloud clip/density/color, Frame gen boost
- **Quit** / window X stops the process and powers the Kinect down. **Hide** keeps the process.
- RGB/IR session publishes **Kinect Webcam** (`v4l2loopback`):

```bash
./setup_loopback.sh --load
```

Do not stream RGB/IR and Cloud at the same time — they share the USB camera.

## Headless API

```bash
./build/bin/kinect_panel --headless --mode rgb    # or ir | cloud
./build/bin/kinect_panel --status                 # JSON SoT
./build/bin/kinect_panel --off                    # dark
./build/bin/kinect_panel --quit
```

C API: `src/kinect_session.h` (`kinect_session_off`, `kinect_session_start`, `kinect_headless_run`).
Control socket: `$XDG_RUNTIME_DIR/kinect-panel.sock` — `OFF` / `START rgb|ir|cloud` / `STATUS` / `QUIT` / `RAISE`.

## Frame gen (ROCm)

`src/fg/` — HIP motion-compensated interpolation (`gfx1201`).  
**In-memory only:** two live frames + generated mids. No disk, no telemetry.

## Layout

```
CMakeLists.txt     project definition
CMakePresets.json  default / debug / cpu
docs/PIPELINE.md   session / USB / FG plate
src/               C + HIP
src/kinect_session.*  power SoT + ctl socket
src/fg/            ROCm frame-gen library
setup_loopback.sh  one-shot v4l2loopback
```
