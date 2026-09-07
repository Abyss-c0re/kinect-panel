# Kinect Panel

C + GTK3 control panel for an Xbox 360 Kinect: RGB / IR webcam, IR vision grade, tilt, live depth cloud, and ROCm frame-gen.

**C only.** There is no Python in this tree.

## Build (CMake)

Needs CMake ≥ 3.21, `libfreenect`, GTK 3, and a C11 compiler. HIP/ROCm is used when `hipcc` is present.

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

- **Camera** tab: preview, RGB / IR / Cloud, tilt
- **Image** tab: IR grade, Cloud clip/density/color, Frame gen boost
- RGB/IR **Camera ON** publishes **Kinect Webcam** (`v4l2loopback`):

```bash
./setup_loopback.sh --load
```

Do not stream RGB/IR and Cloud at the same time — they share the USB camera.

## Frame gen (ROCm)

`src/fg/` — HIP motion-compensated interpolation (`gfx1201`).  
**In-memory only:** two live frames + generated mids. No disk, no telemetry.

## Layout

```
CMakeLists.txt     project definition
CMakePresets.json  default / debug / cpu
src/               C + HIP
src/fg/            ROCm frame-gen library
setup_loopback.sh  one-shot v4l2loopback
```
