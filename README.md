# Kinect Panel

C + GTK3 control panel for an Xbox 360 Kinect and a Kinect 2: RGB / IR from either camera, IR vision grade, tilt, live depth clouds, ROCm frame-gen, and a C library that draws that cloud from the player’s OpenXR pose.

**C only**, except `src/kinect2_device.cpp`, which wraps libfreenect2 (it has no C API). There is no Python in this tree.

**Both sensors are OFF unless a session is live.** GUI starts dark. `--headless` is the only auto-start.
Pipeline, ports, and power law: [`docs/PIPELINE.md`](docs/PIPELINE.md).

## Build (CMake)

Needs CMake ≥ 3.21, `libfreenect`, GTK 3, and a C11 compiler. Kinect 2 also needs `libfreenect2` in `/usr/local` (udev `90-kinect2.rules`, USB `045e:02c4`). Without that library the build still succeeds and Kinect 2 calls return “not built”. HIP uses ROCm Clang (`gfx1201`) when present.

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

- **Camera** tab: preview, RGB / IR / Cloud, sensor buttons Auto / Kinect / Kinect 2 / Both, Align, tilt. Camera ON starts the session. The sensor applies to RGB, IR, and Cloud. It does not switch the view.
- **F11** fills the screen with the current view (RGB, IR, or the point cloud). F11 or Esc leaves it. In that view, Left and Right cycle Auto / Kinect / Kinect 2 / Both, and Up and Down cycle RGB / IR / Cloud.
- **Share** listens on UDP port 8765 (`KINECT_SHARE_PORT`). **Connect** dials `host[:port]` or `KINECT_PEER`. The link is latest-frame only: depth moves as 8×8 block deltas, previews as 320×240 RGB565. A sensor this machine does not have shows up on the same Auto / Kinect / Kinect 2 / Both control. Share does not turn a camera on. `./build/bin/kinect_share --listen 8765` is the headless side (no GTK).
- **Image** tab: IR grade, Cloud clip/density/color, Align (fits Kinect 2 onto Kinect), Frame gen boost
- **Quit** / window X stops the process and powers both sensors down. **Hide** keeps the process.
- RGB/IR session publishes **Kinect Webcam** (`v4l2loopback`):

```bash
./setup_loopback.sh --load
```

Do not stream RGB/IR and Cloud at the same time. RGB/IR uses the selected camera: Kinect (gspca) or Kinect 2 color / IR (libfreenect2, scaled to 640×480). **Both** in RGB or IR shows Kinect 2. A cloud can be Kinect, Kinect 2, or both at once. Kinect 2 color does not turn the IR projector on. IR does.

Point clouds are paired on the host clock (no hardware genlock) and, after **Align**, one rigid transform. Until then the rig is identity, and Kinect 2 points are tinted blue. The rigid is stored in `$KINECT_EXTRINSIC` or `$CUBE_TRACK_DIR/kinect2_extrinsic.txt`.

`KINECT_SRC=auto|v1|v2|both` picks the sensor for RGB, IR, and Cloud. Auto uses the Kinect webcam when that node exists, otherwise Kinect 2. `KINECT2_PIPELINE=cpu|cl|gl|clkde` picks the Kinect 2 packet pipeline. **cpu** is the default. ROCm OpenCL often rejects the libfreenect2 kernels. Color JPEG is TurboJPEG (`KINECT2_RGB=vaapi` opts back into the hardware decoder, which is a flat green frame on the AMD driver).

## Headless API

```bash
./build/bin/kinect_panel --headless --mode rgb    # or ir | cloud
./build/bin/kinect_panel --headless --src v2 --mode rgb   # Kinect 2 color
./build/bin/kinect_panel --headless --src v2 --mode ir    # Kinect 2 IR
./build/bin/kinect_panel --headless --mode v2     # Kinect 2 cloud only
./build/bin/kinect_panel --headless --src both --mode cloud
./build/bin/kinect_panel --status                 # JSON SoT, does not touch USB
./build/bin/kinect_panel --off                    # dark
./build/bin/kinect_panel --quit
./build/bin/kinect_sync_proof                     # offline pair + align
```

C API: `src/kinect_session.h` (`kinect_session_off`, `kinect_session_start`, `kinect_headless_run`).
Control socket: `$XDG_RUNTIME_DIR/kinect-panel.sock` — `OFF` / `START rgb|ir|cloud [v1|v2|both|auto]` / `ALIGN` / `ALIGN RESET` / `STATUS` / `QUIT` / `RAISE`.
`START cloud` with no suffix keeps the current sensor. `v1` and `v2` in the status line mean that sensor is streaming, not merely plugged. `skew_ms` is `-1` when the frame is not a pair.

## OpenXR (`libkinect_xr`)

The panel orbit view is **not** the player. `src/xr/kinect_xr.h` locks the
Kinect-frame cloud to the OpenXR head pose and projects with the compositor
FOV so the room stays put when you turn.

```bash
./build/bin/kxr_proof                          # lock + perspective selftest
./kxr-openxr                                   # WiVRn + live depth cloud
./build/bin/kxr_openxr --src both              # fused cloud, no dashboard
./build/bin/kxr_openxr --no-live --xyz PATH    # watch a published cloud
```

How to wire an app, frames, lock, and controller grips: [`docs/OPENXR.md`](docs/OPENXR.md).

Controllers: left stick moves, right stick looks. Solid 3D / Map fly turns a Kinect-only depth frame into a lit object mesh. Kinect 2 and a fused cloud stay point splats, in the panel and in the headset. Right menu opens a settings panel that stays in front of the HMD. That panel cycles Auto / Kinect / Kinect 2 / Both and can Align Kinect 2 or reset the fit. A sensor change while the cloud is live restarts the stream.

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
src/xr/            libkinect_xr — cloud in OpenXR player space
src/fg/            ROCm frame-gen library
setup_loopback.sh  one-shot v4l2loopback
```
