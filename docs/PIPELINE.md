# Kinect Panel — pipeline (SoT)

Cube is the source of truth. This file is the plate. HUD labels are annotations.
Energy must flow: the sensor is **dark** unless a **session** is live.
`--headless` is the only auto-start. GUI boots **OFF**.

**C only**, except `src/kinect2_device.cpp` (libfreenect2 has no C API). No Python. No frame files. No telemetry (`collect:false`).

## Law → this cube

| Cube law | Here |
|---|---|
| Lattice wins | `kinect_session_status_json` / `--status` / socket `STATUS` |
| IN and OUT | socket IN command → OUT one JSON line |
| Core decides I/O | `kinect_session_*` owns USB / LED / stream |
| Chain only wires | GUI and headless both call the same Core |
| Prevention first | X/Quit/SIGTERM/`--off`/`atexit` all `session_off` |
| CT 101 | not touched |

Heresy if either LED, the IR projector, or a gspca stream is live with **no owner process**.

## Ports

```
  world (USB 045e:02ae camera, 045e:02b0 motor, 045e:02c4 Kinect 2)
        │
        ▼
     session Core          IN:  START rgb|ir|cloud [v1|v2|both|auto]
        │                       ALIGN | ALIGN RESET | OFF | STATUS | QUIT | RAISE
        │                  OUT: one JSON line
        ├─ RGB/IR  selected camera
        │            Kinect    gspca /dev/videoN
        │            Kinect 2  libfreenect2 color 1920×1080 or IR 512×424 → 640×480
        │            Both      Kinect 2 picture (cloud Both still fuses depth)
        │                  → grade (IR) → FG → preview
        │                  └→ v4l2loopback “Kinect Webcam”
        └─ Cloud   kinect_fuse
                     ├─ v1  libfreenect depth 640×480
                     └─ v2  libfreenect2 depth 512×424 (cpu pipeline unless KINECT2_PIPELINE)
                   host-clock pair → rigid v2→v1 → one xyz
                     → FG → preview
                     └→ libkinect_xr → OpenXR view-proj (player eyes)
```

One owner process. Kinect and Kinect 2 are different USB devices and may stream together. RGB/IR and a Kinect cloud still must not share the Kinect camera. `--status` does not open USB. `v1` / `v2` in the JSON mean that sensor is streaming. `skew_ms` is `-1` when the frame is not a pair.

## Power

| State | Stream | LED | IR projector | USB |
|---|---|---|---|---|
| no process | off | off | off | released |
| GUI, Camera OFF | off | off | off | Kinect tilt only, LED forced off. Kinect 2 closed. |
| GUI, Camera ON | live | Kinect green / blink / yellow when that camera or a Kinect cloud is live. Kinect 2 lamp on while its color, IR, or depth runs. | Kinect IR, Kinect depth, or Kinect 2 IR/depth. Kinect 2 color leaves the projector off. | held by session |
| `--headless` | live | as mode | as mode | held by session |

`kinect_session_off()`:

1. stop bridge thread (VIDIOC_STREAMOFF, close fds)
2. stop cloud thread: Kinect `freenect_stop_depth`, LED off, close; Kinect 2 lamp off, `stop`, `close`
3. `freenect_set_led(LED_OFF)` on the Kinect motor (orphan cleanup)
4. close motor, release libusb

Kinect 2 depth pipeline order is cpu, then OpenCL, then OpenGL. `KINECT2_PIPELINE=cl|cpu|gl|clkde` overrides. cpu is the default because ROCm OpenCL often rejects the upstream kernels. Color JPEG is TurboJPEG. The AMD libva JPEG decoder returns a flat green frame, so VAAPI is skipped unless `KINECT2_RGB=vaapi`. The rigid lives at `$KINECT_EXTRINSIC` or `$CUBE_TRACK_DIR/kinect2_extrinsic.txt`. Align needs both clouds. Reset writes identity.

Window **X** and **Quit** destroy the process (then off).
**Hide** keeps the process — session unchanged.
`--off` against a live owner sends `OFF` on the socket.
`--off` with no owner still opens motor once, kills the LED, closes (orphan cleanup after SIGKILL).

## Headless API

Same binary. Same Core.

```bash
kinect_panel --headless --mode rgb          # or ir | cloud
kinect_panel --headless --src v2 --mode rgb # Kinect 2 color
kinect_panel --headless --src v2 --mode ir  # Kinect 2 IR
kinect_panel --headless --mode v2          # Kinect 2 cloud
kinect_panel --headless --src both --mode cloud
kinect_panel --status                      # one JSON line, no USB
kinect_panel --off                         # dark
kinect_panel --quit                        # owner process exits
```

Status shape (old fields first):

```json
{"ok":true,"running":true,"mode":"cloud","headless":true,"collect":false,"src":"both","v1":true,"v2":true,"skew_ms":8,"paired":true,"pipe":"cpu"}
```

C (`src/kinect_session.h`):

```c
int kinect_session_off(void);
int kinect_session_start(kinect_mode mode);   /* RGB / IR / CLOUD */
int kinect_headless_run(kinect_mode mode);    /* no GTK, until signal or OFF */
void kinect_session_status_json(char *buf, size_t n);
int kinect_ctl_send(const char *cmd, char *reply, size_t n);
```

Socket `$XDG_RUNTIME_DIR/kinect-panel.sock` (else `/tmp`):

```
IN   OFF | START rgb|ir|cloud [v1|v2|both|auto]
     ALIGN | ALIGN RESET | STATUS | QUIT | RAISE
OUT  {"ok":true,"running":false,"mode":"off","headless":false,"collect":false,"src":"auto","v1":false,"v2":false,"skew_ms":-1,"paired":false,"pipe":""}
```

`START` with no suffix keeps the current sensor. `START rgb v2` and `START ir both` open the Kinect 2 camera. A second launch does not steal USB. It writes one command to the owner and exits. `--src` is forwarded with the mode. A cloud source alias (`v1`, `v2`, `cloud2`, `kinect2`, `both`, `sync`) still means a cloud.

## Frame path (hot)

```
real frame (gspca, Kinect 2 color/IR, or freenect depth)
    → memcpy preview          ← panel reads this. LIVE only after a live frame.
    → fg_push_rgb             ← two real frames + mids, RAM only
    → vcam write (RGB/IR)     ← O_NONBLOCK, never blocks preview
```

ROCm HIP `gfx1201` when present, else CPU lerp. Probe:

```bash
./build/bin/fg_probe    # {"ok":true,"backend":"hip",…,"collect":false}
```

## Prove (bits, not essays)

```bash
cmake --preset default && cmake --build build
./build/bin/kinect_panel --status
# no owner → running:false  (does not touch USB)

./build/bin/kinect_panel --off
# orphan LED dead

./build/bin/kinect_panel --headless --mode cloud &
./build/bin/kinect_panel --status
# running:true mode:cloud headless:true  plus src/v1/v2/skew_ms/paired/pipe

./build/bin/kinect_panel --off
# running:false
```

```bash
./build/bin/kinect_sync_proof
# {"ok":true,"fails":0,"lib":"kinect_sync"}

./build/bin/kxr_proof
# {"ok":true,"fails":0,"lib":"kinect_xr"}
```

If `--status` says `running:true` and `pgrep kinect_panel` is empty, that is heresy.
