# Kinect Panel — pipeline (SoT)

Cube is the source of truth. This file is the plate. HUD labels are annotations.
Energy must flow: the sensor is **dark** unless a **session** is live.
`--headless` is the only auto-start. GUI boots **OFF**.

**C only.** No Python. No frame files. No telemetry (`collect:false`).

## Law → this cube

| Cube law | Here |
|---|---|
| Lattice wins | `kinect_session_status_json` / `--status` / socket `STATUS` |
| IN and OUT | socket IN command → OUT one JSON line |
| Core decides I/O | `kinect_session_*` owns USB / LED / stream |
| Chain only wires | GUI and headless both call the same Core |
| Prevention first | X/Quit/SIGTERM/`--off`/`atexit` all `session_off` |
| CT 101 | not touched |

Heresy if the LED, IR projector, or gspca stream is live with **no owner process**.

## Ports

```
  world (USB 045e:02ae camera, 045e:02b0 motor)
        │
        ▼
     session Core          IN:  START rgb|ir|cloud | OFF | STATUS | QUIT | RAISE
        │                  OUT: one JSON line
        ├─ RGB/IR  gspca /dev/videoN  →  grade (IR)  →  FG  →  preview
        │                                              └→ v4l2loopback “Kinect Webcam”
        └─ Cloud   libfreenect depth  →  project xyz  →  FG  →  preview
```

One USB owner. RGB/IR and Cloud never share the camera.

## Power

| State | Stream | LED | IR projector | Motor USB |
|---|---|---|---|---|
| no process | off | off | off | released |
| GUI, Camera OFF | off | off | off | tilt only, LED forced off |
| GUI, Camera ON | live | green / blink / yellow | on if IR/Cloud | held by session |
| `--headless` | live | as mode | as mode | held by session |

`kinect_session_off()`:

1. stop bridge thread (VIDIOC_STREAMOFF, close fds)
2. stop cloud thread (`freenect_stop_depth`, LED_OFF, close)
3. `freenect_set_led(LED_OFF)`
4. close motor, release libusb

Window **X** and **Quit** destroy the process (then off).
**Hide** keeps the process — session unchanged.
`--off` against a live owner sends `OFF` on the socket.
`--off` with no owner still opens motor once, kills the LED, closes (orphan cleanup after SIGKILL).

## Headless API

Same binary. Same Core.

```bash
kinect_panel --headless --mode rgb     # or ir | cloud
kinect_panel --status                  # {"ok":true,"running":…,"mode":…,"headless":…,"collect":false}
kinect_panel --off                     # dark
kinect_panel --quit                    # owner process exits
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
IN   OFF\n | START rgb\n | START ir\n | START cloud\n | STATUS\n | QUIT\n | RAISE\n
OUT  {"ok":true,"running":false,"mode":"off","headless":false,"collect":false}
```

Second launch does not steal USB. It writes one command to the owner and exits.

## Frame path (hot)

```
real frame (gspca or freenect)
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
# running:true mode:cloud headless:true

./build/bin/kinect_panel --off
# running:false
```

If `--status` says `running:true` and `pgrep kinect_panel` is empty, that is heresy.
