# Kinect Panel

C + GTK3 control panel for an Xbox 360 Kinect: RGB / IR webcam, IR vision grade, tilt, and a live depth point cloud in the same preview.

**v0.3.0** — C + GTK3, ROCm frame gen (no Python on the run path).

## Build

Needs `libfreenect`, `gtk+-3.0`, and a C11 compiler.

```bash
make
./kinect-panel
# or
make install   # ~/.local/bin/kinect-panel
```

## Use

- **Camera** tab: preview, RGB / IR / Cloud, tilt (always on this tab)
- **Image** tab: IR boost / bright / contrast / gamma / black / white, plus **SEE**
- **Cloud**: same preview; drag to orbit, scroll to zoom
- RGB/IR **Camera ON** publishes **Kinect Webcam** (`v4l2loopback`). Load once if missing:

```bash
./setup_loopback.sh --load
```

Do not stream RGB/IR and Cloud at the same time — they share the USB camera.

## Frame gen (ROCm)

`src/fg/` — HIP motion-compensated interpolation on the Radeon (`gfx1201`).  
**In-memory only:** two live frames + generated mids. No disk, no telemetry.

Image tab → **Frame gen (ROCm)** → Boost `0` (off) … `3` (4×). Applies to RGB, IR, and Cloud.

```bash
make
./bin/fg_probe    # {"backend":"hip", ... "collect":false}
```

## Layout

```
src/          C sources (GTK panel, v4l, motor, IR grade, cloud)
bin/          build output
setup_loopback.sh
```
