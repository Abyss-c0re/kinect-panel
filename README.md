# Kinect Panel

C + GTK3 control panel for an Xbox 360 Kinect: RGB / IR webcam, IR vision grade, tilt, and a live depth point cloud in the same preview.

**v0.2.0** — no Python on the run path.

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

## Layout

```
src/          C sources (GTK panel, v4l, motor, IR grade, cloud)
bin/          build output
setup_loopback.sh
```
