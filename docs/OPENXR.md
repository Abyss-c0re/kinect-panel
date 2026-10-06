# Kinect XR — C library for an OpenXR app

The panel orbit camera looks at the cloud from **outside**. That is not the player.

`libkinect_xr` puts the same Kinect-frame cloud (`cube_cloud_from_depth`) into the
player’s OpenXR local space and draws it with the compositor’s per-eye pose + FOV.

## Frames

| Frame | Axes | Origin |
|---|---|---|
| Kinect `K` | +X right, +Y up, +Z into the room | depth camera (panel cloud) |
| OpenXR local `X` | +X right, +Y up, −Z forward | session LOCAL or STAGE |

Default axis map: `(x,y,z)_K → (x,y,−z)_X`. Then `P_X = R · map(P_K) + t`.

`t` (and optionally `R`) is solved so the Kinect-seen HMD blob sits on the
OpenXR **VIEW** pose — the head, not an eye.

## Build

```bash
cmake --preset default && cmake --build build
./build/bin/kxr_proof
./build/bin/kxr_openxr            # live Kinect stream, no dashboard
./build/bin/kxr_openxr --no-live --xyz state/trackers/kinect_latest.xyz
```

Desktop shortcut **Kinect OpenXR** uses the same host path as gVRMod:

1. pin `XR_RUNTIME_JSON` + `~/.config/openxr/1/active_runtime.json` to WiVRn
2. start `wivrn-server` if it is not up
3. launch `kxr_openxr --live` (owns the Kinect USB — HMD is the display)

```bash
./kxr-openxr                 # launcher (WiVRn + live stream)
./build/bin/kxr_openxr       # same, live default
./build/bin/kxr_local --live # desktop-only debug, no compositor
```

## App loop (your OpenXR code)

```c
#include <openxr/openxr.h>
#include "kinect_xr.h"

kxr_session *s = kxr_create();
kxr_live_start(s);                 /* starts depth now; no panel needed */

/* each frame, after xrLocateSpace(VIEW, LOCAL) and xrLocateViews: */
kxr_set_head(s, &kxr_pose_from_xr(&view_space_location.pose));
kxr_view v[2] = { kxr_view_from_xr(&views[0]), kxr_view_from_xr(&views[1]) };
kxr_set_views(s, v, 2);

if (!locked) kxr_lock_head(s);     /* first confident human head */
/* walk around, then: kxr_lock_sample(); kxr_lock_solve(); */

float mvp[16];
kxr_mvp(s, eye, 0.05f, 12.f, mvp); /* identity model — points already in XR */
kxr_fill_vertices(s, xyzrgb, maxn, &n);
/* bind VBO, glDrawArrays(GL_POINTS, 0, n) with mvp */
```

`kxr_pose_from_xr` / `kxr_view_from_xr` appear only if `<openxr/openxr.h>` is
included **before** `kinect_xr.h`. The library itself does not link OpenXR.

## Lock

| Call | What it solves |
|---|---|
| `kxr_rig_place(pos, yaw, pitch)` | you know where the sensor sits |
| `kxr_lock_head` | translation only — human head ↔ OpenXR VIEW |
| `kxr_lock_sample` + `kxr_lock_solve` | full rigid (Umeyama, ≥3 walked samples) |
| `kxr_lock_yaw_walk` | yaw from the first→last sample delta |

Without a lock the cloud hangs at the XR origin looking the wrong way.
`kxr_track_head` finds a standing/sitting body (torso + crown), not the
tallest lamp. After `kxr_lock_head` that head sits on the OpenXR VIEW pose
and turning the headset moves the **view**, not the orbit target.

Use `xrLocateSpace(VIEW, LOCAL)` for the head. Eye poses from `xrLocateViews`
are only for `kxr_set_views` / `kxr_mvp`.

## Live USB

One owner. The GTK panel and `--live` cannot share the camera.

- XR app default: `kxr_live_start()` — sends `OFF` to the panel socket if a
  dashboard is holding USB, then opens depth. No Camera-tab click required.
- Panel still running in cloud mode: load the published `kinect_latest.xyz`
  with `--no-live --xyz PATH`.

FACE_BLUR_LAW: depth + matrix colours only. No RGB faces.

## Keys in `kxr_openxr`

`L` lock · `S` sample · `R` solve · `Y` yaw-from-walk · `+/-` scale · `0` reset · `Q` quit

## Controllers

Grips are the gesture. Buttons still do the lock commands so you do not need the desktop window.

| Input | Action |
|---|---|
| **Both grips** | Twist the pair = rotate the cloud about the grab. Pinch / spread = zoom. Move both hands = slide the room. |
| **One grip** | Drag around your head = orbit yaw / pitch |
| Thumbstick X / Y | Yaw about head / zoom about head |
| A or X | `kxr_lock_head` |
| B | lock sample |
| Y (left) | lock solve |
| Menu (left) | yaw from walk |
| Stick click | reset scale |

A grab turns autolock off so the room does not snap back onto the HMD. Press A/X (or `L`) to pin your head again.

Quest / Touch (WiVRn), Index, Vive, and WMR bindings are suggested. Hands draw as cyan / amber dots in the cloud.

## In-headset menu (CubeUI / gVRMod launcher)

Press the **right controller menu button** (or `M` on the desktop window) to open a panel that stays **in front of the HMD**. Quest Touch has no right menu path, so that profile uses the left ☰ button. Stick clicks are not used (they conflict with WiVRn).

| Stick / click | Menu |
|---|---|
| Stick Y | move row |
| Stick X or A / trigger | cycle / run the row |
| Laser + trigger | click a row |
| Right menu (Quest: left ☰) | close |
| Left stick | move / fly |
| Right stick | look (camera) |
| Solid 3D / Map fly | lit object mesh; fly the room as a 3D map |

Rows follow the panel. Cloud tune is near / far / density / dot / color / tilt. The LOCK block is the head lock (scale / hide / autolock / lock / sample / solve / yaw / reset). Then Solid 3D, Map fly, and the SENSOR block: Sensor (Auto / Kinect / Kinect 2 / Both), Align v2, Reset align, Close.

Sensor is the same choice as the panel radios. Stick X or the laser cycles it. Changing it while the cloud is live restarts depth on a worker, so the OpenXR frame does not wait on USB. The opened cloud follows that choice: Auto keeps whatever is plugged in, Kinect or Kinect 2 stays on that camera, Both requires both. Align v2 runs the same grid fit as the panel (Kinect 2 onto Kinect, not a new solver). Reset align puts that rigid back to identity. Until Align, a fused cloud stays at identity. The cloud stays depth-coloured. RGB and IR stay on the panel.
