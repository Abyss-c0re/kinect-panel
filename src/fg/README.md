# fg — ROCm frame generation

Artificial FPS boost for RGB, IR, and Cloud previews.

- HIP block-match + warp on AMD GPU (`gfx1201`), CPU lerp fallback
- Holds **two** real frames in RAM/VRAM and emits mids
- **No data collection:** no files, no network, no logs of pixels
