# IRS_Capture

High-throughput recorder for the **Intel RealSense D435i** that captures color, depth,
and IMU **at the correct frame rate with zero dropped frames** — plus tooling to
time-synchronize everything.

## Why this exists

The RealSense SDK's built-in recorder (`pipeline.enable_record_to_file` → `.db3`)
serializes every raw frame through a single-threaded SQLite writer that tops out around
**~120 MB/s** on typical hardware. Full 1080p30 color alone needs ~187 MB/s, so the
built-in recorder **drops ~50% of color/depth frames at 1080p30** and starves the IMU.
`rs-convert` only extracts what the `.db3` already saved, so the frames are gone.

`rs_capture_fast.py` still uses the SDK (`pyrealsense2`) to grab frames, but replaces the
built-in file recorder with a **parallel encoder sink**: a lightweight callback copies each
frame and hands it to a pool of worker threads that encode color (lossless PNG) and save
depth (`.npy`) concurrently. Result on a D435i over USB3:

| Stream | Rate | Dropped frames |
|---|---|---|
| Color 1920×1080 | 30 fps | **0** (hardware frame counter contiguous) |
| Depth 1280×720 | 30 fps | **0** |
| Accel / Gyro | 200 Hz | **0** |

## Install

```bash
pip install -r requirements.txt   # pyrealsense2, opencv-python, numpy
```

> On some environments the RealSense SDK / `pyrealsense2` version must match your firmware.
> The two IR streams and IMU work on the D435i (firmware 5.x).

## Quick start

```bash
python rs_capture_fast.py                 # record until Ctrl-C / ESC (preview window)
python rs_capture_fast.py --secs 30       # record a fixed 30 seconds
python rs_capture_fast.py --no-preview    # headless (no GUI window)
```

Each run creates a fresh folder: `D435I/<date>/<time>/`.

### Options

| Flag | Default | Description |
|---|---|---|
| `--secs N` | `0` | Auto-stop after N seconds (`0` = until ESC / window close) |
| `--no-preview` | off | Headless — no preview window (auto-falls back if OpenCV has no GUI) |
| `--ir {both,left,right,none}` | `none` | Record infrared imager(s). See [Infrared](#infrared-streams) |
| `--powerline {50,60,auto,off}` | `50` | RGB anti-flicker for mains lighting (50 = India/EU, 60 = US/Japan) |
| `--color-exposure N` | auto | Fix RGB exposure (µs); disables color auto-exposure (stops brightness pulsing) |
| `--color-gain N` | auto | Fix RGB gain (use with `--color-exposure`) |
| `--color-format {jpeg,png,png0,raw}` | `jpeg` | Color encoding. `jpeg` q100 = fast + visually lossless (no drops); `png`/`png0` = lossless but slow; `raw` = `.npy` lossless huge |
| `--color-quality N` | `100` | JPEG quality when `--color-format jpeg` |
| `--max-minutes N` | `0` | Safety cap (`0` = no limit) |
| `--camera-name NAME` | `D435I` | Top-level output folder name |
| `--base-dir PATH` | `.` | Parent dir; output goes to `<base-dir>/<camera-name>/<date>/<time>/` |
| `--out DIR` | — | Write **directly** into this exact folder (skips the date/time nesting) |

Examples:
```bash
python rs_capture_fast.py --base-dir /data/recordings        # /data/recordings/D435I/<date>/<time>/
python rs_capture_fast.py --out /data/run_042                # writes straight into /data/run_042/
```

## Output

Per run, into `D435I/<date>/<time>/`:

| File | Contents |
|---|---|
| `color_frames/000123.jpg` | Color 1920×1080 — the color data (`.jpg` default; `.png`/`.npy` with `--color-format`) |
| `depth_npy/depth_00123.npy` | Raw `uint16` depth, 1280×720 (millimetres = value × `depth_scale_m`) |
| `color.mp4` / `depth_video.mp4` | Lossy previews only (not the data) |
| `imu_accel.csv` | `timestamp_ms, ax_m_s2, ay_m_s2, az_m_s2` |
| `imu_gyro.csv` | `timestamp_ms, gx_rad_s, gy_rad_s, gz_rad_s` |
| `frames_index.csv` | Per-frame: timestamps, hardware frame counter, actual exposure, gain, laser power |
| `ir_left/`, `ir_right/` | Infrared imagers, lossless 8-bit PNG (only if `--ir`) |
| `intrinsics.json` | Color + depth intrinsics, depth→color **and camera↔IMU** extrinsics, depth scale, device serial/firmware |
| `camera_settings.json` | Snapshot of every sensor option at record start |
| `depth_scale.txt` | Metres per depth unit |
| `capture_report.json` | Per-stream true rate (from timestamps), drop stats, startup latency |

## Synchronization

All streams share one hardware clock (milliseconds). Color and depth are paired by frame
index (same frameset); IMU is higher-rate and asynchronous, so align it by interpolating on
timestamp. `sync.py` does this:

```bash
python sync.py D435I/2026-07-05/19-53-30 [--warmup 8]
```

Writes into the run folder:
- `synced_frames.csv` — one row per frame: `frame_index, timestamp_ms, color_file,
  depth_file, accel_xyz, gyro_xyz` (IMU interpolated to each frame's timestamp).
- `imu_merged.csv` — single 6-axis IMU stream (accel interpolated onto gyro timestamps),
  the form most VIO/SLAM systems want.

`--warmup N` drops the first N frames (startup / auto-exposure settle; default 8).

> For real VIO/SLAM fusion, feed the estimator the **raw** full-rate IMU + the frame
> timestamps — don't downsample IMU to frame rate. The camera↔IMU extrinsics needed for
> fusion are in `intrinsics.json` (`depth_to_accel_extrinsics`, `color_to_accel_extrinsics`).

## Infrared streams

The D435i has two IR imagers (the stereo pair; imager 1 = left is the one depth is aligned
to). They're **off by default** because recording them alongside 1080p color exceeds USB3
bandwidth:

| `--ir` | Effect (1080p color + depth + IMU) |
|---|---|
| `none` (default) | full 30 fps |
| `left` / `right` | one imager, ~a few fps of USB headroom cost |
| `both` | both imagers, larger USB cost |

To keep a solid 30 fps *with* IR, drop color to 720p: `--ir both --cw 1280 --ch 720`.
(Frame-rate impact is USB-bandwidth-bound, not a pipeline limitation — the pipeline never
drops frames.)

## Platforms

Pure `pyrealsense2` + OpenCV + NumPy — no OS-specific code — so it runs on **Linux and
Windows**.

### Ubuntu 22.04 (e.g. Intel NUC)

Works well — RealSense is first-class on Linux, and a NUC's Intel USB controller is a great
host for the D435i. Setup:

```bash
# Option A: Intel apt repo (recommended - includes udev rules)
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt update
sudo apt install librealsense2-utils librealsense2-dev python3-pyrealsense2

# Option B: pip (Python 3.10 on 22.04)
pip install pyrealsense2

pip install opencv-python numpy      # (or python3-opencv via apt)
```

Linux notes:
- **udev rules**: installed by `librealsense2-utils` (or from the librealsense repo). They let
  you access the camera without `sudo`. Replug the camera after installing. If you used pip
  only, grab `99-realsense-libusb.rules` from the librealsense repo and install it.
- **Headless NUC** (no desktop): run with `--no-preview`. The script also auto-falls back to
  headless if OpenCV has no GUI backend.
- **CPU**: lossless-PNG color encoding at 1080p30 is multi-threaded. On an i5/i7 NUC it keeps
  up easily; on a low-power NUC (Celeron/N-series) watch `capture_report.json` — if
  `color queue_drops > 0`, lower resolution/fps or reduce the color worker count.
- The USB-power-management tip below is Windows-specific; on Linux the default USB autosuspend
  is generally fine for the D435i.

## Flicker / banding under artificial light

If the color video shows rolling brightness **flicker/banding**, it's a mismatch between the
camera exposure and the mains frequency of the lights (lights pulse at 50 Hz or 60 Hz). The
RGB camera's default `Power Line Frequency = Auto` often mis-detects. Force it:

```bash
python rs_capture_fast.py --powerline 50     # 50 Hz mains (India, EU) - DEFAULT
python rs_capture_fast.py --powerline 60     # 60 Hz mains (US, Japan)
```

If brightness still *pulses* frame-to-frame (auto-exposure "hunting"), lock the exposure:
```bash
python rs_capture_fast.py --powerline 50 --color-exposure 156 --color-gain 64
```
The chosen values are recorded in `camera_settings.json` for each run.

## Video timing & the `make_video.py` tool

`color.mp4` / `depth_video.mp4` are built on the **real timeline from frame timestamps**, not
at a naive fixed fps — so even if frames arrive irregularly (or a few drop), the video plays at
correct real-time speed with gaps shown as a brief hold rather than a jerky jump.

To rebuild a real-time video from an already-recorded run's frames:
```bash
python make_video.py <run_dir>            # rebuilds color_realtime.mp4 from color_frames + timestamps
python make_video.py <run_dir> --depth    # depth_realtime.mp4
```

> Frame drops: color encoding must keep up with 30 fps. **PNG at 1080p is ~239 ms/frame**, which
> 6 workers can't sustain (drops frames); **JPEG q100 is ~17 ms** with huge margin. That's why
> `jpeg` is the default. If `capture_report.json` shows `color queue_drops > 0`, your encoder is
> too slow — use `--color-format jpeg` or lower the resolution/fps.

## Notes / gotchas

- **Close the RealSense Viewer first.** Only one process can own the video sensors; if the
  Viewer is open, a capture script gets IMU but zero color/depth frames.
- **Disable USB selective suspend** (Windows Power settings / Device Manager) to avoid
  power-management hiccups on the high-rate IMU.
- **Startup latency:** there's ~2 s between `pipe.start()` and the first frame. The report's
  `capture_hz` is computed from real frame timestamps (so it reflects true fps), and startup
  latency is reported separately.
- **Depth needs a scene:** point the camera at a textured surface **0.3–6 m** away — a blank
  wall or object against the lens returns almost no depth (not a bug).
- No `raw.db3` is produced — that recorder is the frame-loss bottleneck this tool avoids.

## What's captured vs. the SDK `.db3`

This tool records a **superset of the db3's useful data** — color, depth, IMU, both
intrinsics, depth→color + camera↔IMU extrinsics, per-frame metadata, all sensor settings,
and device info — **without the frame drops**. The only db3 items intentionally skipped are
the identity IMU intrinsics and per-sample IMU metadata (both low-value on the D435i). The
D435i has **no pose stream** (that's the T265); pose must be computed via VIO from these
images + IMU.
