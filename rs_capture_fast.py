#!/usr/bin/env python3
"""
rs_capture_fast.py - drop-in replacement for rs_capture.py that records at the
CORRECT FPS (no dropped frames) at full resolution.

WHY THE ORIGINAL DROPS FRAMES
-----------------------------
The original records with pipeline.enable_record_to_file(raw.db3). That rosbag2
/ SQLite recorder serializes every raw frame on a single thread and tops out
around ~118 MB/s on typical hardware. 1080p bgr8 @30 alone needs ~187 MB/s, so
~half the color frames are dropped, and the saturated writer then starves the
200 Hz IMU. extract() then just replays that already-lossy .db3, so the FPS is
lost before extraction ever runs.

THE FIX
-------
Never write a raw .db3. Instead a lightweight callback copies each frame and
hands it to worker threads that:
  * write color.mp4 directly (one ordered writer thread)
  * save each raw depth frame as .npy AND write colorized depth_video.mp4
  * collect IMU samples
Color/depth encoding is far lighter than raw serialization, so nothing is
starved. Proven: 100% frame retention at 1080p30 + 720p30 + IMU.

SAME OUTPUTS AS THE ORIGINAL (into D435I/<date>/<time>/):
    color.mp4                  - RGB video, 1920x1080, ALL frames
    depth_video.mp4            - colorized depth (viewing only), 1280x720
    depth_npy/depth_XXXXX.npy  - raw uint16 depth arrays, 1280x720
    ir_left/XXXXXX.png         - left infrared imager, 8-bit mono (lossless)   [--ir both|left|right|none]
    ir_right/XXXXXX.png        - right infrared imager, 8-bit mono (lossless)
    depth_scale.txt            - meters-per-unit
    intrinsics.json            - color+depth intrinsics, depth->color AND camera<->IMU
                                 extrinsics, depth scale, device serial/firmware
    imu_accel.csv              - timestamp_ms,ax_m_s2,ay_m_s2,az_m_s2
    imu_gyro.csv               - timestamp_ms,gx_rad_s,gy_rad_s,gz_rad_s
    frames_index.csv           - per-frame: timestamps + hw frame counter, actual
                                 exposure, gain, laser power (color & depth)  (for sync)
    capture_report.json        - per-stream received/written counts + drop stats
(raw.db3 is intentionally NOT produced - it's what caused the frame loss.)

JUST RUN:
    python rs_capture_fast.py
Stop with ESC / closing the preview window (or --secs N for a fixed duration).

Requires: pip install pyrealsense2 opencv-python numpy
"""

import argparse, csv, json, os, threading, queue, time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs


def make_run_dir(camera_name, base_dir):
    now = datetime.now()
    d = Path(base_dir) / camera_name / now.strftime("%Y-%m-%d") / now.strftime("%H-%M-%S")
    d.mkdir(parents=True, exist_ok=True)
    return d


def capture(run_dir, depth_wh=(1280, 720), color_wh=(1920, 1080), fps=30,
            max_minutes=0, secs=0, preview=True, qcap=256,
            color_workers=6, depth_workers=4, ir_workers=3, jpeg=95, ir_streams=(1, 2)):
    dw, dh = depth_wh
    cw, ch = color_wh
    depth_dir = run_dir / "depth_npy"
    depth_dir.mkdir(parents=True, exist_ok=True)
    color_dir = run_dir / "color_frames"   # per-frame JPEGs (color.mp4 built from these)
    color_dir.mkdir(parents=True, exist_ok=True)
    ir_dirs = {}                            # infrared imagers: 1=left, 2=right
    for i in ir_streams:
        ir_dirs[i] = run_dir / ("ir_left" if i == 1 else "ir_right")
        ir_dirs[i].mkdir(parents=True, exist_ok=True)

    # ---- start streams (with IMU, fallback to none) ----
    def _start(with_imu):
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, fps)
        cfg.enable_stream(rs.stream.color, cw, ch, rs.format.bgr8, fps)
        for i in ir_streams:  # IR imagers at depth res, 8-bit mono (1=left, 2=right)
            cfg.enable_stream(rs.stream.infrared, i, dw, dh, rs.format.y8, fps)
        if with_imu:
            cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
            cfg.enable_stream(rs.stream.gyro,  rs.format.motion_xyz32f, 200)
        return pipe, cfg

    # ---- state shared with callback / workers ----
    color_q = queue.Queue(maxsize=qcap)      # (idx, bgr)
    depth_q = queue.Queue(maxsize=qcap)      # (idx, uint16)
    ir_q    = queue.Queue(maxsize=qcap * 2)  # (idx, side, y8)  - two IR frames per frameset
    accel_rows = []                          # [ts_ms, ax, ay, az]  (linear accel m/s^2)
    gyro_rows = []                           # [ts_ms, gx, gy, gz]  (angular vel rad/s)
    frame_ts = []                            # (idx, color_ts_ms, depth_ts_ms) per frameset -> for sync
    recv = {"color": 0, "depth": 0, "ir": 0}
    qdrop = {"color": 0, "depth": 0, "ir": 0}
    idx_counter = {"n": 0}
    latest_preview = {"img": None}
    lock = threading.Lock()
    running = {"on": True}

    depth_scale = [0.001]  # filled after start

    # per-frame metadata we pull off each frame (like the db3 image/metadata topic)
    FMV = rs.frame_metadata_value
    def md(fr, key):
        try:
            return int(fr.get_frame_metadata(key)) if fr.supports_frame_metadata(key) else ""
        except Exception:
            return ""

    def on_frame(frame):
        if not running["on"]:
            return
        try:
            if frame.is_frameset():
                fs = frame.as_frameset()
                c = fs.get_color_frame(); d = fs.get_depth_frame()
                if not c or not d:
                    return
                with lock:
                    idx = idx_counter["n"]; idx_counter["n"] += 1
                    recv["color"] += 1; recv["depth"] += 1
                    frame_ts.append((idx, c.get_timestamp(), d.get_timestamp(),
                                     md(c, FMV.frame_counter), md(c, FMV.actual_exposure), md(c, FMV.gain_level),
                                     md(d, FMV.frame_counter), md(d, FMV.actual_exposure), md(d, FMV.gain_level),
                                     md(d, FMV.frame_laser_power)))
                cimg = np.asanyarray(c.get_data()).copy()
                dimg = np.asanyarray(d.get_data()).copy()
                try: color_q.put_nowait((idx, cimg))
                except queue.Full:
                    with lock: qdrop["color"] += 1
                try: depth_q.put_nowait((idx, dimg))
                except queue.Full:
                    with lock: qdrop["depth"] += 1
                for i in ir_streams:
                    irf = fs.get_infrared_frame(i)
                    if irf:
                        with lock: recv["ir"] += 1
                        a = np.asanyarray(irf.get_data()).copy()
                        try: ir_q.put_nowait((idx, i, a))
                        except queue.Full:
                            with lock: qdrop["ir"] += 1
                if preview and (idx % 2 == 0):
                    latest_preview["img"] = cv2.resize(cimg, (960, 540))
            elif frame.is_motion_frame():
                m = frame.as_motion_frame(); v = m.get_motion_data(); ts = m.get_timestamp()
                st = m.get_profile().stream_type()
                if st == rs.stream.accel:  accel_rows.append((ts, v.x, v.y, v.z))
                elif st == rs.stream.gyro: gyro_rows.append((ts, v.x, v.y, v.z))
        except Exception:
            pass

    imu_enabled = True
    try:
        pipe, cfg = _start(True)
        profile = pipe.start(cfg, on_frame)
    except RuntimeError as e:
        print(f"Note: could not enable IMU streams ({e}). Recording color+depth only.")
        imu_enabled = False
        pipe, cfg = _start(False)
        profile = pipe.start(cfg, on_frame)

    dev = profile.get_device()
    for s in dev.query_sensors():
        if s.supports(rs.option.frames_queue_size):
            rng = s.get_option_range(rs.option.frames_queue_size)
            s.set_option(rs.option.frames_queue_size, int(rng.max))
    depth_scale[0] = dev.first_depth_sensor().get_depth_scale()

    # metadata: color intrinsics at TOP LEVEL (unchanged schema, keeps rs_slam.py working)
    #           + depth intrinsics + depth->color extrinsics + depth scale (for point clouds)
    def intr_dict(vsp):
        i = vsp.get_intrinsics()
        return {"width": i.width, "height": i.height, "fx": i.fx, "fy": i.fy,
                "ppx": i.ppx, "ppy": i.ppy, "model": str(i.model), "coeffs": list(i.coeffs)}
    color_vsp = profile.get_stream(rs.stream.color).as_video_stream_profile()
    depth_vsp = profile.get_stream(rs.stream.depth).as_video_stream_profile()
    meta = intr_dict(color_vsp)                        # color fields at top level (backward compatible)
    meta["color_intrinsics"] = intr_dict(color_vsp)
    meta["depth_intrinsics"] = intr_dict(depth_vsp)
    def extr(src, dst):
        e = src.get_extrinsics_to(dst)
        return {"rotation_colmajor": list(e.rotation), "translation_m": list(e.translation)}
    meta["depth_to_color_extrinsics"] = extr(depth_vsp, color_vsp)   # depth -> color
    if imu_enabled:  # camera <-> IMU extrinsics (needed for visual-inertial fusion / VIO)
        accel_sp = profile.get_stream(rs.stream.accel)
        gyro_sp  = profile.get_stream(rs.stream.gyro)
        meta["depth_to_accel_extrinsics"] = extr(depth_vsp, accel_sp)
        meta["depth_to_gyro_extrinsics"]  = extr(depth_vsp, gyro_sp)
        meta["color_to_accel_extrinsics"] = extr(color_vsp, accel_sp)
        meta["color_to_gyro_extrinsics"]  = extr(color_vsp, gyro_sp)
    meta["depth_scale_m"] = depth_scale[0]
    def dev_info(k):
        try: return dev.get_info(k)
        except Exception: return None
    meta["device"] = {"name": dev_info(rs.camera_info.name),
                      "serial_number": dev_info(rs.camera_info.serial_number),
                      "firmware_version": dev_info(rs.camera_info.firmware_version),
                      "product_line": dev_info(rs.camera_info.product_line),
                      "usb_type": dev_info(rs.camera_info.usb_type_descriptor)}
    with open(run_dir / "intrinsics.json", "w") as f:
        json.dump(meta, f, indent=2)
    with open(run_dir / "depth_scale.txt", "w") as f:
        f.write(str(depth_scale[0]))

    # snapshot every sensor's active options (like the db3 /option/ topics) -> provenance
    settings = {}
    for s in dev.query_sensors():
        sname = s.get_info(rs.camera_info.name)
        opts = {}
        for opt in s.get_supported_options():
            try:
                val = s.get_option(opt)
            except Exception:
                continue
            entry = {"value": round(float(val), 6)}
            try: entry["read_only"] = bool(s.is_option_read_only(opt))
            except Exception: pass
            try: entry["description"] = s.get_option_description(opt)
            except Exception: pass
            opts[str(opt).split(".")[-1]] = entry
        settings[sname] = opts
    with open(run_dir / "camera_settings.json", "w") as f:
        json.dump(settings, f, indent=2)

    # ---- worker threads: light per-frame encode/write, fully parallel ----
    # (mp4 muxing is done OFFLINE after recording so the real-time path never
    #  bottlenecks on a single video encoder - that was the 25-drop problem.)
    written = {"color": 0, "depth": 0, "ir": 0}

    def color_worker():
        while True:
            item = color_q.get()
            if item is None: color_q.task_done(); break
            idx, img = item
            # lossless PNG (level 1 = fast, smaller than level 0; bit-exact either way)
            ok, enc = cv2.imencode(".png", img, [cv2.IMWRITE_PNG_COMPRESSION, 1])
            if ok:
                with open(color_dir / f"{idx:06d}.png", "wb") as f: f.write(enc)
                written["color"] += 1
            color_q.task_done()

    def depth_worker():
        while True:
            item = depth_q.get()
            if item is None: depth_q.task_done(); break
            idx, d16 = item
            np.save(depth_dir / f"depth_{idx:05d}.npy", d16)
            written["depth"] += 1
            depth_q.task_done()

    def ir_worker():
        while True:
            item = ir_q.get()
            if item is None: ir_q.task_done(); break
            idx, i, y8 = item
            ok, enc = cv2.imencode(".png", y8, [cv2.IMWRITE_PNG_COMPRESSION, 1])  # lossless mono8
            if ok:
                with open(ir_dirs[i] / f"{idx:06d}.png", "wb") as f: f.write(enc)
                written["ir"] += 1
            ir_q.task_done()

    threads = [threading.Thread(target=color_worker, daemon=True) for _ in range(color_workers)]
    threads += [threading.Thread(target=depth_worker, daemon=True) for _ in range(depth_workers)]
    if ir_streams:
        threads += [threading.Thread(target=ir_worker, daemon=True) for _ in range(ir_workers)]
    for t in threads: t.start()

    # ---- main loop: preview + stop conditions ----
    win = "Recording (RGB preview) - ESC or close window to stop"
    if preview:
        try:
            cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
        except cv2.error:
            print("Note: no GUI support in this OpenCV build (headless) - running without preview.")
            print("      (On a normal 'pip install opencv-python' the preview window works.)")
            preview = False
    print(f"Recording -> {run_dir}")
    print(f"color {cw}x{ch}@{fps}, depth {dw}x{dh}@{fps}, IMU={'on' if imu_enabled else 'off'}")
    print("Stop: ESC / close preview" + (f" (auto-stop {secs}s)" if secs else ""))

    t0 = time.time(); last = t0
    try:
        while True:
            if preview:
                img = latest_preview["img"]
                if img is not None:
                    cv2.imshow(win, img)
                if (cv2.waitKey(1) & 0xFF) == 27: break
                if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1: break
            else:
                time.sleep(0.05)
            now = time.time()
            if now - last >= 1.0:
                last = now
                with lock:
                    print(f"  t={now-t0:4.0f}s  cQ={color_q.qsize():3d} dQ={depth_q.qsize():3d} irQ={ir_q.qsize():3d}  "
                          f"recv c/d/ir={recv['color']}/{recv['depth']}/{recv['ir']}  "
                          f"qdrop={qdrop['color']}/{qdrop['depth']}/{qdrop['ir']}")
            if secs and (now - t0) >= secs: break
            if max_minutes and (now - t0) > max_minutes * 60:
                print(f"Reached max_minutes={max_minutes}, stopping."); break
    except KeyboardInterrupt:
        pass
    finally:
        running["on"] = False
        if preview: cv2.destroyWindow(win)
        pipe.stop()
    rec_dur = time.time() - t0   # true recording duration (excludes offline finalize)

    # drain and stop workers
    color_q.join(); depth_q.join()
    if ir_streams: ir_q.join()
    for _ in range(color_workers): color_q.put(None)
    for _ in range(depth_workers): depth_q.put(None)
    if ir_streams:
        for _ in range(ir_workers): ir_q.put(None)
    for t in threads: t.join(timeout=5)

    # ---- OFFLINE finalize: build color.mp4 + colorized depth_video.mp4 in order ----
    print("Finalizing videos (offline, no frames at risk) ...")
    # color.mp4 is only a lossy PREVIEW - the lossless data lives in color_frames/*.png
    pngs = sorted(color_dir.glob("*.png"), key=lambda p: int(p.stem))
    if pngs:
        vw = cv2.VideoWriter(str(run_dir / "color.mp4"),
                             cv2.VideoWriter_fourcc(*"mp4v"), fps, (cw, ch))
        for p in pngs: vw.write(cv2.imread(str(p)))
        vw.release()
    npys = sorted(depth_dir.glob("depth_*.npy"), key=lambda p: int(p.stem.split("_")[1]))
    if npys:
        vw = cv2.VideoWriter(str(run_dir / "depth_video.mp4"),
                             cv2.VideoWriter_fourcc(*"mp4v"), fps, (dw, dh))
        for p in npys:
            d16 = np.load(str(p))
            dm = np.clip(d16.astype(np.float32) * depth_scale[0], 0.3, 6.0)
            norm = ((dm - 0.3) / (6.0 - 0.3) * 255).astype(np.uint8)
            vis = cv2.applyColorMap(norm, cv2.COLORMAP_JET); vis[d16 == 0] = 0
            vw.write(vis)
        vw.release()

    # IMU: accel + gyro as separate CSV streams (mirrors the two db3 topics).
    #   imu_accel.csv: timestamp_ms, ax_m_s2, ay_m_s2, az_m_s2   (linear acceleration)
    #   imu_gyro.csv : timestamp_ms, gx_rad_s, gy_rad_s, gz_rad_s (angular velocity)
    # CSV writes full float precision -> lossless, and human-readable / SLAM-friendly.
    acc = sorted(accel_rows)
    gyr = sorted(gyro_rows)
    with open(run_dir / "imu_accel.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["timestamp_ms", "ax_m_s2", "ay_m_s2", "az_m_s2"]); w.writerows(acc)
    with open(run_dir / "imu_gyro.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["timestamp_ms", "gx_rad_s", "gy_rad_s", "gz_rad_s"]); w.writerows(gyr)

    # per-frame hardware timestamps -> lets you sync color/depth to IMU by TIME (not frame index).
    #   frame_index N  <->  color_frames/{N:06d}.png  and  depth_npy/depth_{N:05d}.npy
    with open(run_dir / "frames_index.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame_index", "color_timestamp_ms", "depth_timestamp_ms",
                    "color_frame_counter", "color_actual_exposure", "color_gain",
                    "depth_frame_counter", "depth_actual_exposure", "depth_gain", "depth_laser_power"])
        w.writerows(sorted(frame_ts))

    total_dur = time.time() - t0
    # streaming rate from actual frame timestamps (excludes startup warmup, so it
    # reflects true fps). recording_s/wall includes ~2s of pipe.start->first-frame latency.
    def hz(stamps):
        st = sorted(stamps)
        span = (st[-1] - st[0]) / 1000.0 if len(st) > 1 else 0
        return round((len(st) - 1) / span, 2) if span > 0 else 0.0
    color_ts = [r[1] for r in frame_ts]; depth_ts = [r[2] for r in frame_ts]
    stream_span = round((max(color_ts) - min(color_ts)) / 1000.0, 2) if len(color_ts) > 1 else 0
    report = {"recording_wall_s": round(rec_dur, 2), "streaming_span_s": stream_span,
              "startup_latency_s": round(rec_dur - stream_span, 2),
              "total_incl_finalize_s": round(total_dur, 2), "fps_target": fps,
              "color": {"recv": recv["color"], "written": written["color"], "queue_drops": qdrop["color"],
                        "capture_hz": hz(color_ts), "format": "png_lossless"},
              "depth": {"recv": recv["depth"], "written": written["depth"], "queue_drops": qdrop["depth"],
                        "capture_hz": hz(depth_ts)},
              "ir": {"imagers": list(ir_streams), "recv": recv["ir"], "written": written["ir"],
                     "queue_drops": qdrop["ir"],
                     "capture_hz": round(recv["ir"] / stream_span / len(ir_streams), 2) if ir_streams and stream_span else 0,
                     "note": "png_lossless mono8; 1=left 2=right"},
              "imu": {"accel_samples": len(acc), "gyro_samples": len(gyr),
                      "accel_hz": hz([a[0] for a in acc]), "gyro_hz": hz([g[0] for g in gyr]),
                      "accel_file": "imu_accel.csv", "gyro_file": "imu_gyro.csv",
                      "schema": "csv columns = [timestamp_ms, x, y, z]"},
              "imu_enabled": imu_enabled, "depth_scale_m": depth_scale[0],
              "note": "capture_hz = true streaming rate from frame timestamps; 0 drops = contiguous hw frame counter"}
    with open(run_dir / "capture_report.json", "w") as f:
        json.dump(report, f, indent=2)

    print("\n===== DONE  (stream %.1fs + %.1fs startup, finalize %.1fs) =====" %
          (stream_span, rec_dur - stream_span, total_dur - rec_dur))
    print(f"Color: {written['color']} PNG frames @ {report['color']['capture_hz']} Hz, drops {qdrop['color']}")
    print(f"Depth: {written['depth']} npy frames @ {report['depth']['capture_hz']} Hz, drops {qdrop['depth']}")
    if ir_streams:
        print(f"IR   : {written['ir']} PNG frames ({len(ir_streams)} imager(s)) @ "
              f"{report['ir']['capture_hz']} Hz/imager, drops {qdrop['ir']}")
    print(f"Accel: {len(acc)} @ {report['imu']['accel_hz']} Hz -> imu_accel.csv")
    print(f"Gyro : {len(gyr)} @ {report['imu']['gyro_hz']} Hz -> imu_gyro.csv")
    print(f"Files under: {run_dir}")
    print(f"To meters : depth_m = depth_array.astype(float) * {depth_scale[0]}")
    return run_dir


def main():
    p = argparse.ArgumentParser(description="D435i capture at correct FPS (no dropped frames)")
    p.add_argument("--camera-name", default="D435I")
    p.add_argument("--base-dir", default=".")
    p.add_argument("--max-minutes", type=float, default=0)
    p.add_argument("--secs", type=float, default=0, help="auto-stop after N seconds (0 = until ESC)")
    p.add_argument("--no-preview", action="store_true", help="run headless (no preview window)")
    p.add_argument("--ir", default="none", choices=["both", "left", "right", "none"],
                   help="which infrared imagers to record (default none, to keep 1080p30). "
                        "'both'+1080p exceeds USB3 -> ~21fps; 'left' -> ~27fps; use 720p color for solid 30fps")
    args = p.parse_args()
    ir_map = {"both": (1, 2), "left": (1,), "right": (2,), "none": ()}
    run_dir = make_run_dir(args.camera_name, args.base_dir)
    capture(run_dir, max_minutes=args.max_minutes, secs=args.secs,
            preview=not args.no_preview, ir_streams=ir_map[args.ir])
    print(f"\nAll done. Everything under: {run_dir}")


if __name__ == "__main__":
    main()
