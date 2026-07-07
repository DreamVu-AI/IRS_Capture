#!/usr/bin/env python3
"""
Rebuild a real-time, jitter-free video from captured frames + their timestamps.

The recorder's color.mp4 is written by dumping surviving frames back-to-back at a
fixed fps, so dropped/irregular frames make it run fast and jerky. This rebuilds the
video on the actual timeline: for each output slot it holds the most recent captured
frame (nearest-previous), so gaps become a brief freeze (continuous) rather than a jump,
and the clip plays at true real-time length.

Usage:
  python make_video.py <run_dir> [--fps 30] [--out color_realtime.mp4]
  python make_video.py <run_dir> --depth        # rebuild depth_video instead
"""
import argparse, os, glob, csv
import numpy as np, cv2


def load_index(run_dir, which):
    # returns sorted [(timestamp_ms, filepath)] for frames that actually exist on disk
    rows = list(csv.DictReader(open(os.path.join(run_dir, "frames_index.csv"))))
    tcol = "color_timestamp_ms" if which == "color" else "depth_timestamp_ms"
    out = []
    for r in rows:
        idx = int(r["frame_index"])
        if which == "color":
            p = os.path.join(run_dir, "color_frames", f"{idx:06d}.png")
        else:
            p = os.path.join(run_dir, "depth_npy", f"depth_{idx:05d}.npy")
        if os.path.exists(p):
            out.append((float(r[tcol]), p))
    out.sort()
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--fps", type=float, default=30.0, help="output frame rate")
    ap.add_argument("--depth", action="store_true", help="rebuild depth video (colorized) instead of color")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    which = "depth" if args.depth else "color"

    frames = load_index(args.run_dir, which)
    if len(frames) < 2:
        print("not enough frames"); return
    ts = np.array([t for t, _ in frames])
    paths = [p for _, p in frames]

    t0, tN = ts[0], ts[-1]
    dur = (tN - t0) / 1000.0
    n_out = int(round(dur * args.fps))
    out = args.out or (f"{which}_realtime.mp4")
    out_path = os.path.join(args.run_dir, out)

    # probe frame size
    if which == "color":
        h, w = cv2.imread(paths[0]).shape[:2]
    else:
        scale = float(open(os.path.join(args.run_dir, "depth_scale.txt")).read())
        h, w = np.load(paths[0]).shape

    vw = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (w, h))
    last_i, cache_i, cache_img = -1, -1, None
    dup = 0
    for k in range(n_out):
        t = t0 + (k / args.fps) * 1000.0
        i = int(np.searchsorted(ts, t, side="right") - 1)     # most recent captured frame at time t
        if i < 0:
            i = 0
        if i == last_i:
            dup += 1
        last_i = i
        if i != cache_i:                                       # decode only when the source frame changes
            if which == "color":
                cache_img = cv2.imread(paths[i])
            else:
                d16 = np.load(paths[i])
                dm = np.clip(d16.astype(np.float32) * scale, 0.3, 6.0)
                norm = ((dm - 0.3) / (6.0 - 0.3) * 255).astype(np.uint8)
                cache_img = cv2.applyColorMap(norm, cv2.COLORMAP_JET); cache_img[d16 == 0] = 0
            cache_i = i
        vw.write(cache_img)
    vw.release()

    print(f"wrote {out_path}")
    print(f"  {len(frames)} captured frames over {dur:.2f}s -> {n_out} output frames @ {args.fps}fps "
          f"(plays {n_out/args.fps:.2f}s = real time)")
    print(f"  {dup} held-frame slots ({100*dup/n_out:.1f}%) fill the drop gaps -> continuous, correct-speed motion")


if __name__ == "__main__":
    main()
