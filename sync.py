#!/usr/bin/env python3
"""
sync.py - build time-synced CSVs from an rs_capture_fast.py run folder.

All streams share ONE hardware clock (ms). Color+depth are paired per frame index;
IMU (accel/gyro, different rates) is aligned by interpolating onto the target
timestamp. This script writes:

  synced_frames.csv - one row per color/depth frame, with IMU interpolated to that
                      frame's timestamp + the color/depth file paths. This is the
                      "one row per frame, everything aligned" table.
  imu_merged.csv    - single 6-axis IMU stream (accel interpolated onto the gyro
                      timestamps), full IMU rate. This is the form most VIO/SLAM want.

Usage:
  python sync.py <run_dir> [--warmup N]     # N = frames to drop at start (default 8)
"""
import argparse, os, csv
import numpy as np


def load(path):
    a = np.genfromtxt(path, delimiter=",", names=True)
    return np.atleast_1d(a)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir")
    ap.add_argument("--warmup", type=int, default=8,
                    help="drop this many frames at the start (startup/auto-exposure settle)")
    args = ap.parse_args()
    D = args.run_dir

    frames = load(os.path.join(D, "frames_index.csv"))
    accel  = load(os.path.join(D, "imu_accel.csv"))
    gyro   = load(os.path.join(D, "imu_gyro.csv"))

    at = accel["timestamp_ms"]; gt = gyro["timestamp_ms"]

    # ---------- per-frame synced table ----------
    fidx = frames["frame_index"].astype(int)
    ft   = frames["color_timestamp_ms"]          # frame time = color stamp (depth is paired)
    if args.warmup:
        fidx, ft = fidx[args.warmup:], ft[args.warmup:]

    ax = np.interp(ft, at, accel["ax_m_s2"]); ay = np.interp(ft, at, accel["ay_m_s2"]); az = np.interp(ft, at, accel["az_m_s2"])
    gx = np.interp(ft, gt, gyro["gx_rad_s"]);  gy = np.interp(ft, gt, gyro["gy_rad_s"]);  gz = np.interp(ft, gt, gyro["gz_rad_s"])

    out = os.path.join(D, "synced_frames.csv")
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame_index", "timestamp_ms", "color_file", "depth_file",
                    "accel_x_m_s2", "accel_y_m_s2", "accel_z_m_s2",
                    "gyro_x_rad_s", "gyro_y_rad_s", "gyro_z_rad_s"])
        for i in range(len(fidx)):
            n = int(fidx[i])
            w.writerow([n, f"{ft[i]:.4f}",
                        f"color_frames/{n:06d}.png", f"depth_npy/depth_{n:05d}.npy",
                        f"{ax[i]:.6f}", f"{ay[i]:.6f}", f"{az[i]:.6f}",
                        f"{gx[i]:.6f}", f"{gy[i]:.6f}", f"{gz[i]:.6f}"])
    print(f"wrote {out}  ({len(fidx)} frames, dropped {args.warmup} warmup)")

    # ---------- merged 6-axis IMU at gyro rate ----------
    axg = np.interp(gt, at, accel["ax_m_s2"]); ayg = np.interp(gt, at, accel["ay_m_s2"]); azg = np.interp(gt, at, accel["az_m_s2"])
    out2 = os.path.join(D, "imu_merged.csv")
    with open(out2, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp_ms", "accel_x_m_s2", "accel_y_m_s2", "accel_z_m_s2",
                    "gyro_x_rad_s", "gyro_y_rad_s", "gyro_z_rad_s"])
        gxa, gya, gza = gyro["gx_rad_s"], gyro["gy_rad_s"], gyro["gz_rad_s"]
        for i in range(len(gt)):
            w.writerow([f"{gt[i]:.4f}", f"{axg[i]:.6f}", f"{ayg[i]:.6f}", f"{azg[i]:.6f}",
                        f"{gxa[i]:.6f}", f"{gya[i]:.6f}", f"{gza[i]:.6f}"])
    print(f"wrote {out2}  ({len(gt)} samples @ gyro rate)")


if __name__ == "__main__":
    main()
