"""
Motion Data Plotter
-------------------
Reads the sparse CSV log (odrv0_pos / cmd_pos / u / velocity columns),
reconstructs clean time-series for position and velocity, then derives
acceleration via a Savitzky-Golay filter.

Usage:
    python plot_motion.py [path_to_csv]

If no path is given it defaults to  data.csv  in the working directory.
"""

import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy.signal import savgol_filter

# ── configuration ──────────────────────────────────────────────────────────────
CSV_FILE      = "plotting/sinusoid_test_for_report.csv"
GR = 18
t_start = 0
t_end = 3
MOTOR_REV_TO_FINGER_RAD = np.pi*2/GR
# ───────────────────────────────────────────────────────────────────────────────


def load_and_merge(path: str) -> pd.DataFrame:
    """
    The CSV has two interleaved row types:
      • position rows  – odrv0_pos is filled, velocity may be empty
      • velocity rows  – velocity is filled, odrv0_pos may be empty
    We read them all, sort by timestamp, then forward-fill each column so
    every row carries the most-recent known value for every signal.
    """
    df = pd.read_csv(
        path,
        header=0,
        #names=["t", "pos", "velocity"],
        names=["t", "pos", "cmd_pos", "u", "velocity"],
        dtype=str,
        index_col=False,   # prevent first column being used as index
    )

    # clean up quoted values
    for col in df.columns:
        df[col] = df[col].str.strip('"')

    df["t"]        = pd.to_numeric(df["t"],        errors="coerce")
    df["pos"]      = pd.to_numeric(df["pos"],      errors="coerce")
    df["cmd_pos"]      = pd.to_numeric(df["cmd_pos"],      errors="coerce")
    df["velocity"] = pd.to_numeric(df["velocity"], errors="coerce")

    df = df.dropna(subset=["t"]).sort_values("t").reset_index(drop=True)

    # forward-fill so each row has the latest known pos and velocity
    df["pos"]      = df["pos"].ffill()
    df["velocity"] = df["velocity"].ffill()
    df["cmd_pos"] = df["cmd_pos"].ffill()

    # drop rows where we still have no data for either signal
    df = df.dropna(subset=["pos", "velocity", "cmd_pos"]).reset_index(drop=True)

    # normalise time to start at 0
    df["t"] = df["t"] - df["t"].iloc[0]

    return df


def resample_uniform(df: pd.DataFrame, fs: float = 500.0) -> pd.DataFrame:
    """
    The S-G derivative requires (approximately) uniform sampling.
    Resample to a fixed grid via linear interpolation.
    """
    t_uniform = np.arange(df["t"].iloc[0], df["t"].iloc[-1], 1.0 / fs)
    pos_i  = np.interp(t_uniform, df["t"].values, df["pos"].values)
    cmd_pos_i =  np.interp(t_uniform, df["t"].values, df["cmd_pos"].values)
    vel_i  = np.interp(t_uniform, df["t"].values, df["velocity"].values)
    return pd.DataFrame({"t": t_uniform, "pos": pos_i, "velocity": vel_i, "cmd_pos": cmd_pos_i})


def main():
    print(f"Loading {CSV_FILE} …")
    raw = load_and_merge(CSV_FILE)
    print(f"  {len(raw):,} rows after merge & forward-fill")

    # ── resample to uniform grid ──────────────────────────────────────────────
    fs = 1.0 / np.median(np.diff(raw["t"].values))
    print(f"  Median sample rate ≈ {fs:.1f} Hz  →  resampling to 500 Hz")
    data = resample_uniform(raw, fs=500.0)
    print(data)
    data = data[(data["t"] > t_start) & (data["t"] < t_end)]
    dt   = data["t"].iloc[1] - data["t"].iloc[0]

    t = data["t"].values



    # ── plot ──────────────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(14, 9))
    fig.suptitle("Motor Motion Analysis", fontsize=14, fontweight="bold")
    gs  = gridspec.GridSpec(1, 1, hspace=0.45)

    # — position ——————————————————————————————————————————————————————————————
    ax1 = fig.add_subplot(gs[0])
    ax1.plot(t, data["pos"].values*MOTOR_REV_TO_FINGER_RAD, color="blue", lw=1.0,
             label="actual position", alpha=1.0)
    ax1.plot(t, data["cmd_pos"].values*MOTOR_REV_TO_FINGER_RAD, color="orange", lw=0.8,
            label="command position", alpha=0.7)
    ax1.set_ylabel("Position of arm [rad]")
    ax1.set_title("Position")
    ax1.legend(fontsize=8, loc="upper right")
    ax1.grid(True, alpha=0.3)

    plt.savefig("plotting/motion_plot.png", dpi=150, bbox_inches="tight")
    print("Saved  motion_plot.png")
    plt.show()


if __name__ == "__main__":
    main()