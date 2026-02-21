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
CSV_FILE      = "backdrive-test/backdrive_test_3.csv"
SG_WINDOW     = 3    # Savitzky-Golay window length (must be odd, >= polyorder+1)
SG_POLYORDER  = 2     # polynomial order for S-G filter
GR = 18
t_start = 2.3
t_end = 2.7
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
        names=["t", "pos", "velocity"],
        #names=["t", "pos", "cmd_pos", "u", "velocity"],
        dtype=str,
        index_col=False,   # prevent first column being used as index
    )

    # clean up quoted values
    for col in df.columns:
        df[col] = df[col].str.strip('"')

    df["t"]        = pd.to_numeric(df["t"],        errors="coerce")
    df["pos"]      = pd.to_numeric(df["pos"],      errors="coerce")
    df["velocity"] = pd.to_numeric(df["velocity"], errors="coerce")

    df = df.dropna(subset=["t"]).sort_values("t").reset_index(drop=True)

    # forward-fill so each row has the latest known pos and velocity
    df["pos"]      = df["pos"].ffill()
    df["velocity"] = df["velocity"].ffill()

    # drop rows where we still have no data for either signal
    df = df.dropna(subset=["pos", "velocity"]).reset_index(drop=True)

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
    vel_i  = np.interp(t_uniform, df["t"].values, df["velocity"].values)
    return pd.DataFrame({"t": t_uniform, "pos": pos_i, "velocity": vel_i})


def sg_derivative(y: np.ndarray, dt: float,
                  window: int, polyorder: int) -> np.ndarray:
    """
    Compute the first derivative of y with respect to time using the
    Savitzky-Golay filter (deriv=1, delta=dt).
    """
    # ensure window is odd and > polyorder
    if window % 2 == 0:
        window += 1
    window = max(window, polyorder + 2 if (polyorder + 2) % 2 != 0 else polyorder + 3)
    return savgol_filter(y, window_length=window, polyorder=polyorder,
                         deriv=1, delta=dt)

def calculate_inertia(t, pos, vel, accel):
    T_arm = 0.060*0.333 # [N*m]
    m_weight = 0.037 # [Kg]
    r_weight = 0.10396 #[m]
    T_static = 0.020

    iloc = np.argmin(np.abs(t-t_start))
    angle_offset = pos[iloc]

    accel_output = accel*2*np.pi/GR # [rad/s^2]
    angle = ((pos-angle_offset)*2*np.pi)/GR #[rad]


    #F_N = np.clip(m_weight*(9.81-(np.cos(angle)*accel_output*r_weight)), 0, np.inf) # effective gravity needs to subtract out current accel
    F_N = m_weight*9.81

    T = (T_arm + F_N*r_weight)*np.cos(angle)

    i_eq = np.argmin(np.abs(t - 2.5) )
    b = (T[i_eq] - T_static)/(vel[i_eq])
    #b = 0
    b2 = (T[i_eq] - T_static)/(vel[i_eq]/GR)
    print(b2)
    
    T-= b*vel + T_static

    return T/accel_output, T


def main():
    print(f"Loading {CSV_FILE} …")
    raw = load_and_merge(CSV_FILE)
    print(f"  {len(raw):,} rows after merge & forward-fill")

    # ── resample to uniform grid ──────────────────────────────────────────────
    fs = 1.0 / np.median(np.diff(raw["t"].values))
    print(f"  Median sample rate ≈ {fs:.1f} Hz  →  resampling to 500 Hz")
    data = resample_uniform(raw, fs=500.0)
    data = data[(data["t"] > t_start) & (data["t"] < t_end)]
    dt   = data["t"].iloc[1] - data["t"].iloc[0]

    # ── smooth velocity with S-G, then differentiate for acceleration ─────────
    vel_smooth = savgol_filter(data["velocity"].values,
                               window_length=SG_WINDOW,
                               polyorder=SG_POLYORDER)

    accel = sg_derivative(data["velocity"].values, dt, window=SG_WINDOW, polyorder=SG_POLYORDER)
    #accel = savgol_filter(data["pos"].values, window_length=SG_WINDOW, polyorder=SG_POLYORDER, deriv=2)

    # also smooth position and derive velocity independently as a sanity check
    pos_smooth   = savgol_filter(data["pos"].values,
                                 window_length=SG_WINDOW,
                                 polyorder=SG_POLYORDER)
    vel_from_pos = sg_derivative(pos_smooth, dt,
                                 window=SG_WINDOW, polyorder=SG_POLYORDER)

    t = data["t"].values

    inertia, torque = calculate_inertia(t, pos_smooth, vel_smooth, accel)
    iloc_1 = np.argmin(np.abs(2.44-t))
    iloc_2 = np.argmin(np.abs(2.475-t))
    #print(inertia[iloc]*10**7)
    print(inertia[iloc_1:iloc_2]*10**7)



    # ── plot ──────────────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(16, 12))
    fig.suptitle("Motor Motion Analysis", fontsize=14, fontweight="bold")
    gs  = gridspec.GridSpec(5, 1, hspace=0.55)

    # — position ——————————————————————————————————————————————————————————————
    ax1 = fig.add_subplot(gs[0])
    ax1.plot(t, data["pos"].values, color="lightsteelblue", lw=0.8,
             label="raw position", alpha=0.7)
    ax1.plot(t, pos_smooth,         color="royalblue",      lw=1.5,
             label="S-G smoothed")
    ax1.set_ylabel("Position (rev)")
    ax1.set_title("Position")
    ax1.legend(fontsize=8, loc="upper right")
    ax1.grid(True, alpha=0.3)

    # — velocity ——————————————————————————————————————————————————————————————
    ax2 = fig.add_subplot(gs[1], sharex=ax1)
    ax2.plot(t, data["velocity"].values, color="lightcoral",   lw=0.8,
             label="raw velocity (logged)", alpha=0.7)
    ax2.plot(t, vel_smooth,               color="crimson",      lw=1.5,
             label="S-G smoothed velocity")
    ax2.plot(t, vel_from_pos,             color="darkorange",   lw=1.0,
             ls="--", label="velocity derived from pos (S-G)")
    ax2.set_ylabel("Velocity (rev/s)")
    ax2.set_title("Velocity")
    ax2.legend(fontsize=8, loc="upper right")
    ax2.grid(True, alpha=0.3)

    # — acceleration ——————————————————————————————————————————————————————————
    ax3 = fig.add_subplot(gs[2], sharex=ax1)
    ax3.plot(t, accel, color="seagreen", lw=1.5,
             label="acceleration (d/dt smoothed vel, S-G)")
    ax3.set_ylabel("Acceleration (rev/s²)")
    #ax3.set_xlabel("Time (s)")
    ax3.set_title("Acceleration")
    ax3.legend(fontsize=8, loc="upper right")
    ax3.grid(True, alpha=0.3)
    ax3.axhline(0, color="black", lw=0.5, ls="--")
 
    ax4 = fig.add_subplot(gs[3], sharex=ax1)
    ax4.plot(t, torque)
    ax4.set_title("Torque")
    ax4.set_ylabel("Torque (N*m)")
    ax4.grid(True, alpha=0.3)

    ax5 = fig.add_subplot(gs[4], sharex=ax1)
    ax5.plot(t, inertia*10**7)
    ax5.set_xlabel("Time (s)")
    ax5.set_title("Inertia")
    ax5.set_ylabel("Inertia (kg*cm^2)")
    ax5.grid(True, alpha=0.3)

    #ax5.set_xlim([2.4, 2.6])
    ax5.set_ylim([-500, 8000])

    plt.savefig("motion_plot.png", dpi=150, bbox_inches="tight")
    print("Saved  motion_plot.png")
    plt.show()


if __name__ == "__main__":
    main()