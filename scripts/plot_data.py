#!/usr/bin/env python3

import sys
import pandas as pd
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent.parent
DATA_DIR = ROOT_DIR / "data"


def safe_frequency(dt):
    dt = np.asarray(dt)
    freq = np.full_like(dt, np.nan, dtype=float)
    valid = dt > 0.0
    freq[valid] = 1.0 / dt[valid]
    return freq


def print_stats(name, freq):
    freq = np.asarray(freq)
    freq = freq[np.isfinite(freq)]

    if len(freq) == 0:
        print(f"{name}: no valid samples")
        return

    print(f"{name}:")
    print(f"  average = {np.mean(freq):.2f} Hz")
    print(f"  std     = {np.std(freq):.2f} Hz")
    print(f"  min     = {np.min(freq):.2f} Hz")
    print(f"  max     = {np.max(freq):.2f} Hz")


def main():
    if len(sys.argv) < 2:
        print("Usage: python plot_ft9235.py filename.csv")
        print(f"Files are expected in: {DATA_DIR}")
        sys.exit(1)

    filename = DATA_DIR / sys.argv[1]

    if not filename.exists():
        print(f"File not found: {filename}")
        sys.exit(1)

    data = pd.read_csv(filename)

    required_cols = [
        "sample_index",
        "hardware_time", "dt_hw",
        "read_time", "dt_read",
        "batch_time", "dt_batch",
        "Fx", "Fy", "Fz", "Tx", "Ty", "Tz"
    ]

    missing = [c for c in required_cols if c not in data.columns]
    if missing:
        print("Missing columns in CSV:")
        for c in missing:
            print(f"  {c}")
        print("\nYour CSV header should be:")
        print(",".join(required_cols))
        sys.exit(1)

    cols = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
    units = ["N", "N", "N", "Nm", "Nm", "Nm"]

    sample_index = data["sample_index"].values
    hardware_time = data["hardware_time"].values
    read_time = data["read_time"].values
    batch_time = data["batch_time"].values

    dt_hw = data["dt_hw"].values
    dt_read = data["dt_read"].values
    dt_batch = data["dt_batch"].values

    freq_hw = safe_frequency(dt_hw)
    freq_read = safe_frequency(dt_read)
    freq_batch = safe_frequency(dt_batch)

    # ignore first row because dt_read and dt_batch are usually zero there
    print_stats("Hardware frequency from dt_hw", freq_hw[1:])
    print_stats("Software read frequency from dt_read", freq_read[1:])
    print_stats("DAQ read/batch frequency from dt_batch", freq_batch[1:])

    # -------------------------
    # Plot force/torque data
    # -------------------------
    fig1, axs = plt.subplots(6, 1, figsize=(10, 10), sharex=True)

    for i, col in enumerate(cols):
        axs[i].plot(hardware_time, data[col])
        axs[i].set_ylabel(f"{col} [{units[i]}]")
        axs[i].grid(True)

    axs[0].set_title("FT9235 Force/Torque Data")
    axs[-1].set_xlabel("Hardware time [s]")

    plt.tight_layout()

    # -------------------------
    # Plot dt values
    # -------------------------
    fig2, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    axs[0].plot(hardware_time, dt_hw)
    axs[0].set_ylabel("dt_hw [s]")
    axs[0].set_title("Hardware dt")
    axs[0].grid(True)

    axs[1].plot(hardware_time, dt_read)
    axs[1].set_ylabel("dt_read [s]")
    axs[1].set_title("Software read dt")
    axs[1].grid(True)

    axs[2].plot(hardware_time, dt_batch)
    axs[2].set_ylabel("dt_batch [s]")
    axs[2].set_xlabel("Hardware time [s]")
    axs[2].set_title("DAQ read/batch dt")
    axs[2].grid(True)

    plt.tight_layout()

    # -------------------------
    # Plot frequency values
    # -------------------------
    fig3, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    axs[0].plot(hardware_time, freq_hw)
    axs[0].set_ylabel("Freq hw [Hz]")
    axs[0].set_title("Hardware frequency from dt_hw")
    axs[0].grid(True)

    axs[1].plot(hardware_time, freq_read)
    axs[1].set_ylabel("Freq read [Hz]")
    axs[1].set_title("Software read frequency from dt_read")
    axs[1].grid(True)

    axs[2].plot(hardware_time, freq_batch)
    axs[2].set_ylabel("Freq batch [Hz]")
    axs[2].set_xlabel("Hardware time [s]")
    axs[2].set_title("DAQ read/batch frequency from dt_batch")
    axs[2].grid(True)

    plt.tight_layout()

    plt.show()


if __name__ == "__main__":
    main()