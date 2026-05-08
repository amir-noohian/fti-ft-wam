#!/usr/bin/env python3

import os
import sys
import pandas as pd
import matplotlib.pyplot as plt


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_file = os.path.join(script_dir, "..", "data", "ft_libbarrett_log.csv")

    filename = sys.argv[1] if len(sys.argv) > 1 else default_file

    print("Reading file:", filename)

    # ✅ Read with header
    data = pd.read_csv(filename)

    print(data.head())
    print("Shape:", data.shape)

    required = ["time", "Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
    for col in required:
        if col not in data.columns:
            print(f"Missing column: {col}")
            return

    t = data["time"]
    Fx = data["Fx"]
    Fy = data["Fy"]
    Fz = data["Fz"]
    Tx = data["Tx"]
    Ty = data["Ty"]
    Tz = data["Tz"]

    # -------- Force plot --------
    plt.figure()
    plt.plot(t, Fx, label="Fx")
    plt.plot(t, Fy, label="Fy")
    plt.plot(t, Fz, label="Fz")
    plt.xlabel("Time [s]")
    plt.ylabel("Force [N]")
    plt.title("Force Data")
    plt.legend()
    plt.grid(True)

    # -------- Torque plot --------
    plt.figure()
    plt.plot(t, Tx, label="Tx")
    plt.plot(t, Ty, label="Ty")
    plt.plot(t, Tz, label="Tz")
    plt.xlabel("Time [s]")
    plt.ylabel("Torque [Nm]")
    plt.title("Torque Data")
    plt.legend()
    plt.grid(True)

    plt.show()


if __name__ == "__main__":
    main()