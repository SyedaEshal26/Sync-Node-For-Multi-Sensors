#!/usr/bin/env python3

import os
import pandas as pd
import matplotlib.pyplot as plt


DATA_DIR = "/home/eshal/ros2_ws/src/sync_node_pkg/sync_data"

CSV_FILE = os.path.join(
    DATA_DIR,
    "sync_statistics.csv"
)

PLOT_DIR = os.path.join(
    DATA_DIR,
    "graphs"
)

os.makedirs(PLOT_DIR, exist_ok=True)


df = pd.read_csv(CSV_FILE)

if len(df) == 0:
    print("CSV is empty.")
    exit(1)


# ============================================================
# 1. Sensor timestamp synchronization error
# ============================================================

columns = [
    "camera_delta_ms",
    "lidar_delta_ms",
    "imu_delta_ms",
    "depth_delta_ms",
    "odom_delta_ms",
    "gps_delta_ms"
]

plt.figure(figsize=(12, 6))

for col in columns:
    if col in df.columns:
        valid = df[col] >= 0

        if valid.any():
            plt.plot(
                df.loc[valid, "fusion_id"],
                df.loc[valid, col],
                label=col.replace("_delta_ms", "")
            )

plt.xlabel("Fusion ID")
plt.ylabel("Timestamp difference (ms)")
plt.title("Sensor Synchronization Error")
plt.legend()
plt.grid(True)
plt.tight_layout()

plt.savefig(
    os.path.join(
        PLOT_DIR,
        "01_sensor_sync_error.png"
    ),
    dpi=200
)

plt.close()


# ============================================================
# 2. Fusion latency
# ============================================================

plt.figure(figsize=(12, 6))

plt.plot(
    df["fusion_id"],
    df["fusion_latency_ms"]
)

plt.xlabel("Fusion ID")
plt.ylabel("Fusion latency (ms)")
plt.title("Fusion Processing Latency")
plt.grid(True)
plt.tight_layout()

plt.savefig(
    os.path.join(
        PLOT_DIR,
        "02_fusion_latency.png"
    ),
    dpi=200
)

plt.close()


# ============================================================
# 3. Average synchronization error
# ============================================================

means = {}

for col in columns:

    if col in df.columns:

        valid = df[col] >= 0

        if valid.any():
            means[col] = df.loc[
                valid,
                col
            ].mean()

if means:

    plt.figure(figsize=(10, 6))

    labels = [
        x.replace("_delta_ms", "")
        for x in means.keys()
    ]

    values = list(means.values())

    plt.bar(
        labels,
        values
    )

    plt.xlabel("Sensor")
    plt.ylabel("Mean timestamp difference (ms)")
    plt.title("Mean Sensor Synchronization Error")
    plt.grid(axis="y")
    plt.tight_layout()

    plt.savefig(
        os.path.join(
            PLOT_DIR,
            "03_mean_sync_error.png"
        ),
        dpi=200
    )

    plt.close()


# ============================================================
# 4. Fusion completeness
# ============================================================

availability_columns = [
    "camera_found",
    "lidar_found",
    "gps_found",
    "imu_found",
    "depth_found",
    "odom_found"
]

availability = {}

for col in availability_columns:

    if col in df.columns:
        availability[col] = (
            df[col].astype(bool).mean() * 100.0
        )

if availability:

    plt.figure(figsize=(10, 6))

    labels = [
        x.replace("_found", "")
        for x in availability.keys()
    ]

    values = list(availability.values())

    plt.bar(
        labels,
        values
    )

    plt.xlabel("Sensor")
    plt.ylabel("Availability (%)")
    plt.title("Sensor Availability During Fusion")
    plt.ylim(0, 100)
    plt.grid(axis="y")
    plt.tight_layout()

    plt.savefig(
        os.path.join(
            PLOT_DIR,
            "04_sensor_availability.png"
        ),
        dpi=200
    )

    plt.close()


# ============================================================
# 5. GPS interpolation
# ============================================================

if "gps_interpolated" in df.columns:

    gps_interp = (
        df["gps_interpolated"]
        .astype(bool)
        .sum()
    )

    gps_total = len(df)

    gps_direct = gps_total - gps_interp

    plt.figure(figsize=(8, 6))

    plt.bar(
        ["Direct GPS", "Interpolated GPS"],
        [gps_direct, gps_interp]
    )

    plt.ylabel("Fusion cycles")
    plt.title("GPS Direct vs Interpolated")
    plt.grid(axis="y")
    plt.tight_layout()

    plt.savefig(
        os.path.join(
            PLOT_DIR,
            "05_gps_interpolation.png"
        ),
        dpi=200
    )

    plt.close()


# ============================================================
# Summary
# ============================================================

print()
print("==============================================")
print("SYNC GRAPH GENERATION COMPLETE")
print("==============================================")
print()
print("CSV:")
print(CSV_FILE)
print()
print("Graphs:")
print(PLOT_DIR)
print()
print("Fusion cycles:", len(df))

print()

if "fusion_latency_ms" in df.columns:

    print(
        "Fusion latency mean: %.3f ms"
        % df["fusion_latency_ms"].mean()
    )

    print(
        "Fusion latency min : %.3f ms"
        % df["fusion_latency_ms"].min()
    )

    print(
        "Fusion latency max : %.3f ms"
        % df["fusion_latency_ms"].max()
    )

print()

for sensor in columns:

    if sensor in df.columns:

        valid = df[sensor] >= 0

        if valid.any():

            print(
                "%s mean: %.3f ms"
                % (
                    sensor,
                    df.loc[valid, sensor].mean()
                )
            )

print()
