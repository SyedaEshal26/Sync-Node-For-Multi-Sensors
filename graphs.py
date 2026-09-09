#!/usr/bin/env python3

import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Configure matplotlib for better-looking plots
plt.style.use('seaborn-v0_8-darkgrid')
plt.rcParams['figure.figsize'] = (12, 6)
plt.rcParams['font.size'] = 10
plt.rcParams['axes.labelsize'] = 12
plt.rcParams['axes.titlesize'] = 14

DATA_DIR = "/home/scl-ncai/ros2_ws/src/Sync-Node-For-Multi-Sensors/sync_data"
CSV_FILE = os.path.join(DATA_DIR, "sync_statistics_v2.csv")
PLOT_DIR = os.path.join(DATA_DIR, "graphs")

os.makedirs(PLOT_DIR, exist_ok=True)

# Read CSV
df = pd.read_csv(CSV_FILE)

if len(df) == 0:
    print("CSV is empty.")
    exit(1)

print(f"Loaded {len(df)} fusion records")

# ============================================================
# 1. Fusion Latency Over Time (as in image)
# ============================================================

plt.figure(figsize=(12, 6))

# Calculate statistics
latency_min = df['fusion_latency_ms'].min()
latency_max = df['fusion_latency_ms'].max()
latency_std = df['fusion_latency_ms'].std()
latency_mean = df['fusion_latency_ms'].mean()
latency_median = df['fusion_latency_ms'].median()

# Plot with scatter for individual points and line for trend
plt.plot(df['fusion_id'].to_numpy(), 
         df['fusion_latency_ms'].to_numpy(), 
         'b-', alpha=0.6, linewidth=1, label='Fusion Latency')

# Add a moving average for trend (window of 20)
window = min(20, len(df))
moving_avg = df['fusion_latency_ms'].rolling(window=window, center=True).mean()
plt.plot(df['fusion_id'].to_numpy(), 
         moving_avg.to_numpy(), 
         'r-', linewidth=2, label=f'Moving Avg (window={window})')

# Add horizontal lines for mean and median
plt.axhline(y=latency_mean, color='g', linestyle='--', linewidth=2, 
            label=f'Mean: {latency_mean:.2f} ms')
plt.axhline(y=latency_median, color='orange', linestyle='--', linewidth=2, 
            label=f'Median: {latency_median:.2f} ms')

# Add legend with statistics
stats_text = f'Min: {latency_min:.3f} ms\nMax: {latency_max:.3f} ms\nStd: {latency_std:.3f} ms'
plt.text(0.02, 0.98, stats_text, transform=plt.gca().transAxes, 
         verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

plt.xlabel('Total Received')
plt.ylabel('Fused Latency (ms)')
plt.title('Fusion Latency Over Time')
plt.legend(loc='upper right')
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig(os.path.join(PLOT_DIR, "01_fusion_latency.png"), dpi=200)
plt.close()

# ============================================================
# 2. Sensor Availability (% of triggers with data)
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
        availability[col] = (df[col].astype(bool).mean() * 100.0)

if availability:
    plt.figure(figsize=(12, 6))
    
    labels = ['Camera', 'LiDAR', 'GPS', 'IMU', 'Depth', 'Odom']
    values = [availability.get('camera_found', 0), 
              availability.get('lidar_found', 0),
              availability.get('gps_found', 0),
              availability.get('imu_found', 0),
              availability.get('depth_found', 0),
              availability.get('odom_found', 0)]
    
    bars = plt.bar(labels, values, color=['blue', 'green', 'red', 'purple', 'orange', 'brown'])
    
    # Add value labels on bars
    for bar in bars:
        height = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2., height + 1,
                 f'{height:.1f}%', ha='center', va='bottom')
    
    plt.xlabel('Sensor')
    plt.ylabel('Available (%)')
    plt.title('Sensor Availability (% of triggers with data)')
    plt.ylim(0, 105)
    plt.grid(axis='y', alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(PLOT_DIR, "02_sensor_availability.png"), dpi=200)
    plt.close()

# ============================================================
# 3. GPS Interpolation Status
# ============================================================

if "gps_reused" in df.columns:
    plt.figure(figsize=(10, 6))
    
    gps_matched = df['gps_found'].astype(bool).sum() if 'gps_found' in df.columns else len(df)
    gps_reused_count = df['gps_reused'].astype(bool).sum()
    
    direct_gps = gps_matched - gps_reused_count
    interpolated = gps_reused_count
    
    if gps_matched > 0:
        direct_pct = 100.0 * direct_gps / gps_matched
        interp_pct = 100.0 * interpolated / gps_matched
        
        # Create donut chart
        fig, ax = plt.subplots(figsize=(10, 8))
        colors = ['#2E86C1', '#F39C12']
        labels = [f'Direct GPS\n{direct_pct:.1f}%', 
                  f'Interpolated\n{interp_pct:.1f}%']
        sizes = [direct_pct, interp_pct]
        
        wedges, texts, autotexts = ax.pie(sizes, labels=labels, colors=colors,
                                          autopct='%1.1f%%', startangle=90,
                                          textprops={'fontsize': 12})
        
        # Draw circle for donut
        centre_circle = plt.Circle((0, 0), 0.70, fc='white')
        fig.gca().add_artist(centre_circle)
        
        ax.set_title(f'GPS Interpolation Status\n({gps_matched}/{len(df)} fusions with GPS)', 
                     fontsize=14)
        plt.tight_layout()
        plt.savefig(os.path.join(PLOT_DIR, "03_gps_interpolation.png"), dpi=200)
        plt.close()

# ============================================================
# 4. Data Loss Rate by Sensor
# ============================================================

loss_columns = [
    "camera_found",
    "lidar_found",
    "gps_found",
    "imu_found",
    "depth_found",
    "odom_found"
]

loss_rates = {}

for col in loss_columns:
    if col in df.columns:
        loss_rates[col] = (1 - df[col].astype(bool).mean()) * 100.0

if loss_rates:
    plt.figure(figsize=(12, 6))
    
    labels = ['Camera', 'LiDAR', 'GPS', 'IMU', 'Depth', 'Odom']
    values = [loss_rates.get('camera_found', 0),
              loss_rates.get('lidar_found', 0),
              loss_rates.get('gps_found', 0),
              loss_rates.get('imu_found', 0),
              loss_rates.get('depth_found', 0),
              loss_rates.get('odom_found', 0)]
    
    bars = plt.bar(labels, values, color='red')
    
    # Add value labels
    for bar in bars:
        height = bar.get_height()
        plt.text(bar.get_x() + bar.get_width()/2., height + 1,
                 f'{height:.1f}%', ha='center', va='bottom')
    
    plt.xlabel('Sensor')
    plt.ylabel('Loss Rate (%)')
    plt.title('Data Loss Rate by Sensor')
    plt.ylim(0, 105)
    plt.grid(axis='y', alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(PLOT_DIR, "04_data_loss_rate.png"), dpi=200)
    plt.close()

# ============================================================
# 5. Sensor Rate (When data is found) - Multi-threshold plot
# ============================================================

# Create a plot showing different thresholds for sensor availability
thresholds = [5, 20, 30, 40, 50, 60, 70, 80, 90, 100]
sensors = ['Camera', 'LiDAR', 'GPS', 'IMU', 'Depth', 'Odom']
sensor_found_cols = ['camera_found', 'lidar_found', 'gps_found', 'imu_found', 'depth_found', 'odom_found']

plt.figure(figsize=(14, 8))

# Calculate availability for each sensor
avail_values = []
for col in sensor_found_cols:
    if col in df.columns:
        avail_values.append(df[col].astype(bool).mean() * 100)
    else:
        avail_values.append(0)

# Create bar chart with threshold lines
bars = plt.bar(sensors, avail_values, color=['#3498DB', '#2ECC71', '#E74C3C', '#9B59B6', '#F39C12', '#1ABC9C'])

# Add value labels
for bar in bars:
    height = bar.get_height()
    plt.text(bar.get_x() + bar.get_width()/2., height + 1,
             f'{height:.1f}%', ha='center', va='bottom')

# Add threshold lines with different colors
threshold_colors = ['#FF6B6B', '#FF9F43', '#FECA57', '#48DBFB', '#0ABDE3', '#10AC84', '#EE5A24', '#5F27CD', '#341F97', '#000000']
for i, threshold in enumerate(thresholds):
    plt.axhline(y=threshold, color=threshold_colors[i % len(threshold_colors)], 
                linestyle='--', alpha=0.5, linewidth=0.8)
    if i % 2 == 0:  # Show every other threshold label
        plt.text(len(sensors) - 0.5, threshold + 1, f'{threshold}%', 
                 fontsize=8, color=threshold_colors[i % len(threshold_colors)])

plt.xlabel('Sensor')
plt.ylabel('Availability (%)')
plt.title('Sensor Availability with Threshold Indicators')
plt.ylim(0, 105)
plt.grid(axis='y', alpha=0.3)
plt.legend(['Threshold Lines'], loc='upper right')
plt.tight_layout()
plt.savefig(os.path.join(PLOT_DIR, "05_sensor_threshold_analysis.png"), dpi=200)
plt.close()

# ============================================================
# 6. Synchronization Summary Dashboard
# ============================================================

fig, axes = plt.subplots(2, 2, figsize=(14, 10))

# Summary statistics
total_triggers = len(df)
avg_latency = df['fusion_latency_ms'].mean()
min_latency = df['fusion_latency_ms'].min()
max_latency = df['fusion_latency_ms'].max()

# Plot 1: Fusion Latency Distribution
axes[0, 0].hist(df['fusion_latency_ms'], bins=20, color='skyblue', edgecolor='black', alpha=0.7)
axes[0, 0].axvline(x=avg_latency, color='red', linestyle='--', linewidth=2, label=f'Mean: {avg_latency:.3f} ms')
axes[0, 0].set_xlabel('Latency (ms)')
axes[0, 0].set_ylabel('Frequency')
axes[0, 0].set_title('Fusion Latency Distribution')
axes[0, 0].legend()
axes[0, 0].grid(True, alpha=0.3)

# Plot 2: Sensor Availability Bar Chart (like in image)
sensors_avail = ['Camera', 'LiDAR', 'GPS', 'IMU', 'Depth', 'Odom']
avail_values = []
for col in sensor_found_cols:
    if col in df.columns:
        avail_values.append(df[col].astype(bool).mean() * 100)
    else:
        avail_values.append(0)

bars = axes[0, 1].bar(sensors_avail, avail_values, color='steelblue')
for bar in bars:
    height = bar.get_height()
    axes[0, 1].text(bar.get_x() + bar.get_width()/2., height + 1,
                    f'{height:.1f}%', ha='center', va='bottom', fontsize=9)
axes[0, 1].set_ylabel('Availability (%)')
axes[0, 1].set_title('Sensor Availability')
axes[0, 1].set_ylim(0, 105)
axes[0, 1].grid(axis='y', alpha=0.3)

# Plot 3: Cumulative Latency Over Time
cumulative_avg = df['fusion_latency_ms'].expanding().mean()
axes[1, 0].plot(df['fusion_id'], cumulative_avg, 'b-', linewidth=2)
axes[1, 0].axhline(y=avg_latency, color='r', linestyle='--', label=f'Overall Avg: {avg_latency:.3f} ms')
axes[1, 0].set_xlabel('Fusion ID')
axes[1, 0].set_ylabel('Cumulative Average Latency (ms)')
axes[1, 0].set_title('Cumulative Latency Over Time')
axes[1, 0].legend()
axes[1, 0].grid(True, alpha=0.3)

# Plot 4: Sensor Data Loss Rates (like in image)
loss_values = [100 - val for val in avail_values]
bars = axes[1, 1].bar(sensors_avail, loss_values, color=['#FF6B6B', '#FF9F43', '#FECA57', '#48DBFB', '#0ABDE3', '#10AC84'])
for bar in bars:
    height = bar.get_height()
    axes[1, 1].text(bar.get_x() + bar.get_width()/2., height + 1,
                    f'{height:.1f}%', ha='center', va='bottom', fontsize=9)
axes[1, 1].set_ylabel('Loss Rate (%)')
axes[1, 1].set_title('Data Loss Rate by Sensor')
axes[1, 1].set_ylim(0, 105)
axes[1, 1].grid(axis='y', alpha=0.3)

# Add summary statistics as figure title
fig.suptitle(f'Synchronization Summary\nTotal Triggers: {total_triggers} | Avg Latency: {avg_latency:.3f} ms | Min: {min_latency:.3f} ms | Max: {max_latency:.3f} ms', 
             fontsize=12, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(PLOT_DIR, "06_sync_summary_dashboard.png"), dpi=200)
plt.close()

# ============================================================
# Print Summary
# ============================================================

print()
print("==============================================")
print("SYNC GRAPH GENERATION COMPLETE")
print("==============================================")
print()
print("CSV:", CSV_FILE)
print("Graphs:", PLOT_DIR)
print()
print(f"Total Triggers: {len(df)}")
print()

if 'fusion_latency_ms' in df.columns:
    print(f"Fusion Latency Statistics:")
    print(f"  Mean:  {df['fusion_latency_ms'].mean():.3f} ms")
    print(f"  Min:   {df['fusion_latency_ms'].min():.3f} ms")
    print(f"  Max:   {df['fusion_latency_ms'].max():.3f} ms")
    print(f"  Std:   {df['fusion_latency_ms'].std():.3f} ms")
    print(f"  Median:{df['fusion_latency_ms'].median():.3f} ms")
    print()

print("Sensor Availability:")
for sensor, col in zip(['Camera', 'LiDAR', 'GPS', 'IMU', 'Depth', 'Odom'], 
                       ['camera_found', 'lidar_found', 'gps_found', 'imu_found', 'depth_found', 'odom_found']):
    if col in df.columns:
        avail = df[col].astype(bool).mean() * 100
        print(f"  {sensor:8s}: {avail:.1f}%")
print()

if "gps_reused" in df.columns:
    gps_matched = df['gps_found'].astype(bool).sum() if 'gps_found' in df.columns else len(df)
    gps_reused_count = df['gps_reused'].astype(bool).sum()
    if gps_matched > 0:
        direct_gps = gps_matched - gps_reused_count
        print(f"GPS Status:")
        print(f"  Direct GPS:   {direct_gps:3d} ({100.0 * direct_gps / gps_matched:.1f}%)")
        print(f"  Interpolated: {gps_reused_count:3d} ({100.0 * gps_reused_count / gps_matched:.1f}%)")
        print()

print("Generated graphs:")
for i in range(1, 7):
    print(f"  {i:02d}_*.png")
print()
