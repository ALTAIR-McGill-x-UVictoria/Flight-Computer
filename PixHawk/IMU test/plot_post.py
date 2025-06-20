import matplotlib.pyplot as plt

# Read every 3rd line starting from line 4 (i.e., lines 4, 7, 10, ...)
timestamps = []
values = []

with open('attitude_data.csv', 'r') as f:
    lines = f.readlines()

# Start from line 4 (index 3), then every 3rd line
for i in range(3, len(lines), 3):
    parts = lines[i].strip().split(',')
    if len(parts) >= 3:
        try:
            timestamps.append(int(parts[0]))
            values.append(float(parts[2]))
        except ValueError:
            continue

# Convert timestamps to relative seconds
if timestamps:
    t0 = timestamps[0]
    time_sec = [(t - t0) / 1000000 for t in timestamps]

    plt.figure(figsize=(10, 5))
    plt.plot(time_sec, values, label='Value')
    plt.xlabel('Time (s)')
    plt.ylabel('Value')
    plt.title('Yaw vs Time')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    # Plot vertical line at timestamp 10358272.52
    vline_time = 10358272.52  / 1_000_000
    plt.axvline(x=vline_time, color='orange', linestyle='--', label='Event')

    plt.legend()
    plt.show()
else:
    print("No valid data found.")