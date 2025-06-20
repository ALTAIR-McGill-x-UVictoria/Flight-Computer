import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from collections import deque
import time
import threading
import queue
import csv
import re

# Configuration
PORT = 'COM6'
BAUD_RATE = 115200   # Adjust based on your serial configuration
BUFFER_SIZE = 500   # Number of data points to display
UPDATE_INTERVAL = 1  # Update plot everpriny 20ms (50 FPS) for smoother real-time display
CSV_FILENAME = 'attitude_data.csv'

# Create figure and subplots
plt.style.use('dark_background')  # Better for real-time visualization
fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 8))
fig.suptitle('Pixhawk IMU Data (Real-time)')

# Initialize data storage - using deques for efficient append/pop
time_data = deque(maxlen=BUFFER_SIZE)
# Attitude data (roll, pitch, yaw)
roll_data = deque(maxlen=BUFFER_SIZE)
pitch_data = deque(maxlen=BUFFER_SIZE)
yaw_data = deque(maxlen=BUFFER_SIZE)


# Create a queue for thread-safe data passing
data_queue = queue.Queue()
# Flag for signaling thread termination
stop_thread = False

# Initialize plot lines
# Attitude plot
att_lines = []
att_lines.append(ax1.plot([], [], label='Roll', color='r')[0])
att_lines.append(ax1.plot([], [], label='Pitch', color='g')[0])
att_lines.append(ax1.plot([], [], label='Yaw', color='b')[0])


# Configure plots
ax1.set_title('Attitude (rad)')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Angle (deg)')
ax1.legend(loc='upper left')
ax1.grid(True)

# Set fixed y-axis limits for more stable visualization
# These can be adjusted based on typical value ranges
ax1.set_ylim(-180, 180)  # Attitude in radians


# Adjust layout
plt.tight_layout()
plt.subplots_adjust(top=0.9)

# Open CSV file for writing attitude data
csv_file = open(CSV_FILENAME, 'w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['timestamp', 'roll', 'pitch', 'yaw'])  # header

# Thread function to read serial data
def read_serial_data(ser):
    global stop_thread
    # Use local variables for roll, pitch, yaw, timestamp
    roll = pitch = yaw = None
    timestamp = None
    while not stop_thread:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                # print(line)
                parsed = parse_line(line)
                if parsed:
                    ts = parsed
                    csv_writer.writerow([ts])
                    csv_file.flush()
                    print(f"Received: {ts}")
                    # Put in queue for plotting (time in seconds, values)
                    # data_queue.put((timestamp / 1e4, [roll, pitch, yaw]))
                    # Reset for next set
                    roll = pitch = yaw = None
                    timestamp = None
            except Exception as e:
                print(f"Error reading serial data: {e}")
        # time.sleep(0.001)

def parse_line(line):
    timestamp = int(time.time())
    return timestamp
    exit()
    return None

try:
    # Open serial port
    ser = serial.Serial(PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {PORT} at {BAUD_RATE} baud")
    
    # Start the serial reading thread
    serial_thread = threading.Thread(target=read_serial_data, args=(ser,))
    serial_thread.daemon = True
    serial_thread.start()
    
    # Counter for reducing autoscale frequency
    autoscale_counter = 0
    
    # Function to update the plots
    def update_plot(frame):
        # Get all available data from the queue
        new_data = False
        while not data_queue.empty():
            new_data = True
            current_time, values = data_queue.get()
            time_data.append(current_time)
            roll_data.append(values[0])
            pitch_data.append(values[1])
            yaw_data.append(values[2])

        if new_data:
            time_array = list(time_data)
            att_lines[0].set_data(time_array, roll_data)
            att_lines[1].set_data(time_array, pitch_data)
            att_lines[2].set_data(time_array, yaw_data)
            # Only update ax1
            if time_array:
                xmin = time_array[0]
                xmax = time_array[-1] + 0.2
                ax1.set_xlim(xmin, xmax)
                # Optionally, autoscale y if needed:
                # ax1.set_ylim(min(min(roll_data), min(pitch_data), min(yaw_data))-5, max(max(roll_data), max(pitch_data), max(yaw_data))+5)
        return att_lines
    
    # Create animation - using a faster update interval for real-time responsiveness
    ani = FuncAnimation(fig, update_plot, interval=UPDATE_INTERVAL, blit=True, cache_frame_data=False)
    
    # Show plot with blocking call to prevent script termination
    plt.show()
    
except Exception as e:
    print(f"Error: {e}")

finally:
    # Signal thread to stop
    stop_thread = True
    if 'serial_thread' in locals():
        serial_thread.join(timeout=1.0)
    # Close serial port when done
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed")
    # Close CSV file
    if 'csv_file' in locals():
        csv_file.close()
        print(f"Data saved to {CSV_FILENAME}")