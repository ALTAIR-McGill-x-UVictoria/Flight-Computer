import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from collections import deque
import time
import threading
import queue

# Configuration
PORT = 'COM5'
BAUD_RATE = 115200  # Adjust based on your serial configuration
BUFFER_SIZE = 500   # Number of data points to display
UPDATE_INTERVAL = 20  # Update plot every 20ms (50 FPS) for smoother real-time display

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
# Angular velocities (rollspeed, pitchspeed, yawspeed)
rollspd_data = deque(maxlen=BUFFER_SIZE)
pitchspd_data = deque(maxlen=BUFFER_SIZE)
yawspd_data = deque(maxlen=BUFFER_SIZE)
# Linear accelerations (xacc, yacc, zacc)
xacc_data = deque(maxlen=BUFFER_SIZE)
yacc_data = deque(maxlen=BUFFER_SIZE)
zacc_data = deque(maxlen=BUFFER_SIZE)
# Gyroscope data (xgyro, ygyro, zgyro)
xgyro_data = deque(maxlen=BUFFER_SIZE)
ygyro_data = deque(maxlen=BUFFER_SIZE)
zgyro_data = deque(maxlen=BUFFER_SIZE)

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

# Angular velocity plot
angvel_lines = []
angvel_lines.append(ax2.plot([], [], label='Roll Speed', color='r')[0])
angvel_lines.append(ax2.plot([], [], label='Pitch Speed', color='g')[0])
angvel_lines.append(ax2.plot([], [], label='Yaw Speed', color='b')[0])

# Acceleration plot
acc_lines = []
acc_lines.append(ax3.plot([], [], label='X Accel', color='r')[0])
acc_lines.append(ax3.plot([], [], label='Y Accel', color='g')[0])
acc_lines.append(ax3.plot([], [], label='Z Accel', color='b')[0])

# Gyroscope plot
gyro_lines = []
gyro_lines.append(ax4.plot([], [], label='X Gyro', color='r')[0])
gyro_lines.append(ax4.plot([], [], label='Y Gyro', color='g')[0])
gyro_lines.append(ax4.plot([], [], label='Z Gyro', color='b')[0])

# Configure plots
ax1.set_title('Attitude (rad)')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Angle (rad)')
ax1.legend(loc='upper left')
ax1.grid(True)

ax2.set_title('Angular Velocity (rad/s)')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Angular Velocity (rad/s)')
ax2.legend(loc='upper left')
ax2.grid(True)

ax3.set_title('Acceleration (m/s²)')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Acceleration (m/s²)')
ax3.legend(loc='upper left')
ax3.grid(True)

ax4.set_title('Gyroscope (rad/s)')
ax4.set_xlabel('Time (s)')
ax4.set_ylabel('Angular Velocity (rad/s)')
ax4.legend(loc='upper left')
ax4.grid(True)

# Set fixed y-axis limits for more stable visualization
# These can be adjusted based on typical value ranges
ax1.set_ylim(-3.5, 3.5)  # Attitude in radians
ax2.set_ylim(-2.0, 2.0)  # Angular velocity
ax3.set_ylim(-20, 20)    # Acceleration
ax4.set_ylim(-2.0, 2.0)  # Gyroscope

# Adjust layout
plt.tight_layout()
plt.subplots_adjust(top=0.9)

# Thread function to read serial data
def read_serial_data(ser):
    global stop_thread
    start_time = time.time()
    
    while not stop_thread:
        if ser.in_waiting > 0:
            try:
                # Read and decode the line
                line = ser.readline().decode('utf-8').strip()
                
                # Parse the CSV data
                values = list(map(float, line.split(',')))
                
                # Ensure we have all 12 values
                if len(values) == 12:
                    current_time = time.time() - start_time
                    # Put the data in the queue
                    data_queue.put((current_time, values))
                    
            except Exception as e:
                print(f"Error reading serial data: {e}")
                
        # Small sleep to prevent CPU hogging
        time.sleep(0.001)

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
            
            # Extract and store values
            roll_data.append(values[0])
            pitch_data.append(values[1])
            yaw_data.append(values[2])
            
            rollspd_data.append(values[3])
            pitchspd_data.append(values[4])
            yawspd_data.append(values[5])
            
            xacc_data.append(values[6])
            yacc_data.append(values[7])
            zacc_data.append(values[8])
            
            xgyro_data.append(values[9])
            ygyro_data.append(values[10])
            zgyro_data.append(values[11])
        
        # Only update plots if we have new data
        if new_data:
            # Update plot data
            time_array = list(time_data)
            
            # Update attitude plot
            att_lines[0].set_data(time_array, roll_data)
            att_lines[1].set_data(time_array, pitch_data)
            att_lines[2].set_data(time_array, yaw_data)
            
            # Update angular velocity plot
            angvel_lines[0].set_data(time_array, rollspd_data)
            angvel_lines[1].set_data(time_array, pitchspd_data)
            angvel_lines[2].set_data(time_array, yawspd_data)
            
            # Update acceleration plot
            acc_lines[0].set_data(time_array, xacc_data)
            acc_lines[1].set_data(time_array, yacc_data)
            acc_lines[2].set_data(time_array, zacc_data)
            
            # Update gyro plot
            gyro_lines[0].set_data(time_array, xgyro_data)
            gyro_lines[1].set_data(time_array, ygyro_data)
            gyro_lines[2].set_data(time_array, zgyro_data)
            
            # Adjust x-axis to fill entire plot width with available data
            for ax in [ax1, ax2, ax3, ax4]:
                if time_array:
                    # Use full range of available data
                    xmin = time_array[0]  # Start from earliest data point 
                    xmax = time_array[-1] + 0.2  # Add small padding
                    ax.set_xlim(xmin, xmax)
        
        # Return all lines that need to be updated
        return att_lines + angvel_lines + acc_lines + gyro_lines
    
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