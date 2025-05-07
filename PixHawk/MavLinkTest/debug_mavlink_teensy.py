import serial
import datetime
import argparse
import os

def log_serial_data(port, baud_rate, log_file_path):
    """Read data from a serial port and log it to a text file."""
    try:
        # Open serial port
        ser = serial.Serial(port=port, baudrate=baud_rate, timeout=None)
        print(f"Connected to {port} at {baud_rate} baud")
        
        # Make sure the log directory exists
        os.makedirs(os.path.dirname(log_file_path) or '.', exist_ok=True)
        
        # Open log file
        with open(log_file_path, 'w', buffering=1) as log_file:
            # Write header with timestamp
            start_time = datetime.datetime.now()
            log_file.write(f"# Serial log started at {start_time}\n")
            log_file.write(f"# Port: {port}, Baud rate: {baud_rate}\n\n")
            
            print(f"Logging to {log_file_path}. Press Ctrl+C to stop.")
            
            # Read and log data
            while True:
                data = ser.readline()
                if data:
                    # Get current timestamp
                    timestamp = datetime.datetime.now()
                    
                    # Try to decode the data
                    try:
                        data_str = data.decode('utf-8').strip()
                    except UnicodeDecodeError:
                        # If decoding fails, use hex representation
                        data_str = data.hex()
                    
                    # Write data to log file with timestamp
                    log_file.write(f"[{timestamp}] {data_str}\n")
                    
                    # Also print to console
                    print(f"[{timestamp}] {data_str}")

    except KeyboardInterrupt:
        print("\nLogging stopped by user")
    except serial.SerialException as e:
        print(f"Error with serial port: {e}")
    finally:
        # Close the serial port if it's open
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print(f"Serial port {port} closed")

def main():
    parser = argparse.ArgumentParser(description='Log data from a serial port to a file')
    parser.add_argument('--port', default='COM1', help='Serial port to read from')
    parser.add_argument('--baud', type=int, default=9600, help='Baud rate')
    parser.add_argument('--file', default='serial_log.txt', help='Path to output log file')
    
    args = parser.parse_args()
    
    log_serial_data(
        port=args.port,
        baud_rate=args.baud,
        log_file_path=args.file
    )

if __name__ == '__main__':
    main()