#!/usr/bin/env python3

import sys
import serial
import re
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
# Optionally import seaborn if using Option B
# import seaborn as sns

# Configuration
SERIAL_PORT = '/dev/serial/by-id/usb-FTDI_Quad_RS232-HS-if02-port0'
BAUD_RATE = 9600

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_sensor_data_bme.py <PROJECT_NAME>")
        sys.exit(1)

    project_name = sys.argv[1]

    # Determine what to plot based on the project name
    if 'temp' in project_name.lower():
        data_type = 'Temperature'
        y_label = 'Temperature (°C)'
        regex_pattern = r'Temperature\[\d+\]:\s+(-?\d+\.?\d*)\s+deg C'
    elif 'humidity' in project_name.lower():
        data_type = 'Humidity'
        y_label = 'Humidity (%RH)'
        regex_pattern = r'Humidity\[\d+\]:\s+(-?\d+\.?\d*)\s+%RH'
    elif 'pressure' in project_name.lower():
        data_type = 'Pressure'
        y_label = 'Pressure (Pa)'
        regex_pattern = r'Pressure\[\d+\]:\s+(-?\d+\.?\d*)\s+Pa'
    else:
        print("Unknown project type. Please ensure the project name includes 'temp', 'humidity', or 'pressure'.")
        sys.exit(1)

    # Initialize serial connection
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
    except serial.SerialException as e:
        print(f"Failed to connect to serial port: {e}")
        sys.exit(1)

    # Initialize plot
    # Option A: Update the style name
    plt.style.use('seaborn-v0_8-darkgrid')
    
    # Option B: Use seaborn directly
    # sns.set_style('darkgrid')

    fig, ax = plt.subplots()
    plt.title(f'Real-Time {data_type} Data')
    plt.xlabel('Sample Number')
    plt.ylabel(y_label)

    data_queue = deque(maxlen=100)  # Store last 100 data points
    x_data = deque(maxlen=100)      # Sample numbers

    line, = ax.plot([], [], 'b-', linewidth=2)

    def update(frame):
        # Read a line from the serial port
        try:
            line_bytes = ser.readline()
            line_str = line_bytes.decode('utf-8').strip()
            print(line_str)  # Print to console
        except UnicodeDecodeError:
            return line,

        # Use regex to extract the data value
        match = re.search(regex_pattern, line_str)
        if match:
            value = float(match.group(1))
            sample_match = re.search(r'\[(\d+)\]', line_str)
            if sample_match:
                sample_number = int(sample_match.group(1))
            else:
                sample_number = x_data[-1] + 1 if x_data else 0

            x_data.append(sample_number)
            data_queue.append(value)

            line.set_data(x_data, data_queue)
            ax.relim()
            ax.autoscale_view()
        return line,

    ani = animation.FuncAnimation(fig, update, interval=100, cache_frame_data=False)

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        print("Serial connection closed.")

if __name__ == '__main__':
    main()