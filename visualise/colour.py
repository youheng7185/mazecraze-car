import serial
import re
import tkinter as tk

# Configure your serial port (change this accordingly)
SERIAL_PORT = "/dev/ttyACM0"  # Windows: "COMx" (e.g., "COM3"), Linux: "/dev/ttyUSBx"
BAUD_RATE = 115200  

# Open Serial Connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# Regular expression to parse your format
pattern = re.compile(r"RGB Values from (\d+): R = (\d+), G = (\d+), B = (\d+)")

# Tkinter GUI Setup
root = tk.Tk()
root.title("Color Sensor Visualization")

# Create color rectangles
canvas = tk.Canvas(root, width=300, height=100)
canvas.pack()

# Initial rectangles
rects = [
    canvas.create_rectangle(10, 10, 90, 90, fill="black", outline="black"),
    canvas.create_rectangle(110, 10, 190, 90, fill="black", outline="black"),
    canvas.create_rectangle(210, 10, 290, 90, fill="black", outline="black")
]

# Function to read and update colors
sensor_colors = {0: "black", 1: "black", 2: "black"}

def read_color():
    while True:
        try:
            line = ser.readline().decode().strip()
            match = pattern.match(line)
            if match:
                sensor_id = int(match.group(1))
                r = int(match.group(2))
                g = int(match.group(3))
                b = int(match.group(4))
                color = f"#{r:02x}{g:02x}{b:02x}"  # Convert to hex color
                sensor_colors[sensor_id] = color
                
                # Update color in the Tkinter GUI
                canvas.itemconfig(rects[sensor_id], fill=color)
        except Exception as e:
            print("Error:", e)

# Run the serial reading in the background
import threading
threading.Thread(target=read_color, daemon=True).start()

# Run the Tkinter main loop
root.mainloop()

