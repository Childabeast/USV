import serial
import re
from time import time, sleep
from vpython import *
import math

# Serial port config
SERIAL_PORT = '/dev/cu.usbserial-1420'  # Windows: 'COMX',  Mac: '/dev/ttyUSB0'
BAUD_RATE = 115200

# Initialize serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
ser.flushInput()

# Create 3D scene
scene = canvas(title='MPU-6500 Orientation', width=1650, height=1000)
scene.forward = vector(-0.5, -0.5, -1)  # Adjust camera angle

# Boat dimensions
box_length = 2
box_height = 1
box_width = 0.5
edge_thickness = 0.05

# Create box frame
edges = [
    cylinder(pos=vector(-box_length/2, -box_height/2, -box_width/2), axis=vector(box_length, 0, 0), radius=edge_thickness),
    cylinder(pos=vector(-box_length/2, box_height/2, -box_width/2), axis=vector(box_length, 0, 0), radius=edge_thickness),
    cylinder(pos=vector(-box_length/2, -box_height/2, box_width/2), axis=vector(box_length, 0, 0), radius=edge_thickness),
    cylinder(pos=vector(-box_length/2, box_height/2, box_width/2), axis=vector(box_length, 0, 0), radius=edge_thickness),
    
    cylinder(pos=vector(-box_length/2, -box_height/2, -box_width/2), axis=vector(0, box_height, 0), radius=edge_thickness),
    cylinder(pos=vector(box_length/2, -box_height/2, -box_width/2), axis=vector(0, box_height, 0), radius=edge_thickness),
    cylinder(pos=vector(-box_length/2, -box_height/2, box_width/2), axis=vector(0, box_height, 0), radius=edge_thickness),
    cylinder(pos=vector(box_length/2, -box_height/2, box_width/2), axis=vector(0, box_height, 0), radius=edge_thickness),
    
    cylinder(pos=vector(-box_length/2, -box_height/2, -box_width/2), axis=vector(0, 0, box_width), radius=edge_thickness),
    cylinder(pos=vector(box_length/2, -box_height/2, -box_width/2), axis=vector(0, 0, box_width), radius=edge_thickness),
    cylinder(pos=vector(-box_length/2, box_height/2, -box_width/2), axis=vector(0, 0, box_width), radius=edge_thickness),
    cylinder(pos=vector(box_length/2, box_height/2, -box_width/2), axis=vector(0, 0, box_width), radius=edge_thickness)
]

# Group edges into boat object
sensor_box = compound(edges)
# Set initial orientation
sensor_box.axis = vector(1, 0, 0)
sensor_box.up = vector(0, 1, 0)

# Make reference axes
axes = compound([
    arrow(pos=vector(0,0,0), axis=vector(1,0,0), color=color.red, shaftwidth=0.05),
    arrow(pos=vector(0,0,0), axis=vector(0,1,0), color=color.green, shaftwidth=0.05),
    arrow(pos=vector(0,0,0), axis=vector(0,0,1), color=color.blue, shaftwidth=0.05)
])

# Gyro calibration variables
CALIBRATION_SAMPLES = 50
gyro_z_bias = 0.0

print("Calibrating gyro - keep sensor stationary...")
calibration_data = []
while len(calibration_data) < CALIBRATION_SAMPLES:
    line = ser.readline().decode().strip()
    # Expecting (ax,ay,az,gx,gy,gz)
    numbers = re.findall(r"[-+]?\d*\.\d+|\d+", line)
    if len(numbers) >= 6:
        _, _, _, _, _, gz = map(float, numbers)
        calibration_data.append(gz)
        
gyro_z_bias = sum(calibration_data) / len(calibration_data)
print(f"Calibration complete. Gyro Z bias: {gyro_z_bias:.2f} dps")

# Set boat orientation forwards after calibration
sensor_box.axis = vector(1, 0, 0)
sensor_box.up = vector(0, 1, 0)

# Global yaw angle (radians)
yaw = 0

# Each bump marker is stored as a dictionary with keys:
# marker (VPython arrow), local_offset, local_axis, creation_time
bump_markers = []
BUMP_DISPLAY_DURATION = 1.0  # Seconds

def update_orientation(gyro_z, dt):
    # Update based off z-axis gyro reading
    global yaw
    angular_velocity = math.radians(gyro_z - gyro_z_bias)
    yaw -= angular_velocity * dt
    # Reset orientation of sensor_box to calibrated zero state
    sensor_box.axis = vector(1, 0, 0)
    sensor_box.up = vector(0, 1, 0)
    sensor_box.rotate(angle=yaw, axis=vector(0, 1, 0))

def add_bump_marker(side):
    # Front is at +x.
    # For "Front Right", right is -z; for "Front Left", right is +z.
    if side == "Front Right":
        local_offset = vector(box_length/2, 0, -box_width/2)
    elif side == "Front Left":
        local_offset = vector(box_length/2, 0, box_width/2)
    else:
        local_offset = vector(box_length/2, 0, 0)
    # Define local arrow axis pointing +x
    local_axis = vector(0.5, 0, 0)
    # Compute global position and direction
    global_pos = sensor_box.pos + rotate(local_offset, angle=yaw, axis=vector(0,1,0))
    global_axis = rotate(local_axis, angle=yaw, axis=vector(0,1,0))
    marker = arrow(pos=global_pos, axis=global_axis, color=color.red, shaftwidth=0.1)
    bump_markers.append({
        'marker': marker,
        'local_offset': local_offset,
        'local_axis': local_axis,
        'creation_time': time()
    })

def update_bump_markers():
    # Update so arrows follow boat
    for bm in bump_markers:
        marker = bm['marker']
        local_offset = bm['local_offset']
        local_axis = bm['local_axis']
        # Recompute marker's global position and direction using current yaw
        marker.pos = sensor_box.pos + rotate(local_offset, angle=yaw, axis=vector(0,1,0))
        marker.axis = rotate(local_axis, angle=yaw, axis=vector(0,1,0))

# Main loop variable
last_time = time()

try:
    while True:
        rate(100)  # Limit loop to 100 iterations per second
        
        # Serial input
        if ser.in_waiting > 0:
            line = ser.readline().decode().strip()
            if "BUMP:" in line:
                if "Front Right" in line:
                    print("Bump detected on Front Left")
                    add_bump_marker("Front Left")
                elif "Front Left" in line:
                    print("Bump detected on Front Right")
                    add_bump_marker("Front Right")
                else:
                    print("Bump detected (unknown side)")
                    add_bump_marker("Unknown")
            else:
                # Parse regular data (ax,ay,az,gx,gy,gz)
                numbers = re.findall(r"[-+]?\d*\.\d+|\d+", line)
                if len(numbers) >= 6:
                    ax, ay, az, gx, gy, gz = map(float, numbers)
                    current_time = time()
                    dt = current_time - last_time
                    last_time = current_time
                    update_orientation(gz, dt)
        
        # Update markers to follow boat
        update_bump_markers()
        
        # Remove markers longer than BUMP_DISPLAY_DURATION
        for bm in bump_markers[:]:
            if time() - bm['creation_time'] > BUMP_DISPLAY_DURATION:
                bm['marker'].visible = False
                bump_markers.remove(bm)
                
except KeyboardInterrupt:
    ser.close()
    print("Serial connection closed")
