

#Modify the following line with your own serial port details
#   Currently set COM3 as serial port at 115.2kbps 8N1
#   Refer to PySerial API for other options.  One option to consider is
#   the "timeout" - allowing the program to proceed if after a defined
#   timeout period.  The default = 0, which means wait forever.

import serial
import numpy as np
import open3d as o3d


s = serial.Serial('COM3', 115200)
                            
print("Opening: " + s.name)

# Reset the buffers of the UART port to delete the remaining data in the buffers
s.reset_output_buffer()
s.reset_input_buffer()

# Wait for the user's signal to start the program
input("Press Enter to start communication...")

# Send the character 's' to MCU via UART to signal MCU to start the transmission
s.write('s'.encode())
points = []
count = 0;
while count<48:
    line = s.readline().decode()
    try:
        x, y, z = map(float, line.split(','))
        points.append([x, y, z])
        print("Received point:", x, y, z)
        count+=1
    except ValueError:
        print(line)

# Close the serial port
print("Closing: " + s.name)
s.close()

# Convert the received points to a point cloud
point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(points)

# Create a line set geometry
lines = []
for i in range(0, len(points), 16):
    for j in range(15):
        lines.append([i + j, i + j + 1])
    lines.append([i + 15, i])

line_set = o3d.geometry.LineSet()
line_set.points = point_cloud.points
line_set.lines = o3d.utility.Vector2iVector(lines)

# Visualize the point cloud and lines
o3d.visualization.draw_geometries([point_cloud, line_set])
