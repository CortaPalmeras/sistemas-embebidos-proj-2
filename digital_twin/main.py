import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Qt5Agg')  # Use the Qt5Agg backend
import numpy as np
import time

# Initialize the plot
fig, ax = plt.subplots()
ax.set_xlim(-1.5, 1.5)
ax.set_ylim(-1.5, 1.5)
ax.set_aspect('equal')

# Create an initial arrow at (0, 0) with a random direction
arrow_length = 1.0
arrow = ax.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1, color='blue')

# Initialize the angle of rotation (in radians)
angle = 0.0
spin_speed = np.pi / 20  # Speed of the rotation, radians per update

# Function to update the arrow direction
def update_arrow():
    global angle
    # Generate a random angle between 0 and 2π
    angle = (angle + spin_speed) % (2 * np.pi)

    # Calculate the new x and y components of the vector
    dx = arrow_length * np.cos(angle)
    dy = arrow_length * np.sin(angle)

    # Update the quiver object with new direction
    arrow.set_UVC(dx, dy)

# Run the update loop
while plt.get_fignums():  # While the figure is open
    update_arrow()
    plt.draw()
    time.sleep(0.2)  # Wait for 0.2 second before changing direction
    plt.pause(0.001)  # Allow the plot to update and be responsive