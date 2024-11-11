""" 
Author: Kyara Gandara
Lab: SDSU DSIM Lab
Date: 11/10/2024
Code Name: Extremum Seeking Control Follow Line Test
Version: 01
Objective:
    For the robot to follow a straight line. The line represents a signal. 
    The purpose is for the robot to detect a sound and adjust its location accordingly.
    
[Perturbation]:
A controlled disturbance or oscillation applied to a system's input. 
This technique aids in estimating the gradient of the cost function 
and enables adaptive control by allowing the system to explore how changes in input affect output.

[Adaptive Control]:
Purpose: By using perturbation, the controller can adaptively adjust its inputs based on real-time feedback, 
helping the robot minimize the cost and align with the target line.
"""

# Imports Libraries
import numpy as np
import matplotlib.pyplot as plt

# Parameters for the simulation
initial_position = np.array([0, 0]) # Declares initial position of robot
initial_heading = 0                 # Declared heading value, will change later
linear_speed = 0.5                  # Constant forward speed
dt = 0.1                            # Time step
steps = 1000                        # Number of steps in the for loop

x_target = []                       # Array to store the values of the iterating x value.
y_target = []                       # Array to store the values of the iterating y value.

for t in range (steps):             # For loop to simulate the iterating x and y values.
    x_star = 0.1*t*dt               # 0.1 is the slope. dt represents a time step.
    y_star = 0.1*t*dt               # t represents the time index.
    
    x_target.append(0.1*t*dt)       # Appends the calculated x value to the x_target array.
    y_target.append(0.1*t*dt)       # Appends the calculated y value to the y_target array.

# Kinematic update function for a differential drive robot
def update_position (position, heading, linear_speed, angular_speed, dt):   # Position: x & y cooridnates of the robot.
                                                                            # Heading: direction the robot is facing (radians).
                                                                            # Linear velocity: rate of change of a robot's position along a straight path.
                                                                            # Angular velocity: rate of change of a robot's rotational position around a fixed axis (radians per unit time).
                                                                            # dt: time step.
    new_x = position[0] + linear_speed * np.cos(heading) * dt               # x update: the kinematic equation new_x = x + v*cos(theta)*dt is used.
    new_y = position[1] + linear_speed * np.sin(heading) * dt               # y update: the kinematic equation new_y = y + v*sin(theta)*dt is used.
    new_heading = heading + angular_speed * dt                              # heading update: the kinematic equation new_theta = theta + omega*dt is used.
    return np.array([new_x, new_y]), new_heading                            # Return the new position and heading as an array and scalar.

# Define the cost function based on the distance to the target path
def cost_function (position, t_index):                                      # Function that calculates the cost based on distance to target path.
    target_position_x = x_target[t_index]                                   # Extracts the target x coordinate from x_target at index t_index.
    target_position_y = y_target[t_index]                                   # Extracts the target y coordinate from y_target at index t_index.
    
    distance_to_target_x = position[0] - target_position_x                  # Calculates x distance (error) of the actual robot and target position.
    distance_to_target_y = position[1] - target_position_y                  # Calculates y distance (error) of the actual robot and target position.
    
    distance_to_target = np.sqrt(distance_to_target_x**2 + distance_to_target_y**2) # Calculate Euclidean distance (magnitude of the vector) from the robot to the target.
    return distance_to_target**2                                                    # Return squared distance as the cost. Penalizes larger deviations from the target.
# Cost is proportional to squared distance        
# It quantifies the deviation from the target path, so minimizing the cost aligns the robot's trajectory with the target path.


# ESC Parameters
omega = 1.0             # Frequency of the perturbation in ESC, defining oscillation rate.
amplitude = 0.01        # Amplitude of oscillatory input, influencing ESC's search range.
k = 0.5                 # Gain for ESC adjustment, controlling response to cost changes.
epsilon = 1e-2          #  Small constant to stabilize gradient estimation, preventing large jumps.

# Lists to store the robot's path
x_history = []          # Stores x coordinates to plot the robot's trajectory.
y_history = []          # Stores y coordinates to plot the robot's trajectory.

# Initial position, heading, and cost for ESC
position = initial_position                 # Starting position of the robot, initially set to [0, 0].
heading = initial_heading                   # Starting heading of the robot, initially set to 0 radians.
previous_cost = cost_function(position, 0)  # Initial cost based on starting position.


# Run the simulation
for t in range(steps):                                  # Loop through each time step to calculate the robot's position.
    time = t*dt                                         # Calculate the current time by multiplying the step counter by the time step.
    
    perterbation = amplitude*np.sin(omega*time)         # Calculate the ESC perturbation as a sine wave influenced by amplitude, omega (frequency), and time.
    current_cost = cost_function (position,t)           # Compute the current cost based on the robot's position and the target path at step t.
    gradient = (current_cost - previous_cost)/epsilon   # Estimate the gradient (rate of cost change) by taking the difference in cost, scaled by epsilon.
                                                        # Epsilon is a small constant that controls the gradient sensitivity.
    
    linear_speed = 0.5                                  # Constant linear speed to keep the robot moving at a steady pace.
    angular_speed = perterbation - k*gradient           # Calculate angular speed; it's influenced by ESC perturbation and adjusted by the cost gradient.
    
    # Update the position and heading
    position, heading = update_position(position, heading, linear_speed, angular_speed, dt)
    
    # Store the position for plotting
    x_history.append(position[0])
    y_history.append(position[1])
    
    # Update the previous cost for the next iteration
    previous_cost = current_cost

# Plot the robot's path and target path
plt.figure(figsize=(10, 6))
plt.plot(x_history, y_history, label='Robot Path (with ESC)', color='orange')
plt.plot(x_target, y_target, label='Target Path', color='green')
plt.scatter(x_history, y_history, color='red', s=10)  # Show robot positions

# Plot labels and legend
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Robot Following Target Path Using ESC')
plt.legend()
plt.grid()
plt.show()
