""" 
Code Name: Extremum Seeking Control Sine Signal Test
Objective:
    For the robot to follow a sine wave signal. The point is for
    the robot to follow the line which can be used for path planning
    when detecting a sound.
"""

# Imports Libraries
import numpy as np
import matplotlib.pyplot as plt

# This class will generate a sine wave representing a signal.
# Objective is to calculate the y value of position x.
# x is the position of the robot.
amp = 1.0         # Amplitude of sine wave
omega = 2.0       # Frequency of sine wave

"""
[Perturbation]:
Controlled disturbance or oscillation to a system's input. 
This technique helps in estimating the gradient of the 
sine wave and enables adaptive control.
Explores how changes in the input affect the output.
[Adaptive Control]:
Purpose: By using perturbation, the controller can adaptively adjust its inputs based on real-time feedback. 
"""
# Perturbation Parameters
per_amp = 0.1      # Amplitude of perturbation
per_omega = 0.5    # Frequency of perturbation

def signal (x):
    return amp*np.sin(omega*x)

def update_position(x, y, lr):
    """Update the robot's vertical position based on the distance from the sine wave."""
    find_y_sine = signal(x)  # Get the sine wave value at current x
    distance = y - find_y_sine  # Find the distance between y and find_y_sine

    # Adjust y based on the distance
    if distance > 0:                        # If the robot is above the sine wave
        y -= distance * lr                  # Move down to reduce distance
    elif distance < 0:                      # If the robot is below the sine wave
        y += abs(distance) * lr             # Move up to reduce distance
                                            # If distance is zero, no adjustment needed

    return y # Returns the value of y.

"""
This class moves the robot.
x_i represents the x coordinate incrementing.
steps counts how many steps should be added, more for smoother
learning_rate stability adjustment
"""
def move_robot (x_i, steps, lr):
    
    #Initializes the coordinates of the robot.
    x = 0.0
    y = 0.0
    x_history = []                     # Stores data values in x array to plot
    y_history = []                     # Stores data values in y array to plot
    for _ in range(steps):
        per = per_amp*np.sin(per_omega*x) # Add perturbation to the x-coordinate
        x += x_i + per                    #Robot moves forward with perturbation influence
        y = update_position(x,y,lr)       #Update vertical position
        
        # Store history for plotting
        x_history.append(x)
        y_history.append(y)
    return x_history, y_history
        
        
#Parameters
steps = 500   # The more steps, the smoother the robot's path
lr = 0.8      # Learning Rate
x_i = 0.1     # Robot's x-coordinate will increase 
              # by 0.1 for each of the 500 iterations
              # 50.0 (i.e., 0 + 0.1 * 500)

# Run the simulation
x_history, y_history = move_robot (x_i, steps, lr)

# Plotting results
plt.figure(figsize=(12, 6))

# Plot the sine wave
x_line = np.linspace(0, 20, 100)
y_line = signal(x_line)
plt.plot(x_line, y_line, label='Sine Wave: y = A * sin(B * x)', color='blue')

# Plot the robot's path
plt.plot(x_history, y_history, label='Robot Path', color='orange')
plt.scatter(x_history, y_history, color='red', s=10)  # Show robot positions

plt.title('Line Following Using Extremum Seeking Control on a Sine Wave')
plt.xlabel('Horizontal Position (x)')
plt.ylabel('Vertical Position (y)')
plt.legend()
plt.grid()
plt.axhline(0, color='gray', lw=0.5, ls='--')
plt.axvline(0, color='gray', lw=0.5, ls='--')
plt.xlim(0, 20)
plt.ylim(-2, 2)
plt.show()