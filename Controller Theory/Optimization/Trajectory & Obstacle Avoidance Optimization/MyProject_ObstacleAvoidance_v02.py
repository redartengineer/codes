#==============================================================================
# Assignment: Obstacle Avoidance
# Programming Language: Python 
# Version: 02
# Summary: A robot will avoid obstacles while reaching its goal.
#==============================================================================

#=========Library=========
import numpy as np                                         # Import numerical computing library.
import matplotlib.pyplot as plt                            # Import plotting library.
from scipy.optimize import minimize                        # Import optimization solver.

#=========Object Placement and Parameters=========
num_waypoints = 20                                         # Number of control steps.
obstacles = [(3, 3), (7, 6), (6, 8)]                       # List of obstacle positions.
obstacle_radius = 0.6                                      # Radius of each circular obstacle.
goal = np.array([8, 9])                                    # Target position to reach.
start = np.array([0, 0, 0])                                # Starting position and orientation (x, y, theta).
dt = 0.5                                                   # Time step between waypoints.

#=========Weights (Lambdas)=========
lambda_goal = 2.0                                            # Penalty weight for being far from the goal.
lambda_obstacle = 15.0                                       # Penalty weight for being near obstacles.
lambda_smooth = 5.0                                          # Penalty for rapid changes in control inputs (v,w).

v_max = 1.5                                                # Maximum linear velocity.
w_max = np.pi / 4                                          # Maximum angular velocity.
epsilon = 1e-6                                             # Small value to avoid division by zero.
safety_d = 0.8                                             # Extra margin to keep from obstacles.

#=========Dynamics=========
def dynamics(state, control, dt):                          # Computes the next state from current state and control.
    x, y, theta = state                                    # Unpack current state.
    v, w = control                                         # Unpack control inputs.
    x_new = x + v * np.cos(theta) * dt                     # Compute new x position using linear velocity and orientation.
    y_new = y + v * np.sin(theta) * dt                     # Compute new y position using linear velocity and orientation.
    theta_new = theta + w * dt                             # Computes robot's orientation using angular velocity.
    
    return np.array([x_new, y_new, theta_new])             # Return new state as array.

#=========Cost Function=========
def cost_function(U):                                      # Defines total cost based on trajectory.
    U = U.reshape((-1, 2))                                 # Reshape flat vector into [v, w] pairs.
    trajectory = [start]                                   # Initialize trajectory with start state.
    cost = 0                                               # Start cost at zero.

    for t in range(len(U)):                                # Loop through each time step.
        v_t, w_t = U[t]                                    # Get current control inputs.
        state_t = trajectory[t]                            # Get current state.
        next_state = dynamics(state_t, [v_t, w_t], dt)     # Compute next state from dynamics.
        trajectory.append(next_state)                      # Append next state to trajectory.

        #==Goal Attraction Equation==
        J_goal = lambda_goal * np.linalg.norm(next_state[:2] - goal)**2  # Penalize distance to goal.
                                                                         # Weight multiplies into the squared euclidean distance--
                                                                         # of the robot position and goal position.
        #==Obstacle Avoidance Equation==
        J_obs = 0                                                        # Initialize obstacle penalty.
        for (ox, oy) in obstacles:                                       # Loop over each obstacle (ox, oy).
            d = np.linalg.norm(next_state[:2] - np.array([ox, oy]))      # Distance to obstacle, which is---
                                                                         # the Euclidean distance of the robot position and the obstacle. 
            if d < obstacle_radius + safety_d:                           # If the distance is less that the radius plus safety distance:
                J_obs += lambda_obstacle / (d - obstacle_radius + epsilon)**2   # Apply penalty.
                                                                                # Weight is divided by distance which is-- 
                                                                                # subtracted by the size of the-- 
                                                                                # obstacle's radius and an epsilon value.
                                                              
        #==v and w Smoothness Equation==
        if t > 0:                                           # When the simulation starts, the following activates:
            v_prev, w_prev = U[t - 1]                       # Get previous v and w control inputs.
            J_smooth = lambda_smooth * ((v_t - v_prev)**2 + (w_t - w_prev)**2)  # Penalize abrupt changes.
                                                                                # Obtains the difference of the--
                                                                                # linear and angular to find the squared euclidean,--
                                                                                # and multiplied by the weight.
        else:                                               # Otherwise:
            J_smooth = 0                                    # No penalty at first step.

        cost += J_goal + J_obs + J_smooth                   # Sum up cost terms.

    return cost                                             # Return total cost.

#=========Constraint Function=========
def dynamics_constraints(U):                               # Defines constraints to ensure safe and feasible trajectory.
    U = U.reshape((-1, 2))                                 # Reshape flat vector into [v, w] pairs.
    trajectory = [start]                                   # Initialize trajectory.
    constraint_list = []                                   # List of inequality constraints.

    for t in range(len(U)):
        v_t, w_t = U[t]                                    # Get current v and w control inputs represented as U[t].
        state_t = trajectory[t]                            # Get current state.
        next_state = dynamics(state_t, [v_t, w_t], dt)     # Simulate next state.
        trajectory.append(next_state)                      # Append to trajectory.
        
        #==v and w Limit Constraints Equations==
        constraint_list.append(v_max - abs(v_t))           # v ≤ v_max.
        constraint_list.append(w_max - abs(w_t))           # w ≤ w_max.

        #==Obstacle Avoidance Limit Constraints Equations==
        for ox, oy in obstacles:                           # For each obstacle:
            d = np.linalg.norm(next_state[:2] - np.array([ox, oy]))      # Distance to obstacle.
            clearance = d - obstacle_radius - safety_d     # Ensure robot has distance to avoid collision.
            constraint_list.append(clearance)              # Enforce clearance constraint.

    #==Goal Attraction Limit Constraints Equations==
    constraint_list.append(np.linalg.norm(trajectory[-1][:2] - goal))    # Ensure final position reaches goal.
    return np.array(constraint_list)                                     # Return constraints array.

#=========Optimization=========
U0 = np.tile(np.array([1.0, 0.0]), (num_waypoints - 1, 1)).flatten()  # Initial guess.
constraints = [{"type": "ineq", "fun": dynamics_constraints}]         # Define constraints.

result = minimize(cost_function, U0, constraints=constraints, method='SLSQP')  # Run solver.
U_opt = result.x.reshape((num_waypoints - 1, 2))                               # Get optimal controls.

#=========Simulate Trajectory=========
trajectory = [start]                                       # Initialize trajectory.
for v_t, w_t in U_opt:                                     # Loop over optimal controls:
    next_state = dynamics(trajectory[-1], [v_t, w_t], dt)  # Simulate next state.
    trajectory.append(next_state)                          # Append to trajectory.

#=========Plot=========
plt.figure(figsize=(6, 6))                                 # Set figure size.
for ob in obstacles:                                       # Draw each obstacle as a circle.
    plt.gca().add_patch(plt.Circle(ob, obstacle_radius, color='r', alpha=0.5))

x_vals = [s[0] for s in trajectory]                        # Get x values.
y_vals = [s[1] for s in trajectory]                        # Get y values.

plt.plot(x_vals, y_vals, 'bo-', label='Trajectory')        # Plot robot trajectory.
plt.scatter(start[0], start[1], color='b', label='Start')  # Mark start point.
plt.scatter(goal[0], goal[1], color='g', label='Goal', marker='x')  # Mark goal point.
plt.xlim(0, 10)                                            # Set x-axis limits.
plt.ylim(0, 10)                                            # Set y-axis limits.
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.title("Obstacle Avoidance on Python")
plt.grid()
plt.show()
