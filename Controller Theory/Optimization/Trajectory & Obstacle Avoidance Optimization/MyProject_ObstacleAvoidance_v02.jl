#-------------------------------------------------------------------------
# Assignment: Obstacle Avoidance
# Programming Language: Julia (JuMP)
# Version: 02
# Summary: A robot will avoid obstacles while reaching its goal.
#-------------------------------------------------------------------------

#-------Library-------
using JuMP                    # Optimization modeling language
using Ipopt                   # Nonlinear solver
using Plots                   # Plotting library

#-------Object Placement and Parameters-------
num_waypoints = 20                                # Number of control steps.
obstacles = [(3.0, 3.0), (7.0, 6.0), (6.0, 8.0)]  # List of obstacle positions.
obstacle_radius = 0.6                             # Radius of each circular obstacle.
goal = [8.0, 9.0]                                 # Target position to reach.
start = [0.0, 0.0, 0.0]                           # Starting position and orientation (x, y, theta).
dt = 0.5                                          # Time step between waypoints.

#-------Weights-------
lambda_goal = 2.0         # Penalty weight for being far from the goal.
lambda_obstacle = 15.0    # Penalty weight for being near obstacles.
lambda_smooth = 5.0       # Penalty for rapid changes in control inputs (v,w).

v_max = 1.5               # Max linear velocity.
w_max = pi / 4            # Max angular velocity.
epsilon = 1e-6            # Small value to avoid division by zero.
safety_d = 0.8            # Extra margin to keep from obstacles.

#-------Dynamics and Constraints-------
# Model
model = Model(Ipopt.Optimizer)   # Initialize optimization model with Ipopt solver

# Control variables: linear and angular velocities
@variable(model, v[1:num_waypoints-1])   # Linear velocity v[t].
@variable(model, w[1:num_waypoints-1])   # Angular velocity w[t].

# State variables: position and heading
@variable(model, x[1:num_waypoints])     # x position.
@variable(model, y[1:num_waypoints])     # y position.
@variable(model, theta[1:num_waypoints]) # Orientation.

# Initial State Constraints
@constraint(model, x[1] == start[1])     # Start x position.
@constraint(model, y[1] == start[2])     # Start y position.
@constraint(model, theta[1] == start[3]) # Start orientation.

# Dynamics
for t in 1:num_waypoints-1
    @NLconstraint(model, x[t+1] == x[t] + v[t] * cos(theta[t]) * dt)     # Compute new x position using linear velocity and orientation.
    @NLconstraint(model, y[t+1] == y[t] + v[t] * sin(theta[t]) * dt)     # Compute new y position using linear velocity and orientation.
    @NLconstraint(model, theta[t+1] == theta[t] + w[t] * dt)             # Computes robot's orientation using angular velocity.
end

# Control Limits
for t in 1:num_waypoints-1
    @constraint(model, -v_max <= v[t] <= v_max)   # Velocity bounds: v ≤ v_max.
    @constraint(model, -w_max <= w[t] <= w_max)   # Angular velocity bounds: w ≤ w_max.
end

# Obstacle Avoidance Constraints
for t in 1:num_waypoints
    for (ox, oy) in obstacles
        @NLconstraint(model, ((x[t] - ox)^2 + (y[t] - oy)^2) >= (obstacle_radius + safety_d)^2)  # Keep outside safety radius
    end
end
 # Distance constraint adding more distance to prevent robot from colliding with obstacles.                                                   

#-------Objective Functions-------
@NLobjective(model, Min,
    # 1. Goal Attraction Cost
    sum(lambda_goal * ((x[t] - goal[1])^2 + (y[t] - goal[2])^2) for t in 1:num_waypoints) +
# Penalize distance to goal.
# Weight multiplies into the squared euclidean distance of the robot position and goal position.

    # 2. Obstacle Repulsion Cost
    sum(lambda_obstacle / (((x[t] - ox)^2 + (y[t] - oy)^2 - obstacle_radius^2 + epsilon)^2)
        for t in 1:num_waypoints, (ox, oy) in obstacles) +
# Apply penalty.
# Weight is divided by distance which is subtracted by the size of the obstacle's radius and an epsilon value.

    # 3. Smoothness Cost
    sum(lambda_smooth * ((v[t+1] - v[t])^2 + (w[t+1] - w[t])^2) for t in 1:num_waypoints-2)
)
# Penalize abrupt changes.
# Obtains the difference of the linear and angular to find the squared euclidean and multiplied by the weight.

#-------Solve-------
optimize!(model)  # Run the optimization solver

#-------Extract & Plot-------
x_vals = value.(x)  # Extract optimized x coordinates
y_vals = value.(y)  # Extract optimized y coordinates

#-------Main trajectory plot-------
plot(x_vals, y_vals,
     label="Trajectory",
     marker=:circle,
     lw=2,
     xlabel="X",
     ylabel="Y",
     title="Obstacle Avoidance on Julia",
     xlims=(0, 10),
     ylims=(0, 10),
     aspect_ratio=:equal,
     size=(700, 700))

#-------Plot obstacles as red shaded circles-------
for (ox, oy) in obstacles
    θ = range(0, 2π, length=50)
    x_circle = ox .+ obstacle_radius * cos.(θ)
    y_circle = oy .+ obstacle_radius * sin.(θ)
    plot!(x_circle, y_circle, seriestype=:shape, color=:red, alpha=0.3, label="")
end

#-------Start and goal markers-------
scatter!([start[1]], [start[2]], color=:blue, label="Start")
scatter!([goal[1]], [goal[2]], color=:green, label="Goal", marker=:x)

#-------Display Plot on Preview Window-------
display(current())  # Show the plot in VS Code or REPL

#-------Save the final plot to PNG-------
savefig(joinpath(@__DIR__, "Julia_ObstacleAvoidance_Plot_v01.png")) # Save plot image as PNG in current directory