from math import sin, cos, tan
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

def move(old_pose, d, beta, L, steering_drift):
    x, y, theta = old_pose
    beta += steering_drift
    alpha = (d / L) * tan(beta)
    R = d / alpha

    
    if abs(alpha) < 0.0001: #Move Straight
        new_x = x + d*cos(theta)
        new_y = y + d*sin(theta)
        new_theta = theta    
        
    else:
        cx = x - R*sin(theta)
        cy = y + R*cos(theta)        
        new_x = cx + R*sin(theta + alpha)
        new_y = cy - R*cos(theta + alpha)
        new_theta = (theta + alpha) % (2*np.pi)   

    return new_x, new_y, new_theta
    
def control_run(n, initial_state, d, lambda_p, lambda_d, L, steering_drift):
    x_trajectory = []
    y_trajectory = []
    orientation_trajectory = []  
    
    x,y,theta = initial_state    
    CTE = y
    CTE_tminus1 = CTE

    for _ in range (n):
        ddt_CTE = CTE-CTE_tminus1
        delta_time = 1
        beta = (-lambda_p*CTE - lambda_d*ddt_CTE)/1
        beta = max(-np.pi/4, min(beta, np.pi/4))
        
        x_trajectory.append(x)
        y_trajectory.append(y)
        orientation_trajectory.append(theta)
        
        x,y,theta = move ((x, y, theta), d, beta, L, steering_drift)
        CTE_tminus1 = CTE       
        CTE = y - np.sin(theta)
        
    return x_trajectory, y_trajectory, orientation_trajectory
        
n = 100
lambda_p = 0.2

initial_state = (0,1,0)
L = 20.0
v = 1.0
steering_drift = (10.0*np.pi)/180
d = 1.0

# Plotting the trajectory
x_ref = np.arange(0, n * d, d)
y_ref = np.zeros_like(x_ref)

#Graph of lambda_p = 0.2 & lambda_d = 0
lambda_d = 0
x_trajectory, y_trajectory, orientation_trajectory = control_run(n, initial_state, v, lambda_p, lambda_d, L,steering_drift)
plt.plot(x_trajectory, y_trajectory, color='blue',label=f'No Derivative λd=0.0')
plt.plot(x_ref, y_ref, label='Reference Trajectory',linestyle="--" ,color='red')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Car Trajectory')
plt.legend()
plt.grid(True)
plt.show()

#Graph of lambda_p = 0.2 & lambda_d = 3.0
lambda_d = 3.0
x_trajectory_with_differential, y_trajectory_with_differential, orientation_trajectory_with_differential = control_run(n, initial_state, v, lambda_p, lambda_d, L, steering_drift)
plt.plot(x_trajectory_with_differential, y_trajectory_with_differential, color='red', label=f'With Derivative λd=3.0')
plt.plot(x_ref, y_ref, label='Reference Trajectory',linestyle="--" ,color='red')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Car Trajectory with Steering Drift (With Differential)')
plt.legend()
plt.grid(True)
plt.show()

# Create DataFrame for lambda_d = 0
trajectory_data_no_differential = pd.DataFrame({
    'X': x_trajectory,
    'Y': y_trajectory,
    'Orientation': orientation_trajectory
})

# Create DataFrame for lambda_d = 3.0
trajectory_data_with_differential = pd.DataFrame({
    'X': x_trajectory_with_differential,
    'Y': y_trajectory_with_differential,
    'Orientation': orientation_trajectory_with_differential
})

# Specify the file path where you want to save the Excel file
excel_file_path = 'trajectory_data_ex1_partb-1.xlsx'

# Export DataFrames to different sheets in the same Excel file
with pd.ExcelWriter(excel_file_path) as writer:
    trajectory_data_no_differential.to_excel(writer, sheet_name='Lambda_d_0', index=False)
    trajectory_data_with_differential.to_excel(writer, sheet_name='Lambda_d_3.0', index=False)

print(f'Trajectory data exported to {excel_file_path}')