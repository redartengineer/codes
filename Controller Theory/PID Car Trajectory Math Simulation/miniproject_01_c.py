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
    
def control_run(n, initial_state, d, lambda_p, lambda_d, lambda_I, L, steering_drift):
    x_trajectory = []
    y_trajectory = []
    orientation_trajectory = []  
    
    x,y,theta = initial_state    
    CTE = y
    CTE_tminus1 = CTE
    CTE_sum = 0

    for _ in range (n):
        ddt_CTE = CTE-CTE_tminus1
        delta_time = 1
        CTE_sum += CTE
        beta = (-lambda_p*CTE - lambda_d*ddt_CTE - lambda_I*CTE_sum)/1
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
lambda_d = 3.0
initial_state = (0,1,0)
L = 20.0
v = 1.0
steering_drift = (10.0*np.pi)/180
d = 1.0

# Plotting the trajectory
x_ref = np.arange(0, n * d, d)
y_ref = np.zeros_like(x_ref)

#Graph of lambda_I = 0.004
lambda_I = 0.004
x_trajectory, y_trajectory, orientation_trajectory = control_run(n, initial_state, v, lambda_p, lambda_d, lambda_I, L,steering_drift)
plt.plot(x_trajectory, y_trajectory, color='blue',label=f'λI=0.004')
plt.plot(x_ref, y_ref, label='Reference Trajectory',linestyle="--" ,color='red')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Car Trajectory with Steering Drift (With Integral)')
plt.legend()
plt.grid(True)
plt.show()

#Graph of lambda_I = 0.001
lambda_I = 0.001
x_trajectory_with_integral, y_trajectory_with_integral, orientation_trajectory_with_integral = control_run(n, initial_state, v, lambda_p, lambda_d, lambda_I, L, steering_drift)
plt.plot(x_trajectory_with_integral, y_trajectory_with_integral, color='red', label=f'λI=0.001')
plt.plot(x_ref, y_ref, label='Reference Trajectory',linestyle="--" ,color='red')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Car Trajectory with Steering Drift (With Integral)')
plt.legend()
plt.grid(True)
plt.show()

#Graph of lambda_I = 0.1
lambda_I = 0.1
x_trajectory_with_integral, y_trajectory_with_integral, orientation_trajectory_with_integral = control_run(n, initial_state, v, lambda_p, lambda_d, lambda_I, L, steering_drift)
plt.plot(x_trajectory_with_integral, y_trajectory_with_integral, color='Green', label=f'λI=0.1')
plt.plot(x_ref, y_ref, label='Reference Trajectory',linestyle="--" ,color='red')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Car Trajectory with Steering Drift (With Integral)')
plt.legend()
plt.grid(True)
plt.show()

# Export data for lambda_I = 0.004
lambda_I = 0.004
x_traj, y_traj, orientation_traj = control_run(n, initial_state, v, lambda_p, lambda_d, L, steering_drift, lambda_I)
trajectory_data = pd.DataFrame({
    'X': x_traj,
    'Y': y_traj,
    'Orientation': orientation_traj
})

# Export data for lambda_I = 0.001
lambda_I = 0.001
x_traj_001, y_traj_001, orientation_traj_001 = control_run(n, initial_state, v, lambda_p, lambda_d, L, steering_drift, lambda_I)
trajectory_001_data = pd.DataFrame({
    'X': x_traj_001,
    'Y': y_traj_001,
    'Orientation': orientation_traj_001
})

# Export data for lambda_I = 0.1
lambda_I = 0.1
x_traj_1, y_traj_1, orientation_traj_1 = control_run(n, initial_state, v, lambda_p, lambda_d, L, steering_drift, lambda_I)
trajectory_1_data = pd.DataFrame({
    'X': x_traj_1,
    'Y': y_traj_1,
    'Orientation': orientation_traj_1
})

# Specify the file path where you want to save the Excel file
excel_file_path = 'trajectory_data_ex1_partc.xlsx'

# Export DataFrame to Excel file
with pd.ExcelWriter(excel_file_path) as writer:
    trajectory_data.to_excel(writer, sheet_name=f'Lambda_I_0.004', index=False)
    trajectory_001_data.to_excel(writer, sheet_name=f'Lambda_I_0.001', index=False)
    trajectory_1_data.to_excel(writer, sheet_name=f'Lambda_I_0.1', index=False)

print(f'Trajectory data exported to {excel_file_path}')