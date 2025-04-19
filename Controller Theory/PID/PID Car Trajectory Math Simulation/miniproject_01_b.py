from math import sin, cos, tan
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

def move(old_pose, d, beta, L):
    x, y, theta = old_pose

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
    
def control_run(n, initial_state, d, lambda_p, lambda_d, L):
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
        
        x,y,theta = move ((x, y, theta), d, beta, L)
        CTE_tminus1 = CTE       
        CTE = y - np.sin(theta)
        
    return x_trajectory, y_trajectory, orientation_trajectory
        
n = 100
lambda_p = 0.2
lambda_d = 3.0
initial_state = (0,1,0)
L = 20.0
v = 1.0
d = 1.0

# Plotting the trajectory
x_ref = np.arange(0, n * d, d)
y_ref = np.zeros_like(x_ref)
plt.plot(x_ref, y_ref, label='Reference Trajectory',linestyle="--" ,color='red')

#Graph of Just 0.3
x_trajectory, y_trajectory, orientation_trajectory = control_run(n, initial_state, v, lambda_p, lambda_d, L)
plt.plot(x_trajectory, y_trajectory, color='blue',label=f'λp=0.2')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Car Trajectory')
plt.legend()
plt.grid(True)
plt.show()

trajectory_data = pd.DataFrame({
    'X': x_trajectory,
    'Y': y_trajectory,
    'Orientation': orientation_trajectory
})

excel_file_path = 'trajectory_data_ex1_partb.xlsx'
trajectory_data.to_excel(excel_file_path, index=False)

print(f'Trajectory data exported to {excel_file_path}')