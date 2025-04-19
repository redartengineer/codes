from math import sin, cos, tan
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

def move(old_pose, d, beta, L, steering_drift):
    x, y, theta = old_pose
    beta += steering_drift
    alpha = (d / L) * tan(beta)
    R = d / alpha

    if abs(alpha) < 0.001: #Move Straight
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
    total_CTE_squared = 0

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
        total_CTE_squared += CTE**2
        
    average_CTE = total_CTE_squared/n
        
    return x_trajectory, y_trajectory, average_CTE
        
def coordinate_ascent (n, d, L, steering_drift):
    p = [0.0, 0.0, 0.0]
    dp = [1.0, 1.0, 1.0]
    initial_state = (0,1,0)
    best_CTE = 1.0e6
    
    while sum(dp) > 0.0001:
        for i in range(len(p)):
            p[i] = p[i] + dp[i]
            lambda_p, lambda_d, lambda_I = p 
            x_traj, y_traj, average_CTE = control_run(n, initial_state, d, lambda_p, lambda_d, lambda_I, L, steering_drift)
            
            if average_CTE < best_CTE: #Found smaller CTE
                best_CTE = average_CTE
                dp[i] = 0.8*dp[i]
                    
            else:
                p[i] = p[i] - 2*dp[i]
                lambda_p, lambda_d, lambda_I = p  # Unpack controller parameters
                x_traj, y_traj, average_CTE = control_run(n, initial_state, d, lambda_p, lambda_d, lambda_I, L, steering_drift)
                    
                if average_CTE < best_CTE:
                    best_CTE = average_CTE
                    dp[i] = 1.1*dp[i]
                else:
                    p[i] = p[i] + dp[i]
                    dp[i] = 0.9*dp[i]
    return p, best_CTE
    
# Main function to execute the optimization process
def main():
    n = 100  # Number of steps
    d = 1.0  # Distance per time step
    L = 20.0  # Wheelbase of the car
    steering_drift = (10.0 * np.pi) / 180.0  # Drift in steering angle

    # Perform coordinate ascent to optimize controller parameters
    optimal_params, min_CTE = coordinate_ascent(n, d, L, steering_drift)
    lambda_p, lambda_d, lambda_I = optimal_params  # Optimal controller parameters
    
    # Print the values
    print("Optimized Controller Parameters:")
    print(f"Lambda_P: {lambda_p}")
    print(f"Lambda_D: {lambda_d}")
    print(f"Lambda_I: {lambda_I}")
    print(f"Minimum Average CTE: {min_CTE}")
    
    # Plotting the trajectory
    x_ref = np.arange(0, n * d, d)
    y_ref = np.zeros_like(x_ref)

    x_traj, y_traj, average_CTE = control_run(n, (0, 1, 0), d, lambda_p, lambda_d, lambda_I, L, steering_drift)

    plt.plot(x_ref, y_ref, label='Reference Trajectory', linestyle="--" ,color='red')
    plt.plot(x_traj, y_traj, label='Car Trajectory', color='blue')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Car Trajectory vs Reference Trajectory')
    plt.legend()
    plt.grid(True)
    plt.show()
    
    # Export to Excel
    df = pd.DataFrame({'X': x_traj, 'Y': y_traj})
    df.to_excel('trajectory_data_ex2.xlsx', index=False)

    # Append controller parameters and min_CTE to the same Excel file
    df_params = pd.DataFrame({
        'Parameter': ['Lambda_P', 'Lambda_D', 'Lambda_I', 'Minimum_CTE'],
        'Value': [lambda_p, lambda_d, lambda_I, min_CTE]
    })
    with pd.ExcelWriter('trajectory_data_ex2.xlsx', mode='a', engine='openpyxl') as writer:
        df_params.to_excel(writer, sheet_name='Parameters', index=False)

if __name__ == "__main__":
    main()
