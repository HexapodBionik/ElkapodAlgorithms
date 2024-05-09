from ...kinematics.kinematics_solvers import KinematicsSolver
from .joint_trajectories import LSPBTwoPoints, SimpleJointSpaceTrajectory
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Qt5Agg')

# Wszystkie wartości translacji w metrach
# Translacja mocowania - pomiędzy world frame, a J1
mount_t = np.array([0, 0, 0.05, 1])

# Translacja T1 - pomiędzy J1, a J2
t1 = np.array([0.05, 0, -0.035, 1])

# Translacja T2 - pomiędzy J2, a J3
t2 = np.array([0.09, 0, 0, 1])

# Translacja T3 - pomiędzy J3, a FCP
t3 = np.array([0.1, 0, 0, 1])


kinematics_solver = KinematicsSolver(mount_t, t1, t2, t3, 0, -np.pi/2)
linear_trajectory = LSPBTwoPoints(kinematics_solver)

start_coordinates = np.array([0.01, -0.02, -0.15])
end_coordinates = np.array([0.01, 0.02, -0.135])

velocity_start = np.array([0, 0, 0])
velocity_end = np.array([0, 0, 0])
time_end = 3
max_acceleration = np.array([4, 0.1, 4]) # rads/s^2

# Time steps for whole movement probed with f=200Hz (T=5ms)
x = np.arange(0, time_end, 0.005)


linear_trajectory_polynomials = linear_trajectory.generate_trajectory(start_coordinates, end_coordinates, velocity_start, velocity_end, max_acceleration, time_end)


joint = 1
midtrajectory_positions = linear_trajectory_polynomials[joint](x)
midtrajectory_vels = linear_trajectory_polynomials[joint].get_velocity(x)
midtrajectory_accels = linear_trajectory_polynomials[joint].get_acceleration(x)

fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 5))

# Plot data on the first subplot
ax1.plot(x, midtrajectory_positions, color='blue')
ax1.set_title("Joint 0 position relative to timesteps")
ax1.set_xlabel("Timesteps")
ax1.set_ylabel("Position [rad]")
ax1.grid()

# Plot data on the second subplot
ax2.plot(x, midtrajectory_vels, color='green')
ax2.set_title("Joint 0 velocity relative to timesteps")
ax2.set_xlabel("Timesteps")
ax2.set_ylabel("Velocity [rad/s]")
ax2.grid()

# Plot data on the third subplot
ax3.plot(x, midtrajectory_accels, color='red')
ax3.set_title("Joint 0 acceleration relative to timesteps")
ax3.set_xlabel("Timesteps")
ax3.set_ylabel("Acceleration [rad/s^2]")
ax3.grid()

# Adjust layout to prevent overlap
plt.tight_layout()


x_points = []
y_points = []
z_points = []

theta1 = linear_trajectory_polynomials[0](x)
theta2 = linear_trajectory_polynomials[1](x)
theta3 = linear_trajectory_polynomials[2](x)
for i in range(len(x)):
    forward_coordinates = kinematics_solver.forward(theta1[i], theta2[i], theta3[i])
    x_points.append(forward_coordinates[0])
    y_points.append(forward_coordinates[1])
    z_points.append(forward_coordinates[2])

# Create a 3D scatter plot
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')
ax.set_title(label="Trajectory with 2 via points generated with spline polynomial")
scatter = ax.scatter(x_points, y_points, z_points, marker='o', label="Spline polynomial")
ax.view_init(elev=45, azim=135)

coordinate_points_x = [start_coordinates[0], end_coordinates[0]]
coordinate_points_y = [start_coordinates[1], end_coordinates[1]]
coordinate_points_z = [start_coordinates[2], end_coordinates[2]]
ax.scatter(coordinate_points_x, coordinate_points_y, coordinate_points_z, color='red', s=100, label="Interpolated points")

# Add labels
ax.set_xlabel('X Axis [m]')
ax.set_ylabel('Y Axis [m]')
ax.set_zlabel('Z Axis [m]')
ax.legend()

# Show the interactive plot
plt.show()