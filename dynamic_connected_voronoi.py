#!/usr/bin/env python3

import csv
import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from scipy.spatial.distance import cdist
from scipy.spatial import Voronoi, voronoi_plot_2d
from scipy.sparse.csgraph import minimum_spanning_tree

import rps.robotarium as robotarium
from rps.utilities.graph import *
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *

from utils import generate_gaussian_distribution

# Simulation Variables
# fmt: off
N = 5                    # Number of robots
iterations = 1000        # How many iterations do we want (about N*0.033 seconds)
L = lineGL(N)            # Generated a connected graph Laplacian (for a cylce graph).
x_min = -1.5             # Upper bound of x coordinate
x_max = 1.5              # Lower bound of x coordinate
y_min = -1               # Upper bound of y coordinate
y_max = 1                # Lower bound of x coordinate
res = 0.05               # Resolution of coordinates
kv = 1                   # Constant Gain for velocity controller

max_range = np.sqrt(((x_max-x_min) ** 2) + ((y_max-y_min) ** 2))   # Calculate the maximum range to cover entire simulation
Rc = max_range/4.0                                                 # Sets the static communication range for all of the robots
gamma = 1                                                          # Sets the constant that is used in the "h" calculation
convergence_threshold = 2e-3                                       # Threshold to determine if convergence has occurred
initial_conditions = np.asarray([                                  # Sets the initial positions of the robots
    [1.25, 0.25, 0],
    [1, 0.5, 0],
    [1, -0.5, 0],
    [-0.1, -0.65, 0],
    [0.1, 0.2, 0],
    [0.2, -0.6, 0],
    [-0.75, -0.1, 0],
    [-1, 0, 0],
    [-0.8, -0.25, 0],
    [1.3, -0.4, 0]
])
# fmt: on

# Status and Performance Variables
poses = []
wij = []
Cwi = []
ui = []
dist_robot = []
total_Hg = []
total_Hp = []
total_Hv = []
total_Hr = []
total_Hc = []
previous_x = None
previous_y = None
converged_iteration = -1

# Instantiate Robotarium object
r = robotarium.Robotarium(number_of_robots=N, sim_in_real_time=False, initial_conditions=initial_conditions[0:N].T)

# Helper Functions for Simulation
si_barrier_cert = create_single_integrator_barrier_certificate_with_boundary()
si_to_uni_dyn, uni_to_si_states = create_si_to_uni_mapping()

# Initialize figure
fig, ax = plt.subplots()

# Generate a distribution function for the environment that respresents communication ability
X, Y, density = generate_gaussian_distribution(False)

# Iterate for the amount defined
for k in range(iterations):
    # Get the poses of the robots and convert to single-integrator poses
    x = r.get_poses()
    x_si = uni_to_si_states(x)
    poses.append(x_si.tolist())
    current_x = x_si[0, :, None]
    current_y = x_si[1, :, None]

    # Instantiate calculation variables to zero
    Hp = 0
    c_v = np.zeros((N, 2))
    w_v = np.zeros(N)
    cwi = np.zeros((2, N))
    weights = np.zeros((N, N))
    u_desired = np.zeros((2, N))
    distance_traveled = np.zeros(N)
    si_velocities = np.zeros((2, N))

    # Nested loop that occurs for each coordinate, this is the calculation portion of the code for the Voronoi cells
    for ix in np.arange(x_min, x_max, res):
        for iy in np.arange(y_min, y_max, res):
            # Set the importance value to 1, this represents the distribution function
            importance_value = 1

            # Instantiate the distance array to zero for all values
            distances = np.zeros(N)

            # Calculate the distance of each robot to the current point
            for robots in range(N):
                distances[robots] = np.sqrt(np.square(ix - current_x[robots].item()) + np.square(iy - current_y[robots].item()))

            # Find the robot with the smallest distance
            min_index = np.argmin(distances)

            # Calculations for the Standard Voronoi Partitioning
            c_v[min_index][0] += ix * importance_value
            c_v[min_index][1] += iy * importance_value
            w_v[min_index] += importance_value

            # Calculate cost
            Hp += 0.5 * (distances[min_index] ** 2) * importance_value

    # Compute distances between robots
    robot_distances = cdist(np.array(x_si).T, np.array(x_si).T, 'euclidean')

    # Iterate for the number of robots, this is the velocity and weight controller portion of the code
    for robots in range(N):
        # Calculate the distance travled from the previous iteration
        # fmt: off
        if previous_x is not None and previous_y is not None:
            distance_traveled[robots] = float(np.sqrt((abs(current_x[robots][0] - previous_x[robots][0]) ** 2) +
                                                      (abs(current_y[robots][0] - previous_y[robots][0]) ** 2)))
        # fmt: on

        # Calculate the x and y coordinates of the centroids of the Weighted Voronoi Partitions
        if not w_v[robots] == 0:
            c_x = c_v[robots][0] / w_v[robots]
            c_y = c_v[robots][1] / w_v[robots]

            # Calculate the optimal velocity of each robot
            u_desired[:, robots] = kv * np.array([(c_x - current_x[robots][0]), (c_y - current_y[robots][0])])

            # Append the current centroid and velocity to the lists
            cwi[:, robots] = np.array([c_x, c_y])

    # Calculate and fill the weights array to calculate the MST
    for robot in range(N):
        for neighbor in range(N):
            h = (Rc ** 2 - robot_distances[robot][neighbor] ** 2)
            h_dot = -2 * (np.array(x_si).T[robot] - np.array(x_si).T[neighbor]) @ (u_desired[:, robot] - u_desired[:, neighbor])
            weights[robot][neighbor] = -1 * (h_dot + gamma * h)
    
    # Calculate the MST from the weights array and get the edges
    mst = minimum_spanning_tree(weights).toarray()
    edges = np.array(np.where(mst > 0)).T.tolist()

    # Create a 2x5 Variable for the solver
    u = cp.Variable((2, 5))

    # Create an objective function to minimize the difference between the velocity and optimal velocity
    objective = cp.Minimize(cp.sum_squares(u - u_desired))

    # Create an initial constraint that ensures the velocity is in same direction as the optimal velocity
    constraints = [cp.sum(cp.multiply(u, u_desired)) >= 0]
    
    # Calculate additional constraints for the communication range
    for edge in edges:
        h = (Rc ** 2 - robot_distances[edge[0]][edge[1]] ** 2)
        diff = np.array(x_si).T[edge[0]] - np.array(x_si).T[edge[1]]
        constraint = 2 * diff @ u[:, edge[0]] <= 2 * diff @ u[:, edge[1]] + gamma * h
        constraints.append(constraint)
    
    # Define the problem with the objective and constraints and solve for the robots velocity
    prob = cp.Problem(objective, constraints)
    prob.solve()

    # Assign the robot velocities
    si_velocities = u.value if u.value is not None else u_desired

    # Make a copy of distance traveled array to calculate total distances
    total_distances = distance_traveled.copy()

    # If there is a value in the distance array, add the latest calculated distance to the previous iteration
    if dist_robot:
        total_distances += dist_robot[-1]

    # Update the variables for the previous pose with the current pose to be used in the next calculation
    previous_x = current_x
    previous_y = current_y

    # Add the current iteration values to the global lists
    Cwi.append(cwi.tolist())
    ui.append(si_velocities.tolist())
    dist_robot.append(total_distances.tolist())
    total_Hp.append(Hp)

    # Check for convergence
    if np.all(distance_traveled < convergence_threshold) and k > 3:
        converged_iteration = k + 1
        print(f"Converged at iteration {converged_iteration}")
        break

    # Use the barrier certificate to avoid collisions
    si_velocities = si_barrier_cert(si_velocities, x_si)

    # Transform single integrator to unicycle
    dxu = si_to_uni_dyn(si_velocities, x)

    # Set the velocities of agents 1,...,N
    r.set_velocities(np.arange(N), dxu)
    # Iterate the simulation
    r.step()

    # Clear the previous plot
    ax.clear()

    # Plot the distribution function
    ax.pcolor(X, Y, density, shading="auto", zorder=-1)

    # Create a Voronoi diagram based on the robot positions
    points = np.array([current_x.flatten(), current_y.flatten()]).T
    vor = Voronoi(points)

    # Plot the Voronoi diagram
    voronoi_plot_2d(vor, ax=ax, show_vertices=False, line_colors="r")

    # Plot robots' positions
    ax.scatter(current_x.flatten(), current_y.flatten(), c="b", marker="o")

    # Plot the ideal centroids
    ax.scatter(cwi[0].flatten(), cwi[1].flatten().flatten(), c="r", marker="x")

    # Plot the communication ranges
    for robot in range(N):
        circle = Circle((current_x[robot], current_y[robot]), radius=Rc, fill=False, color='green')
        ax.add_patch(circle)

    # Set the aspect ratio so the circles are not stretched
    ax.set_aspect('equal')

    # Set plot limits and labels
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title(f"Iteration {k+1} - Voronoi Partitioning")

    # Draw the plot
    plt.draw()

# Outputs the data to a .csv file
with open("output.csv", mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow([f"Number of Iterations: {converged_iteration}"])
    writer.writerow(["X Poses", "Y Poses"])
    for index, value in enumerate(poses, start=1):
        writer.writerow([f"Iteration {index}", value])
    writer.writerow([])
    writer.writerow(["Weights"])
    for index, value in enumerate(wij, start=1):
        writer.writerow([f"Iteration {index}", value])
    writer.writerow([])
    writer.writerow(["Centroids"])
    for index, value in enumerate(Cwi, start=1):
        writer.writerow([f"Iteration {index}", value])
    writer.writerow([])
    writer.writerow(["Control Inputs"])
    for index, value in enumerate(ui, start=1):
        writer.writerow([f"Iteration {index}", value])
    writer.writerow([])
    writer.writerow(["Distance Traveled"])
    for index, value in enumerate(dist_robot, start=1):
        writer.writerow([f"Iteration {index}", value])
    writer.writerow([])
    writer.writerow(["Locational Cost for all Sensor Types"])
    for index, value in enumerate(total_Hg, start=1):
        writer.writerow([f"Iteration {index}: {value}"])
    writer.writerow([])
    writer.writerow(["Power Cost"])
    for index, value in enumerate(total_Hp, start=1):
        writer.writerow([f"Iteration {index}: {value}"])
    writer.writerow([])
    writer.writerow(["Temporal Cost"])
    for index, value in enumerate(total_Hv, start=1):
        writer.writerow([f"Iteration {index}: {value}"])
    writer.writerow([])
    writer.writerow(["Range-Limited Cost"])
    for index, value in enumerate(total_Hr, start=1):
        writer.writerow([f"Iteration {index}: {value}"])
    writer.writerow([])
    writer.writerow(["Custom Heterogeneous Cost"])
    for index, value in enumerate(total_Hc, start=1):
        writer.writerow([f"Iteration {index}: {value}"])
    writer.writerow([])
    writer.writerow(["Standard Voronoi Baseline"])
    writer.writerow([])

# Call at end of script to print debug information and for your script to run on the Robotarium server properly
r.call_at_scripts_end()
