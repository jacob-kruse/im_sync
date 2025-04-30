#!/usr/bin/env python3

import os
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

from scipy.linalg import eigh
from scipy.spatial.distance import cdist
from scipy.spatial import Voronoi, voronoi_plot_2d

import rps.robotarium as robotarium
from rps.utilities.graph import *
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *

from utils import generate_gaussian_distribution, generate_random_distribution


def voronoi(density, max_density, min_density):
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
    convergence_threshold = 2e-3                                       # Threshold to determine if convergence has occurred
    range_diff = 0.3                                                   # Determines how much the range can deviate
    initial_conditions = np.asarray([                                  # Sets the initial positions of the robots
        [1.25, 0.25, 0],
        [1, 0.3, 0],
        [1, -0.4, 0],
        [0.2, -0.35, 0],
        [0.4, 0.2, 0],
        [0.2, -0.6, 0],
        [-0.75, -0.1, 0],
        [-1, 0, 0],
        [-0.8, -0.25, 0],
        [1.3, -0.4, 0]
    ])

    # Status and Performance Variables
    Cwi = []
    poses = []
    total_H = []
    total_edges = []
    total_ranges = []
    total_connectivity = []
    previous_x = None
    previous_y = None
    converged_iteration = -1

    # Instantiate Robotarium object
    r = robotarium.Robotarium(number_of_robots=N, sim_in_real_time=False, initial_conditions=initial_conditions[0:N].T, show_figure=False)
    # fmt: on

    # Close the initial figure that is opened by Robotarium
    plt.close(1)

    # Helper Functions for Simulation
    si_barrier_cert = create_single_integrator_barrier_certificate_with_boundary()
    si_to_uni_dyn, uni_to_si_states = create_si_to_uni_mapping()

    # Initialize figure
    fig, ax = plt.subplots()

    # Iterate for the amount defined
    for k in range(iterations):
        # Get the poses of the robots and convert to single-integrator poses
        x = r.get_poses()
        x_si = uni_to_si_states(x)
        poses.append(x_si.tolist())
        current_x = x_si[0, :, None]
        current_y = x_si[1, :, None]

        # Instantiate calculation variables to zero
        H = 0
        comm_ranges = []
        w_v = np.zeros(N)
        A = np.zeros((N, N))
        D = np.zeros((N, N))
        c_v = np.zeros((N, 2))
        cwi = np.zeros((2, N))
        distance_traveled = np.zeros(N)
        si_velocities = np.zeros((2, N))

        # Nested loop that occurs for each coordinate, this is the calculation portion of the code for the Voronoi cells
        for ix in np.arange(x_min, x_max, res):
            for iy in np.arange(y_min, y_max, res):
                """Need to make this different over environment"""
                # Set the importance value to 1, this represents the distribution function
                importance_value = 1

                # Instantiate the distance array to zero for all values
                distances = np.zeros(N)

                # Calculate the distance of each robot to the current point
                for robots in range(N):
                    distances[robots] = np.sqrt(
                        np.square(ix - current_x[robots].item()) + np.square(iy - current_y[robots].item())
                    )

                # Find the robot with the smallest distance
                min_index = np.argmin(distances)

                # Calculations for the Standard Voronoi Partitioning
                c_v[min_index][0] += ix * importance_value
                c_v[min_index][1] += iy * importance_value
                w_v[min_index] += importance_value

                # Calculate cost
                H += (distances[min_index] ** 2) * importance_value

        # Iterate for the number of robots, this is the velocity and weight controller portion of the code
        for robots in range(N):
            # Calculate the distance travled from the previous iteration
            # fmt: off
            if previous_x is not None and previous_y is not None:
                distance_traveled[robots] = float(np.sqrt((abs(current_x[robots][0] - previous_x[robots][0]) ** 2) +
                                                          (abs(current_y[robots][0] - previous_y[robots][0]) ** 2)))
            # fmt: on

            # Get the density value of each robot and modify the communication ranges accordingly
            pose_density = density(current_x[robots][0], current_y[robots][0])
            range_constant = (max_density - pose_density) / (max_density - min_density)
            comm_range = float(Rc * ((1 - range_diff) + (2 * range_diff * range_constant)))
            comm_ranges.append(comm_range)

            # Calculate the x and y coordinates of the centroids of the Weighted Voronoi Partitions
            if not w_v[robots] == 0:
                c_x = c_v[robots][0] / w_v[robots]
                c_y = c_v[robots][1] / w_v[robots]

                # Calculate the velocity of each robot
                si_velocities[:, robots] = kv * np.array([(c_x - current_x[robots][0]), (c_y - current_y[robots][0])])

                # Append the current centroid and velocity to the lists
                cwi[:, robots] = np.array([c_x, c_y])

        # Compute distances between robots
        robot_distances = cdist(np.array(x_si).T, np.array(x_si).T, "euclidean")

        # Calculate the adjacency and diagonal matrix
        row_index = 0
        for row in robot_distances:
            for dist in row:
                dist_index = np.where(row == dist)[0][0]
                if dist_index != row_index and dist <= min(comm_ranges[dist_index], comm_ranges[row_index]):
                    A[row_index][dist_index] = 1
                    D[row_index][row_index] += 1
                else:
                    A[row_index][dist_index] = 0
            row_index += 1

        # Calculate the Laplacian and eigenvalues to find the algebraic connectivity
        L = D - A
        eigenvalues = eigh(L, eigvals_only=True)
        connectivity = sorted(eigenvalues)[1]

        # Update the variables for the previous pose with the current pose to be used in the next calculation
        previous_x = current_x
        previous_y = current_y

        # Create blank edges array for data
        edges = []

        # Add the current iteration values to the global lists
        Cwi.append(cwi.tolist())
        total_H.append(H)
        total_ranges.append(comm_ranges)
        total_edges.append(edges)
        total_connectivity.append(connectivity)

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
        x_vals = np.arange(x_min, x_max + res, res)
        y_vals = np.arange(y_min, y_max + res, res)
        X, Y = np.meshgrid(x_vals, y_vals, indexing="ij")
        Z = density(X, Y)
        ax.pcolor(X, Y, Z, shading="auto", zorder=-1)

        # Create a Voronoi diagram based on the robot positions
        points = np.array([current_x.flatten(), current_y.flatten()]).T
        vor = Voronoi(points)

        # Plot the Voronoi diagram
        voronoi_plot_2d(vor, ax=ax, show_vertices=False, show_points=False, line_colors="black")

        # Plot robots' positions
        colors = ["red", "green", "blue", "purple", "yellow"]
        ax.scatter(current_x.flatten(), current_y.flatten(), c=colors, marker="o")

        # Plot the ideal centroids
        ax.scatter(cwi[0].flatten(), cwi[1].flatten().flatten(), c="black", marker="x")

        # Plot the communication ranges
        for robot in range(N):
            actual_circle = Circle(
                (current_x[robot], current_y[robot]), radius=comm_ranges[robot], fill=False, color=colors[robot], linestyle="--"
            )
            ax.add_patch(actual_circle)

        # Set the aspect ratio so the circles are not stretched
        ax.set_aspect("equal")

        # Set plot limits and labels
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_title(f"Iteration {k+1} - Standard Voronoi Baseline")

        # Draw the plot
        plt.draw()
        if k == 0:
            plt.pause(0.5)
        else:
            plt.pause(0.001)

    # Outputs the data to a .csv file
    repo_root = os.path.abspath(os.path.join(__file__, "..", "..", ".."))
    output_path = os.path.join(repo_root, "data", "last", "voronoi_output.csv")
    with open(output_path, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([f"Number of Iterations: {converged_iteration}"])
        writer.writerow(["X Poses", "Y Poses"])
        for index, value in enumerate(poses, start=1):
            writer.writerow([f"Iteration {index}", value])
        writer.writerow([])
        writer.writerow(["Centroids"])
        for index, value in enumerate(Cwi, start=1):
            writer.writerow([f"Iteration {index}", value])
        writer.writerow([])
        writer.writerow(["Communication Ranges"])
        for index, value in enumerate(total_ranges, start=1):
            writer.writerow([f"Iteration {index}", value])
        writer.writerow([])
        writer.writerow(["Density Array"])
        for row in Z:
            writer.writerow([f"{val:.5f}" for val in row])
        writer.writerow([])
        writer.writerow(["X Array"])
        for row in X:
            writer.writerow([f"{val:.5f}" for val in row])
        writer.writerow([])
        writer.writerow(["Y Array"])
        for row in Y:
            writer.writerow([f"{val:.5f}" for val in row])
        writer.writerow([])
        writer.writerow(["Edges of Connectivity Graph"])
        for index, value in enumerate(total_edges, start=1):
            writer.writerow([f"Iteration {index}", value])
        writer.writerow([])
        writer.writerow(["Algebraic Connectivity"])
        for index, value in enumerate(total_connectivity, start=1):
            writer.writerow([f"Iteration {index}: {value}"])
        writer.writerow([])
        writer.writerow(["Cost"])
        for index, value in enumerate(total_H, start=1):
            writer.writerow([f"Iteration {index}: {value}"])
        writer.writerow([])
        writer.writerow(["Standard Voronoi Baseline"])
        writer.writerow([])

    # Call at end of script to print debug information and for your script to run on the Robotarium server properly
    r.call_at_scripts_end()

    plt.close(1)


def main():
    # Generate a distribution function for the environment that represents communication ability
    # density, max_density, min_density = generate_gaussian_distribution(randomize=False)
    density, max_density, min_density = generate_random_distribution()

    # Call the coverage script
    voronoi(density, max_density, min_density)


if __name__ == "__main__":
    main()
