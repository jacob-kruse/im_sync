import os
import csv
import ast
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from scipy.spatial import Voronoi, voronoi_plot_2d


def plot_poses(csv_file):
    # Define empty variables
    robot_centroids = {}
    robot_positions = {}
    comm_ranges = []
    density = []
    X = []
    Y = []
    conn_edges = []
    label = None

    # Opent the .csv file
    with open(csv_file, "r") as file:
        # Define a reader for the .csv files
        reader = csv.reader(file)

        # Initialize the counter
        counter = 0

        # For each row in the file ...
        for row in reader:
            # Extract the number of iterations from the first row to use for next condition
            if counter == 0:
                total_iterations = int(row[0].split(":")[1].strip())

            # Extract the pose data from the corresponding rows
            elif 1 < counter < (total_iterations + 2):
                # Convert string to list and extract the positions
                positions = ast.literal_eval(row[1])
                x_positions, y_positions = positions

                # Store positions for each robot
                for i in range(len(x_positions)):
                    if i not in robot_positions:
                        robot_positions[i] = {"x": [], "y": []}
                    robot_positions[i]["x"].append(x_positions[i])
                    robot_positions[i]["y"].append(y_positions[i])

            # Extract the centroid data from the corresponding rows
            elif (total_iterations + 3) < counter < (2 * total_iterations + 4):
                # Convert string to list and extract the positions
                positions = ast.literal_eval(row[1])
                x_positions, y_positions = positions

                # Store positions for each robot
                for i in range(len(x_positions)):
                    if i not in robot_centroids:
                        robot_centroids[i] = {"x": [], "y": []}
                    robot_centroids[i]["x"].append(x_positions[i])
                    robot_centroids[i]["y"].append(y_positions[i])

            # Extract the communication range data from the corresponding rows
            elif (2 * total_iterations + 5) < counter < (3 * total_iterations + 6):
                # Convert string to list and extract the ranges
                ranges = ast.literal_eval(row[1])

                comm_ranges.append(ranges)

            # Extract the density data from the corresponding rows
            elif (3 * total_iterations + 7) < counter < (3 * total_iterations + 69):
                # Add each row to the entire density arrays
                density.append(row)

            # Extract the density data from the corresponding rows
            elif (3 * total_iterations + 70) < counter < (3 * total_iterations + 132):
                # Add each row to the entire density arrays
                X.append(row)

            # Extract the density data from the corresponding rows
            elif (3 * total_iterations + 133) < counter < (3 * total_iterations + 195):
                # Add each row to the entire density arrays
                Y.append(row)

            # Extract the edge connectivity data from the corresponding rows
            elif (3 * total_iterations + 196) < counter < (4 * total_iterations + 197):
                # Convert string to list and extract the ranges
                edges = ast.literal_eval(row[1])

                conn_edges.append(edges)

            # Extract the label
            elif counter == (6 * total_iterations + 202):
                label = row[0]

            # Extract the ideal communication range
            elif counter == (6 * total_iterations + 204) and label == "Connectivity Maintenance Baseline":
                ideal_comm_range = float(row[0].split(":")[1].strip())

            counter += 1

    # Plot robot positions over iterations
    fig, ax = plt.subplots(figsize=(8, 6))

    final_x = np.zeros(len(list(robot_positions)))
    final_y = np.zeros(len(list(robot_positions)))

    for robot_id, data in robot_positions.items():
        # line, = plt.plot(data["x"], data["y"], linestyle="-", label=f"Robot {robot_id+1}")
        point = plt.scatter(data["x"][-1], data["y"][-1], marker="o", s=40)

        color = point.get_facecolors()[0]

        index = list(robot_positions).index(robot_id)
        final_x[index] = data["x"][-1]
        final_y[index] = data["y"][-1]
        actual_circle = Circle(
            (final_x[index], final_y[index]), radius=comm_ranges[-1][index], fill=False, color=color, linewidth=1.5
        )
        ax.add_patch(actual_circle)
        if label == "Connectivity Maintenance Baseline":
            ideal_circle = Circle(
                (final_x[index], final_y[index]),
                radius=ideal_comm_range,
                fill=False,
                color="white",
                linewidth=1.5,
                linestyle="--",
            )
            ax.add_patch(ideal_circle)

    for robot_id, data in robot_centroids.items():
        plt.scatter(data["x"][-1], data["y"][-1], marker="x", s=40, color="black")

    final_points = np.array([final_x.flatten(), final_y.flatten()]).T
    vor = Voronoi(final_points)
    voronoi_plot_2d(vor, ax=ax, show_points=False, show_vertices=False, line_colors="black", line_width=1.5)

    ax.pcolor(np.array(X, dtype=float), np.array(Y, dtype=float), np.array(density, dtype=float), shading="auto", zorder=-1)
    ax.set_aspect("equal")

    if label == "Dynamic Communication Extension" or label == "Connectivity Maintenance Baseline":
        for edge in edges:
            x_values = [final_x[edge[0]], final_x[edge[1]]]
            y_values = [final_y[edge[0]], final_y[edge[1]]]
            ax.plot(x_values, y_values, color="white", linewidth=1.5)

    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.0, 1.0)
    ax.set_title("Final Configuration")
    fig.suptitle(f"{label}")
    ax.grid()
    plt.show()


def main():
    repo_root = os.path.abspath(os.path.join(__file__, "..", "..", ".."))
    # csv_file = os.path.join(repo_root, "data", "last", "voronoi_output.csv")
    csv_file = os.path.join(repo_root, "data", "last", "connected_output.csv")
    # csv_file = os.path.join(repo_root, "data", "last", "dynamic_connected_output.csv")
    plot_poses(csv_file)


if __name__ == "__main__":
    main()
