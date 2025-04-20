import os
from plot_poses import plot_poses
from plot_cost_comparison import plot_cost_comparison
from plot_convergence_comparison import plot_convergence_comparison
from plot_connectivty_comparison import plot_connectivity_comparison

repo_root = os.path.abspath(os.path.join(__file__, "..", "..", ".."))
csv_file_1 = os.path.join(repo_root, "data", "last", "voronoi_output.csv")
csv_file_2 = os.path.join(repo_root, "data", "last", "connected_output.csv")
csv_file_3 = os.path.join(repo_root, "data", "last", "dynamic_connected_output.csv")
csv_files = [csv_file_1, csv_file_2, csv_file_3]
for csv_file in csv_files:
    plot_poses(csv_file)
plot_cost_comparison()
plot_convergence_comparison()
plot_connectivity_comparison()