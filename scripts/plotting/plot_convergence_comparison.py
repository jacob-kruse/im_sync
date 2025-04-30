import os
import csv
import matplotlib.pyplot as plt


def plot_convergence_comparison():
    repo_root = os.path.abspath(os.path.join(__file__, "..", "..", ".."))
    csv_file_1 = os.path.join(repo_root, "data", "last", "voronoi_output.csv")
    csv_file_2 = os.path.join(repo_root, "data", "last", "connected_output.csv")
    csv_file_3 = os.path.join(repo_root, "data", "last", "dynamic_connected_output.csv")
    csv_files = [csv_file_1, csv_file_2, csv_file_3]

    convergences = {}

    for csv_file in csv_files:
        with open(csv_file, "r") as file:
            reader = csv.reader(file)

            counter = 0

            for row in reader:
                # Extract the number of iterations from the first row to use for next condition
                if counter == 0:
                    total_iterations = int(row[0].split(":")[1].strip())

                # Extract the label
                elif counter == (6 * total_iterations + 202):
                    label = row[0]
                    if label not in convergences:
                        convergences[label] = {"Iteration": int}
                    convergences[label]["Iteration"] = total_iterations

                counter += 1

    # Plot robot positions over iterations
    plt.figure(figsize=(8, 6))

    for label, data in convergences.items():
        bar = plt.bar(label, data["Iteration"])
        yval = int(bar[0].get_height())
        plt.text(bar[0].get_x() + bar[0].get_width() / 2, yval, f"{yval}", ha="center", va="bottom", fontsize=10)

    # plt.xlabel("Iteration")
    plt.ylabel("Iteration of Convergence")
    plt.title("Convergence Comparison")
    # plt.suptitle("Scenario 1")
    plt.xticks(fontsize=8)
    plt.show()


def main():
    plot_convergence_comparison()


if __name__ == "__main__":
    main()
