import os
import csv
import matplotlib.pyplot as plt


def plot_cost_comparison():
    repo_root = os.path.abspath(os.path.join(__file__, "..", "..", ".."))
    csv_file_1 = os.path.join(repo_root, "data", "last", "voronoi_output.csv")
    csv_file_2 = os.path.join(repo_root, "data", "last", "connected_output.csv")
    csv_file_3 = os.path.join(repo_root, "data", "last", "dynamic_connected_output.csv")
    csv_files = [csv_file_1, csv_file_2, csv_file_3]

    total_locational_costs = {}

    for csv_file in csv_files:
        iterations = []
        locational_costs = []

        with open(csv_file, "r") as file:
            reader = csv.reader(file)

            counter = 0

            for row in reader:
                # Extract the number of iterations from the first row to use for next condition
                if counter == 0:
                    total_iterations = int(row[0].split(":")[1].strip())

                # Extract the locational cost data from the corresponding rows
                # elif (4 * total_iterations + 198) < counter < (5 * total_iterations + 199):
                elif (5 * total_iterations + 200) < counter < (6 * total_iterations + 201):
                    iteration = int(row[0].split()[1].replace(":", ""))
                    locational_cost = float(row[0].split(":")[1].strip())

                    iterations.append(iteration)
                    locational_costs.append(locational_cost)

                # Extract the label
                elif counter == (6 * total_iterations + 202):
                    label = row[0]
                    if label not in total_locational_costs:
                        total_locational_costs[label] = {"Costs": [], "Iterations": []}
                    total_locational_costs[label]["Costs"] = locational_costs
                    total_locational_costs[label]["Iterations"] = iterations

                counter += 1

    # Plot robot positions over iterations
    plt.figure(figsize=(8, 6))

    for label, data in total_locational_costs.items():
        plt.plot(data["Iterations"], data["Costs"], linestyle="-", label=f"{label}")
        plt.scatter(data["Iterations"][-1], data["Costs"][-1], marker="o", s=10)

    plt.xlabel("Iteration")
    plt.ylabel("Cost")
    plt.title("Cost Comparison")
    # plt.suptitle("Scenario 1")
    plt.legend()
    plt.grid()
    plt.show()


def main():
    plot_cost_comparison()


if __name__ == "__main__":
    main()
