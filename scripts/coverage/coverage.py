from voronoi import voronoi
from connected_voronoi import connected_voronoi
from dynamic_connected_voronoi import dynamic_connected_voronoi
from utils import generate_random_distribution, generate_gaussian_distribution

density, max_density, min_density = generate_random_distribution()

voronoi(density, max_density, min_density)
connected_voronoi(density, max_density, min_density)
dynamic_connected_voronoi(density, max_density, min_density)