#!/usr/bin/env python3

import numpy as np
from random import uniform
import matplotlib.pyplot as plt

def generate_gaussian_distribution(randomize=False):

    # Define environment variables
    x_min = -1.5             # Upper bound of x coordinate
    x_max = 1.5              # Lower bound of x coordinate
    y_min = -1               # Upper bound of y coordinate
    y_max = 1                # Lower bound of x coordinate
    resolution = 0.05        # Resolution of coordinates

    # Create 1D arrays of the discretized environment
    x_vals = np.arange(x_min, x_max + resolution, resolution) 
    y_vals = np.arange(y_min, y_max + resolution, resolution)

    # Define 2D arrays for holding the value of each point and the density
    X = np.zeros((len(x_vals), len(y_vals)))
    Y = np.zeros((len(x_vals), len(y_vals)))
    density_array = np.zeros((len(x_vals), len(y_vals)))

    # Fill in X and Y arrays with each discretized point
    for i, x in enumerate(x_vals):
        for j, y in enumerate(y_vals):
            X[i, j] = x
            Y[i, j] = y

    # Generate values for the Gaussian if random argument is True
    if randomize:
        sigma = uniform(0.4, 1)
        x_location = uniform(x_min, x_max)
        y_location = uniform(y_min, y_max)
        mu = [x_location, y_location]
    else:
        sigma = 1
        mu = [0.85, 0.85]

    # Use the sigma value to calculate the normalizing constant for the Gaussian
    sigma_array = sigma * np.eye(2)
    sigma_inv = np.linalg.inv(sigma_array)
    normalizer = 4 / (1 * np.pi * np.sqrt(np.linalg.det(sigma_array)))

    # Create an array to hold each discretized point and find the difference from the mean
    coordinates = np.column_stack((X.ravel(), Y.ravel()))
    diff = coordinates - mu

    # Calculate the 1D distribution function
    for i in range(coordinates.shape[0]):
        exponent = -0.5 * np.dot(diff[i], np.dot(sigma_inv, diff[i]))
        density_array.ravel()[i] = normalizer * (np.exp(exponent))

    # Reshape the density function to 2D
    density = density_array.reshape(X.shape)

    return X, Y, density

def generate_random_distribution():
    pass