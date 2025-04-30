#!/usr/bin/env python3

import numpy as np
from random import uniform


def generate_gaussian_distribution(randomize=False):
    # Generate values for the Gaussian if random argument is True
    if randomize:
        sigma = uniform(0.4, 1)
        x_location = uniform(-1.5, 1.5)
        y_location = uniform(-1.0, 1.0)
        mu = [x_location, y_location]
    else:
        sigma = 1
        mu = np.array([0.85, 0.85])

    # Use the sigma value to calculate the normalizing constant for the Gaussian
    sigma_array = sigma * np.eye(2)
    sigma_inv = np.linalg.inv(sigma_array)
    sigma_det = np.linalg.det(sigma_array)
    normalizer = 4 / (1 * np.pi * np.sqrt(sigma_det))

    # Define Gaussian distribution function
    def distribution_function(x, y):
        # Convert passed coordinates to arrays and join the arrays
        x = np.asarray(x)
        y = np.asarray(y)
        coordinates = np.stack([x, y], axis=-1)

        # Calculate the density value of the point
        diff = coordinates - mu
        # exponent = -0.5 * np.dot(diff, np.dot(sigma_inv, diff))
        exponent = -0.5 * np.einsum("...i,ij,...j->...", diff, sigma_inv, diff)
        density = normalizer * np.exp(exponent)
        return density

    # Calculate the maximum and minimum density
    max_density = distribution_function(mu[0], mu[1])
    min_density = 0

    # Return the generated function so that other scripts can pass coordinates
    return distribution_function, max_density, min_density


def generate_random_distribution():
    # Define random variables for generated function
    freq_x = np.random.uniform(0.5, 2.0)
    freq_y = np.random.uniform(0.5, 2.0)
    phase_x = np.random.uniform(0, 2 * np.pi)
    phase_y = np.random.uniform(0, 2 * np.pi)
    amplitude = np.random.uniform(0.5, 1.0)

    # Define the maximum and minimum values of the function in order to normalize in main script
    max_density = amplitude
    min_density = -1 * amplitude

    # Define a function using the generated random variables
    def distribution_function(x, y):
        value = amplitude * np.sin(freq_x * x + phase_x) * np.sin(freq_y * y + phase_y)
        return value

    return distribution_function, max_density, min_density
