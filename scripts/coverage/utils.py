#!/usr/bin/env python3

import numpy as np
from random import uniform
from scipy.ndimage import gaussian_filter
from scipy.interpolate import RegularGridInterpolator

def generate_gaussian_distribution(randomize=False):

    # Generate values for the Gaussian if random argument is True
    if randomize:
        sigma = uniform(0.4, 1)
        x_location = uniform(x_min, x_max)
        y_location = uniform(y_min, y_max)
        mu = [x_location, y_location]
    else:
        sigma = 1
        mu = np.array([0.85, 0.85])

    # Use the sigma value to calculate the normalizing constant for the Gaussian
    sigma_array = sigma * np.eye(2)
    sigma_inv = np.linalg.inv(sigma_array)
    sigma_det = np.linalg.det(sigma_array)
    normalizer = 4 / (1 * np.pi * np.sqrt(sigma_det))

    # Function for the Gaussian distribution function
    def distribution_function(x, y):
        
        # Convert passed coordinates to arrays and join the arrays
        x = np.asarray(x)
        y = np.asarray(y)
        coordinates = np.stack([x, y], axis=-1)

        # Calculate the density value of the point 
        diff = coordinates - mu
        # exponent = -0.5 * np.dot(diff, np.dot(sigma_inv, diff))
        exponent = -0.5 * np.einsum('...i,ij,...j->...', diff, sigma_inv, diff)
        density = normalizer * np.exp(exponent)
        return density
    
    # Calcualte the max density
    max_density = distribution_function(mu[0], mu[1])
    
    # Return the generated function so that other scripts can pass coordinates
    return distribution_function, max_density

def generate_random_distribution():
    # Define environment variables
    x_min = -1.5
    x_max = 1.5
    y_min = -1
    y_max = 1
    resolution = 0.05

    # Create 1D arrays of the discretized environment
    x_vals = np.arange(x_min, x_max + resolution, resolution)
    y_vals = np.arange(y_min, y_max + resolution, resolution)

    # Define 2D arrays for holding the coordinates
    X = np.zeros((len(x_vals), len(y_vals)))
    Y = np.zeros((len(x_vals), len(y_vals)))

    # Fill in X and Y arrays
    for i, x in enumerate(x_vals):
        for j, y in enumerate(y_vals):
            X[i, j] = x
            Y[i, j] = y

    # Generate random noise
    random_noise = np.random.rand(len(x_vals), len(y_vals))

    # Smooth the random noise with a Gaussian filter
    smooth_random = gaussian_filter(random_noise, sigma=1)

    # Interpolate the discretized points
    interpolator = RegularGridInterpolator((x_vals, y_vals), smooth_random)

    # Find the minimum and max values
    max_density = np.max(random_noise)
    min_density = np.min(random_noise)

    return interpolator, max_density, min_density