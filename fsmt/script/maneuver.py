import math
import numpy as np
import matplotlib.pyplot as plt

length = 1
width = 0.4
pb = np.array([0.2, 0.2])

fig, ax = plt.subplots()

def create_vehicle_geometry(length: float, width: float, pb: np.ndarray) -> np.ndarray:
    """
    Compute the four points that define the geometric are of a vehicle.

    :param width: width of the vehicle.
    :type length: float
    :param length: length of the vehicle.
    :type width: float
    :return: Thhe geometry of the corresponding vehicle
    :rtype: np.ndarray

    :raises ValueError: If `a` or `b` are not integers.

    :example:
        >>> create_vehicle_geometry(1, 0.4, np.array([0.2, 0.2]))
        [[-0.2 -0.2]
          [ 0.8 -0.2]
          [ 0.8  0.2]
          [-0.2  0.2]]
    """
    geometry = np.array(
        [
            [0, 0],
            [length, 0],
            [length, width],
            [0, width]
        ]
    ) - pb

    return geometry


def plot_vehicle_geometry(ax, geometry):
    box = np.vstack((geometry, geometry[0,:]))
    ax.plot(box[:,0], box [:,1])
    plt.show()

geom = create_vehicle_geometry(length, width, pb)
plot_vehicle_geometry(ax, geom)

def radius(r, pi):
    return math.sqrt(pi[0]*pi[0] + (-r+pi[1])*(-r+pi[1]))

def angular_interval(spatial_interval, radius):
    return spatial_interval/radius