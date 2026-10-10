import numpy as np

def vortex_field(location):
    x, y, z = location
    if z > 0:
        return [0, 0, 0]
    strength = 5.0
    r_squared = x**2 + y**2 + 1e-5  # avoid divide by zero
    dx = -y / r_squared * strength
    dy = x / r_squared * strength
    dz = 0.2 * np.cos(0.1 * r_squared)
    return [3*dx, 3*dy, 3*dz]
