""" calc_energy calculates the kinetic energy for a given body traveling in 1D

Arguments:
    - m (float): Mass of body
    - v (float): Velocity of body

Returns:
    - float: Kinetic energy of body
"""
def calc_energy(m, v):
    # Ensure kinetic energy is negative if velocity is negative
    # (Separate logic necessary because velocity looses its sign in the eq due 
    # to being raised to the power of 2)
    dir = 1

    if v < 0:
        dir = -1

    # Kinetic energy eq: (1/2) * m * v^2
    return dir * (m * (v ** 2)) / 2

""" calc_acceleration calculates the acceleration needed to accelerate from an 
initial to final velocity in a specified length of time.

Arguments:
    - vi (float): Initial velocity
    - vf (float): Final velocity
    - dt (float): Time

Returns:
    - float: Acceleration
"""
def calc_acceleration(vi, vf, dt):
    # Derived from formula: vf = vi + (a * t)
    return (vf - vi) / dt

""" calc_delta_distance calculates the 1D distance travelled at a specific 
velocity in a certain time.

Arguments:
    - vi (float): Initial velocity
    - vf (float): Final velocity
    - dt (float): Time duration

Returns:
    - float: Distance covered
"""
def calc_delta_distance(vi, vf, dt):
    # Formula: d = (vi * t) + ((1/2) * a * t^2)
    a = calc_acceleration(vi, vf, dt)
    return (vi * dt) + (0.5 * a * (dt ** 2))

""" calc_force calculates the force needed to add the specified kinetic energy 
to a body in a certain distance.

Arguments:
    - delta_energy (float): Energy to add to the body
    - delta_distance (float): Distance to add energy to body in

Returns:
    - float: Force
"""
def calc_force(delta_energy, delta_distance):
    return delta_energy / delta_distance
