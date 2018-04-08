import time
import threading
import signal
import sys
import math
from . import velocity as vLib

""" Vessel offers several convenient method for controlling a spacecraft's 
position.

To control the spacecraft's position one must declare a desired state. The 
vessel will then run an update loop. Which attempts to reach and maintain the 
desired state.  

Many properties of the vessel can be controlled. However only one can be 
controlled at a time. As they all modify the velocity of the vessel.

A property can be set for control by running the Vessel.target_<property name> 
method. Additionally, the Vessel.clear_<property name> can be used to stop 
controlling a property.

Fields:
    - _controlling_event (threading.Event): Used to stop the control loop once 
        started.
    - vessel (krpc.Vessel): KRPC Vessel to control an in game space craft
    - conn (krpc.client.Client): Used to communicate with the KRPC API
    - control_rate (float): Number of seconds between attempts to control the 
        space craft to reach the desired state.
    - desired_velocity (float): Velocity which vessel will attempt to maintain. 
    - desired_acceleration (float): (Not implemented yet) Acceleration which 
        vessel will attempt to maintain.
    - desired_height (float): (Not implemented yet) Heigh which vessel will 
        attempt to maintain.
"""
class Vessel:
    """ PropertyVelocity is the value used to indicate the velocity property of 
    the vesself
    """
    PropertyVelocity = "velocity"

    def __init__(self, conn, vessel, control_rate):
        self._controlling_event = threading.Event()

        self.conn = conn
        self.vessel = vessel
        self.control_rate = control_rate

        self.velocity_drift = 0
        self.desired_velocity = None
        self.desired_acceleration = None
        self.desired_height = None

    """ start_control_loop starts the control loop which attempts to reach the 
    desired state.
    """
    def start_control_loop(self):
        # Retrieve data stream
        with self.conn.stream(self.vessel.velocity, self.vessel.orbit.body.reference_frame) \
                as velocity_fn:
            # Loop while control loop is running
            while self.can_control():
                # Run one iteration of the update loop
                self.update(velocity_fn)

                # Wait specified amount of time
                time.sleep(self.control_rate)

    """ stop_control_loop signals the control loop logic to stop
    """
    def stop_control_loop(self):
        self._controlling_event.set()

    """ can_control determines if the vessel control loop could be running. 
    It does not however determine if the control loop logic is actually running. 

    This is because the can_control method only checks to see if the control 
    loop stop flag is **not** set. If this stop flag is **not** set, the 
    control loop logic is allowed to run.

    However if the stop flag is set, the control loop logic will exit immediately 
    with an error.

    Returns:
        - bool: True if the control loop is allowed to run, otherwise False.
    """
    def can_control(self):
        return not self._controlling_event.is_set()

    """ update runs one iteration of the control loop. Which will attempt to 
    reach the declared state.

    Arguments:
        - velocity_fn (krpc.client.Client): Stream used to retrieve velocity data

    """
    def update(self, velocity_fn):
        # Enable SAS
        self.vessel.control.sas = True

        # Check which property is to be controlled
        controlled = self.get_controlled()

        # If none controlled, end now
        if controlled is None:
            return

        # Otherwise run property specific update logic
        if controlled == Vessel.PropertyVelocity:
            self.update_velocity(velocity_fn)

    """ update_velocity runs one iteration of the velocity specific control 
    loop.

    Arguments:
        - velocity_fn (krpc.client.Client): Stream used to retrieve velocity data
    """
    def update_velocity(self, velocity_fn):
        # Check to see if vessel has any thrust power yet
        if self.vessel.available_thrust == 0: 
            # If not, stage
            self.vessel.control.activate_next_stage()
        else: # If so, control
            # Get velocity
            velocity = velocity_fn()
            verticle_velocity = (velocity[2] * -1) - self.velocity_drift # Negate b/c z axis is facing 
            #   downwards

            # Determine if we can reach desired velocity in update frame
            # f = m * a
            # vf = vi + a * t
            mass = self.vessel.mass
            max_acceleration = self.vessel.available_thrust / mass
            max_post_frame_velocity = verticle_velocity + (max_acceleration * self.control_rate)

            #self.target_velocity(1000 / self.vessel.flight().mean_altitude)

            # If we can't
            if max_post_frame_velocity <= self.desired_velocity:
                self.vessel.control.throttle = 1
                return

            # If we can
            # Calculate difference between actual and desired velocity
            delta_velocity = self.desired_velocity - verticle_velocity

            # Calculate difference between desired and actual kinetic energy

            target_energy = vLib.calc_energy(mass, self.desired_velocity)
            current_energy = vLib.calc_energy(mass, verticle_velocity)

            delta_energy = target_energy - current_energy

            # Calculate distance rocket should travel if it were to accelerate to 
            # the desired velocity
            delta_distance = math.fabs(vLib.calc_delta_distance(verticle_velocity, 
                                                self.desired_velocity, 
                                                self.control_rate))

            # Calculate force needed to accelerate space craft calculated distance 
            # in the specified amount of time
            force = mass * vLib.calc_acceleration(verticle_velocity, 
                                                  self.desired_velocity, 
                                                  self.control_rate) # vLib.calc_force(delta_energy, delta_distance)

            # Calculate what percentage of vessel's thrust power is needed to 
            # achieve the desired velocity
            required_thrust = force / self.vessel.available_thrust

            # Determine if we can reach velocity in our current update frame
            throttle = required_thrust
            if required_thrust < 0:
                throttle = 0
            elif required_thrust > 1:
                throttle = 1

            # Set throttle
            self.vessel.control.throttle = throttle

            # Auto correct
            #if delta_distance < math.fabs(vLib.calc_delta_distance(self.desired_velocity, self.desired_velocity, self.control_rate)):
            self.velocity_drift = 0.76#delta_velocity

            # Print debug info
            print("tv = {}, v = {}, throttle = {}, dd = {}, vd = {}".format(self.desired_velocity, verticle_velocity + self.velocity_drift, throttle, delta_distance, self.velocity_drift))



    """ get_controlled checks to make sure no vessel properties are being 
    controlled at the moment.

    Returns:
        - None: If no properties are being controlled.
        - string: Name of property being controlled.

    Raises:
        - Error: If more than one property is being controlled.
    """
    def get_controlled(self):
        # Check all properties 
        controlled = []

        if self.desired_velocity is not None:
            controlled.append(Vessel.PropertyVelocity)

        # See if any properties were controlled
        if len(controlled) == 1:
            return controlled[0]
        elif len(controlled) > 1: # Check if state is invalid, and more than one 
            # property was controlled
            raise Error("desired state invalid, more than one property "+\
                        "controlled: {}".format(controlled))

        # Otherwise return None
        return None


    """ target_velocity sets the desired_velocity property.
    Fields:
        - v (float): Desired velocity.

    Raises:
        - Error: If any other properties are being controlled.
    """
    def target_velocity(self, v):
        # Set
        self.desired_velocity = v

    """ clear_velocity makes the vessel stop maintaining the previously 
    specified velocity.
    """
    def clear_velocity(self):
        self.desired_velocity = None
