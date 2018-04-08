import krpc
import time
import sys
import math
import lib
import signal

# configuration
name = "launch"

UpdateInterval = 0.25
TargetVelocity = 1
SmallDistanceValue = 0.001

# helpers
def calc_energy(m, v):
    dir = 1
    if v < 0:
        dir = -1

    return dir * (m * (v ** 2)) / 2

def control_to_velocity(vessel, vel_fn, update_interval, target_velocity):
    # Check if we have any thrust to work with
    if vessel.available_thrust == 0:
        ctl.activate_next_stage()
    else:
        # check velocity
        v = vel_fn()
        v_up = -1*v[2]

        """ calculate force needed to reach target velocity
        """

        # delta velocity
        dv = target_velocity - v_up

        # delta energy
        target_energy = calc_energy(vessel.mass, target_velocity)
        current_energy = calc_energy(vessel.mass, v_up)

        de = target_energy - current_energy

        # distance travelled in 1 tic
        dd = math.fabs(v_up) * update_interval 
        if dd == 0:
            dd = SmallDistanceValue

        force = de / dd
        normalized_force = force / vessel.available_thrust

        if normalized_force < 0:
            new_throttle = 0
        elif normalized_force > 1:
            new_throttle = 1
        else:
            new_throttle = normalized_force

        ctl.throttle = new_throttle

        #print(v_up, normalized_force, de, target_energy, current_energy)

# connect
print("connecting")

conn = krpc.connect(name=name)

print("connection version: {}".format(conn.krpc.get_status().version))

# retrieve current vessel
vessel = conn.space_center.active_vessel

# setup vessel
print("setting up vessel")
vessel_instance = lib.vessel.Vessel(conn, vessel, UpdateInterval)

# control vessel
vessel_instance.target_velocity(50)

signal.signal(signal.SIGINT, signal.default_int_handler)

print("running control loop")
try:
    while True:
        vessel_instance.start_control_loop()
except KeyboardInterrupt:
    print("\ncaught ctrl+c, shutting down")

print("shut down")

"""
vessel.name = "CPU Controlled"

ctl = vessel.control
planet = vessel.orbit.body

if ctl.state != conn.space_center.ControlState.full:
    print("error: script does not have full control of vessel")
    sys.exit()

ctl.sas = True

# setup data stream
frame = planet.reference_frame

# get info
with conn.stream(vessel.velocity, frame) as vel_fn:
    while True:
        print(vessel.flight().vertical_speed)
        control_to_velocity(vessel, vel_fn, UpdateInterval, TargetVelocity)
        
        # wait till next update
        time.sleep(UpdateInterval)
"""
