import time
import numpy as np
import godot_interaction as godot
from models import UAV
from kinematics import UnitQuaternion


def test_apply_force(axis=0):

    # Initialize Drone:
    pos_init = [0, 0, 0]
    vel_init = [0, 0, 0]
    att_init = UnitQuaternion.from_euler_angles_deg(roll=0, pitch=0, yaw=0).as_vector()
    pqr_init = [0, 0, 0]
    x_init = np.concatenate([pos_init, vel_init, att_init, pqr_init])
    uav = UAV(x_0=x_init, mass=5.0, inertia_matrix=np.diag([1.0, 1.0, 1.0]))

    # Set up UDP stream:
    godot_connection = godot.StreamUDP(port=9999, target_port=12345)

    # Set up real-time simulation:
    prev = time.time()
    time.sleep(0.01)
    while True:
        t_i = time.time()
        dt = t_i - prev
        prev = t_i

        # Sinusoid force:
        freq = 0.2
        amplitude = 4.0
        varying_force = amplitude * np.sin(2 * np.pi * freq * t_i)

        # Apply forces and moments:
        FM = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        FM[5] = -0.1

        # Step the system in time:
        uav.rk4(u_i=FM, t_i=t_i, dt=dt)

        # Transform to Godot coordinate system:
        godot_position = godot.R_i2g @ uav.position
        godot_attitude = godot.quaternion_transform_i2g @ uav.orientation.as_vector()

        # Create packet:
        update = godot.StateUpdate(position=list(godot_position), attitude=list(godot_attitude))

        # Send it over:
        godot_connection.send(packet=update)

        # Do not rush too much:
        time.sleep(0.9 * 1/60.)
