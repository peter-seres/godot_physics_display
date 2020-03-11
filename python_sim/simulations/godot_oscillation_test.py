import time
import numpy as np
import godot_interaction as godot


def test_oscillation(axis=0):
    assert 0 <= axis <= 2

    # Set up test oscillation:
    freq = 2
    amplitude = 2.0
    t_0 = time.time()
    dt = 0.9 * 1/60.

    # Set up UDP stream:
    connection = godot.StreamUDP(port=9999, target_port=12345)

    while True:
        t = time.time() - t_0
        x = 3 + amplitude * np.sin(2 * np.pi * freq * t)

        # Create packet:
        position = [0.0, 0.0, 0.0]
        position[axis] = x
        update = godot.StateUpdate(position=position, attitude=[0.0, 0.0, 0.0, 1.0])

        # Send it over:
        print(update)
        connection.send(packet=update)

        # Wait a bit:
        time.sleep(dt)
