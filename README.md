# Using the godot display folder

A tool to display 6DOF physics simulations in 3D using the Godot Engine.
It uses a UDP stream between the simulations and the game engine. Currently implemented for Python and Rust.

## Setup

Clone the repository.

Download the latest version of the Godot Engine from:

## Usage

Run Godot and in the Editor: open the `project.godot` file using the Import button and you will see the simulation scene.

Press the play button to run the display application.

The application is now waiting for StateUpdate JSON packages with two vectors: position (R3) and attitude (R4).

## Python Instructions:

### Make a UDP Stream

Make sure that the target port is the same as the port set in Godot. By selecting the UAV node, the Inspector on the right shows the `Script Variables` -> `Port` setting. 

In the simulation do:
```python
    import visualization.godot_display as godot
    godot_connection = godot.StreamUDP(port=9999, target_port=12345)
```

## Send data packet every 1/60.0 of a second:

```python
    # Coordinate-system transformation from NED to Godot:
    godot_position = godot.transform_translation(uav.position)
    godot_attitude = godot.transform_rotation(uav.orientation)

    # Make packet and send:
    update = godot.StateUpdate(position=godot_position, attitude=godot_attitude)
    godot_connection.send(packet=update)

```
