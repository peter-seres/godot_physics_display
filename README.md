# godot_physics_display
A tool to display 6DOF physics simulations in 3D using the Godot Engine.
It uses a UDP stream between the python simulation and the game engine.

## Use

Run the Godot project and run the game. 

Run the python simulation using:

    python python_sim
    
The simulation is just an applied force or moment within the simulations package.
Change __main__ to different simulations.

Make sure the target port is the same port that 'udp_update.gd' is listening to.

## Setup

Virtual environment:

    python -m venv venv
    
Windows:

    venv\scripts\activate
    
Linux:

    source venv/bin/activate
    
Packages:

    pip install --upgrade pip
    pip install -r requiremens.txt
