# supporting_scripts

This are some python scripts used to pass from Denavit-Hartenberg notation to URDF notation. In particular there is a `rotation_to_rpy.py` script that given a rotation matrix, use the inverse formulas from the book of Robotics to calculate the right roll, pitch and yaw angles that can be put in the URDF file. 
The `script_inertia_matrix_and_center_of_mass.py` is a static script, that takes the inertia matrix and center of mass of our links, and perform rotation and translation to return the same information to respect of the frames of the same link.

