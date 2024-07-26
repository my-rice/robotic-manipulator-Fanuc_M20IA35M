# acg_resources_fanuc_m20ia_35m_description

## Writing of the URDF

The URDF provided in this packages has been made by the group 2 of Robotics class of University of Salerno on December 2, 2023. The robot information has been taken from the official datasheet provided by FANUC company. The URDF itself was written starting from the Denavit-Hartenberg convention and a supporting script `rotation_to_rpy` was used to transform the rotation matrix computed in **roll**, **pitch** and **yaw** angles that can be found in `supporting_scripts` package. 

The only case where Denavit was not used, was in the addition of the slider and the end effector, because they are not part of the robot itself. The dimension of the slider was computed to understand where to put the relative frame and two frame were positioned in the center of both the tools of the end effector: drilling one and fastening one. 

The parameter of inertia, mass and center of mass, where not provided by the FANUC, so they have been computed starting from the mesh provided by Enrico Ferrentino (eferrentino@unisa.it) using tools such as **Blender**, **Meshlab** and **MeshCleaner**, and they were put in the `inertial.yaml` configuration file and added to the urdf using xacro with an additional file `inertial.xacro`. The result obtained by this tools were not rotated with respect of the frame of the relative link, as URDF demand, so to compute this transformations another supporting script `script_inertia_matrix_and_center_of_mass` was used, that can be found in the same package as before.

## Usage

In order to parse the URDF to visualize the FANUCm20ia/35m robot, after building, use the launch file provided with this package:

```bash
ros2 launch acg_resources_fanuc_m20ia_35m_description display.launch.py
```

RViz will be launched and the FANUC robot visualized (with its reference frames).
Use the Joint State Publisher's GUI to move single joints.

