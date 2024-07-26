# matlab_scripts

These are some MATLAB scripts used for parabola path creation made by the `server_trajectory` package. In particular the `launch_matlab_script.sh` is called by the MATLAB server
to let the user choice the parabola in a 3D visualization calling the function `trajectory_msg.m`. Two more scripts were created to perform analysis of the trajectory computed by the robot, one for joint space analysis that is called `MSEgiunti.m`, and one for task space analysis that is called `MSEoperativo.m`.

# Usage of trajectory generation

WARNING!!! This module can be used only if the `Symbolic Math Toolbox` is installed as a MATLAB add-on.
This script should be used just with the `server_trajectory`. When the server is called, a GUI appears and the user can choose the position of the first point of the parabola,
after that the user can choose the orientation of the plane on which the parabola will be calculated by rotating it around the X Y and Z axes through sliders. At the end the user can choose the last point position (whose coordinates can be regulated through sliders) and the concavity
of the parabola. After that some checks are made to:

- check length feasibility, because there is a constraint on lenght that have to be 3.14, that is parameterized

- check if parabola fits the robot workspace, which is assumed as a big cylinder with the round faces on the sides, with radius equal to 4.20 m, height equal to 8.40 m and the center positioned in [0,0,1.2]

- check if start and ending point are different points

After that the parabola is stored in a bag file.

# Usage of MSE Joint side

When using the application, the created trajectory is saved as a bagfile with name `Trajectory_%Y-%m-%d_%H:%M:%S`, and then, after the computation of the trajectory, the desired and actual joint positions are stored in two bagfiles, called `DesiredTrajectory_%Y-%m-%d_%H:%M:%S` and `RealTrajectory_%Y-%m-%d_%H:%M:%S`. These names are used to check the graphs and MSE of the relative trajectory in the MATLAB script. When used the user should just provide the name of the Trajectory and the graphs are computed, by loading the two bagfiles.
WARNING!!! This script name is `MSEgiunti` and should be run in the `matlab_script` folder in order to correctly retrieve the data from the bagfiles.

# Usage of MSE Cartesian side

When using the application, the created trajectory is saved as a bagfile with name `Trajectory_%Y-%m-%d_%H:%M:%S`, and then, after the computation of the trajectory, the desired and actual poses are stored in two bagfiles, called `DesiredTrajectory_%Y-%m-%d_%H:%M:%S` and `RealTrajectory_%Y-%m-%d_%H:%M:%S`. These names are used to check the graphs and MSE of the relative trajectory in the MATLAB script. When used the user should just provide the name of the Trajectory and the graphs are computed, by loading the two bagfiles.
WARNING!!! For this MATLAB script the `Aerospace Toolbox` is required!
WARNING!!! This script name is `MSEoperativo` and should be run in the `matlab_script` folder in order to correctly retrieve the data from the bagfiles.


# IMPORTANT NOTES FOR THE GUI 

When the MATLAB script is started, the plane shown is ortogonal to the initial position of the robot and the drilling tool is positioned behind the plane seeing the figure. The user should check what is the actual pose and orientation of the robot to correctly utilize the MATLAB script to achieve correct rotations and traslations of points. Some useful tip is that the initial pose of the drilling tool with respect to the world coordinate frame is:

- Position x: 0.0064

- Position y: 1.4056

- Position z: 1.80093

- Orientation x: 0.5

- Orientation y: 0.5

- Orientation z: 0.5

- Orientation w: 0.5

So the initial plane will set the orientation of the robot to this one.
The initial pose of the fastening tool, instead, is:

- Position x: 0.12044

- Position y: 1.3476

- Position z: 1.8134

- Orientation x: 0.5

- Orientation y: 0.5

- Orientation z: 0.5

- Orientation w: 0.5

Furthermore opening the RViz configuration, referring to the MATLAB axis, the y axis is along the drill, the z axis is along the robot and the x asis is along the slider in the left direction. Note also that the initial point and the final point coordinate are referred to the world frame that can be seen in Rviz.

