# server_trajectory

This package provides a Service Server that has the role of calling the MATLAB script to compute a trajectory path chosen by the user, it can be
used by itself, and using CLI to compute different parabolas.
It was mainly created to work with `fanuc_m20ia_35m_planning_demo` package.
The executable of this package should be launched in `unisa_acg_ros2_group2` directory to find the relative scripts that the server calls, more information
about this script are in the `matlab_scripts` package.

# Usage
In order to boot the server as a stand alone node, run the following command in a separate terminal:

```bash
ros2 launch server_trajectory demo.launch.py
```