# trajectory_interfaces

This package provide new `srv` and `msg` configurations, but only `srv` is used in this project. This type of message is used to send to the 
`server_trajectory` package server the name of the trajectory that should be computed, in response it will return a message describing the MATLAB output and a boolean status of the execution:
- `true` if the parabola was successfully computed, 
- `false` if there was a problem in creation of the parabola, as described in `matlab_scripts`, or in the saving on the bag file.