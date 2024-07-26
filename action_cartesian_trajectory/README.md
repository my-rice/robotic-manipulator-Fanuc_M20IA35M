# action_cartesian_trajectory

This package provide a new action message. This type of message is used to send to the 
`task_space_velocity_controller` package server the cartesian trajectory that has to be executed, as a feedback the action server returns three instances of CartesianTrajectoryPoint 
that represent the desired pose, the actual one and the error. At the end of the action goal, the action server can return one of the possible results listed in the `FollowCartesianTrajectory.action` file:
- `SUCCESSFUL`
- `INVALID_GOAL`
- `INVALID_POSE`
- `OLD_HEADER_TIMESTAMP`