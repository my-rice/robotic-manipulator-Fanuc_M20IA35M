#!/bin/bash

# Define the parameters for the MATLAB script
bag_file_name="$1"

# Define possible paths to MATLAB and the MATLAB script
MATLAB_PATHS=( # Possible paths to MATLAB
    "/usr/bin/matlab"                   
    "/usr/local/MATLAB/R2023a/bin/matlab"
    "/usr/local/MATLAB/R2023b/bin/matlab"
    "/usr/local/MATLAB/R2022a/bin/matlab"
    "/usr/local/MATLAB/R2022b/bin/matlab"
    "/usr/local/MATLAB/R2021a/bin/matlab"
    "/usr/local/MATLAB/R2021b/bin/matlab"

)

MATLAB_EXEC=""
for path in "${MATLAB_PATHS[@]}"; do
    if [ -x "$path" ]; then
        MATLAB_EXEC="$path"
        break
    fi
done


# Execute MATLAB script with parameters
# -nodesktop: MATLAB does not display the desktop
# -nosplash: MATLAB does not display the splash screen
# -r: MATLAB executes the specified command
# exit: MATLAB exits after executing the command
cd matlab_scripts
# Try to execute the matlab command
matlab -nodesktop -nosplash -r "exit;" > /dev/null 2>&1

# Check if the matlab command was successful
if [ $? -eq 0 ]; then
    echo "Executing MATLAB script..."
    matlab -nodesktop -nosplash  -r "trajectory_script('$bag_file_name'); exit;"
elif [ -n "$MATLAB_EXEC" ]; then
    echo "Executing MATLAB script..."
    "$MATLAB_EXEC" -nodesktop -nosplash  -r "trajectory_script('$bag_file_name'); exit;"
else
    echo "I cannot find MATLAB executable. Please, check the MATLAB installation."  
    echo "Insert the MATLAB executable path:"
    read matlab_path
    if [ -x "$matlab_path" ]; then
        echo "Executing MATLAB script..."
        "$matlab_path" -nodesktop -nosplash  -r "trajectory_script('$bag_file_name'); exit;"
    else
        echo "I cannot find MATLAB executable. Please, check the MATLAB installation."  
    fi
fi