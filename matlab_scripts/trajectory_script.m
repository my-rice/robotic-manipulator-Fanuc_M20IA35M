function trajectory_script(name)
    % The function generate_path is called to compute the trajectory. It takes as input the coordinates (x,y,z) of the starting point and the name of the bag file.
    % Then set the information needed to compute the trajectory.
    % The information are:
    % - trajectory_type: the type of trajectory to be computed. It can be either LINE or PARABOLA
    % - lambda: the length of the trajectory
    % - concavity: the concavity of the parabola. It can be either UP or DOWN
    % - path: the path where to save the bag file
    % - Name: the name of the bag file
    % - topic: the topic of the bag file
    % - points: the number of points of the parabola

    % checks on the input parameters
    if nargin < 1
        error('Not enough input arguments');
        exit(1);
    elseif nargin > 1
        error('Too many input arguments');
        exit(1);
    end

    if ~ischar(name)
        error('The name must be a string');
        exit(1);
    end

    % set the information needed to compute the trajectory
    path = strcat(pwd,"/bagfiles/")
    path = strcat(path,name)
    disp(strcat("The bag file will be saved in: ",path));
    trajectory_info = struct('trajectory_type', 'PARABOLA', ...
                             'lambda', 3.14, ...
                             'concavity', 'DOWN', ...
                             'path', path, ...
                             'Name', name, ...
                             'topic', '/geometry_msgs', ...
                             'points', 100);
    
    % convert the struct to a json string and set the request message
    request.string_value = jsonencode(trajectory_info);
    
    % call the function to compute the trajectory
    try
        generate_path(request)
    catch ME
        disp("Error in the computation of the trajectory");
        exit(1);
    end

end