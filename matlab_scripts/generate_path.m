function response = generate_path(request)
    % This function is called by the ROS2 node to generate the parabolic path.
    % It reads the data from the file and computes the parabolic path between the two points given as input.
    % Then, it returns the points of the parabola in the world frame through a bag file.
    % NOTE: The file must be in the same folder of this script and must be named 'planning_info.txt'. That is a convention.
    
    % Calling the script that manage the GUI. It writes the data in the file 'planning_info.txt', that is read by this script.
    GUI_functions
    
    % Read data from file
    filename = "planning_info.txt";
    [starting_position, final_position, rotation_matrix, traslation_vector] = readDataFromFile(filename);
    
    % Read data from the request. The request is a JSON string. 
    json_string = request.string_value;
    s = jsondecode(json_string);
    trajectory_type = s.trajectory_type;
    lambda = s.lambda;
    concavity = s.concavity;
    path = s.path;
    Name = s.Name;
    topic = s.topic;
    points = s.points;

    %% Compute the parabolic path
    try
        if strcmp(trajectory_type, 'PARABOLA')
            parabolic_path_points = parabola(starting_position,rotation_matrix,traslation_vector,final_position,lambda,concavity,points);
        else 
            fprintf('ERROR: trajectory_type not recognized\n');
            result = 'ERROR: trajectory_type not recognized';
            exit(1);
        end
        

    catch ME
        fprintf('Message: %s\n', ME.message);
        result = ME.message;
    end

    %% Plot the parabolic path
    temp = [0,0,0];
    invert_concavity = plot3D_function_from_points(parabolic_path_points,starting_position, final_position, rotation_matrix, traslation_vector)
    
    if invert_concavity
        if strcmp(concavity, 'UP')
            concavity = 'DOWN';
            disp("Inverting concavity: UP -> DOWN")
            parabolic_path_points = parabola(starting_position,rotation_matrix,traslation_vector,final_position,lambda,concavity, points);
        else
            concavity = 'UP';
            disp("Inverting concavity: DOWN -> UP")
            parabolic_path_points = parabola(starting_position,rotation_matrix,traslation_vector,final_position,lambda,concavity, points);
        end
    end

    %% Check if the path is valid
    radius = 4.20
    half_height = 4.20
    center = [0,0,1.2]
    for c=1:points
        
        % Check if the point is inside the cylinder
        if (parabolic_path_points(c,2)-center(2))^2 + (parabolic_path_points(c,3)-center(3))^2 <= radius && abs(parabolic_path_points(c,1) - center(1)) <= half_height
            %disp("Point inside the cylinder")
        else
            disp("Point outside the cylinder")
            response.string_value = 'FAILURE';
            exit(1);
        end
    end


    %% Compute the quaternion from the rotation matrix
    rotation_matrix= rotation_matrix*[0 1 0; -1 0 0; 0 0 1]
    quat = rotm2quat(rotation_matrix);
    disp("Computing the quaternion from the rotation matrix. The quaternion is:")
    disp(quat)

    %% Write the points of the parabola in a bag file    
    data = ros2message('geometry_msgs/Pose');
    bagWriter = ros2bagwriter(path)

    disp("Writing the points of the parabola in the bag file")
    temp = 0;
    for c=1:points
        % Creating the message to write in the bag file
    
        % Set the timestamp
        timestamp = ros2time(1.6160e+09);
        
        % Set the data
        temp = parabolic_path_points(c,:);
        data.position.x = double(temp(1));
        data.position.y = double(temp(2));
        data.position.z = double(temp(3));
    
        % Set the orientation (considerando una posizione dell'orientamento fissa)
        data.orientation.x = double(quat(2));
        data.orientation.y = double(quat(3));
        data.orientation.z = double(quat(4));
        data.orientation.w = double(quat(1));
    
        % Write the message in the bag file
        write(bagWriter, topic, timestamp, data)
    end

    delete(bagWriter)
    clear bagWriter
    response.string_value = 'SUCCESS';
end