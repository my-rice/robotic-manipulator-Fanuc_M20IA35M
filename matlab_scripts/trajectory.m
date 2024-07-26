function result = trajectory(starting_position, rotation_matrix,traslation_vector, final_position, trajectory_type,lambda,concavity);
    % The function trajectory generates a trajectory between two points, calling the right function according to the specified trajectory type.
    % The trajectory, up to now, can be only a parabola, but in the future it will be possible to add more types of trajectories just adding a new function.
    if numel(starting_position) ~= 3
        error('Starting position must be a 1D array of size 3');
    end

    if numel(final_position) ~= 3
        error('Final position must be a 1D array of size 3');
    end

    if numel(traslation_vector) ~= 3
        error('Orientation must be a 1D array of size 3');
    end

    if ~ismember(trajectory_type, {'LINE', 'PARABOLA'})
        error('Trajectory type must be one of the following: LINE, PARABOLA');
    end

    if strcmp(trajectory_type, 'PARABOLA')
        result = parabola(starting_position,rotation_matrix,traslation_vector,final_position,lambda,concavity);

        plot3D_function_from_points(result,starting_position, final_position)
        
    end
end