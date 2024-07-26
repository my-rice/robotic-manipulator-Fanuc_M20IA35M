function new_concavity = plot3D_function_from_points(points,starting_position, final_position, rotation_matrix, traslation_vector)
    % This function plots a 3D function from a set of points and the starting and final position given as input. 
    % It also takes as input the rotation matrix and the traslation vector.
    % The function returns a boolean value that is true if the user wants to invert the concavity of the function
    % and also plots the 3D function and waits for the user to close the figure in order to end the program.
    
    new_concavity = false;
    fixed_point = final_position;
    disp('PLOTTING THE 3D FUNCTION FROM THE POINTS');

    disp(traslation_vector)
    disp(final_position)

    %% Create a figure
    fig = figure('Position', [100, 100, 1000, 900], 'Name', 'Final result', 'NumberTitle', 'off');
    fig.Resize = 'off';

    % Create the first subplot for the plot
    subplot('Position', [0.1 0.25 0.8 0.7]);

    set(gca,'Xdir','reverse')
    ax = gca;


    % Create the "DONE" button
    done_button = uicontrol('Style', 'pushbutton', 'String', 'DONE', ...
    'Units', 'normalized','Position', [0.3, 0.05, 0.1, 0.05]);

    done_button.Callback = @(src, event) close(fig);

    % Create the "INVERT" button
    invert_button = uicontrol('Style', 'pushbutton', 'String', 'INVERT CONCAVITY', ...
    'Units', 'normalized','Position', [0.05, 0.05, 0.2, 0.05]);

    invert_button.Callback = @invert_buttonCallbackFunction;
    function invert_buttonCallbackFunction(src, event)
        new_concavity = true;
        close(fig);
    end

    % Define the plane
    [xx, yy] = meshgrid(-3.14:3.14, -3.14:3.14);
    zz = zeros(size(xx));
    plane = cat(3, xx, yy, zz);

    reshaped_plane = reshape(plane, 3, []);

    % Apply rotation to each point in the plane
    rotated_plane = zeros(size(plane));
    for i = 1:size(plane, 1)
        for j = 1:size(plane, 2)
            point_i = reshape(plane(i, j, :), 3, 1);
            rotated_point = rotation_matrix * point_i + traslation_vector.';
            
            rotated_plane(i, j, 1) = rotated_point(1);
            rotated_plane(i, j, 2) = rotated_point(2);
            rotated_plane(i, j, 3) = rotated_point(3);
        end
    end

    % Clear the axes
    cla(ax);

    % Plot the rotated surface
    surf(ax, rotated_plane(:,:,1), rotated_plane(:,:,2), rotated_plane(:,:,3), ...
        'FaceAlpha', 0.65, 'EdgeColor', 'none', 'FaceColor', 'blue');
    
    hold(ax, 'on');
    
    % Plot the points
    plot3(points(:,1), points(:,2), points(:,3), 'r', 'LineWidth', 2)
    
    % Set axis labels and limits
    xlabel(ax, 'X-axis');
    ylabel(ax, 'Y-axis');
    zlabel(ax, 'Z-axis');
    title(ax, 'Plane Rotating around a Fixed Point');
    xlim(ax, [-3.44 + fixed_point(1), 3.44 + fixed_point(1)]);
    ylim(ax, [-3.44 + fixed_point(2), 3.44 + fixed_point(2)]);
    zlim(ax, [-3.44 + fixed_point(3), 3.44 + fixed_point(3)]);

    set(ax, 'XDir', 'reverse');
    set(ax, 'YDir', 'reverse');

    % Set the view
    view(ax, 45, 30);
    hold(ax, 'off');

    %% Waiting for the figure to close in order to end the program
    waitfor(fig);

end