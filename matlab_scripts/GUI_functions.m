%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GUI Configuration %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Define the path and the file name of the file to save the data. We choose 'planning_info.txt' as a convention. The file will be saved in the same folder as the script.
PATH = './';
FILE_NAME = 'planning_info.txt';


% Initializing the variables

% Ask the user to insert the values of the fixed point
prompt = {'Insert the x coordinate:', 'Insert the y coordinate:', 'Insert the z coordinate:'};
dlgtitle = 'Insert the coordinates';
dims = [1 50]; % Constrain the input to a single line

% Set the default values
definput = {'0', '1.4', '1.8'};

% Display the dialog box
answer = inputdlg(prompt, dlgtitle, dims, definput);
% If the user has clicked on "Cancel", close the figure and end the program
if isempty(answer)
    close all;
    exit(1);
end


% Check if the user has inserted the values \
if ~isempty(answer)
    x = str2double(answer{1});
    y = str2double(answer{2});
    z = str2double(answer{3});
else
    % In caso di annullamento, assegna valori predefiniti
    x = 0;
    y = 0;
    z = 0;
end

fixed_point = [x; y; z];
angles = [-pi/2 0 0];
green_point = [x; y; z];
rotation_matrix = eye(3);
%translation_vector = zeros(1, 3);
translation_vector = [x;y;z];

%% Create a figure
fig = figure('Position', [100, 100, 1000, 900], 'Name', 'Choose the orientation of the plane', 'NumberTitle', 'off');
fig.Resize = 'off';

% Create the first subplot for the plot
subplot('Position', [0.1 0.25 0.8 0.7]);

set(gca,'Xdir','reverse')
ax = gca;

% Create the second subplot for the sliders
subplot('Position', [0.1 0.1 0.1 0.1]);
ax_sliders = gca;

% Hide the axes for the sliders
ax_sliders.Visible = 'off';

%% Create sliders, buttons and labels
% Create a slider with a label
slider_green_x = uicontrol('Style', 'slider', 'Min', -3.14 + fixed_point(1), 'Max', 3.14 + fixed_point(1), 'Value', fixed_point(1), ...
    'Units', 'normalized', 'Position', [0.2, 0.08, 0.65, 0.03]);
label_slider_green_x = uicontrol('Style', 'text','Units', 'norm', 'Position', [0.85, 0.08, 0.05, 0.03]);

% Create another slider with a label
slider_green_y = uicontrol('Style', 'slider', 'Min', -3.14 + fixed_point(2), 'Max', 3.14 + fixed_point(2), 'Value', fixed_point(2), ...
    'Units', 'normalized', 'Position', [0.2, 0.03, 0.65, 0.03]);
label_slider_green_y = uicontrol('Style', 'text','Units', 'norm', 'Position', [0.85, 0.03, 0.05, 0.03]);


% Set up sliders for rotations
slider_x = uicontrol('Style', 'slider', 'Min', -pi, 'Max', pi, 'Value', -pi/2, ...
    'Units', 'normalized', 'Position', [0.2, 0.13, 0.65, 0.03]);

slider_y = uicontrol('Style', 'slider', 'Min', -pi, 'Max', pi, 'Value', 0, ...
    'Units', 'normalized', 'Position', [0.2, 0.08, 0.65, 0.03]);

slider_z = uicontrol('Style', 'slider', 'Min', -pi, 'Max', pi, 'Value', 0, ...
    'Units', 'normalized', 'Position', [0.2, 0.03, 0.65, 0.03]);

label_x = uicontrol('Style', 'text','Units', 'norm', 'Position', [0.85 0.10 0.1 0.05]);
label_y = uicontrol('Style', 'text','Units', 'norm', 'Position', [0.85 0.05 0.1 0.05]);
label_z = uicontrol('Style', 'text','Units', 'norm', 'Position', [0.85 0.001 0.1 0.05]);

% Create the "PASS" button 
pass_button = uicontrol('Style', 'pushbutton', 'String', 'PASS', ...
    'Units', 'normalized','Position', [0.05, 0.05, 0.1, 0.05]);

% Create the "DONE" button
done_button = uicontrol('Style', 'pushbutton', 'String', 'DONE', ...
    'Units', 'normalized','Position', [0.05, 0.05, 0.1, 0.05]);

% Set up the global variables
global fixed_point, angles, green_point, rotation_matrix, translation_vector;

%% Set up the labels for the sliders
set(label_slider_green_x,'string', sprintf('%s', 'Move X'));
set(label_slider_green_y,'string', sprintf('%s', 'Move Y'));

set(label_x,'string', sprintf('%s', 'X Rotation'));
set(label_y,'string', sprintf('%s', 'Y Rotation'));
set(label_z,'string', sprintf('%s', 'Z Rotation'));

%% Set up the callbacks for the sliders and buttons
addlistener(slider_green_x, 'Value', 'PostSet', @(src, event) update_green_x(src, event, slider_green_x, ax));
addlistener(slider_green_y, 'Value', 'PostSet', @(src, event) update_green_y(src, event, slider_green_y, ax));

addlistener(slider_x, 'Value', 'PostSet', @(src, event) update(src, event, slider_x, slider_y, slider_z, ax));
addlistener(slider_y, 'Value', 'PostSet', @(src, event) update(src, event, slider_x, slider_y, slider_z, ax));
addlistener(slider_z, 'Value', 'PostSet', @(src, event) update(src, event, slider_x, slider_y, slider_z, ax));

pass_button.Callback = @(src, event) on_close(src, event, PATH, FILE_NAME);
done_button.Callback = @(src, event) done_button_clicked(src, event, ax, slider_x, slider_y, slider_z, label_x, label_y, label_z, done_button, pass_button, slider_green_x, slider_green_y, label_slider_green_x, label_slider_green_y);

%% Set up the initial plot with the correct interactivity
slider_green_x.Enable = 'off';
slider_green_y.Enable = 'off';
label_slider_green_x.Enable = 'off';
label_slider_green_y.Enable = 'off';
pass_button.Enable = 'off';

slider_green_x.Visible = 'off';
slider_green_y.Visible = 'off';
label_slider_green_x.Visible = 'off';
label_slider_green_y.Visible = 'off';
pass_button.Visible = 'off';

% Plot the initial plane
plot_plane(ax, fixed_point, angles, green_point);

%% Waiting for the figure to close in order to end the program
waitfor(fig);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    Function definitions   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Function to update the plot based on slider values
function update(src, event, slider_x, slider_y, slider_z, ax)
    % This function is called whenever the slider values for the rotation of the plane are changed. It updates the plot based on the new values.

    global angles green_point fixed_point rotation_matrix translation_vector;
    
    angles(1) = slider_x.Value;
    angles(2) = slider_y.Value;
    angles(3) = slider_z.Value;
    plot_plane(ax, fixed_point, angles, green_point);
    drawnow;
end

% Function to handle the "DONE" button click
function done_button_clicked(src, event, ax,slider_x, slider_y, slider_z, label_x, label_y, label_z, done_button, pass_button, slider_green_x, slider_green_y, label_slider_green_x, label_slider_green_y);
    % This function is called when the "DONE" button is clicked. It disables the rotation sliders and enables the sliders for the green point together with the "PASS" button.
    
    global angles green_point fixed_point rotation_matrix translation_vector;

    %% Disable the rotation sliders and labels and enable the sliders and labels for choosing the green point 
    set(slider_x, 'Visible', 'off');
    set(slider_y, 'Visible', 'off');
    set(slider_z, 'Visible', 'off');

    set(slider_x, 'Enable', 'off');
    set(slider_y, 'Enable', 'off');
    set(slider_z, 'Enable', 'off');
    
    set(label_x, 'Visible', 'off');
    set(label_y, 'Visible', 'off');
    set(label_z, 'Visible', 'off');

    set(slider_green_x, 'Enable', 'on');
    set(slider_green_y, 'Enable', 'on');
    set(label_slider_green_x, 'Enable', 'on');
    set(label_slider_green_y, 'Enable', 'on');

    set(slider_green_x, 'Visible', 'on');
    set(slider_green_y, 'Visible', 'on');
    set(label_slider_green_x, 'Visible', 'on');
    set(label_slider_green_y, 'Visible', 'on');

    % Disable the "DONE" button and enable the "PASS" button
    set(done_button, 'Enable', 'off');
    set(done_button, 'Visible', 'off');

    set(pass_button, 'Enable', 'on');
    set(pass_button, 'Visible', 'on');    

    fprintf('Rotation Matrix:\n');
    disp(rotation_matrix);

    translation_vector = fixed_point;

    fprintf('\nTranslation Vector:\n');
    disp(translation_vector);

    green_point = fixed_point;
    plot_plane(ax, fixed_point, angles, green_point);
    drawnow;
end

% Function to update the green point's x coordinate
function update_green_x(src, event, slider_green_x, ax)
    % This function is called whenever the slider values for the green point's x coordinate are changed. It updates the plot based on the new values.

    global angles green_point fixed_point rotation_matrix translation_vector;

    green_point_plane = rotation_matrix^-1 * (green_point - fixed_point) + fixed_point;
    green_point_plane(1) = slider_green_x.Value;
    green_point = rotation_matrix * (green_point_plane - fixed_point) + fixed_point;
    plot_plane(ax, fixed_point, angles, green_point);
    
    drawnow;
end

% Function to update the green point's y coordinate
function update_green_y(src, event, slider_green_y, ax)
    % This function is called whenever the slider values for the green point's y coordinate are changed. It updates the plot based on the new values.

    global angles green_point fixed_point rotation_matrix translation_vector;
    
    green_point_plane = rotation_matrix^-1 * (green_point - fixed_point) + fixed_point;
    green_point_plane(2) = slider_green_y.Value;
    green_point = rotation_matrix * (green_point_plane - fixed_point) + fixed_point;
    plot_plane(ax, fixed_point, angles, green_point);

    drawnow;
end


% Function to handle the "PASS" button click
function on_close(src, event,PATH, FILE_NAME)
    % This function is called when the "PASS" button is clicked. It saves the final positions and closes the figure.

    global angles green_point fixed_point rotation_matrix translation_vector;
    
    % Display the final points
    fprintf('\nFinal Fixed Green Point:\n');
    disp(green_point);

    fprintf('\nFinal Fixed Red Point:\n');
    disp(fixed_point);

    fprintf('\nFinal Rotation Matrix:\n');
    disp(rotation_matrix);

    fprintf('\nFinal Translation Vector:\n');
    disp(translation_vector);

    % Save the final positions to a file
    % Open the file
    fileID = fopen(fullfile(PATH, FILE_NAME), 'w');
    
    % Check if the file is opened correctly
    if fileID == -1
        error('Cannot open file %s', fullfile(PATH, FILE_NAME));
    end
    
    fprintf(fileID, 'STARTING POSITION\n%.4f %.4f %.4f\n', fixed_point);
    fprintf(fileID, 'FINAL POSITION\n%.4f %.4f %.4f\n', green_point);
    fprintf(fileID, 'ROTATION MATRIX\n%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n', rotation_matrix');
    fprintf(fileID, 'TRANSLATION VECTOR\n%.4f %.4f %.4f\n', translation_vector);
        
    % Close the file
    fclose(fileID);

    % Close the figure
    close all;
end


function plot_plane(ax, point, angles, green_point)
    global angles green_point fixed_point rotation_matrix translation_vector;
    
    % Define the plane
    [xx, yy] = meshgrid(-3.14:3.14, -3.14:3.14);
    zz = zeros(size(xx));
    plane = cat(3, xx, yy, zz);

    rotation_matrix_x = [1, 0, 0; 0, cos(angles(1)), -sin(angles(1)); 0, sin(angles(1)), cos(angles(1))];
    rotation_matrix_y = [cos(angles(2)), 0, sin(angles(2)); 0, 1, 0; -sin(angles(2)), 0, cos(angles(2))];
    rotation_matrix_z = [cos(angles(3)), -sin(angles(3)), 0; sin(angles(3)), cos(angles(3)), 0; 0, 0, 1];

    rotation_matrix = rotation_matrix_z * rotation_matrix_y * rotation_matrix_x;
    
    reshaped_plane = reshape(plane, 3, []);

    % Apply rotation to each point in the plane
    rotated_plane = zeros(size(plane));
    for i = 1:size(plane, 1)
        for j = 1:size(plane, 2)
            point_i = reshape(plane(i, j, :), 3, 1);
            rotated_point = rotation_matrix * point_i + fixed_point;
            
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

    % Plot the fixed and green points
    scatter3(ax, point(1), point(2), point(3), 'red', 'filled');
    scatter3(ax, green_point(1), green_point(2), green_point(3), 'green', 'filled');
    
 
    % Set axis labels and limits
    xlabel(ax, 'X-axis');
    ylabel(ax, 'Y-axis');
    zlabel(ax, 'Z-axis');
    title(ax, 'Plane Rotating around a Fixed Point');
    xlim(ax, [-3.44 + point(1), 3.44 + point(1)]);
    ylim(ax, [-3.44 + point(2), 3.44 + point(2)]);
    zlim(ax, [-3.44 + point(3), 3.44 + point(3)]);
    set(ax, 'XDir', 'reverse');
    set(ax, 'YDir', 'reverse');

    % Set the view
    view(ax, 45, 30);
    hold(ax, 'off');
end