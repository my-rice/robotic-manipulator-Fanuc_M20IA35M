% Read the bag files and plot the joint positions and the Mean Squared Error (MSE) for each joint
close all
clear all

% Ask the user to insert the name of the trajectory
trajectoryname = input('Insert the name of the trajectory ', 's');

% Build the path to the bag files
folderPath_desired = "results_bagfiles/desired_trajectories/Desired" + trajectoryname;
bagReader_desired = ros2bagreader(folderPath_desired);
msgs_desired = readMessages(bagReader_desired);

% Extract the joint positions from the messages
joint_positions_desired = cellfun(@(msg) msg.position, msgs_desired, 'UniformOutput', false);

folderPath_actual = "results_bagfiles/real_trajectories/Real" + trajectoryname;

bagReader_actual = ros2bagreader(folderPath_actual);
msgs_actual = readMessages(bagReader_actual);

joint_positions_actual = cellfun(@(msg) msg.position, msgs_actual, 'UniformOutput', false);

% Check if the number of messages is the same for both desired and actual trajectories
assert(all(cellfun(@length, joint_positions_desired) == cellfun(@length, joint_positions_actual)), 'Dimension of joint positions are different.');


% Plots of the joint positions and the Mean Squared Error (MSE) for each joint
num_joints = length(joint_positions_desired{1}); % Assuming the first message has all joints
num_rows = ceil(num_joints / 3);

num_joints = length(joint_positions_desired{1}); % Assuming the first message has all joints
num_rows = ceil(num_joints / 3);

% Create figures for joint positions and MSE
figure_joint_positions = figure;
figure_mse = figure;


for joint = 1:num_joints
    % clear error_joint mse_joint
    error_joint = [];
    mse_joint = [];
    % Extract joint positions for both desired and actual trajectories
    joint_positions_desired_joint = cellfun(@(pos) pos(joint), joint_positions_desired);
    joint_positions_actual_joint = cellfun(@(pos) pos(joint), joint_positions_actual);

    % Compute the error between desired and actual joint positions
    error_joint = joint_positions_actual_joint - joint_positions_desired_joint;

    % Compute the Mean Squared Error (MSE) for the joint
    mse_joint = mean(error_joint.^2);

    % Determine subplot position
    subplot_row = ceil(joint / 3);
    subplot_col = mod(joint - 1, 3) + 1;
    
    % Plot joint positions in the joint positions figure
    figure(figure_joint_positions);
    subplot(num_rows, 3, (subplot_row - 1) * 3 + subplot_col);
    plot(joint_positions_desired_joint, 'r', 'LineWidth', 1.5);
    hold on;
    plot(joint_positions_actual_joint, 'b', 'LineWidth', 1.5);
    title(['Joint ', num2str(joint), ' - Position']);
    legend('Desired', 'Actual');
    xlabel('Trajectory Points');
    ylabel(['Joint ', num2str(joint),' [rad]']);
    xlim([1, length(joint_positions_desired_joint)]);  % Set x-axis limits
    ylim([min([joint_positions_desired_joint; joint_positions_actual_joint])-0.15, max([joint_positions_desired_joint; joint_positions_actual_joint])]+0.15);  % Set y-axis limits

    % Plot MSE values in the MSE figure
    figure(figure_mse);
    subplot(num_rows, 3, (subplot_row - 1) * 3 + subplot_col);
    plot(error_joint, 'g', 'LineWidth', 1.5);
    title(['Joint ', num2str(joint), ' - Error - MSE: ', num2str(mse_joint)]);
    xlabel('Trajectory Points');
    ylabel(['Error Joint ', num2str(joint),' [rad]']);
    xlim([1, length(joint_positions_desired_joint)]);  % Set x-axis limits
    ylim([min(error_joint)-0.001, max(error_joint)+0.001]);  % Set y-axis limits
end

% Set titles for both figures
figure(figure_joint_positions);
sgtitle('Trajectory - Joint Positions');

figure(figure_mse);
sgtitle('Trajectory - MSE Values');

