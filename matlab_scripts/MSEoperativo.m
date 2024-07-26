close all
clear all

% Ask the user to enter the name of the trajectory
trajectoryname = input('Insert the name of the trajectory ', 's');

% Build the folder paths dynamically
folderPath_desired = "results_bagfiles/desired_trajectories/Desired" + trajectoryname;
bagReader_desired = ros2bagreader(folderPath_desired);
msgs_desired = readMessages(bagReader_desired);

xyzpose_desired = cellfun(@(msg) msg.position, msgs_desired, 'UniformOutput', false);
orientation_desired =  cellfun(@(msg) msg.orientation, msgs_desired, 'UniformOutput', false);

folderPath_actual = "results_bagfiles/real_trajectories/Real" + trajectoryname;
bagReader_actual = ros2bagreader(folderPath_actual);
msgs_actual = readMessages(bagReader_actual);

xyzpose_actual = cellfun(@(msg) msg.position, msgs_actual, 'UniformOutput', false);
orientation_actual =  cellfun(@(msg) msg.orientation, msgs_actual, 'UniformOutput', false);

% Extract XYZ coordinates and X, Y, Z, W orientations
x_desired = cellfun(@(pose) pose.x, xyzpose_desired);
y_desired = cellfun(@(pose) pose.y, xyzpose_desired);
z_desired = cellfun(@(pose) pose.z, xyzpose_desired);

x_actual = cellfun(@(pose) pose.x, xyzpose_actual);
y_actual = cellfun(@(pose) pose.y, xyzpose_actual);
z_actual = cellfun(@(pose) pose.z, xyzpose_actual);

x_orient_desired = cellfun(@(orientation) orientation.x, orientation_desired);
y_orient_desired = cellfun(@(orientation) orientation.y, orientation_desired);
z_orient_desired = cellfun(@(orientation) orientation.z, orientation_desired);
w_orient_desired = cellfun(@(orientation) orientation.w, orientation_desired);

x_orient_actual = cellfun(@(orientation) orientation.x, orientation_actual);
y_orient_actual = cellfun(@(orientation) orientation.y, orientation_actual);
z_orient_actual = cellfun(@(orientation) orientation.z, orientation_actual);
w_orient_actual = cellfun(@(orientation) orientation.w, orientation_actual);

% Calculate Roll, Pitch, and Yaw angles from quaternions
[roll_desired, pitch_desired, yaw_desired] = quat2angle([w_orient_desired, x_orient_desired, y_orient_desired, z_orient_desired],'XYZ');
[roll_actual, pitch_actual, yaw_actual] = quat2angle([w_orient_actual, x_orient_actual, y_orient_actual, z_orient_actual],'XYZ');

% Calculate errors for Roll, Pitch, and Yaw angles
error_roll = roll_actual -  roll_desired;
error_pitch = pitch_actual - pitch_desired;
error_yaw = yaw_actual - yaw_desired;

% Calculate MSE for Roll, Pitch, and Yaw angles
mse_roll = mean(error_roll.^2);
mse_pitch = mean(error_pitch.^2);
mse_yaw = mean(error_yaw.^2);

disp(['MSE Roll: ', num2str(mse_roll)]);
disp(['MSE Pitch: ', num2str(mse_pitch)]);
disp(['MSE Yaw: ', num2str(mse_yaw)]);



% Calculate errors for X, Y, and Z coordinates of position
error_x = x_actual - x_desired;
error_y = y_actual - y_desired;
error_z = z_actual - z_desired;

% Calculate MSE for X, Y, and Z coordinates of position
mse_x = mean(error_x.^2);
mse_y = mean(error_y.^2);
mse_z = mean(error_z.^2);

disp(['MSE X: ', num2str(mse_x)]);
disp(['MSE Y: ', num2str(mse_y)]);
disp(['MSE Z: ', num2str(mse_z)]);

% Plot the graphs
figure;

subplot(3, 2, 1);
plot(x_desired, 'r', 'LineWidth', 1.5);
hold on;
plot(x_actual, 'b', 'LineWidth', 1.5);
title('Position X');
legend('Desired', 'Actual');
xlabel('Time [ms]');
ylabel('Position [m]');
xlim([0, length(x_desired)]);
ylim([min(x_desired)-0.5, max(x_desired)+0.5]);


subplot(3, 2, 2);
plot(error_x, 'g', 'LineWidth', 1.5);
title(['Error X   ---   MSE: ', num2str(mse_x)]);
xlabel('Time [ms]');
ylabel('Error X [m]');
xlim([0, length(error_x)]);
ylim([min(error_x)-0.001, max(error_x)+0.001]);


subplot(3, 2, 3);
plot(y_desired, 'r', 'LineWidth', 1.5);
hold on;
plot(y_actual, 'b', 'LineWidth', 1.5);
title('Position Y');
legend('Desired', 'Actual');
xlabel('Time [ms]');
ylabel('Position [m]');
xlim([0, length(y_desired)]);
ylim([min(y_desired)-0.5, max(y_desired)+0.5]);

subplot(3, 2, 4);
plot(error_y, 'g', 'LineWidth', 1.5);
title(['Error Y   ---   MSE: ', num2str(mse_y)]);
xlabel('Time [ms]');
ylabel('Error Y [m]');
xlim([0, length(error_y)]);
ylim([min(error_y)-0.001, max(error_y)+0.001]);

subplot(3, 2, 5);
plot(z_desired, 'r', 'LineWidth', 1.5);
hold on;
plot(z_actual, 'b', 'LineWidth', 1.5);
title('Position Z');
legend('Desired', 'Actual');
xlabel('Time [ms]');
ylabel('Position [m]');
xlim([0, length(z_desired)]);
ylim([min(z_desired)-0.5, max(z_desired)+0.5]);

subplot(3, 2, 6);
plot(error_z, 'g', 'LineWidth', 1.5);
title(['Error Z   ---   MSE: ', num2str(mse_z)]);
xlabel('Time [ms]');
ylabel('Error Z [m]');
xlim([0, length(error_z)]);
ylim([min(error_z)-0.001, max(error_z)+0.001]);

% Plot the graphs for Roll, Pitch, and Yaw angles
figure;

subplot(3, 2, 1);
plot(roll_desired, 'r', 'LineWidth', 1.5);
hold on;
plot(roll_actual, 'b', 'LineWidth', 1.5);
title('Roll');
legend('Desired', 'Actual');
xlabel('Time [ms]');
ylabel('Angle [rad]');
xlim([0, length(roll_desired)]);
ylim([min(roll_desired)-0.15, max(roll_desired)+0.15]);

subplot(3, 2, 2);
plot(error_roll, 'g', 'LineWidth', 1.5);
title(['Error Roll   ---   MSE: ', num2str(mse_roll)]);
xlabel('Time [ms]');
ylabel('Error Roll [rad]')
xlim([0, length(error_roll)]);
ylim([min(error_roll)-0.001, max(error_roll)+0.001]);

subplot(3, 2, 3);
plot(pitch_desired, 'r', 'LineWidth', 1.5);
hold on;
plot(pitch_actual, 'b', 'LineWidth', 1.5);
title('Pitch');
legend('Desired', 'Actual');
xlabel('Time [ms]');
ylabel('Angle [rad]');
xlim([0, length(pitch_desired)]);
ylim([min(pitch_desired)-0.15, max(pitch_desired)+0.15]);

subplot(3, 2, 4);
plot(error_pitch, 'g', 'LineWidth', 1.5);
title(['Error Pitch   ---   MSE: ', num2str(mse_pitch)]);
xlabel('Time [ms]');
ylabel('Error Pitch [rad]')
xlim([0, length(error_pitch)]);
ylim([min(error_pitch)-0.001, max(error_pitch)+0.001]);


subplot(3, 2, 5);
plot(yaw_desired, 'r', 'LineWidth', 1.5);
hold on;
plot(yaw_actual, 'b', 'LineWidth', 1.5);
title('Yaw');
legend('Desired', 'Actual');
xlabel('Time [ms]');
ylabel('Angle [rad]');
xlim([0, length(yaw_desired)]);
ylim([min(yaw_desired)-0.15, max(yaw_desired)+0.15]);

subplot(3, 2, 6);
plot(error_yaw, 'g', 'LineWidth', 1.5);
title(['Error Yaw   ---   MSE: ', num2str(mse_yaw)]);
xlabel('Time [ms]');
ylabel('Error Yaw [rad]')
xlim([0, length(error_yaw)]);
ylim([min(error_yaw)-0.001, max(error_yaw)+0.001]);

sgtitle(['Trajectory']);

