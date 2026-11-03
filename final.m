% Main Script: Real-time joint angle computation, update and send to robot
clc;
% Define Simulink model path and name
project_dir = fileparts(mfilename('fullpath'));
model_path = fullfile(project_dir, 'cobot600.slx');
model_name = 'cobot600'; % Model name without .slx extension

% Load the Simulink model
disp('Loading Simulink model...');
load_system(model_path);

% Save the captured image for processing (optional)
image_path = fullfile(project_dir, 'unsolved_maze2.png');
%imwrite(img, image_path);

% Release the camera resource (optional)
clear camera;

% Call Python script to detect marker positions
if int64(py.sys.path.count(project_dir)) == 0
    insert(py.sys.path, int32(0), project_dir);
end
centers_py = py.maze_solver.detect_maze_path_points(image_path);

% Convert Python list to MATLAB array
centers_cell = cell(centers_py);
if numel(centers_cell) ~= 10
    error('Expected 10 waypoints, got %d.', numel(centers_cell));
end
marker1 = [double(centers_cell{1}{1}), double(centers_cell{1}{2}), 0];
marker2 = [double(centers_cell{2}{1}), double(centers_cell{2}{2}), 0];
marker3 = [double(centers_cell{3}{1}), double(centers_cell{3}{2}), 0];
marker4 = [double(centers_cell{4}{1}), double(centers_cell{4}{2}), 0];
marker5 = [double(centers_cell{5}{1}), double(centers_cell{5}{2}), 0];
marker6 = [double(centers_cell{6}{1}), double(centers_cell{6}{2}), 0];
marker7 = [double(centers_cell{7}{1}), double(centers_cell{7}{2}), 0];
marker8 = [double(centers_cell{8}{1}), double(centers_cell{8}{2}), 0];
marker9 = [double(centers_cell{9}{1}), double(centers_cell{9}{2}), 0];
marker10 = [double(centers_cell{10}{1}), double(centers_cell{10}{2}), 0];

% Given initial end-effector position
initial_position = [-543.339, -193.854, 153.705]; % Initial position (in mm)
current_joint_angles = [-171.685 -26.152   48.348 -114.346  -87.187  -18.721];

% Calculate marker positions relative to initial end-effector position
disp('Marker 1 Position:');
x1 = initial_position(1) + marker1(1) - 30;
y1 = initial_position(2) - marker1(2) + 20;
z1 = initial_position(3);
marker1 = [x1, y1, z1];
disp(marker1);
disp('Marker 2 Position:');
x2 = initial_position(1) + marker2(1) - 20;
y2 = initial_position(2) - marker2(2);
z2 = initial_position(3);
marker2 = [x2, y2, z2];
disp(marker2);
disp('Marker 3 Position:');
x3 = initial_position(1) + marker3(1) - 20;
y3 = initial_position(2) - marker3(2);
z3 = initial_position(3);
marker3 = [x3, y3, z3];
disp(marker3);
disp('Marker 4 Position:');
x4 = initial_position(1) + marker4(1) - 20;
y4 = initial_position(2) - marker4(2);
z4 = initial_position(3);
marker4 = [x4, y4, z4];
disp(marker4);
disp('Marker 5 Position:');
x5 = initial_position(1) + marker5(1) - 20;
y5 = initial_position(2) - marker5(2);
z5 = initial_position(3);
marker5 = [x5, y5, z5];
disp(marker5);
disp('Marker 6 Position:');
x6 = initial_position(1) + marker6(1) - 20;
y6 = initial_position(2) - marker6(2);
z6 = initial_position(3);
marker6 = [x6, y6, z6];
disp(marker6);
disp('Marker 7 Position:');
x7 = initial_position(1) + marker7(1) - 20;
y7 = initial_position(2) - marker7(2);
z7 = initial_position(3);
marker7 = [x7, y7, z7];
disp(marker7);
disp('Marker 8 Position:');
x8 = initial_position(1) + marker8(1) - 20;
y8 = initial_position(2) - marker8(2);
z8 = initial_position(3);
marker8 = [x8, y8, z8];
disp(marker8);
disp('Marker 9 Position:');
x9 = initial_position(1) + marker9(1) - 20;
y9 = initial_position(2) - marker9(2);
z9 = initial_position(3);
marker9 = [x9, y9, z9];
disp(marker9);
disp('Marker 10 Position:');
x10 = initial_position(1) + marker10(1) - 20;
y10 = initial_position(2) - marker10(2);
z10 = initial_position(3);
marker10 = [x10, y10, z10];
disp(marker10);

% Robot DH parameters
L1 = 210;   % d1
L2 = 250;   % a2
L3 = 250;   % a3
L4 = 109.5; % d4
L5 = 107;   % d5
L6 = 76.2;  % d6

% Compute joint angles for the first marker position
marker1_joint_angles = compute_inverse_kinematics(marker1, L1, L2, L3, L4, L5, L6);
disp('Joint Angles for Marker 1 (degrees):');
disp(marker1_joint_angles);

% Compute joint angles for the second marker position
marker2_joint_angles = compute_inverse_kinematics(marker2, L1, L2, L3, L4, L5, L6);
disp('Joint Angles for Marker 2 (degrees):');
disp(marker2_joint_angles);

% Compute joint angles for the third marker position
marker3_joint_angles = compute_inverse_kinematics(marker3, L1, L2, L3, L4, L5, L6);
disp('Joint Angles for Marker 3 (degrees):');
disp(marker3_joint_angles);

% Compute joint angles for the fourth marker position
marker4_joint_angles = compute_inverse_kinematics(marker4, L1, L2, L3, L4, L5, L6);
disp('Joint Angles for Marker 4 (degrees):');
disp(marker4_joint_angles);

% Compute joint angles for the fifth marker position
marker5_joint_angles = compute_inverse_kinematics(marker5, L1, L2, L3, L4, L5, L6);
disp('Joint Angles for Marker 5 (degrees):');
disp(marker5_joint_angles);

% Compute joint angles for the sixth marker position
marker6_joint_angles = compute_inverse_kinematics(marker6, L1, L2, L3, L4, L5, L6);
disp('Joint Angles for Marker 6 (degrees):');
disp(marker6_joint_angles);

% Compute joint angles for the seventh marker position
marker7_joint_angles = compute_inverse_kinematics(marker7, L1, L2, L3, L4, L5, L6);
disp('Joint Angles for Marker 7 (degrees):');
disp(marker7_joint_angles);

% Compute joint angles for the eighth marker position
marker8_joint_angles = compute_inverse_kinematics(marker8, L1, L2, L3, L4, L5, L6);
disp('Joint Angles for Marker 8 (degrees):');
disp(marker8_joint_angles);

% Compute joint angles for the ninth marker position
marker9_joint_angles = compute_inverse_kinematics(marker9, L1, L2, L3, L4, L5, L6);
disp('Joint Angles for Marker 9 (degrees):');
disp(marker9_joint_angles);

% Compute joint angles for the tenth marker position
marker10_joint_angles = compute_inverse_kinematics(marker10, L1, L2, L3, L4, L5, L6);
disp('Joint Angles for Marker 10 (degrees):');
disp(marker10_joint_angles);

% Real-time simulation: Running continuously
disp('Starting real-time simulation...');
current_joint_angles = move_to_goal(current_joint_angles, marker1_joint_angles, model_name);
current_joint_angles = move_to_goal(current_joint_angles, marker2_joint_angles, model_name);
current_joint_angles = move_to_goal(current_joint_angles, marker3_joint_angles, model_name);
current_joint_angles = move_to_goal(current_joint_angles, marker4_joint_angles, model_name);
current_joint_angles = move_to_goal(current_joint_angles, marker5_joint_angles, model_name);
current_joint_angles = move_to_goal(current_joint_angles, marker6_joint_angles, model_name);
current_joint_angles = move_to_goal(current_joint_angles, marker7_joint_angles, model_name);
current_joint_angles = move_to_goal(current_joint_angles, marker8_joint_angles, model_name);
current_joint_angles = move_to_goal(current_joint_angles, marker9_joint_angles, model_name);
current_joint_angles = move_to_goal(current_joint_angles, marker10_joint_angles, model_name);


%%

function current_joint_angles = move_to_goal(current_joint_angles, goal_joint_angles, model_name)
    %pause(5);
    time_duration= 0.2;
    time_step= 0.1;
    % Calculate the number of steps for interpolation
    num_steps = round(time_duration / time_step);

    % Display movement start
    fprintf('Moving to goal position: %s\n', mat2str(goal_joint_angles, 4));

    % Interpolation loop
    for i = 1:num_steps
        % Compute interpolation factor
        alpha = i / num_steps;

        % Interpolate joint angles
        interpolated_angles = (1 - alpha) * current_joint_angles + alpha * goal_joint_angles;

        % Update Simulink signals for each joint
        for j = 1:6 % Assuming 6 joints
            signal_name = sprintf('Joint%d', j); % Signal name for Joint 1 to Joint 6
            set_param([model_name, '/', signal_name], 'Value', num2str(interpolated_angles(j))); % Update the signal in Simulink
        end

        % Optionally send angles to the robot in real time
         send_joint_angles_to_robot(interpolated_angles, 1500);

        % Real-time pacing
        pause(time_step);
    end

    % Update the current joint angles to the goal joint angles
    current_joint_angles = goal_joint_angles;


    % Display completion
    fprintf('Reached goal position: %s\n', mat2str(goal_joint_angles, 4));
end


% Close the Simulink model after the simulation (model keeps running in the background)
disp('Simulation completed. Closing Simulink model...');
%close_system(model_path, 0);

%% Function: Send Joint Angles to Robot
function send_joint_angles_to_robot(joint_angles, speed)
    % Format the joint angles and speed for the robot command
    message = sprintf('set_angles(%f, %f, %f, %f, %f, %f, %d)', ...
        joint_angles(1), joint_angles(2), joint_angles(3), ...
        joint_angles(4), joint_angles(5), joint_angles(6), speed);

    % Send the message via a TCP/IP connection
    server_ip = '192.168.1.159'; % Replace with your robot's IP
    server_port = 5001; % Replace with your robot's port
    send_tcp_packet(server_ip, server_port, message);
end

%% Function: Compute Inverse Kinematics
function angles_degrees = compute_inverse_kinematics(desired_position, L1, L2, L3, L4, L5, L6)
    % Desired orientation (example rotation matrix, replace as needed)
    desired_rotation = [-0.4621,  0.8866, -0.0199;
                         0.8864,  0.4610, -0.0432;
                        -0.0291, -0.0376, -0.9989];

    % Define the objective function (difference between the forward kinematics and desired position/orientation)
    obj_fun = @(angles) calculate_error(angles, desired_position, desired_rotation, L1, L2, L3, L4, L5, L6);

    % Initial guess for joint angles (in radians)
    initial_guess = deg2rad([-172, -35, 103, -164, -93, -50]); % Example initial guess

    % Use fminunc to minimize the objective function
    options = optimset('Display', 'off', 'TolFun', 1e-6, 'TolX', 1e-6);
    [angles_solution, ~] = fminunc(obj_fun, initial_guess, options);

    % Convert solution to degrees
    angles_degrees = rad2deg(angles_solution);
end

%% Function: Calculate Error for Inverse Kinematics
function error = calculate_error(angles, desired_pos, desired_rot, L1, L2, L3, L4, L5, L6)
    % Extract joint angles
    t1 = angles(1);
    t2 = angles(2);
    t3 = angles(3);
    t4 = angles(4);
    t5 = angles(5);
    t6 = angles(6);

    % Forward kinematics to compute the position and orientation
    T01 = dh_transform(t1, L1, 0, -pi/2);
    T12 = dh_transform(t2, 0, L2, 0);
    T23 = dh_transform(t3, 0, L3, 0);
    T34 = dh_transform(t4, L4, 0, -pi/2);
    T45 = dh_transform(t5 - pi, L5, 0, -pi/2);
    T56 = dh_transform(t6, L6, 0, 0);

    % Compute the overall transformation matrix
    T06 = T01 * T12 * T23 * T34 * T45 * T56;

    % Extract position and rotation
    position = T06(1:3, 4)';
    rotation = T06(1:3, 1:3);

    % Compute position error
    position_error = norm(position - desired_pos);

    % Compute orientation error using geodesic distance
    R_diff = desired_rot' * rotation;  % Relative rotation
    orientation_error = acos((trace(R_diff) - 1) / 2);

    % Weighted sum of position and orientation errors
    position_weight = 1;
    orientation_weight = 100;
    error = position_weight * position_error + orientation_weight * orientation_error;
end

%% Function: DH Transformation Matrix
function T = dh_transform(theta, d, a, alpha)
    % Compute the DH transformation matrix
    T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];
end

function send_tcp_packet(server_ip, server_port, message)
    try
        disp('Attempting to connect to the robot...');
        tcp_client = tcpclient(server_ip, server_port); % Attempt connection

        disp('Connected. Sending message...');
        write(tcp_client, message, "char"); % Send message

        disp('Message sent successfully.');
        clear tcp_client; % Close connection
    catch ME
        % Display error details
        disp('Error communicating with the robot:');
        disp(ME.message);
        disp('Verify the IP address, port, and that the robot server is running.');
    end
end
