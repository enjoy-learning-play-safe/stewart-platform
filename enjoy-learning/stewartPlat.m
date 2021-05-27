% ** SETUP

% * SET CONSTANTS
base_diameter = 18; % [inches], acrylic platform bolt circle
top_diameter = 14; % [inches], acrylic platform bolt circle
a = (base_diameter) / 2; % [inches], distance from center of base to actuator
b = (top_diameter) / 2; % [inches], distance from center of top to actuator

% * SET BASE AND TOP ANGLES
base_angle = [0 60 120 180 240 300]; % [degrees], Spacing of actuator pairs, base
base_angle = sort(base_angle); % sorted for calculation integrity
top_angle = [0 40 120 160 240 280]; % [degrees], Spacing of actuator pairs, top
top_angle = sort(top_angle); % sorted for calculation integrity

% * SET CONFIGURATION
% 6-3 Configuration when the base joints are spaced 60° and the top joints
% pairs are spaced ~120° (Physical joint angle accounted for)
% 6-6 Configuration when all joints are spaced 60° (unstable configuration)
configuration_6_3 = false; % Run preset 6-3 configuration simulation
configuration_6_6 = false; % Run preset 6-6 configuration simulation
animation = true; % Display Animation
plots = false; % Display Plots

% * SET PHYSICAL PARAMETERS
stroke_length = 8; % [inches]
joint_angle = 3.47; % [degrees], for 6_3

% * CONFIGURATION
min_joint_length = 16.7; % [inches], from CAD
max_joint_length = 24.7; % [inches], from CAD
min_joint_velocity = -2; % [inches/sec], from Progressive Automations
max_joint_velocity = 2; % [inches/sec], from Progressive Automations

% * SET UP MOTION VARIABLES
n = 1:400; % Array dimension
[theta, phi, psi, Px, Py, Pz] = deal(zeros(1, length(n))); % Create position array
time_duration = 10; % Seconds
time = 0:(time_duration / (max(n) - 1)):time_duration; % Create time array, position
time_dot = time; % Create time array, velocity
time_dot(end) = []; % Delete array end, discrete differentiation
motion_profile = 2; % Choose motion profile

% * PREALLOCATION, CREATE JOINT LOCATIONS AND DISTANCE ARRAYS
a_i = zeros(4, 6, length(theta));
b_i = zeros(4, 6, length(theta));
b_rot = zeros(4, 6, length(theta));
L_vector = zeros(4, 6, length(theta));
L_length = zeros(length(theta), 6);
L_length_dot = L_length;
L_length_dot(length(n), :) = [];

% ** MOTION PROFILE CREATION
% Create function based motion per degree of freedom (independently) based on the time duration over n
% steps. All motions are allowed. Resulting plots will determine if move profile is valid or not

if motion_profile == 1

    for m = 1:length(n)
        theta(m) = -30 + 30 * m / max(n); % [degrees], relative to x axis
        phi(m) = 0; % [degrees], relative to y axis
        psi(m) = -30 + 60 * m / max(n); % [degrees], relative to z axis
        Px(m) = 0; % [inches], x position
        Py(m) = 0; % [inches], y position
        Pz(m) = 20 + 2 * cosd(m / max(n) * 360); % [inches], z position
    end

end

if motion_profile == 2

    for m = 1:length(n)
        theta(m) = 10; % [degrees], relative to x axis
        phi(m) = 10 * cosd(m / max(n) * 360); % [degrees], relative to y axis
        psi(m) = 0; % [degrees], relative to z axis
        Px(m) = 3 * m / max(n); % [inches], x position
        Py(m) = 0; % [inches], y position
        Pz(m) = 22 - 3 * m / max(n); % [inches], z position
    end

end

if motion_profile == 3

    for m = 1:length(n)
        theta(m) = 0; % [degrees], relative to x axis
        phi(m) = 10; % [degrees], relative to y axis
        psi(m) = -15 + 25 * m / max(n); % [degrees], relative to z axis
        Px(m) = 3; % [inches], x position
        Py(m) = -2 + 3 * m / max(n); % [inches], y position
        Pz(m) = 20; % [inches], z position
    end

end

if motion_profile == 4

    for m = 1:length(n)
        theta(m) = -15 + 30 * m / max(n); % [degrees], relative to x axis
        phi(m) = 10 - 5 * m / max(n); % [degrees], relative to y axis
        psi(m) = 15 * sind(m / max(n) * 360); % [degrees], relative to z axis
        Px(m) = 1.5; % [inches], x position
        Py(m) = -3 + 4 * m / max(n); % [inches], y position
        Pz(m) = 19.8 + 2 * cosd(m / max(n) * 360); % [inches], z position
    end

end

P = [Px; Py; Pz; zeros(1, length(theta))]; % combine into array

% ** CALCULATE EOMS

if configuration_6_6

    for j = 1:length(theta)

        for i = 1:6
            % calculate a vector for each base location
            a_i(1, i, j) = a * cosd((i - 1) * 60);
            a_i(2, i, j) = a * sind((i - 1) * 60);
            a_i(3, i, j) = 0;
            a_i(4, i, j) = 1;
            % calculate b vector for each top location
            b_i(1, i, j) = b * cosd((i - 1) * 60);
            b_i(2, i, j) = b * sind((i - 1) * 60);
            b_i(3, i, j) = 0;
            b_i(4, i, j) = 1;
            % convert b from global to rotated

            b_rot(:, i, j) = stewartrot(theta(j), phi(j), psi(j), b_i(:, i, j));
            L_vector(:, i, j) = P(:, j) + b_rot(:, i, j) - a_i(:, i, j);
            L_length(j, i) = sqrt(L_vector(1, i, j)^2 + L_vector(2, i, j)^2 + L_vector(3, i, j)^2);
        end

    end

elseif configuration_6_3

    for j = 1:length(theta)
        % calculate b vector for each top location
        for i = [1 3 5]
            b_i(1, i, j) = b * cosd((i - 1) * 60 - joint_angle);
            b_i(2, i, j) = b * sind((i - 1) * 60 - joint_angle);
            b_i(3, i, j) = 0;
            b_i(4, i, j) = 1;
            % ------------------------------------------
            b_i(1, i + 1, j) = b * cosd((i - 1) * 60 + joint_angle);
            b_i(2, i + 1, j) = b * sind((i - 1) * 60 + joint_angle);
            b_i(3, i + 1, j) = 0;
            b_i(4, i + 1, j) = 1;
        end

        for i = 1:6
            % calculate a vector for each base location
            a_i(1, i, j) = a * cosd((i - 1) * 60);
            a_i(2, i, j) = a * sind((i - 1) * 60);
            a_i(3, i, j) = 0;
            a_i(4, i, j) = 1;
            % convert b from global to rotated

            b_rot(:, i, j) = stewartrot(theta(j), phi(j), psi(j), b_i(:, i, j));
            L_vector(:, i, j) = P(:, j) + b_rot(:, i, j) - a_i(:, i, j);
            L_length(j, i) = sqrt(L_vector(1, i, j)^2 + L_vector(2, i, j)^2 + L_vector(3, i, j)^2);
        end

    end

else % for all non-true 6-6 and 6-3 cases

    for j = 1:length(theta)

        for i = 1:6
            a_i(1, i, j) = a * cosd(base_angle(i));
            a_i(2, i, j) = a * sind(base_angle(i));
            a_i(3, i, j) = 0;
            a_i(4, i, j) = 1;
            % ------------------------------------------
            b_i(1, i, j) = b * cosd(top_angle(i));
            b_i(2, i, j) = b * sind(top_angle(i));
            b_i(3, i, j) = 0;
            b_i(4, i, j) = 1;

            b_rot(:, i, j) = stewartrot(theta(j), phi(j), psi(j), b_i(:, i, j));
            L_vector(:, i, j) = P(:, j) + b_rot(:, i, j) - a_i(:, i, j);
            L_length(j, i) = sqrt(L_vector(1, i, j)^2 + L_vector(2, i, j)^2 + L_vector(3, i, j)^2);
        end

    end

end

% ** DERIVATIVE CALCULATION

[theta_dot, phi_dot, psi_dot, Px_dot, Py_dot, Pz_dot, P_dot] = deal(zeros(1, length(n) - 1)); % preallocation

for i = 2:length(n)
    Px_dot(i - 1) = (Px(i) - Px(i - 1)) / (time(i) - time(i - 1));
    Py_dot(i - 1) = (Py(i) - Py(i - 1)) / (time(i) - time(i - 1));
    Pz_dot(i - 1) = (Pz(i) - Pz(i - 1)) / (time(i) - time(i - 1));
    P_dot(i - 1) = sqrt((Px_dot(i - 1))^2 + (Py_dot(i - 1))^2 + (Pz_dot(i - 1))^2);
    theta_dot(i - 1) = (theta(i) - theta(i - 1)) / (time(i) - time(i - 1));
    phi_dot(i - 1) = (phi(i) - phi(i - 1)) / (time(i) - time(i - 1));
    psi_dot(i - 1) = (psi(i) - psi(i - 1)) / (time(i) - time(i - 1));

    for j = 1:6
        L_length_dot(i - 1, j) = (L_length(i, j) - L_length(i - 1, j)) / (time(i) - time(i - 1));
    end

end

% ** PLOTS

if plots
    figure(1)
    hold on

    for i = 1:6
        plot(time, L_length(:, i))
    end

    xlabel('Time (seconds)')
    ylabel('Length (inches)')
    plot(time, min_joint_length * ones(1, length(time)), '--k', time, max_joint_length * ones(1, length(time)), '--k', 'LineWidth', 2)
    legend('Actuator 1', 'Actuator 2', 'Actuator 3', 'Actuator 4', 'Actuator 5', 'Actuator 6', 'Min Length', 'Max Length', 'Location', 'north')
    title('Joint Lengths, from Sphere to Sphere')
    hold off
    figure(2)
    hold on

    for i = 1:6
        plot(time_dot, L_length_dot(:, i))
    end

    plot(time, min_joint_velocity * ones(1, length(time)), '--k', time, max_joint_velocity * ones(1, length(time)), '--k', 'LineWidth', 2)
    xlabel('Time (seconds)')
    ylabel('Velocity (inches/sec)')
    ylim([-2.5 2.5])
    % plot(time, min_joint_length*ones(1,length(time_dot)), time, max_joint_length * ones(1, length(time)))
    legend('Actuator 1', 'Actuator 2', 'Actuator 3', 'Actuator 4', 'Actuator 5', 'Actuator 6', 'Min Length', 'Max Length', 'Location', 'northwest')
    title('Joint Velocities')
    hold off
    figure(3)
    plot(time, Px, time, Py, time, Pz)
    xlabel('Time (seconds)')
    ylabel('Coordinate Value (inches)')
    legend('X Centroid', 'Y Centroid', 'Z Centroid', 'Location', 'east')
    title('Position of Centroid P')
    figure(4)
    plot(time, theta, time, phi, time, psi)
    xlabel('Time (seconds)')
    ylabel('Angle of Rotation (degree)')
    legend('Theta (about X)', 'Phi (about Y)', 'Psi (about Z)')
    title('Angle of Rotation about Respective Axis')
    figure(5)
    plot(time_dot, Px_dot, time_dot, Py_dot, time_dot, Pz_dot, time_dot, P_dot)
    xlabel('Time (seconds)')
    ylabel('Coordinate Value Velocity (inches/sec)')
    legend('X Centroid', 'Y Centroid', 'Z Centroid', 'Centroid Velocity', 'Location', 'southeast')
    title('Velocity of Centroid P')
    figure(6)
    plot(time_dot, theta_dot, time_dot, phi_dot, time_dot, psi_dot)
    xlabel('Time (seconds)')
    ylabel('Angular Velocity (degree/sec)')
    legend('Theta Dot (about X)', 'Phi Dot (about Y)', 'Psi Dot (about Z)', 'Location', 'north')
    title('Angular Velocity about Respective Axis')
end

% ** ANIMATION

if animation
    figure(100)
    alpha_base = 0:10:360; % creates angles for circular base plot
    x_base = base_diameter * 0.5 * cosd(alpha_base); % x values for circular base
    y_base = base_diameter * 0.5 * sind(alpha_base); % y values for circular base
    z_base = zeros(length(x_base)); % z values for circular base

    if configuration_6_3 == 0 && configuration_6_6 == 0
        alpha = base_angle;
    else
        alpha = 0:60:360;
    end

    x = a * cosd(alpha); % x values for actuators base side
    y = a * sind(alpha); % y values for actuators base side
    z = zeros(length(x));
    z1 = ones(length(x));

    for j = 1:length(psi)
        hold off
        plot3(x_base, y_base, z_base, 'b-', 'LineWidth', 3); % plot circular base every time
        hold on

        for i = 1:6
            % for each instance in time, plot the xyz values of the end of the actuators based on the length
            x_d(i) = x(i) + L_vector(1, i, j);
            y_d(i) = y(i) + L_vector(2, i, j);
            z_d(i) = z(i) + L_vector(3, i, j);
            plot3([x(i) x_d(i)], [y(i) y_d(i)], [z(i) z_d(i)], 'LineWidth', 3)
        end

        % * periodic boundary condition to complete top plate drawing
        x_d(i + 1) = x_d(1);
        y_d(i + 1) = y_d(1);
        z_d(i + 1) = z_d(1);
        plot3(x_d, y_d, z_d, 'LineWidth', 3)
        % * plot settings
        axis square;
        axis([-(1.5 * a) (1.5 * a) -(1.5 * a) 1.5 * a 0 (max(Pz) + b / 2)]);
        grid on;
        set(gcf, 'WindowState', 'fullscreen')
        pause(1/1000)

        if j < length(psi)
            clf;
        end

    end

end

% ** MAGIC SAUCE
function b_rot = stewartrot(theta, phi, psi, b_home)
    Z = [cosd(psi) -sind(psi) 0 0; sind(psi) cosd(psi) 0 0; 0 0 1 0; 0 0 0 1];
    Y = [cosd(theta) 0 sind(theta) 0; 0 1 0 0; -sind(theta) 0 cosd(theta) 0; 0 0 0 1];
    X = [1 0 0 0; 0 cosd(phi) -sind(phi) 0; 0 sind(phi) cosd(phi) 0; 0 0 0 1];
    b_rot = Z * Y * X * b_home;
end
