function yref = Desired_traj(t)
    % yref_func - Generates a piecewise desired trajectory.
    %
    % Inputs:
    %   t - Current time (scalar)
    %
    % Outputs:
    %   yref - Desired trajectory [x_ref; y_ref; theta_ref] (3x1 vector)

    % Define piecewise desired trajectory
    if t <= 20
        x_ref = 1;   % x = 1 for the first 8 seconds
        y_ref = 1.5; % y = 1.5 for the first 8 seconds
    else
        x_ref = 2;   % x = 2 after 8 seconds
        y_ref = 3;   % y = 3 after 8 seconds
    end

    theta_ref = 0; % Always 0 for theta

    % Combine into the desired trajectory vector
    yref = [x_ref y_ref theta_ref];
end





% % Circular Trajectory
% function yref = Desired_traj(t)
%     % Parameters for the circular trajectory
%     r = 1;              % Radius of the circle
%     T = 40;             % Total time for one full circle
%     omega = 2 * pi / T; % Angular velocity for full trajectory in T seconds
% 
%     % Ensure time is within bounds
%     t = mod(t, T);      % Wrap t around if it exceeds T
% 
%     % Calculate the angular position theta for counterclockwise motion
%     % Start at the origin (0, 0) at t = 0, by setting initial angle to -pi/2
%     theta = omega * t;  % Angle increases linearly with time
% 
%     % Circular trajectory equations centered at (0, 0)
%     x = r * cos(theta); % x-coordinate
%     y = r * sin(theta); % y-coordinate
% 
%     % Offset the trajectory to start from the origin
%     % Subtract the initial position at t = 0 to ensure it starts at (0, 0)
%     initial_x = r * cos(0); % x-coordinate at t = 0 (origin)
%     initial_y = r * sin(0); % y-coordinate at t = 0 (origin)
% 
%     x = x - initial_x; % Apply the offset to x
%     y = y - initial_y; % Apply the offset to y
% 
%     % Orientation theta (optional: you can set this differently if desired)
%     orientation_theta = theta; % Bot's orientation matches the angular position
% 
%     % Return the reference trajectory
%     yref = [x, y, orientation_theta];
% end
% 


% function yref = Desired_traj(t)
% 
% x_ref = 0.1*t;
% y_ref = 0;
% theta_ref = 0;
% yref = [x_ref y_ref theta_ref];
% end



