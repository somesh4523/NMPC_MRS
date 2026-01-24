% Closed-loop simulation

% Simulation duration in seconds
Duration = 20;

% Initial states and control inputs
xHistory1 = x0.';  % Ensure x0 is a row vector for logging
uHistory1 = u0.';  % Ensure u0 is a row vector for logging

% Initialize the current state and control input
xk = xHistory1(1,:).';
uk = uHistory1(1,:).';

% Initialize a timer for performance measurement
timerVal1 = 0;

% Define the desired final position (xd, yd)
xd = 1;  % Replace with your target x-coordinate
yd = 1;  % Replace with your target y-coordinate

% % Ensure that `pvstate` (or any required parameters) is defined
% pvstate = ...;  % Define the additional parameter(s) needed by botStateFcn

% Simulation loop
for k = 1:(Duration/Ts)
    % Update the current state xk for the NMPC optimization
    xk = xHistory1(k,:).';

    % Compute the desired angle using atan2
    theta_desired = atan2((xk(2) - yd), (xk(1) - xd));
    xf = [xd; yd; theta_desired];  % Update the desired state

    % Call the nlmpcControllerMEX function to compute the optimal control input
    tic;
    % [uk, onlinedata] = nlmpcControllerMEX(xk, uk, onlinedata);  % NMPC computation
    [uk, onlinedata] = nlmpcmove(msobj, x0, u0);  % NMPC computation
    timerVal1 = timerVal1 + toc;  % Accumulate the computation time

    % Simulate the system using the state function and control input
    ODEFUN = @(t, xk) botStateFcn(xk, uk, pvstate);  % Include pvstate
    [TOUT, XOUT] = ode45(ODEFUN, [0 Ts], xHistory1(k,:));  % Integrate the state

    % Log the results
    xHistory1(k+1,:) = XOUT(end,:);  % Store the final state from ODE solver
    uHistory1(k+1,:) = uk;  % Store the control input

    % Plot the robot's states (you can customize this as needed)
    % Example: If plotting angles or positions, you can modify the logic
    % hPlot.Theta1 = xHistory1(k+1,1);
    % hPlot.Theta2 = xHistory1(k+1,2);
    % drawnow limitrate;  % Update the plot efficiently
end

% Plot the closed-loop results
figure;
helperPlotResults(xHistory1, uHistory1, Ts, xf, 'show');  % Plot function
