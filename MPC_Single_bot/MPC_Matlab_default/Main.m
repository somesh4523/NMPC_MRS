clear all;
close all;
clc;


% Define the number of states and inputs
nx = 3;
ny = 3;
nu = 2;

nlobj = nlmpc(nx, ny, nu);
Ts = 0.1;
nlobj.Ts = Ts;
nlobj.PredictionHorizon = 10;
nlobj.ControlHorizon = 5;
nlobj.Model.StateFcn = "botDT0";
nlobj.Model.IsContinuousTime = false;
nlobj.Model.NumberOfParameters = 1;
nlobj.Model.OutputFcn = "botOutputFcn";
nlobj.Jacobian.OutputFcn = @(x, u, Ts) [1 0 0; 0 1 0; 0 0 1];
nlobj.Weights.OutputVariables = [5 5 0];
nlobj.Weights.ManipulatedVariables = [0.3 0.3];
nlobj.Weights.ManipulatedVariablesRate = [0.1 0.1];
nlobj.MV(1).Min = -0.3;
nlobj.MV(1).Max = 0.3;
nlobj.MV(2).Min = -1.5;
nlobj.MV(2).Max = 1.5;
nlobj.MV(1).RateMin = -0.03;
nlobj.MV(1).RateMax = 0.03;
nlobj.MV(2).RateMin = -0.15;
nlobj.MV(2).RateMax = 0.15;

x0 = [0; 0; 0];
u0 = [0; 0];

validateFcns(nlobj, x0, u0, [], {Ts});

EKF = extendedKalmanFilter(@botStateFcn, @botMeasurementFcn);

x = [0; 1; 0.5];
y = [x(1); x(2); x(3)];
EKF.State = x;

mv = [0; 0];

yref = [0 0 0];

nloptions = nlmpcmoveopt;
nloptions.Parameters = {Ts};

Duration = 40;
hbar = waitbar(0, 'Simulation Progress');
xHistory = x;
mvHistory = mv;
yrefHistory = yref;  % Initialize to store yref over time

% Initialize the figure for real-time plotting of x vs y
figure;
hold on;
xlabel('x');
ylabel('y');
title('Real-Time x vs y Trajectory');
grid on;

% Initialize a plot for the trajectory
trajectoryPlot = plot(NaN, NaN, 'k-', 'LineWidth', 1.5);  % Empty plot for trajectory
desiredTrajectoryPlot = plot(NaN, NaN, 'b--', 'LineWidth', 1.5);  % Plot desired trajectory (blue dashed line)

% Marker to represent the current position as a triangle
currentPosPlot = patch(NaN, NaN, 'r', 'FaceColor', 'r', 'EdgeColor', 'r');  % Initial empty triangle

% Highlight the front-facing vertex of the triangle
frontVertexPlot = plot(NaN, NaN, 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);  % Front-facing vertex (green)

% Define the size of the triangle marker based on the bot's size
baseSize = 0.17;  % 340 mm base (0.34 meters)

for ct = 1:(Duration / Ts)
    t = ct * Ts;
    yref = Desired_traj(t);  % Get desired trajectory for this time step
    yrefHistory = [yrefHistory; yref];
    
    % EKF update and nonlinear MPC move
    xk = correct(EKF, y);
    [mv, nloptions, info] = nlmpcmove(nlobj, xk, mv, yref, [], nloptions);
    
    predict(EKF, mv, Ts);
    x = botDT0(x, mv, Ts);
    y = x;  % Add noise to the measurement randn(3, 1) * 0.01
    
    % Append new data to histories
    xHistory = [xHistory, x];
    mvHistory = [mvHistory, mv];
    
    % Update the real-time x vs y trajectory plot
    trajectoryXData = get(trajectoryPlot, 'XData');
    trajectoryYData = get(trajectoryPlot, 'YData');
    set(trajectoryPlot, 'XData', [trajectoryXData, x(1)], 'YData', [trajectoryYData, x(2)]);  % Update trajectory
    
    % Update the desired trajectory plot in real-time
    desiredTrajectoryXData = get(desiredTrajectoryPlot, 'XData');
    desiredTrajectoryYData = get(desiredTrajectoryPlot, 'YData');
    set(desiredTrajectoryPlot, 'XData', [desiredTrajectoryXData, yref(1)], 'YData', [desiredTrajectoryYData, yref(2)]);  % Update desired trajectory
    
    % Update the bot marker using the new function
    [triangleX, triangleY, frontX, frontY] = updateBotMarker(x, baseSize);
    
    % Update the position of the triangle marker
    set(currentPosPlot, 'XData', triangleX, 'YData', triangleY);

    % Update the position of the front-facing vertex plot (green spot)
    set(frontVertexPlot, 'XData', frontX, 'YData', frontY);

    % Update waitbar progress
    waitbar(ct * Ts / Duration, hbar);
end

close(hbar);

% Plot final results for x, y, theta, and control variables

figure;

% Time vector for plotting
timeVector = 0:Ts:Duration;

% Plot x-position
subplot(2, 2, 1);
plot(timeVector, xHistory(1, :), 'LineWidth', 1.5);
hold on;
plot(timeVector, yrefHistory(:, 1), 'r--', 'LineWidth', 1.5);  % Time-varying reference for x
xlabel('Time (s)');
ylabel('x');
title('x-position');
legend('x', 'Reference (yref)');
grid on;

% Plot y-position
subplot(2, 2, 2);
plot(timeVector, xHistory(2, :), 'LineWidth', 1.5);
hold on;
plot(timeVector, yrefHistory(:, 2), 'r--', 'LineWidth', 1.5);  % Time-varying reference for y
xlabel('Time (s)');
ylabel('y');
title('y-position');
legend('y', 'Reference (yref)');
grid on;

% Plot theta
subplot(2, 2, 3);
plot(timeVector, xHistory(3, :), 'LineWidth', 1.5);
hold on;
plot(timeVector, yrefHistory(:, 3), 'r--', 'LineWidth', 1.5);  % Time-varying reference for theta
xlabel('Time (s)');
ylabel('\theta');
title('Pose (theta)');
legend('\theta', 'Reference (yref)');
grid on;

figure;

% Extract the control inputs history
mv1History = mvHistory(1, :);  % First manipulated variable
mv2History = mvHistory(2, :);  % Second manipulated variable

% Plot manipulated variable 1
subplot(2, 1, 1);
plot(timeVector, mv1History, 'LineWidth', 1.5);
hold on;
yline(nlobj.MV(1).Min, 'r--', 'LineWidth', 1);  % Min constraint
yline(nlobj.MV(1).Max, 'r--', 'LineWidth', 1);  % Max constraint
title('Manipulated Variable 1 (mv1)');
xlabel('Time (s)');
ylabel('mv1');
legend('mv1', 'Min constraint', 'Max constraint');
grid on;

% Plot manipulated variable 2
subplot(2, 1, 2);
plot(timeVector, mv2History, 'LineWidth', 1.5);
hold on;
yline(nlobj.MV(2).Min, 'r--', 'LineWidth', 1);  % Min constraint
yline(nlobj.MV(2).Max, 'r--', 'LineWidth', 1);  % Max constraint
title('Manipulated Variable 2 (mv2)');
xlabel('Time (s)');
ylabel('mv2');
legend('mv2', 'Min constraint', 'Max constraint');
grid on;
