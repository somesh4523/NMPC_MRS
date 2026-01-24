close all;
clear all;
clc;

% Define the number of robots
numRobots = 6;

% Define the number of states and inputs for each robot
nx = 3;
ny = 3;
nu = 2;

% Create cell arrays to store parameters for each robot
nlobj = cell(1, numRobots);
EKF = cell(1, numRobots);
x = cell(1, numRobots);
y = cell(1, numRobots);
mv = cell(1, numRobots);
yref = cell(1, numRobots);

% Initialize robot parameters
Ts = 0.1; % Simulation timestep
Duration = 50;
kp = 0.35; % Proportional gain
kc = 0.8; % Centroidal coupling gain
baseSize = 0.17; % Size of triangle marker for bots
centroid_velocity = [0.3, 0]; % Velocity of the centroid in the positive x-direction
formation_achieved = false;
xHistory = cell(1, numRobots);
mvHistory = cell(1, numRobots);
yrefHistory = cell(1, numRobots);

formationRadius = 1; % Adjust this value to control the size of the formation

% Generate final positions for the desired formation
finalPositions = generatePolygonVertices(numRobots, formationRadius);

Laplac = ones(numRobots) - numRobots * eye(numRobots); % Laplacian matrix

% Initialize position arrays
pos = zeros(numRobots, 2);
posHistory = cell(1, numRobots);

% Initialize parameters for all robots
for i = 1:numRobots
    initPos = rand(1, 2); % Random initial positions
    x{i} = [initPos(1); initPos(2); 0]; % Initial [x, y, theta]
    y{i} = x{i};  % Initial output (same as initial state)
    pos(i, :) = [x{i}(1), x{i}(2)]; % Initialize planner position with actual position
    posHistory{i} = pos(i, :);
    mv{i} = [0; 0]; % Initial control inputs
    yref{i} = [0 0 0]; % Initial reference trajectory
    xHistory{i} = x{i};
    mvHistory{i} = mv{i};
    yrefHistory{i} = yref{i};

    nlobj{i} = nlmpc(nx, ny, nu);
    nlobj{i}.Ts = Ts;
    nlobj{i}.PredictionHorizon = 10;
    nlobj{i}.ControlHorizon = 5;
    nlobj{i}.Model.StateFcn = "botDT0";
    nlobj{i}.Model.IsContinuousTime = false;
    nlobj{i}.Model.NumberOfParameters = 1;
    nlobj{i}.Model.OutputFcn = "botOutputFcn";
    nlobj{i}.Jacobian.OutputFcn = @(x, u, Ts) eye(nx);
    nlobj{i}.Weights.OutputVariables = [15 15 0.1];
    nlobj{i}.Weights.ManipulatedVariables = [0.3 0.3];
    nlobj{i}.Weights.ManipulatedVariablesRate = [0.3 0.3];
    nlobj{i}.MV(1).Min = -0.3;
    nlobj{i}.MV(1).Max = 0.3;
    nlobj{i}.MV(2).Min = -1.5;
    nlobj{i}.MV(2).Max = 1.5;
    nlobj{i}.MV(1).RateMin = -0.03;
    nlobj{i}.MV(1).RateMax = 0.03;
    nlobj{i}.MV(2).RateMin = -0.15;
    nlobj{i}.MV(2).RateMax = 0.15;

    validateFcns(nlobj{i},x{i},mv{i},[],{Ts});
    EKF{i} = extendedKalmanFilter(@botStateFcn, @botMeasurementFcn);
    EKF{i}.State = x{i};
end

nloptions = nlmpcmoveopt;
nloptions.Parameters = {Ts};

figure;
hold on;
xlabel('x');
ylabel('y');
title('Real-Time x vs y Trajectory for All Robots');
grid on;

trajectoryPlots = gobjects(1, numRobots);
desiredTrajectoryPlots = gobjects(1, numRobots);
currentPosPlots = gobjects(1, numRobots);
frontVertexPlots = gobjects(1, numRobots);

colors = lines(numRobots);
for i = 1:numRobots
    trajectoryPlots(i) = plot(NaN, NaN, '-', 'LineWidth', 1.5, 'Color', colors(i, :));
    desiredTrajectoryPlots(i) = plot(NaN, NaN, '--', 'LineWidth', 1.5, 'Color', colors(i, :));
    currentPosPlots(i) = patch(NaN, NaN, colors(i, :), 'FaceColor', colors(i, :), 'EdgeColor', colors(i, :));
    frontVertexPlots(i) = plot(NaN, NaN, 'o', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
end

hbar = waitbar(0, 'Simulation Progress');
tic;

for ct = 1:(Duration / Ts)
    t = ct * Ts;
    elapsedTime = toc;
    waitbar(ct * Ts / Duration, hbar, ...
        sprintf('Simulation Progress: %.2f%%\nElapsed Time: %.2f seconds', ...
        (ct * Ts / Duration) * 100, elapsedTime));

    % Update planner position with current actual positions
    for i = 1:numRobots
        pos(i, :) = [x{i}(1), x{i}(2)];
    end

    if ~formation_achieved
        v = Laplac * pos - Laplac * finalPositions;
        error = norm(v, 'fro');
        if error < 1
            formation_achieved = true;
            disp("Formation achieved. Starting centroid motion.")
        end
        centroid = (ones(numRobots, numRobots) * pos) / numRobots;
        pos = pos + (kp * v - kc * centroid) * Ts;
    else
        drift = centroid_velocity * Ts;
        pos = pos + drift;
    end

    for i = 1:numRobots
        posHistory{i} = [posHistory{i}; pos(i, :)];
    end

    for i = 1:numRobots
        yref{i} = [pos(i, :), atan2(pos(i,2), pos(i,1))];
        yrefHistory{i} = [yrefHistory{i}; yref{i}];

        xk = correct(EKF{i}, y{i});
        [mv{i}, nloptions, info] = nlmpcmove(nlobj{i}, xk, mv{i}, yref{i}, [], nloptions);
        predict(EKF{i}, mv{i}, Ts);

        x{i} = botDT0(x{i}, mv{i}, Ts);
        y{i} = x{i} + randn(3, 1) * 0.01;

        xHistory{i} = [xHistory{i}, x{i}];
        mvHistory{i} = [mvHistory{i}, mv{i}];

        trajectoryXData = get(trajectoryPlots(i), 'XData');
        trajectoryYData = get(trajectoryPlots(i), 'YData');
        set(trajectoryPlots(i), 'XData', [trajectoryXData, x{i}(1)], 'YData', [trajectoryYData, x{i}(2)]);

        desiredTrajectoryXData = get(desiredTrajectoryPlots(i), 'XData');
        desiredTrajectoryYData = get(desiredTrajectoryPlots(i), 'YData');
        set(desiredTrajectoryPlots(i), 'XData', [desiredTrajectoryXData, yref{i}(1)], 'YData', [desiredTrajectoryYData, yref{i}(2)]);

        [triangleX, triangleY, frontX, frontY] = updateBotMarker(x{i}, baseSize);
        set(currentPosPlots(i), 'XData', triangleX, 'YData', triangleY);
        set(frontVertexPlots(i), 'XData', frontX, 'YData', frontY);
    end
end

% Close the waitbar
close(hbar);

% Extract constraints for manipulated variables (mv1 and mv2)
mv1Min = -0.3;
mv1Max = 0.3;
mv2Min = -1.5;
mv2Max = 1.5;

% Initialize figures for each bot
for botIdx = 1:numRobots
    % Retrieve the time vector and data for the current bot
    timeVector = 0:Ts:Duration;
    xHistoryBot = xHistory{botIdx};       % State history for botIdx
    yrefHistoryBot = yrefHistory{botIdx}; % Reference trajectory for botIdx
    mvHistoryBot = mvHistory{botIdx};     % Manipulated variables history for botIdx

    % Plot states (x, y, theta) for the current bot
    figure('Name', sprintf('State Plots for Bot %d', botIdx));

    % Plot x-position
    subplot(2, 2, 1);
    plot(timeVector, xHistoryBot(1, :), 'LineWidth', 1.5);
    hold on;
    plot(timeVector, yrefHistoryBot(:, 1), 'r--', 'LineWidth', 1.5); % Reference for x
    xlabel('Time (s)');
    ylabel('x');
    title(sprintf('x-position (Bot %d)', botIdx));
    legend('x', 'Reference');
    grid on;

    % Plot y-position
    subplot(2, 2, 2);
    plot(timeVector, xHistoryBot(2, :), 'LineWidth', 1.5);
    hold on;
    plot(timeVector, yrefHistoryBot(:, 2), 'r--', 'LineWidth', 1.5); % Reference for y
    xlabel('Time (s)');
    ylabel('y');
    title(sprintf('y-position (Bot %d)', botIdx));
    legend('y', 'Reference');
    grid on;

    % Plot theta
    subplot(2, 2, 3);
    plot(timeVector, xHistoryBot(3, :), 'LineWidth', 1.5);
    hold on;
    plot(timeVector, yrefHistoryBot(:, 3), 'r--', 'LineWidth', 1.5); % Reference for theta
    xlabel('Time (s)');
    ylabel('\theta');
    title(sprintf('Pose (theta) (Bot %d)', botIdx));
    legend('\theta', 'Reference');
    grid on;

    % Plot manipulated variables for the current bot
    figure('Name', sprintf('Manipulated Variables for Bot %d', botIdx));

    % Plot manipulated variable 1
    subplot(2, 1, 1);
    plot(timeVector, mvHistoryBot(1, :), 'LineWidth', 1.5);
    hold on;
    yline(mv1Min, 'r--', 'LineWidth', 1); % Min constraint
    yline(mv1Max, 'r--', 'LineWidth', 1); % Max constraint
    title(sprintf('Manipulated Variable 1 (mv1) (Bot %d)', botIdx));
    xlabel('Time (s)');
    ylabel('mv1');
    legend('mv1', 'Min constraint', 'Max constraint');
    grid on;

    % Plot manipulated variable 2
    subplot(2, 1, 2);
    plot(timeVector, mvHistoryBot(2, :), 'LineWidth', 1.5);
    hold on;
    yline(mv2Min, 'r--', 'LineWidth', 1); % Min constraint
    yline(mv2Max, 'r--', 'LineWidth', 1); % Max constraint
    title(sprintf('Manipulated Variable 2 (mv2) (Bot %d)', botIdx));
    xlabel('Time (s)');
    ylabel('mv2');
    legend('mv2', 'Min constraint', 'Max constraint');
    grid on;
end
