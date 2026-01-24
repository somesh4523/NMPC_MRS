clear all;
close all;
clc;


% Define system dimensions and parameters
numRobots = 6;
nx = numRobots * 3; % Total number of states
nu = numRobots * 2; % Total number of inputs
ny = nx; % Assuming full-state output

% Simulation parameters
Ts = 0.1; % Simulation timestep
Duration = 20; % Total simulation time
kp = 0.35; % Proportional gain
kc = 0.8; % Centroidal coupling gain
baseSize = 0.17; % Size of triangle marker for bots
centroid_velocity = [0.3, 0]; % Velocity of the centroid
formation_achieved = false;

% Initialize storage for history
posHistory = cell(1, numRobots);
postxHistory = cell(1, numRobots);
postrefHistory = cell(1, numRobots);
postmvHistory = cell(1, numRobots);

% Define formation and Laplacian matrix
finalPositions = [[(3^0.5)/2, 0.5]; [0, 1]; [-(3^0.5)/2, 0.5]; ...
                  [-(3^0.5)/2, -0.5]; [0, -1]; [(3^0.5)/2, -0.5]];
Laplac = ones(numRobots) - numRobots * eye(numRobots);
pos = rand(numRobots, 2); % Random initial positions
for i = 1:numRobots
    posHistory{i} = pos(i, :);
end

% Setup nonlinear MPC
nlobj = nlmpc(nx, ny, nu);
nlobj.Ts = Ts;
nlobj.PredictionHorizon = 10;
nlobj.ControlHorizon = 5;
nlobj.Model.StateFcn = "botDT0"; % Placeholder for dynamics
nlobj.Model.IsContinuousTime = false;
nlobj.Model.NumberOfParameters = 1;
nlobj.Model.OutputFcn = "botOutputFcn";
nlobj.Jacobian.OutputFcn = @(x, u, Ts) eye(nx);
nlobj.Weights.OutputVariables = repmat([3, 3, 0], 1, numRobots);
nlobj.Weights.ManipulatedVariables = ones(1, nu) * 0.3;
nlobj.Weights.ManipulatedVariablesRate = ones(1, nu) * 0.1;

% Define input constraints
for i = 1:numRobots
    idx = 2 * (i - 1) + (1:2);
    nlobj.MV(idx(1)).Min = -0.3;
    nlobj.MV(idx(1)).Max = 0.3;
    nlobj.MV(idx(2)).Min = -1.5;
    nlobj.MV(idx(2)).Max = 1.5;
    nlobj.MV(idx(1)).RateMin = -0.03;
    nlobj.MV(idx(1)).RateMax = 0.03;
    nlobj.MV(idx(2)).RateMin = -0.15;
    nlobj.MV(idx(2)).RateMax = 0.15;
end

% Initialize state and inputs
x = reshape([pos, zeros(numRobots, 1)]', [], 1);
y = x;
mv = zeros(nu, 1);
yref = zeros(1, ny);

xHistory = x;
mvHistory = mv;
yrefHistory = yref;

EKF = extendedKalmanFilter(@botStateFcn, @botMeasurementFcn);
EKF.State = x;

% Options for nonlinear MPC
nloptions = nlmpcmoveopt;
nloptions.Parameters = {Ts};

% Initialize plotting
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

% Simulation loop
hbar = waitbar(0, 'Simulation Progress');
tic;

for ct = 1:(Duration / Ts)
    t = ct * Ts;
    elapsedTime = toc;
    waitbar(ct * Ts / Duration, hbar, ...
        sprintf('Simulation Progress: %.2f%%\nElapsed Time: %.2f seconds', ...
        (ct * Ts / Duration) * 100, elapsedTime));

    if ~formation_achieved
        v_x = Laplac * pos(:, 1) - Laplac * finalPositions(:, 1);
        v_y = Laplac * pos(:, 2) - Laplac * finalPositions(:, 2);
        v = [v_x, v_y];
        error = norm(v, 'fro');
        if error < 1e-3
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

    yref = reshape([pos, zeros(numRobots, 1)]', 1, []);
    xk = correct(EKF, y);
    [mv, nloptions, info] = nlmpcmove(nlobj, xk, mv, yref, [], nloptions);
    predict(EKF, mv, Ts);

    x = botDT0(x, mv, Ts);
    y = x + randn(3*numRobots, 1) * 0.01;

    postx = reshape(x, 3, [])';
    postref = reshape(yref, 3, [])';
    postmv = reshape(mv, 2, [])';

    for i = 1:numRobots
        postxHistory{i} = [postxHistory{i}; postx(i, :)];
        postrefHistory{i} = [postrefHistory{i}; postref(i, :)];
        postmvHistory{i} = [postmvHistory{i}; postmv(i, :)];
    end

    for i = 1:numRobots
        trajectoryXData = get(trajectoryPlots(i), 'XData');
        trajectoryYData = get(trajectoryPlots(i), 'YData');
        set(trajectoryPlots(i), 'XData', [trajectoryXData, postx(i,1)], 'YData', [trajectoryYData, postx(i,2)]);

        desiredTrajectoryXData = get(desiredTrajectoryPlots(i), 'XData');
        desiredTrajectoryYData = get(desiredTrajectoryPlots(i), 'YData');
        set(desiredTrajectoryPlots(i), 'XData', [desiredTrajectoryXData, postref(i,1)], 'YData', [desiredTrajectoryYData, postref(i,2)]);

        [triangleX, triangleY, frontX, frontY] = updateBotMarker(postx(i,:), baseSize);
        set(currentPosPlots(i), 'XData', triangleX, 'YData', triangleY);
        set(frontVertexPlots(i), 'XData', frontX, 'YData', frontY);
    end
end

close(hbar);

% Constraints for manipulated variables
mv1Min = -0.3;
mv1Max = 0.3;
mv2Min = -1.5;
mv2Max = 1.5;

% Plot results for each robot
for botIdx = 1:numRobots
    timeVector = (0:(size(postxHistory{botIdx}, 1) - 1)) * Ts;

    % Trajectory plots
    figure('Name', sprintf('Trajectory for Bot %d', botIdx));
    subplot(2, 2, 1);
    plot(timeVector, postxHistory{botIdx}(:, 1), 'LineWidth', 1.5);
    hold on;
    plot(timeVector, postrefHistory{botIdx}(:, 1), 'r--', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('x');
    title(sprintf('X Position for Bot %d', botIdx));
    legend('Actual', 'Reference'); grid on;

    subplot(2, 2, 2);
    plot(timeVector, postxHistory{botIdx}(:, 2), 'LineWidth', 1.5);
    hold on;
    plot(timeVector, postrefHistory{botIdx}(:, 2), 'r--', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('y');
    title(sprintf('Y Position for Bot %d', botIdx));
    legend('Actual', 'Reference'); grid on;

    subplot(2, 2, 3);
    plot(timeVector, postxHistory{botIdx}(:, 3), 'LineWidth', 1.5);
    hold on;
    plot(timeVector, postrefHistory{botIdx}(:, 3), 'r--', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel('\theta');
    title(sprintf('Pose (\theta) (Bot %d)', botIdx));
    legend('\theta', 'Reference'); grid on;

    % Manipulated variable plots
    figure('Name', sprintf('Manipulated Variables for Bot %d', botIdx));
    subplot(2, 1, 1);
    plot(timeVector, postmvHistory{botIdx}(:, 1), 'LineWidth', 1.5);
    hold on;
    yline(mv1Min, 'r--', 'LineWidth', 1);
    yline(mv1Max, 'r--', 'LineWidth', 1);
    title(sprintf('Manipulated Variable 1 (mv1) (Bot %d)', botIdx));
    xlabel('Time (s)'); ylabel('mv1');
    legend('mv1', 'Min constraint', 'Max constraint'); grid on;

    subplot(2, 1, 2);
    plot(timeVector, postmvHistory{botIdx}(:, 2), 'LineWidth', 1.5);
    hold on;
    yline(mv2Min, 'r--', 'LineWidth', 1);
    yline(mv2Max, 'r--', 'LineWidth', 1);
    title(sprintf('Manipulated Variable 2 (mv2) (Bot %d)', botIdx));
    xlabel('Time (s)'); ylabel('mv2');
    legend('mv2', 'Min constraint', 'Max constraint'); grid on;
end
