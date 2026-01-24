function helperPlotResults(xHistory, uHistory, Ts, xf, LegendControl)
% HELPERPLOTRESULTS
%   This helper function plots the closed-loop simulation results, 
%   including state and control trajectories.
%
%   Inputs:
%       xHistory - Matrix of states (rows correspond to time steps)
%       uHistory - Matrix of control inputs (rows correspond to time steps)
%       Ts - Sample time in seconds
%       xf - Desired final state (optional)
%       LegendControl - Control legend visibility ("show" | "hide")

% Argument validation
arguments
    xHistory = []
    uHistory = []
    Ts = []
    xf = []
    LegendControl {mustBeMember(LegendControl, {'show', 'hide'})} = "hide"
end

% Check if Ts is specified for time vector generation
if isempty(Ts)
    error('Sample time Ts must be provided.');
end

% Generate the time vectors
timeX = (0:(size(xHistory, 1)-1)) * Ts;
timeMv = (0:(size(uHistory, 1)-1)) * Ts;

% Number of states and control inputs
nx = size(xHistory, 2);
nmv = size(uHistory, 2);

% Define the state and control labels
stateLabels = {'X Position', 'Y Position', 'Orientation (\theta)'};
controlLabels = {'Linear Velocity (v)', 'Angular Velocity (\omega)'};

% Plot the states
figure;
for ix = 1:nx
    subplot(3, 2, ix); hold on; box on;
    
    % Plot the state trajectory
    plot(timeX, xHistory(:, ix), 'DisplayName', stateLabels{ix}, 'LineWidth', 1.5);
    
    % Plot the desired state if xf is provided
    if ~isempty(xf) && length(xf) >= ix
        plot(timeX, ones(size(timeX)) * xf(ix), '--', 'DisplayName', ['Target ' stateLabels{ix}], 'LineWidth', 1.2);
    end

    % Title and labels
    title(stateLabels{ix}, 'Interpreter', 'latex');
    xlabel('Time (s)', 'Interpreter', 'latex');
    ylabel(stateLabels{ix}, 'Interpreter', 'latex');
    
    % Show legend based on LegendControl
    if LegendControl == "show"
        legend('show', 'Location', 'best');
    else
        legend('hide');
    end
end

% Plot the control inputs
for imv = 1:nmv
    subplot(3, 2, nx + imv); hold on; box on;
    
    % Plot the control input trajectory
    plot(timeMv, uHistory(:, imv), 'DisplayName', controlLabels{imv}, 'LineWidth', 1.5);
    
    % Title and labels
    title(controlLabels{imv}, 'Interpreter', 'latex');
    xlabel('Time (s)', 'Interpreter', 'latex');
    ylabel(controlLabels{imv}, 'Interpreter', 'latex');
    
    % Show legend based on LegendControl
    if LegendControl == "show"
        legend('show', 'Location', 'best');
    else
        legend('hide');
    end
end

% Adjust layout for better visibility
sgtitle('Closed-Loop Simulation Results', 'FontWeight', 'bold');
end
