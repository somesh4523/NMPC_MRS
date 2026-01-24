function dxdt = botCT0(x, u)
    % Combined continuous-time dynamics for all robots
    numRobots = length(x) / 3; % Assuming each robot has 3 states: [x, y, theta]
    dxdt = zeros(size(x));

    for i = 1:numRobots
        % Extract the state and input for the i-th robot
        idx = 3 * (i - 1) + (1:3);
        uidx = 2 * (i - 1) + (1:2);
        xi = x(idx);
        ui = u(uidx);

        % Compute dynamics for the i-th robot
        v = ui(1);
        omega = ui(2);
        ptheta = xi(3);

        % Update the state derivatives
        dxdt(idx) = [v * cos(ptheta);
                     v * sin(ptheta);
                     omega];
    end
end
