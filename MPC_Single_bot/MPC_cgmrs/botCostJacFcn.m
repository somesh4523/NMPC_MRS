function [Gx, Gmv] = botCostJacFcn(stage, x, u, pvcost)
    % botCostJacFcn computes the Jacobian of the cost function with respect to x and u.
    %   stage - Current stage (1 through p+1)
    %   x - Current state vector [x_r; y_r; theta_r]
    %   u - Control input [v; omega]
    %   pvcost - Stage parameters containing [xf; Sf; Q; R; p]

    % Extract Desired State
    xd = 1;
    yd = 1;
    xf = [xd;yd;atan2((x(2)-yd), (x(1)-xd))];

    % Extract and Initialize Weighting Matrices
    Sf = diag(pvcost(1:3));                 % Terminal Weight Matrix
    Q  = diag(pvcost(4:6));                 % State Weight Matrix
    R  = diag(pvcost(7:8));               % Control Weight Matrix
    p  = pvcost(9);                        % Prediction Horizon

    % Compute Jacobians
    if stage == p + 1
        % Terminal stage
        Gx = Sf * (x - xf);
        Gmv = zeros(2, 1);
    else
        % Intermediate stages
        Gx = Q * (x - xf);
        Gmv = R * u;
    end
end
