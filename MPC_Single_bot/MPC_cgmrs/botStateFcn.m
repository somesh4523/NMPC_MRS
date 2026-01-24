function dxdt = botStateFcn(x, tau, pvstate)
    % botStateFcn
    %   dxdt = botStateFcn(x, tau)
    %
    %   State-space function for the TurtleBot model. The states x are
    %   [x_r; y_r; theta_r], and the control input is [v; omega].
    
    % States
    x_r = x(1);
    y_r = x(2);
    theta_r = x(3);

    % Control inputs
    ux = tau(1);
    uy = tau(2);

    l = pvstate;

    v = ux*cos(theta_r) + uy*sin(theta_r);
    omega = (-1/l)*(ux*sin(theta_r) - uy*cos(theta_r));

    % Model equations (differential equations)
    dxdt = [v * cos(theta_r) - l*omega*sin(theta_r); 
            v * sin(theta_r) + l*omega*cos(theta_r); 
            omega];
end
