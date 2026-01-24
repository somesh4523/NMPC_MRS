function xk1 = botDT0(xk, uk, Ts)
    % Combined discrete-time dynamics for all robots
    M = 10; % Number of Euler integration steps
    delta = Ts / M;
    xk1 = xk;

    for ct = 1:M
        xk1 = xk1 + delta * botCT0(xk1, uk);
    end
end
