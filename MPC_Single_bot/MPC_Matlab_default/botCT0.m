function dxdt = botCT0(x,u)

ptheta = x(3);

v = u(1);
omega = u(2);

dxdt = [v*cos(ptheta);
    v*sin(ptheta);...
    omega];
