function dxdt = botCT0(x,mv)

ptheta = x(3);

v = mv(1);
omega = mv(2);

dxdt = [v*cos(ptheta);
    v*sin(ptheta);...
    omega];
