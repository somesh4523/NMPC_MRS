% Define symbolic variables 
syms x [3 1] real        % State vector
syms tau [2 1] real      % Control input vector 
syms pvstate [1 1] real  % Additional parameters

% Compute the derivative of the state vector 
dxdt = botStateFcn(x, tau, pvstate);

% Compute the Jacobian matrices 
[A, B] = deal(jacobian(dxdt,x), jacobian(dxdt,tau));

% Create a MATLAB function for the Jacobian matrices
matlabFunction(A, B, ...
    'Vars', {x,tau,pvstate}, ...
    'File', 'botStateJacFcn')