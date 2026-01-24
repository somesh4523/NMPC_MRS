
pvstate = 0.2;
% Stage Parameters
Sf = [10; 10; 20];  % Terminal Weight Matrix.
Q  = [10; 10; 30];  % State Weight Matrix.
R  = [0.1; 0.1];            % Control Weighting Matrix.

p  = 50;                    % Prediction Horizon.


% Combine stage parameters into a column array.
pvcost = [Sf; Q; R; p];


