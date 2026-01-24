% Design Nonlinear MPC controller
nx  = 3;       % Number of states
nmv = 2;       % Number of manipulated variables (control inputs)

% Create an NLMP controller object
msobj = nlmpcMultistage(p, nx, nmv);
Ts = 2e-3;     % Sample time
msobj.Ts = Ts;

% Set the state function and Jacobian functions
msobj.Model.StateFcn = "botStateFcn";
msobj.Model.StateJacFcn = "botStateJacFcn";
msobj.Model.ParameterLength = length(pvstate);

% Set up cost functions for each stage
for k = 1:p+1
    msobj.Stages(k).CostFcn = "botCostFcn";
    msobj.Stages(k).CostJacFcn = "botCostJacFcn";
    msobj.Stages(k).ParameterLength = length(pvcost);
end

% Set the control input bounds
msobj.ManipulatedVariables(1).Min = -0.4;
msobj.ManipulatedVariables(1).Max =  0.4;
msobj.ManipulatedVariables(2).Min = -0.4;
msobj.ManipulatedVariables(2).Max =  0.4;

% Simulation Data
simdata = getSimulationData(msobj);
simdata.StateFcnParameter = pvstate;
simdata.StageParameter = repmat(pvcost, p+1, 1);

% Set the Optimization Solver
msobj.Optimization.Solver = "cgmres";
msobj.Optimization.SolverOptions.StabilizationParameter = 1 / msobj.Ts;
msobj.Optimization.SolverOptions.MaxIterations = 10;
msobj.Optimization.SolverOptions.Restart = 3;
msobj.Optimization.SolverOptions.BarrierParameter = 1e-3;
msobj.Optimization.SolverOptions.TerminationTolerance = 1e-6;

% Generate code for MPC controller
x0 = [0; 0; 0];
u0 = [0; 0];
[coredata, onlinedata] = getCodeGenerationData(msobj, x0, u0, StateFcnParameter=pvstate, StageParameter=repmat(pvcost, p+1, 1));
buildMEX(msobj, "nlmpcControllerMEX", coredata, onlinedata);

% Update onlinedata with StageParameter
onlinedata.StateFcnParameter = simdata.StateFcnParameter;
onlinedata.StageParameter = simdata.StageParameter;
simdata.InitialGuess = onlinedata.InitialGuess;

% Run simulations with nlmpc, nlmpcCodeGeneration, and MEX controller
[~, ~, info1] = nlmpcmove(msobj, x0, u0);
[~, ~, info2] = nlmpcmoveCodeGeneration(coredata, x0, u0, onlinedata); 
[~, ~, info3] = nlmpcControllerMEX(x0, u0, onlinedata); 

% Plot results
figure;
for ix = 1:nx
    subplot(3, 2, ix); hold on; box on;
    plot(info1.Topt, info1.Xopt(:, ix), "-.");
    plot(info2.Topt, info2.Xopt(:, ix), "-x");
    plot(info3.Topt, info3.Xopt(:, ix), "-+");
    title(['State, x_' num2str(ix)]);
    xlabel("Time, s");
end

% Plot control inputs
for imv = 1:nmv
    subplot(3, 2, ix + imv); hold on; box on;
    plot(info1.Topt, info1.MVopt(:, imv), "-.");
    plot(info2.Topt, info2.MVopt(:, imv), "-x");
    plot(info3.Topt, info3.MVopt(:, imv), "-+");
    title(['Control, u_' num2str(imv)]);
    xlabel("Time, s");
end

% Add legend
legend("nlmpcmove", "nlmpcmoveCodeGeneration", "nlmpcControllerMEX");