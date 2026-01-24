pos = randi(30,6,2);
final = [[(3^0.5)/2, 0.5];
    [0,1];
    [-(3^0.5)/2, 0.5];
    [-(3^0.5)/2, -0.5];
    [0,-1];
    [(3^0.5)/2, -0.5]];
quad = [[-1/8,-1/8];[-1/8,1/8];[1/8,1/8];[1/8,-1/8];[-1/8,-1/8];];
dt = 0.03;
kp = 0.35;
kc = 0.8;
N = 6;
pos1 = [];
pos2 = [];
pos3 = [];
pos4 = [];
pos5 = [];
pos6 = [];
Laplac = ones(N) - N*eye(N);
q1 = [];
q2 = [];
q3 = [];
q4 = [];
q5 = [];
q6 = [];
centroid_velocity = [0.1, 0]; % Velocity of the centroid in the positive x-direction

formation_achieved = false;
t = 0;

while t <= 40
    if ~formation_achieved
        % Formation control phase
        v = Laplac * pos - Laplac * final;
        error = norm(v, 'fro'); % Calculate error as Frobenius norm of the velocity vector
        if error < 1e-3 % Threshold for considering formation achieved
            formation_achieved = true;
            disp("Formation achieved. Starting centroid motion.");
        end
        pos = pos + (kp * v - kc * (ones(6, 6) * pos / 6)) * dt;
    else
        % Centroid drift phase
        centroid = (ones(6, 6) * pos) / 6; % Calculate current centroid
        drift = centroid_velocity * dt; % Drift due to centroid velocity
        pos = pos + drift; % Update positions to maintain formation with centroid drift
    end

    % Plotting
    qpos1 = quad + pos(1, :);
    delete(q1);
    q1 = plot(qpos1(:, 1), qpos1(:, 2), '-r');
    plot(pos(1, 1), pos(1, 2), 'r.');
    hold on;
    qpos2 = quad + pos(2, :);
    delete(q2);
    q2 = plot(qpos2(:, 1), qpos2(:, 2), '-b');
    plot(pos(2, 1), pos(2, 2), 'b.');
    hold on;
    qpos3 = quad + pos(3, :);
    delete(q3);
    q3 = plot(qpos3(:, 1), qpos3(:, 2), '-g');
    plot(pos(3, 1), pos(3, 2), 'g.');
    hold on;
    qpos4 = quad + pos(4, :);
    delete(q4);
    q4 = plot(qpos4(:, 1), qpos4(:, 2), '-k');
    plot(pos(4, 1), pos(4, 2), 'k.');
    hold on;
    qpos5 = quad + pos(5, :);
    delete(q5);
    q5 = plot(qpos5(:, 1), qpos5(:, 2), '-m');
    plot(pos(5, 1), pos(5, 2), 'm.');
    hold on;
    qpos6 = quad + pos(6, :);
    delete(q6);
    q6 = plot(qpos6(:, 1), qpos6(:, 2), '-c');
    plot(pos(6, 1), pos(6, 2), 'c.');
    hold on;
    pause(dt / 10);

    t = t + dt;
end
