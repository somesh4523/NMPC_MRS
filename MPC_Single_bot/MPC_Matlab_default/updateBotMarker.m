function [triangleX, triangleY, frontX, frontY] = updateBotMarker(botPosition, baseSize)
    % Update the bot's marker and front-facing vertex based on its position and orientation
    %
    % Inputs:
    % - botPosition: [x; y; theta] (current position and orientation of the bot)
    % - baseSize: Size of the triangle marker (defines its scale)
    %
    % Outputs:
    % - triangleX: x-coordinates of the triangle vertices
    % - triangleY: y-coordinates of the triangle vertices
    % - frontX: x-coordinate of the front-facing vertex
    % - frontY: y-coordinate of the front-facing vertex

    % Extract position and orientation
    x = botPosition(1);  % x-coordinate of the bot
    y = botPosition(2);  % y-coordinate of the bot
    theta = botPosition(3);  % Orientation (theta) of the bot

    % Define the triangle vertices relative to the bot's center
    % The front vertex will align with the bot's heading
    halfBase = baseSize / 2;  % Half of the base for back vertices
    triangleLocalX = [baseSize, -halfBase, -halfBase];
    triangleLocalY = [0, -halfBase, halfBase];

    % Apply rotation matrix to adjust triangle vertices based on orientation
    rotationMatrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    rotatedTriangle = rotationMatrix * [triangleLocalX; triangleLocalY];

    % Translate the rotated triangle to the bot's current position
    triangleX = rotatedTriangle(1, :) + x;
    triangleY = rotatedTriangle(2, :) + y;

    % The front vertex is the first vertex in the local triangle definition
    frontX = triangleX(1);  % x-coordinate of the front-facing vertex
    frontY = triangleY(1);  % y-coordinate of the front-facing vertex
end
