function positions = generatePolygonVertices(numSides, radius)
    % Generate vertices of a regular n-sided polygon
    angles = linspace(0, 2*pi, numSides + 1);
    x = radius * cos(angles(1:end-1));
    y = radius * sin(angles(1:end-1));
    positions = [x', y'];
end