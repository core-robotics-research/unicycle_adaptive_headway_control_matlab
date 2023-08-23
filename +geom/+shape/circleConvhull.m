% Example:
%   r = 1;
%   n = 40;
%   [X, K, V] = geom.shape.circleConvhull(d,n);
%   figure; hold on; box on; grid on; axis equal;
%   patch('Faces', K, 'Vertices', X, 'FaceColor', 'r', 'FaceAlpha', 0.5);  

function [X, K, V] = circleConvhull(r, n)

    X = geom.shape.circlePoint(double(r), n);
    [K, V] = convhulln(X);
    
end