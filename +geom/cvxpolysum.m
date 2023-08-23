% Return minkoski sum of two convex polygons 
% Input: 
%   X1, Y1  : Vertex Coordinates of a convex polygon
%   X2, Y2  : Vertex Coordinates of a convex polygon
% Output:
%   XS, YS  : Vertex coordinates of Minskowski sum
% Usage:
%   X1 = rand(3,1);
%   Y1 = rand(3,1);
%   X2 = rand(3,1);
%   Y2 = rand(3,1);
%   [XS, YS] =cvxpolysum(X1, Y1, X2, Y2);
%   patch(X1, Y1, 'r', 'EdgeColor', 'r', 'FaceColor', 'none');
%   patch(X2, Y2, 'b', 'EdgeColor', 'b', 'FaceColor', 'none');
%   patch(XS, YS, 'g', 'EdgeColor', 'g', 'FaceColor', 'none');


function [XS, YS] = cvxpolysum(X1, Y1, X2, Y2)

    X1 = reshape(X1, 1, []);
    Y1 = reshape(Y1, 1, []);
    X2 = reshape(X2, [], 1);
    Y2 = reshape(Y2, [], 1);
    n1 = length(X1);
    n2 = length(X2);
    X = repmat(X1, [1, n2]) + repmat(X2, [n1, 1]);
    Y = repmat(Y1, [1, n2]) + repmat(Y2, [n1, 1]);
    K = convhull(X(:), Y(:));
    
    XS = X(K);
    YS = Y(K);
    
end