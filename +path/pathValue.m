% Computes the points of a unit-speed piecewise linear path at given path parameters  
%
% Usage:
%   Y = path.pathValue(X, t)
% Input:
%   X - Piecewise linear path points in n-dimensional space, m x n matrix
%   t - Path parameters, 1 x k vector 
% Output:
%   Y - Path points, k x n matrix
% Example:
%   X = rand(4,2);
%   L = path.pathLength(X);
%   t = linspace(0, L, 20);
%   t = t(randperm(length(t)));
%   Y = path.pathValue(X, t);
%   figure; hold on; box on; grid on; axis equal; 
%   plot(X(:,1), X(:,2), 'k-');
%   scatter(Y(:,1), Y(:,2), [], 'r', 'filled');

function Y = pathValue(X, t)
% Author: Omur Arslan, omurarslan.phd@gmail.com
% Created: August 31, 2022
% Modified: August 31, 2022
% Modified: September 21, 2022 (A bug associated with invalid path parameter is corrected) 

    t = reshape(t, [], 1);
    n = size(X,2);
    Y = NaN(numel(t), size(X,2));
    
    [L, L2] = path.pathLength(X);

    IN = (t >= 0) & (t <= L);
    t = t(IN);
    
    if isempty(t)
        return;
    end
     
    [J, I] = find(diff([t >= L2', zeros(numel(t), 1)]', 1, 1));
    
    X = [X; X(end,:)];
    L2 = [L2; L2(end,:)];
    w = (t - L2(J))./(L2(J+1) - L2(J));
    w = min(max(w, 0), 1);
    YIN = repmat(1-w, 1, n).*X(J,:) + repmat(w, 1, n).* X(J+1,:);
    Y(IN,:) = YIN;

end