% [K, V] = convhulln(X, TOL) returns the indices K of the points in X that
% comprise the facets of the convex hull of X and the volume V of convex 
% hull. 
%
% To handle degenerate data that lie in an affine subspace, the convex hull 
% is computed in the corresponsing affine subspaces whose dimension is 
% determined based on the data covariance matrix rank associated with 
% tolerance TOL.
%
% If data covariance has rank r and the convex hull of the projected point
% in the corresponding affine space has p facets, then K is a p-by-r
% matrix.
%
% For more information, see the reference page of convhulln and rank.
%
% Usage:
%   import geom.convhulln
%   [K, V, V2] = convhulln(X, OPTIONS, Name, Value)
% Input:
%   X: m Data points in n-dimensional space, m x n matrix
%   OPTIONS: convexhull options
%   Name,Value Pairs
%       'Tol': Rank tolerance, 1 x 1 scalar, default TOL = max(size(A)) * eps(norm(A)) 
% Output:
%   K: Point indices defining convex hull facets, p x r matrix
%   V: Convex hull volume in the original n-dimensional space , 1 x 1 scalar
%   V2: Convex hull volume in the r-dimensional affine subspace, 1 x 1 scalar 
% Example:
%   n = 5;
%   X = [ones(n,1), rand(n,2)];
%   [K, V, V2] = geom.convhulln(X);
%   figure; hold on; grid on; box on;
%   scatter3(X(:,1), X(:,2), X(:,3), [], 'r', 'filled');
%   for t = 1:size(K,1)
%       patch('Faces', K, 'Vertices', X, 'FaceColor', 'y', 'FaceAlpha', 0.5);  
%   end

function [K, V, V2] = convhulln(X, varargin)
% Author: Omur Arslan, omurarslan.phd@gmail.com
% Created: February 01, 2022
% Modified: February 01, 2022

    TOL =  {};
    OPTIONS = cell(1,0);
    for k = 1:(length(varargin))
       switch lower(varargin{k})
           case 'tol'
               TOL = {varargin{k+1}};
               k = k+1;
           otherwise
               OPTIONS{end+1} = varargin{k};
       end
    end
    
    n = size(X, 2);
    Sigma = cov(X);
    r = rank(Sigma, TOL{:});
    
    if (r == 0)
        K = 1;
        V = 0;
        V2 = 0;
        return;
    end
    
    if (r == n)
        [K, V] = convhulln(X, OPTIONS{:});
        V2 = V;
        return;
    end
    
     % Handle rank deficiency
    [V, ~] = eigs(Sigma, r);
    mu = mean(X,1);
    Y = X - mu;

    if (r == 1)
        % 1D convex hull
        Y = Y*V;
        [Ymin, Imin] = min(Y);
        [Ymax, Imax] = max(Y);
        K = [Imin, Imax; Imax Imin];
        V = 0;
        V = Ymax - Ymin;
    else
        Y = Y*V;
        [K, V2] = convhulln(Y);
        V = 0;
    end
    
end