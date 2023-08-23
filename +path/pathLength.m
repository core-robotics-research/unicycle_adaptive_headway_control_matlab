% Computes the path length of points of a piecewise linear path with
% respect to the start point 
%
% Usage:
%   [L, L2] = path.pathLength(X)
% Input:
%   X - Piecewise linear path in n dimensional space, m x n matrix
% Output:
%   L - Path length, 1 x 1 scalar
%   L2 - Path length of points wrt the start, m x 1 matrix
% Example:
%   X = rand(10,2);
%   [L, L2] = path.pathLength(X);

function [L, L2] = pathLength(X)
% Author: Omur Arslan, omurarslan.phd@gmail.com
% Created: February 19, 2019
% Modified: February 19, 2022

L2 = cumsum([0; sqrt(sum(diff(X, 1, 1).^2,2))]);
L = L2(end);