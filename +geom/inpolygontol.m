% Determines if given 2D points are contained in a 2D polygon
%
% Syntax:
%   [IN, ON] = inpolygontol(X, Y, XV, YV, tol)
% Input:  
%   X, Y - Point coordinates, n x 1 vertices
%   XV, YV - Polygon vertices, m x 1 vertices
%   tol - Inclusion tolerance, a scalar
% Output: 
%   IN - Logical inclusion indicator, n x 1 vector
%   ON - Logical boundary indicator, n x 1 vector
% Usage:
%   n = 100;
%   X = 2*rand(n,1)-1;
%   Y = 2*rand(n,1)-1;
%   m = 7;
%   th = linspace(0, 2*pi, m)';
%   XV = rand(m,1).*cos(th);
%   YV = rand(m,1).*sin(th);
%   tol = 1e-1;
%   [IN, ON] = inpolygontol(X, Y, XV, YV, tol);
%   figure; hold on; axis equal;
%   patch('XData', XV, 'YData', YV, 'FaceColor', 'y');
%   scatter(X(~IN), Y(~IN), [], 'b', 'filled');
%   scatter(X(IN), Y(IN), [], 'g', 'filled');
%   scatter(X(ON), Y(ON), [], 'r', 'filled');

function [IN, ON] = inpolygontol(X, Y, XV, YV, tol)
% Author: Omur Arslan, omurarslan.phd@gmail.com
% Date: November 07, 2018

D = geom.polydist(X, Y, XV, YV);

IN = D <= tol;
ON = abs(D) <= tol;