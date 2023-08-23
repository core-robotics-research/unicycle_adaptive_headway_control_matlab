% Computes the signed Euclidean distance of a point to a polygon region, 
% and returns the closest point on the polygon boundary.  
%
% Here, the sign of distance is negative if the point is included in the 
% region of interest (i.e., inside, outside, or boundary), and the 
% magnitude of the distance equals to the minimum Euclidean distance 
% between the input point and any point on the polygon boundary.
%
% Input:  
%    X, Y - point coordinates
%    XV, YV - coordinates of polygon vertices
% Optional Name-Value Inputs
%   'ROI' - Region of Interest, 'inside' (default) | outside | boundary 
% Output: 
%    D - distance between points and the polygon 
%    CX, CY: coordinates of the closest points on the polygon to the input points
% Usage:
%   n = 10;
%   v = 8;
%   X = 2 * rand(n,1) - 1;
%   Y = 2 * rand(n,1) - 1;
%   th = linspace(0, 2*pi, v+1)';
%   XV = 0.5*cos(th);
%   YV = 0.5*sin(th);
%   [D, CX, CY] = polydist(X, Y, XV, YV);
%   figure, hold on;
%   patch(XV, YV, 'y');
%   plot(X, Y, 'ro');
%   plot(CX, CY, 'rx');
%   plot([X CX]', [Y CY]', 'b');
%   axis equal;
%   axis([-1 1 -1 1]);
%   box on;
%   grid on;

function [D, CX, CY] = polydist(X, Y, XV, YV, varargin)
% Author: Omur Arslan, omurarslan.phd@gmail.com
% Created: A while ago in 2018
% Modified: August 03, 2022

%Input validation
if ~isequal(size(X), size(Y))
    error('MATLAB:polydist:DimensionMismatch', 'Point Coordinates, X and Y, must have the same size!')
end
if ~isequal(size(XV), size(XV))
    error('MATLAB:polydist:DimensionMismatch', 'Polygon coordinates, XV and YV, must have the same size!');
end

roi = 'inside'; % Default Region of Interest
for k = 2:2:numel(varargin)
    if strcmpi(varargin{k-1}, 'ROI')
        roi = varargin{k};
    end
end

% Distance to empty set is infinity
if isempty(XV)
   D = Inf;
   CX = Inf;
   CY = Inf;
   return;
end

% Remove leading singleton dimensions
[X, nshifts] = shiftdim(X);
Y = shiftdim(Y);
XV = shiftdim(XV);
YV = shiftdim(YV);
% Relative coordinates of polygon rims
dX = circshift(XV,1) - XV; 
dY = circshift(YV,1) - YV;

% Vectorization 
n = size(X,1);  % number of points
v = size(XV,1); % number of polygon vertices
MdX = repmat(dX', [n, 1]);
MdY = repmat(dY', [n, 1]);
MXV = repmat(XV', [n, 1]);
MYV = repmat(YV', [n, 1]);
MX = repmat(X, [1, v]);
MY = repmat(Y, [1, v]);

% Compute the convex combination coefficients that define the closest
% points on polygon rims and input points 
A = (MdX.*(MX - MXV) + MdY.*(MY - MYV))./(MdX.^2 + MdY.^2);
A = max(min(A,1),0); 

% Minimum distance to polygon rims 
MD = (A.*MdX + MXV - MX).^2 + (A.*MdY + MYV - MY).^2;
% Minimum distance to the polygon
[D, I] = min(MD, [], 2);
D = sqrt(D);
I = sub2ind([n, v], (1:n)', I);
% Closest points on the polygon boundary
CX = A(I).*MdX(I) + MXV(I);
CY = A(I).*MdY(I) + MYV(I);

% Adjust the sign of distance for points in the polygon
IN = inpolygon(X, Y, XV, YV); 
D(IN) = -D(IN);

% Restore orginal dimensions
CX = shiftdim(CX, -nshifts);
CY = shiftdim(CY, -nshifts);
D = shiftdim(D, -nshifts);

% Distance sign correction according to the region of interest

switch lower(roi)
    case 'inside'
        % Do Nothing 
    case 'outside'
        D = -D;
    case 'boundary'
        D = abs(D);
    otherwise
        error('Unknown region of interest!');
end
