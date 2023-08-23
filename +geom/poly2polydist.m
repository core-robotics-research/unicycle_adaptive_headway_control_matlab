% Computes the signed Euclidean distance of a polygon inside to a polygon 
% region and also returns a pair of points of the polygons achieving their 
% distance. 
% 
% Here, the sign of the Euclidean distance between a point from the first polygon inside 
% and a point from the second polygon region is defined to be negative if the first 
% polygon inside point is in the second polygon region. Hence, the signed
% Euclidean distance between a polygon inside and a polygon region is
% defined as the minimum signed Euclidean distance between their points.
%
% Note that the signed Euclidean distance between a polygon inside and a 
% polygon region is achieved by a vertex of one of the polygons and its 
% closest point on the other polygon. Accordingly, the distance between 
% two polygons can be computed using the distance of vertices of one 
% polygon to the other polygon body.
%
% Warning: While the nonnegative signed distance calculation is correct, 
% the negative signed distance is not correct and can be larger than the
% actual signed distance. For single vertex overlap, the negatie signed
% distance calsulation is still correct.
%
% Input:
%   X1, Y1, X2, Y2 : vertex coordinates of the input polygons
% Optional Name-Value Inputs
%   'ROI' - Region of Interest, 'inside' (default) | outside 
% Output:
%   D : distance between the input polygons
%   CX1, CY1, CX2, CY2 : coordinate of a pair of points  achieveing the
%                        minimum distance between the polygons
% Usage : 
%   n1 = 3;
%   n2 = 3;
%   X1 = rand(n1,1) - 1; Y1 = rand(n1,1) - 1;
%   X2 = rand(n2,1) + 1; Y2 = rand(n2,1) + 1;
%   [D, CX1, CY1, CX2, CY2] = poly2polydist(X1, Y1, X2, Y2);
%   figure, hold on;
%   patch(X1, Y1, 'r');
%   patch(X2, Y2, 'b');
%   plot([CX1, CX2], [CY1, CY2], '-ko', 'LineWidth', 2, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
%   axis equal;
%   box on;
%   grid on;

function [D, CX1, CY1, CX2, CY2] = poly2polydist(X1, Y1, X2, Y2, varargin)
% Author: Omur Arslan, omurarslan.phd@gmail.com
% Created: A while ago in 2018
% Modified: August 03, 2022

    roi = 'inside'; % Default Region of Interest
    for k = 2:2:numel(varargin)
        if strcmpi(varargin{k-1}, 'ROI')
            roi = varargin{k};
        end
    end

    % Compute the distance of the vertices of the first polygon to the second polybody
    [D12, CX12, CY12] = geom.polydist(X1, Y1, X2, Y2, 'ROI', roi);
    % Compute the distance of the vertices of the second polygon to the first polybody
    [D21, CX21, CY21] = geom.polydist(X2, Y2, X1, Y1, 'ROI', 'inside');
    
    % Find the pair of points achiving the minimum distance
    [Dmin12, I12] = min(D12);
    [Dmin21, I21] = min(D21);
    if (Dmin12 < Dmin21)
        D = Dmin12;
        CX1 = X1(I12);
        CY1 = Y1(I12);
        CX2 = CX12(I12);
        CY2 = CY12(I12);
    else
        D = Dmin21;
        CX1 = CX21(I21);
        CY1 = CY21(I21);
        CX2 = X2(I21);
        CY2 = Y2(I21);
    end

end 
