% A Linear Path Class for representing piecewise linear paths parametrized
% by arc length
%
% Example:
%   n = 10;
%   d = 2;
%   X = rand(n,d);
%   mypath = path.Path(X);
%   L = mypath.Length;

classdef Path <  matlab.mixin.Copyable
    
    % Essential Path Properties
    properties
        Points % Path Points
    end

    % Dependent Path Properties
    properties(Dependent)
        Length % Path Length
        Interval % Path Interval
        Domain % Path Domain (Interval)
        Dimension % Path Dimension
    end
    
    methods
        function obj = Path(X)
            obj.Points = X;
        end

        function value = get.Length(obj)
            value = path.pathLength(obj.Points);
        end

        function value = get.Interval(obj)
            value = [0, obj.Length];
        end

        function value = get.Domain(obj)
            value = [0, obj.Length];
        end

        function value = get.Dimension(obj)
            value = size(obj.Points, 2);
        end
        
    end

    methods

        function Y = value(obj, t)
            Y = path.pathValue(obj.Points, t);    
        end

        % function dX = derivative(obj, t, k)
        %     if nargin < 3
        %         k = 1;
        %     end
        %     dX = path.pathDerivative(obj.Points, t, k);
        % end

        function [L, L2] = length(obj)
            [L, L2] = path.pathLength(obj.Points); 
        end
        
        % function [D, C] = dist(obj, X)
        %    [D, C] = path.pathDist(obj.Points, X); 
        % end
        
        % function Y = goal(obj, X, r)
        %     Y = path.pathGoal(obj.Points, X, r);
        % end
        
        function h = plot(obj, t, varargin)
            h = plot(obj.Points(:,1), obj.Points(:,2), varargin{:});
        end

        function h = nplot(obj, t, s, varargin)
            t = reshape(t, [], 1);
            s = reshape(s, [], 1);
            X = obj.value(t);
            dX = obj.derivative(t);
            Y = [-s.*dX(:,2), s.*dX(:,1)];
            
            h = quiver(X(:,1), X(:,2), Y(:,1), Y(:,2), 'off', 'ShowArrowHead','off', varargin{:});
            %h = plot([X(:,1), X(:,1) + Y(:,1)]', [X(:,2), X(:,2) + Y(:, 2)]', varargin{:});

        end
        
        

    end
    
end
