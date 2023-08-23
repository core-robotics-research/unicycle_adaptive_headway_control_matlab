% 2D Polygonal Map Class for World Modelling in Robotics
% 
% Usage:
%   map = world.PolygonMap2D(boundary, obstacles)
% Input:
%   boundary: Map boundary, vertices of a simple polygon, n x 2 matrix
%   obstacles: Map obstacles, a cell array of simple polygon vertices, m x 1 cell of r x 2 matrices  
%
% Example:
%   boundary = [0 0; 0 10; 10 10; 10 0; 0 0];
%   obstacles{1} = [2 2; 2 3; 3 3; 3 2; 2 2];
%   obstacles{2} = [6 6; 6 7; 8 7; 8 6; 6 6];
%   map = world.PolygonMap2D(boundary, obstacles);
%   h = show(map);

classdef PolygonMap2D < matlab.mixin.Copyable
% Authors:  Omur Arslan, omurarslan.phd@gmail.com
%           Aykut Isleyen,
% Created: August 09, 2018
% Modified: August 02, 2022

    % Read-Only Object Properties
    properties(GetAccess='public', SetAccess='private')
        Boundary % Polygonal workspace boundary in the map coordinates
        Obstacle % Polygonal workspace obstacle list in the map coordinates
    end
    
    % Object Properties
    properties(GetAccess='public', SetAccess='public')
        MapOriginInWorld % Bottom-left map corner position [x,y] in the world coordinates, [0 0] (default) | two-element vector
        LocalOriginInMap % Local coordinate origin in the map coordinate system, [0, 0] (default) | two-element vector
    end

    % Dependent Object Properties
    properties(Dependent=true, SetAccess='public')
        LocalOriginInWorld % Local coordinate frame origin in the world coordinates, [0, 0] (default) | two-element [x, y] vector
    end

    % Dependent Read-Only Object Properties
    properties(Dependent=true, SetAccess='private')
        XWorldLimits % Minimum and maximum map range values of x-coordinates, [min max] vector
        YWorldLimits % Minimum and maximum map range values of y-coordinates, [min max] vector
        XLocalLimits % Minimum and maximum values of x-coordinates in local frame, [min max] vector
        YLocalLimits % Minimum and maximum values of y-coordinates in local frame, [min max] vector
        Width % Map width, a positive scalar
        Height % Map height, a positive scalar
    end

    % Class Constructor
    methods
        function map = PolygonMap2D(boundary, obstacles)
        % 2D Polygonal map class constructor
            if nargin ~= 0
                map.Boundary = boundary;
                map.Obstacle = obstacles;
            end

            map.MapOriginInWorld = [0, 0];
            map.LocalOriginInMap = [0, 0];

        end
    end

    % Get & Set Methods for Object Properties
    methods
        
        function value = get.Boundary(map)
            value = map.Boundary;
        end

        function value = get.Obstacle(map)
            value = map.Obstacle;
        end

        function value = get.MapOriginInWorld(map)
            value = map.MapOriginInWorld;
        end

        function set.MapOriginInWorld(map, value)
            map.MapOriginInWorld = value;
        end

        function value = get.LocalOriginInMap(map)
            value = map.LocalOriginInMap;
        end

        function set.LocalOriginInMap(map, value)
            map.LocalOriginInMap = value;
        end

        function value = get.LocalOriginInWorld(map)
            value = map.map2world(map.LocalOriginInMap);
        end

        function set.LocalOriginInWorld(map, value)
            map.LocalOriginInMap = map.world2map(value);
        end

        function value = get.XWorldLimits(map)
        % Returns the minimum and maximum map limits along x-axis
        %
        % Syntax:
        %   XLim = XWorldLimits(map)
        % Input:
        %   map - 2D polygonal map, a PolygonMap2D object
        % Output:
        %   XLim - x-coordinates limits, [min, max] vector 

            boundary = map.map2world(map.Boundary);  
            value = [min(boundary(:,1)), max(boundary(:,1))];
        
        end
        
        function value = get.YWorldLimits(map)
        % Returns the minimum and maximum map limits along y-axis
        %
        % Syntax:
        %   YLim = YWorldLimits(map)
        % Input:
        %   map - 2D polygonal map, a PolygonMap2D object
        % Output:
        %   YLim - y-coordinates limits, [min, max] vector 

            boundary = map.map2world(map.Boundary);
            value = [min(boundary(:,2)), max(boundary(:,2))]; 
        
        end

        function value = get.XLocalLimits(map)

            boundary = map.map2local(map.Boundary);
            value = [min(boundary(:,1)), max(boundary(:,1))];

        end

        function value = get.YLocalLimits(map)

            boundary = map.map2local(map.Boundary);
            value = [min(boundary(:,2)), max(boundary(:,2))];

        end

        function value = get.Width(map)
            value = diff(map.XWorldLimits);
        end

        function value = get.Height(map)
            value = diff(map.YWorldLimits);
        end
        
    end

    % Object Functions - Coordinate Transformations
    methods
        function position = world2map(map, position)
            position = position - repmat(map.MapOriginInWorld, [size(position,1), 1]);
        end

        function position = map2world(map, position)
            position = position + repmat(map.MapOriginInWorld, [size(position,1), 1]);
        end

        function position = local2map(map, position)
            position = position + repmat(map.LocalOriginInMap, [size(position,1), 1]);
        end
   
        function position = map2local(map, position)
            position = position - repmat(map.LocalOriginInMap, [size(position,1), 1]);
        end

        function position = local2world(map, position)
            position = map.map2world(map.local2map(position));
        end

        function position = world2local(map, position)
            position = map.map2local(map.world2map(position));
        end

    end

    % Object Functions - Collision Functions
    methods

        function I = isfree(map, X)
          [IN, ON] = inpolygon(X(:,1), X(:,2), map.Boundary(:,1), map.Boundary(:,2));
          I = and(IN, ~ON);
          for k = 1:numel(map.Obstacle)
            IN = geom.inpolygontol(X(I,1), X(I,2), map.Obstacle{k}(:,1), map.Obstacle{k}(:,2), 1e-10);
            I(I,1) = not(IN);
          end
        end

        function [D, X] = dist2coll(map, position, varargin)

            % Default Settings
            frame = 'world';

            for k = 2:2:numel(varargin)
                switch lower(varargin{k-1})
                    case 'frame'
                        frame = varargin{k};
                end
            end

            position = reshape(position, [], 2);
            position = map.frame2map(position, frame);

            [D, CX, CY] = geom.polydist(position(:,1), position(:,2), map.Boundary(:,1), map.Boundary(:,2));
            D = - D;
            
            for k = 1 : numel(map.Obstacle)
                [Dtemp, CXtemp, CYtemp] = geom.polydist(position(:,1), position(:,2), map.Obstacle{k}(:,1), map.Obstacle{k}(:,2));
                I = Dtemp < D; 
                D(I) = Dtemp(I);
                CX(I) = CXtemp(I);
                CY(I) = CYtemp(I);
            end

            X = map.map2frame([CX(:), CY(:)], frame);

        end

        function [D, XP, XM] = polydist2coll(map, polygon, varargin)

            % Default Settings
            frame = 'world';            
            for k = 1:2:numel(varargin)
                switch lower(varargin{k})
                    case 'frame'
                        frame = varargin{k+1};
                end
            end

            polygon = map.frame2map(polygon, frame);
            [D, XP1, XP2, XM1, XM2] = geom.poly2polydist(polygon(:,1), polygon(:,2), map.Boundary(:,1), map.Boundary(:,2), 'ROI', 'outside');

            for k = 1 : numel(map.Obstacle)
                [Dtemp, XP1temp, XP2temp, XM1temp, XM2temp] = geom.poly2polydist(polygon(:,1), polygon(:,2), map.Obstacle{k}(:,1), map.Obstacle{k}(:,2));
                if Dtemp < D 
                    D = Dtemp;
                    XP1 = XP1temp;
                    XP2 = XP2temp;
                    XM1 = XM1temp;
                    XM2 = XM2temp;
                end
            end
            XP = map.map2frame([XP1, XP2], frame);
            XM = map.map2frame([XM1, XM2], frame);

        end

        function occgrid = binaryOccupancyGrid(map, resolution)

            boundaryThickness = 1;
            nRows = ceil(map.Height*resolution) + 2*boundaryThickness;
            nCols = ceil(map.Width*resolution) + 2*boundaryThickness;

            polygon = ceil(map.Boundary*resolution) + boundaryThickness;
            occupancyMatrix = 1 - poly2mask(polygon(:,1), polygon(:,2), nRows, nCols);
            for k = 1:numel(map.Obstacle)
                polygon = ceil(map.Obstacle{k}*resolution) + boundaryThickness;
                mask = poly2mask(polygon(:,1), polygon(:,2), nRows, nCols);
                occupancyMatrix(mask) = 1;
            end

            occgrid = world.BinaryOccupancyGrid2D(flipud(occupancyMatrix), resolution);
            %occgrid.LocalOriginInMap = map.LocalOriginInMap + boundaryThickness/resolution;
            occgrid.MapOriginInWorld = map.MapOriginInWorld - boundaryThickness/resolution;

        end
        
    end

    % Private Object Functions

    methods(Access='private')

        function position = map2frame(map, position, frame)
            switch lower(frame)
                case 'world'
                    position = map.map2world(position);
                case 'local'
                    position = map.map2local(position);
                case 'map'
                    % Do nothing;
                otherwise
                    error('Unknown coordinate frame!');
            end
        end

        function position = frame2map(map, position, frame)
            switch lower(frame)
                case 'world'
                    position = map.world2map(position);
                case 'local'
                    position = map.local2map(position);
                case 'map'
                    % Do nothing;
                otherwise
                    error('Unknown coordinate frame!');
            end
        end

    end

    % Visualization Functions
    methods

        function h = patch(map, varargin)
        % Patch visualization of map boundary (with no facecolor) and obstacles (with no edgecolor)

            h = zeros(1, 1 + numel(map.Obstacle));
            
            for k = 1:numel(map.Obstacle)
                obstacle = map.map2world(map.Obstacle{k});
                h(k) = patch('XData', obstacle(:,1), 'YData', obstacle(:,2), varargin{:}, 'EdgeColor', 'none');
            end

            boundary = map.map2world(map.Boundary);
            h(end) = patch('XData', boundary(:,1), 'YData', boundary(:,2), varargin{:}, 'FaceColor', 'none');

        end


        function h = patchConfSpace(map, inflationRadius, varargin)
        % Patch plot of configuration space obstacles
        % Input:
        %   map - a PolygonMap2D object
        %   inflationRadius - Inflation radius for configuration space obstacles, 1x1 positive scalar
        % Optional Name-Value Inputs:
        %       Name, Value: Any name-value input pair for a patch plot, see patch   

            angles = linspace(0,2*pi,60)'; % Angle samples
            inflationBall = inflationRadius*[cos(angles), sin(angles)];
            
            h = [];
            % Augmented Obstacles
            for k = 1:numel(map.Obstacle)
               [Xa, Ya] = geom.cvxpolydilate(map.Obstacle{k}(:,1), map.Obstacle{k}(:,2), inflationRadius);
               h(end+1) = patch('XData', Xa, 'YData', Ya, varargin{:});
               for v = 1:size(map.Obstacle{k},1)
                   h(end+1) = patch('XData', map.Obstacle{k}(v,1) + inflationBall(:,1), 'YData', map.Obstacle{k}(v,2) + inflationBall(:,2), varargin{:});
               end
            end

        end


        function h = plot(map, varargin)
        % Visualize a map using patch plot    
            h = map.patch(varargin{:});
        end

        function h = plotConfSpace(map, inflationRadius, varargin)
        % Visualize the configuration space obstacles of a map using patch plot
        %
        % Input:
        %   map - a PolygonMap2D object
        %   inflationRadius - Inflation radius for configuration space obstacles, 1x1 positive scalar
        % Optional Name-Value Inputs:
        %       Name, Value: Any name-value input pair for a patch plot, see patch
        
            h = map.patchConfSpace(inflationRadius, varargin{:});
        end

        function h = show(map)
        % Show map    
            
            figureOption = {};
            axisOption = {'NextPlot', 'add', 'DataAspectRatio', [1 1 1],...
                'Box', 'on', 'XGrid', 'on', 'YGrid', 'on', ...
                'XLim', map.XWorldLimits, 'YLim', map.YWorldLimits};
            patchOption = {'FaceColor', 'k', 'EdgeColor', 'k', 'LineWidth', 2};
            
            h.figure = figure(figureOption{:});
            h.axis = axes(axisOption{:});
            h.patch = patch(map, patchOption{:});

        end


    end
end