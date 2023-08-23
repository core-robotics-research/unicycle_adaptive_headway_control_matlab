% A custom binary occupancy grid map built based on the binaryOccupancyMap class

classdef BinaryOccupancyGrid2D < matlab.mixin.Copyable
% Authors:  Omur Arslan, omurarslan.phd@gmail.com
%           Aykut Isleyen,
% Created:  August 21, 2018
% Modified: August 02, 2022

    properties(GetAccess='private', SetAccess='private')
        occgrid % Binary occupancy grid map, a binaryOccupancyMap object
    end
    
    % Renamed Inherited Properties from binary OccupancyMap 
    properties(Dependent=true, SetAccess='public')
        MapOriginInWorld % [x,y] world coordinates of grid, [0 0] (default) | two-element vector  
        LocalOriginInMap % [x,y] local coordinate origin in the map coordinates | two-element vector
        LocalOriginInWorld % [x, y] local coordinate origin in the world coordinates | two-element vector
    end
    
    % Inherited Properties from binaryOccupancyMap
    properties(Dependent=true, SetAccess='private')
        GridSize % Number of rows and columns in grid, [rows cols] vector
        Resolution % Grid resolution, 1 (default) | scalar in cells per meter
        XWorldLimits % Minimum and maximum world range values of x-coordinates, [min max] vector
        YWorldLimits % Minimum and maximum world range values of y-coordinates, [min max] vector
        XLocalLimits % Minimum and maximum values of x-coordinates in local frame, [min max] vector
        YLocalLimits % Minimum and maximum values of y-coordinates in local frame, [min max] vector
        DefaultValue % Default value for unspecified map locations, 0 (default) or 1
    end
    
    % Custom Dependent Properties 
    properties(Dependent=true, SetAccess='private')
        Width % Map width, a positive scalar
        Height % Map height, a positive scalar
        OccupiedGrids % Occupied grid indices, n x 2 matrix
        OccupiedPoints % World coordinates of occuppied grids, n x 2 matrix 
    end
    
    % Custom Object Properties

    properties(GetAccess='public', SetAccess='public')
        DistanceMetric = 'euclidean' % Distance metric for distance field (bwdist) calculation, 'euclidean' (default) | 'chessboard' | 'cityblock' | 'quasi-euclidean'
    end

    % Custom Private Object Properties
    properties(GetAccess='private', SetAccess='private')
        DistanceField % Distance field (trasnform) of the binary occupancy grid map
        NearestNeighbor % Linear index of the nearest nonzero grid 
        isValidDistanceField % Checks if the distance field is up-to-date
    end


    % Class Constructor
    methods
        function map = BinaryOccupancyGrid2D(varargin)
        % Binary occupancy grid class constructor
        % Refer to binaryOccupancyMap for details
            map.occgrid = binaryOccupancyMap(varargin{:});
            map.isValidDistanceField = false;
        end
    end

    % Get & Set Methods for Renamed Inherited Properties
    methods
        
        function value = get.MapOriginInWorld(map)
            value = map.occgrid.GridLocationInWorld;
        end
        
        function set.MapOriginInWorld(map, value)
            map.occgrid.GridOriginInLocal = -(map.LocalOriginInWorld - value);
            map.occgrid.GridLocationInWorld = value;
        end
        
        function value = get.LocalOriginInMap(map)
            value = - map.occgrid.GridOriginInLocal;
        end

        function set.LocalOriginInMap(map, value)
            temp = map.occgrid.GridLocationInWorld;
            map.occgrid.GridOriginInLocal = - value;
            map.occgrid.GridLocationInWorld = temp;
        end

        function value = get.LocalOriginInWorld(map)
            value = map.occgrid.LocalOriginInWorld;
        end

        function set.LocalOriginInWorld(map, value)
            map.occgrid.GridOriginInLocal = -(value - map.occgrid.GridLocationInWorld);
            map.occgrid.LocalOriginInWorld = value;
        end

    end

    % Get & Set Methods for Inherited Properties
    methods
        
        function  value = get.GridSize(map)
            value = map.occgrid.GridSize;
        end
        
        function value = get.Resolution(map)
            value = map.occgrid.Resolution;
        end
        
        function value = get.XWorldLimits(map)
            value = map.occgrid.XWorldLimits;
        end
        
        function value = get.YWorldLimits(map)
            value = map.occgrid.YWorldLimits;
        end
        
        function value = get.XLocalLimits(map)
            value = map.occgrid.XLocalLimits;
        end

        function value = get.YLocalLimits(map)
            value = map.occgrid.YLocalLimits;
        end

        function value = get.DefaultValue(map)
            value = map.occgrid.DefaultValue;
        end

        function set.DefaultValue(map, value)
            map.occgrid.DefaultValue = value;
        end
    end

    % Get & Set Methods for Custom Dependent Properties
    methods

        function value = get.Width(map)
           value = diff(map.XWorldLimits); 
        end
        
        function value = get.Height(map)
            value = diff(map.YWorldLimits);
        end

        function ij = get.OccupiedGrids(map)
        % Determines the indices of occupied map grids    
           mat = map.occupancyMatrix();
           [I, J] = find(mat);
           ij = [I J];
        end
        
        function xy = get.OccupiedPoints(map)
        % Determines the world coordinates of occupied map grids    
            xy = map.grid2world(map.OccupiedGrids); 
        end

    end

    % Inherited Object Functions
    methods
        function ij = world2grid(map, xyWorld)
        % Convert world coordinates to grid indices    
            ij = map.occgrid.world2grid(xyWorld);
        end

        function xyWorld = grid2world(map, ij)
        % Convert grid indices to world coordinates    
            xyWorld = map.occgrid.grid2world(ij);
        end

        function xyLocal = world2local(map, xyWorld)
            xyLocal = map.occgrid.world2local(xyWorld);
        end

        function xyWorld = local2world(map, xyLocal)
            xyWorld = map.occgrid.local2world(xyLocal);
        end

        function xyMap = world2map(map, xyWorld)
            xyMap = xyWorld - map.MapOriginInWorld;
        end

        function xyWorld = map2world(map, xyMap)
            xyWorld = xyMap + map.MapOriginInWorld;
        end   

        function xyLocal = grid2local(map, ij)
        % Convert grid indices to local coordinates    
            xyLocal = map.occgrid.grid2local(ij);
        end
        
        function ij = local2grid(map, xyLocal)
        % Convert local coordinates to grid indices    
            ij = map.occgrid.local2grid(xyLocal);
        end

        function xyMap = grid2map(map, ij)
        % Convert grid indices to map coordinates    
            xyMap = map.local2map(map.occgrid.grid2local(ij));
        end
        
        function ij = map2grid(map, xyMap)
        % Convert local coordinates to grid indices    
            ij = map.occgrid.local2grid(map.map2local(xyMap));
        end

        function xyMap = local2map(map, xyLocal)
            xyMap = xyLocal + map.LocalOriginInMap;
        end

        function xyLocal = map2local(map, xyMap)
            xyLocal = xyMap - map.LocalOriginInMap;
        end
        
        function occval = getOccupancy(map, varargin)
        % Get occupancy of a location    
            occval = map.occgrid.getOccupancy(varargin{:});
        end
        
        function setOccupancy(map, varargin)
        % Set occupancy of a location
           map.occgrid.setOccupancy(varargin{:}); 
           map.isValidDistanceField = false;
        end
        
        function mat = occupancyMatrix(map)
        % Convert occupancy grid to logical matrix    
            mat = map.occgrid.occupancyMatrix();
        end
        
    end 

    % Custom Object Functions
    methods

        function [D, I] = distanceField(map)
            if map.isValidDistanceField
                D = map.DistanceField;
                I = map.NearestNeighbor;
            else
                [D, I] = bwdist(map.occupancyMatrix, map.DistanceMetric);
                D = D/map.Resolution;
                map.DistanceField = D;
                map.NearestNeighbor = I;
                map.isValidDistanceField = true;
            end
        end

        function [D, X] = dist2coll(map, position, varargin)
        % D: Distance-to-collision at given positions
        % X: Collision point in the given coordinate frame
        % Optional Name-Value Inputs:
        %   'Frame' - Coordinate frame of input position, 'world' (default) | 'map', 'local', 'grid' 
        %   ''

            frame = 'world';
            for k = 2:2:numel(varargin)
                if strcmpi(varargin{k-1}, 'frame')
                    frame = varargin{k};
                end
            end

            D = zeros(size(position,1),1);
            X = zeros(size(position,1),2);
            position = map.frame2grid(position, frame);
            IN = map.isInMap(position, 'grid');

            [DF, NN] = map.distanceField();
            posIND = sub2ind(map.GridSize, position(IN,1),position(IN,2));
            D(IN) = DF(posIND);
            [XI, XJ] = ind2sub(map.GridSize, NN(posIND));
            X(IN,:) = [XI(:), XJ(:)];

            % Positions outside the map
            OUT = not(IN);
            X(OUT,:) = min(max(position(OUT,:), 1), map.GridSize);
            D(OUT) = - sqrt(sum((position(OUT,:) - X(OUT,:)).^2,2))/map.Resolution;

            X = map.grid2frame(X, frame);

        end

        function  [D, XP, XM] = polydist2coll(map, polygon, varargin)

            frame = 'world';
            for k = 2:2:numel(varargin)
                if strcmpi(varargin{k-1}, 'frame')
                    frame = varargin{k};
                end
            end


            polygon = map.frame2grid(polygon, frame);
            ij = map.poly2grid(polygon, 'grid');
            [D, XM] = map.dist2coll(ij, 'Frame', 'grid');

            [Dtemp, XMtemp] = map.dist2coll(polygon, 'Frame', 'grid');

            D = [D; Dtemp];
            ij = [ij; polygon];
            XM = [XM; XMtemp];

            [D, I] = min(D);
            XM = map.grid2frame(XM(I,:), frame);
            XP = map.grid2frame(ij(I,:), frame);

        end
 
    end

    % Custom Private Object Functions
    methods(Access='private')

        function ij = frame2grid(map, position, frame)
            
            switch lower(frame)
                case 'world'
                    ij = map.world2grid(position);
                case 'local'
                    ij = map.local2grid(position);
                case 'map'
                    ij = map.map2grid(position);
                otherwise
                    %  frame is grid
                    ij = position;
            end
        end

        function position = grid2frame(map, ij, frame)

             switch lower(frame)
                case 'world'
                    position = map.grid2world(ij);
                case 'local'
                    position = map.grid2local(ij);
                 case 'map'
                     position = map.grid2map(ij);
                otherwise
                    %  frame is grid
                    position = ij;
            end

        end

        function  ij = poly2grid(map, polygon, frame)

            polygon = map.frame2grid(polygon, frame);

            GS = map.GridSize;

            M = poly2mask(polygon(:,1), polygon(:,2), GS(1), GS(2));

            [I, J] = meshgrid(1:GS(2), 1:GS(1));
            
            ij = [I(M), J(M)];
        end

        function I = isInMap(map, position, frame)
            position = map.frame2grid(position, frame);
            I = and(all(position>0,2), all(position<=map.GridSize,2));
        end

    end

    % Visualization Methods
    methods

        function h = surf(map, varargin)

            % Surf visualization of map boundary and obstacles
            C = double(not(flipud(map.occupancyMatrix)));
            C = [C, C(:,end)];
            C = [C; C(end,:)];
            C = repmat(C, [1 1 3]);
            
            X = linspace(map.XWorldLimits(1), map.XWorldLimits(2), map.GridSize(2)+1);
            Y = linspace(map.YWorldLimits(1), map.YWorldLimits(2), map.GridSize(1)+1);      
            h = surf(X, Y, zeros(map.GridSize + 1), C, varargin{:});
        end
        
        function h = surfConfSpace(map, inflationRadius, varargin)
        % Patch plot of configuration space obstacles
        % Input:
        %   map - a BinaryOccupancyGrid2D object
        %   inflationRadius - Inflation radius for configuration space obstacles, 1x1 positive scalar
        % Optional Name-Value Inputs:
        %       Name, Value: Any name-value input pair for a patch plot, see patch   
    
            [D, ~] = map.distanceField;
            D(D>inflationRadius) = -1;
            D(D ~= 0 & D~=-1) = 0.8;
            D(D==-1) = 1;
            D = repmat(flipud(D), [1 1 3]);

            X = linspace(map.XWorldLimits(1), map.XWorldLimits(2), map.GridSize(2)+1);
            Y = linspace(map.YWorldLimits(1), map.YWorldLimits(2), map.GridSize(1)+1);      
            h = surf(X, Y, zeros(map.GridSize + 1), D, varargin{:});

        end

        function h = plot(map, varargin)
        % Visualize a map using surf plot    
            h = map.surf(varargin{:});
        end

        function h = plotConfSpace(map, inflationRadius, varargin)
        % Visualize the configuration space obstacles of a map using patch plot
        %
        % Input:
        %   map - a BinaryOccupancyGrid2D object
        %   inflationRadius - Inflation radius for configuration space obstacles, 1x1 positive scalar
        % Optional Name-Value Inputs:
        %       Name, Value: Any name-value input pair for a patch plot, see patch
        
            h = map.surfConfSpace(inflationRadius, varargin{:});
        end

        function h = show(map, varargin)
            h = map.occgrid.show(varargin{:});
        end

        function h = imagesc(map)
           h = imagesc(map.XWorldLimits, fliplr(map.YWorldLimits), map.occupancyMatrix(), [0, 1]);
        end

    end
    
    methods(Access=protected)
        % Override copyElement method:
        function copyMap = copyElement(map)
            % Make a shallow copy of all properties
            copyMap = copyElement@matlab.mixin.Copyable(map);
            % Make a deep copy of the occgrid object
            copyMap.occgrid = copy(map.occgrid);
        end
    end
end