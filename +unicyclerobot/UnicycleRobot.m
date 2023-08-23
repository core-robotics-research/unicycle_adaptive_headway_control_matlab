classdef UnicycleRobot
    
    properties
        Control % Robot Control Policy
        BodyRadius % Robot Body Radius
        BodyResolution % Robot Body Resolution
    end
    
    % Dependent Robot Properties
    properties(Dependent=true)
        BodyPolygon % Robot Body Polygon
    end

    
    methods
        
        function obj = UnicycleRobot(~, ~)
            
            
            % Default Properties
            obj.Control = unicyclesys.UnicycleHeadwayControl();
            obj.BodyRadius = 1;
            obj.BodyResolution = 60;

        end


        
        function value = get.BodyPolygon(obj)
            angles = linspace(0, 2*pi, obj.BodyResolution)';
            value = obj.BodyRadius*[cos(angles), sin(angles)];
        end

        function P = bodyPolygon(obj, resolution)
            if nargin < 2
                resolution = obj.BodyResolution;
            end

            angles = linspace(0, 2*pi, resolution)';
            P = obj.BodyRadius*[cos(angles), sin(angles)];
        end
                


        function x = mat2vec(obj, X)
        % Converts a state matrix X to a state vector x as specified by control  
        % as x = obj.Control.mat2vec(X)
        %
        % Input:
        %   obj: Fully actuated robot object
        %   X: State matrix, n x d matrix 
        % Output:
        %   x: State vector, n*d x 1 vector
            x = obj.Control.mat2vec(X);
        end

        function X = vec2mat(obj, x)
        % Converts a state vector x to a state matrix as specified by control 
        % as X = obj.Control.vec2mat(x)
        %
        % Input: 
        %   obj: Fully actuated robot object
        %   x: State vector, n*d x 1 matrix
        % Output:
        %   X: State matrix, n x d matrix    
            X = obj.Control.vec2mat(x);
        end

        function dx = vectorField(obj, x, xref)
        % Vector field as specified by control, 
        % i.e., obj.vectorField = obj.Control.vectorField
        % 
        % Input: 
        %   obj: Fully actuated robot object
        %   x: State vector, [x^{(0)}, ..., x^{(n-1)}]', n*d x 1 vector 
        %   xref: Reference state vector, n*d x 1 vector
        % Output:
        %   dx: State time-rate-of-change vector, n*d x 1 vector

            dx = obj.Control.vectorfield(x, xref);
        end

        function P = motionPolygon(obj, x, xref, varargin)
        % Closed-loop motion polygon as specified by control
        %
        % A motion polygon bounds the closed-loop robot motion trajectory
        % relative to a given constant reference position.  
        %
        % Input:
        %   obj: Fully actuated robot object, 
        %   x: State vector, [x^{(0)}, ..., x^{(n-1)}]', n*d x 1 vector
        %   xref: Reference state vector, n*d x 1 vector
        % Output:
        %   P: Motion polygon, m x 2 vector

            P = obj.Control.motionpolygon(x, xref, varargin{:});
        end

        function P = bodyMotionPolygon(obj, x, xref, varargin)
        % Closed-loop body motion polygon as specified by control
        %
        % A body motion polygon bounds the closed-loop robot body motion 
        % trajectory relative to a given constant reference position.  
        %
        % Input:
        %   obj: Fully actuated robot object, 
        %   x: State vector, [x^{(0)}, ..., x^{(n-1)}]', n*d x 1 vector
        %   xref: Reference state vector, n*d x 1 vector
        % Output:
        %   P: Motion polygon, m x 2 vector
            
            if nargin < 3
                xref = zeros(1, 3);
            end
            
            P = obj.motionPolygon(x, xref, varargin{:});
            minResolution = 16;
            resolution = ceil(max(size(P,1), obj.BodyResolution)*minResolution/size(P,1));
            R = obj.bodyPolygon(resolution);
            [P1, P2] = geom.cvxpolysum(P(:,1), P(:,2), R(:,1), R(:,2));
            P = [P1(:), P2(:)];

        end

        function P = augmentedMotionPolygon(obj, x, xref, varargin)
            
            if nargin < 3
                xref = zeros(1, 3);
            end
            
            P = obj.motionPolygon(x, xref, varargin{:});
            R = obj.BodyPolygon;
            [P1, P2] = geom.cvxpolysum(P(:,1), P(:,2), R(:,1), R(:,2));
            P = [P1(:), P2(:)];

        end

    end
    
end