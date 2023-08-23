classdef TimeGovernedPathFollower <  matlab.mixin.Copyable

    properties
        Robot
        Map
        Path
    end

    properties
        PathFeedback % Path Feedback Type - 'PositionOnly' (default) | 'PositionVelocity' 
        ConvergenceRate % Unconstrained Parameter Convergence Rate
        SafetyGain % Safety Gain for Parameter Governor 
        MaxRate % Maximum Time Rate of Change of Path Parameter
    end

    methods
        function obj = TimeGovernedPathFollower(myrobot, mymap, mypath)
            obj.Robot = myrobot;
            obj.Map = mymap;
            obj.Path = mypath;

            obj.PathFeedback = 'PositionOnly';
            obj.SafetyGain = 1.0;
            obj.ConvergenceRate = 1.0;
            obj.MaxRate = 1.0; % Maximum Time Rate of Change of Path Parameter
        end

        function [dx, dk] = vectorField(obj, x, k)
 
            % Maximum Parameter Rate
            dkMax = obj.MaxRate; 
            if (k < obj.Path.Interval(1)) || (k > obj.Path.Interval(2))
                k = max(min(k, obj.Path.Interval(2)), obj.Path.Interval(1));
                dkMax = 0;
            end

            x = obj.Robot.vec2mat(x); 
            pathPoint = obj.Path.value(k);
            xref = [pathPoint];

            % Safety Level
            P = obj.Robot.motionPolygon(x, xref);
            safetyLevel = obj.Map.polydist2coll(P) - obj.Robot.BodyRadius;
            
            dkSafe = obj.SafetyGain*safetyLevel; % Safe Parameter Rate
            dkConv = obj.ConvergenceRate*(obj.Path.Interval(2) - k); % Converging Parameter Rate
            dk = min([dkMax, dkSafe, dkConv]);
            dk = max([0, dk]);


            if strcmpi(obj.PathFeedback, 'PositionVelocity')
                pathVelocity = obj.Path.derivative(k, 1)*dk;
                xref(2,:) = pathVelocity;
            end

            dx = obj.Robot.vectorField(x, xref);

        end

        function dxk = vectorField2(obj, xk)
            x = xk(1:3);
            k = xk((3+1):end);

            [dx, dk] = obj.vectorField(x, k);

            dxk = [obj.Robot.mat2vec(dx); dk(:)];

        end

        function [T, X, K] = ode45(obj, tspan, x0, k0, options)

            if nargin < 5
                options = odeset();
            end

            options = odeset(options, 'Events', @(t, xk) obj.convergenceEvent2(t, xk, options.RelTol));

            x0 = obj.Robot.mat2vec(x0);
            k0 = k0(:);
            xk0 = [x0; k0];
            [T, XK] = ode45(@(t, xk) obj.vectorField2(xk), tspan, xk0, options);
            X = XK(:, 1:3);
            K = XK(:,(3+1):end);

        end

        function s = safetyLevel(obj, x, k)
   
            x = obj.Robot.vec2mat(x);
            k = max(min(k, obj.Path.Interval(2)),obj.Path.Interval(1));

            % Reference Dynamics
            pathPoint = obj.Path.value(k);
            xref = [pathPoint];
            % Motion Polygon
            P = obj.Robot.motionPolygon(x, xref);
            % Motion Safety 
            s = obj.Map.polydist2coll(P) - obj.Robot.BodyRadius;
            
        end

        function MP = bodyMotionPolygon(obj, x, k)

            x = obj.Robot.vec2mat(x);
            k = max(min(k, obj.Path.Interval(2)),obj.Path.Interval(1));

            % Reference Dynamics
            pathPoint = obj.Path.value(k);
            xref = [pathPoint];
            
            % Motion Polygon
            MP = obj.Robot.bodyMotionPolygon(x, xref);
            
        end

    end

    methods(Access='private')

        function [value, isterminal, direction] = convergenceEvent2(obj, t, xk, Tol)

            if nargin < 4
                Tol = 1e-3;
            end

            if isempty(Tol)
                Tol = 1e-3;
            end

            x = xk(1:3);
            k = xk((3+1):end);
            [dx, dk] = obj.vectorField(x, k);
            dx = obj.Robot.vec2mat(dx);
            value = max([sqrt(sum(dx.^2, 2)); abs(dk)]) - Tol;
            isterminal = 1;
            direction = 0;

        end

    end

end



