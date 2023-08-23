% Unicycle Adaptive Headway Control Class

classdef UnicycleHeadwayControl <  matlab.mixin.Copyable
% Authors: Aykut Isleyen, a.isleyen@tue.nl
% Created: February 6, 2023
% Modified: August 08, 2023


    % Object Properties
    properties
       EpsilonGain % Epsilon Gain
       ReferenceGain % Reference Gain
    end

    properties
       MotionPolygonMethod 
       MotionPolygonResolution
    end
    
    
    methods 

        function obj = UnicycleHeadwayControl(EpsGain, RefGain)
            switch nargin
                case 0
                    EpsGain = 0.5;
                    RefGain = 1;
                case 1
                    RefGain = 1;
                otherwise 
                    % Do nothing
            end
            obj.EpsilonGain = EpsGain;
            obj.ReferenceGain = RefGain;
            obj.MotionPolygonMethod = "Circular";
            obj.MotionPolygonResolution = 60;
        end
    end


    % Object Functions for Motion Planning
    methods

        function  x = mat2vec(obj, X)
            x = reshape(X', [3, 1]);
        end
        
        function X = vec2mat(obj, x)
            X = reshape(x', [3,1])';
        end
    end

%%
    % Object Functions for Headway Reference Dynamics
    methods

       function [T, headwaypos] = headway_traj(obj, tspan, x0, options)
            % Numerically solves the headway position trajectory using
            % unicycle motion trajectory
            
            if nargin < 4
                options = odeset();
            end
            [T, X] = obj.traj(tspan, x0, options);
            headwaydist =   obj.EpsilonGain * sqrt( X(:,1).^2+X(:,2).^2); %epsilon length
            headwaypos = X(:,1:2)+[headwaydist.*cos(X(:,3)), headwaydist.*sin(X(:,3))];
        end

        function xp = projected_position(obj, x0, goal)
            
            if nargin < 3
                goal = zeros(1,2);
            end
            x0 = reshape(x0,1,3);
            goal = reshape(goal,1,2);
            
            headwaydist = obj.EpsilonGain * norm(x0(1:2)-goal);
            headwaypos = x0(1:2)+headwaydist*[cos(x0(3)) sin(x0(3))];
            projdist = ((goal-x0(1:2))*(goal - headwaypos)')/norm(headwaypos - goal);
            if norm(headwaypos-goal) > 1e-3
                xp = goal + projdist*(headwaypos - goal)/norm(headwaypos-goal);
            else
                xp = goal;
            end
        end

        function xe = extended_position(obj, x0, goal)
            
            if nargin < 3
                goal = zeros(1,2);
            end
            x0 = reshape(x0,1,3);
            goal = reshape(goal,1,2);
            
            headwaydist = obj.EpsilonGain * norm(x0(1:2)-goal);
            headwaypos = x0(1:2)+headwaydist*[cos(x0(3)) sin(x0(3))];
            projdist = ((goal-x0(1:2))*(goal - headwaypos)')/norm(headwaypos - goal);
            xp = obj.projected_position(x0, goal);
            if norm(headwaypos-goal) > 1e-3 &&  norm(x0(1:2)-xp) > 1e-3
                xe = xp + projdist*obj.EpsilonGain/sqrt(1-obj.EpsilonGain^2)*(x0(1:2) - xp)/norm(x0(1:2) - xp);
            elseif  norm(x0(1:2)-xp) > 1e-3
                xe = xp;
            else
                xe = goal;
            end
        end


    end

%%
    % Object Functions - Trajectory Functions
    methods
        function dX = vectorfield(obj, x, goal)
            
            if nargin < 3
            goal = zeros(2,1);
            end

            [v, w] = unicyclesys.adaptive_headway_control(x, goal, obj.EpsilonGain, obj.ReferenceGain);
            dx = v*cos(x(3));
            dy = v*sin(x(3));
            dtheta = w;
            dX = [dx;dy;dtheta];
        end

        function dX = vectorfield2(obj, t, x)
            
            goal = zeros(2,1);
            [v, w] = unicyclesys.adaptive_headway_control(x, goal, obj.EpsilonGain, obj.ReferenceGain);
            dx = v*cos(x(3));
            dy = v*sin(x(3));
            dtheta = w;
            dX = [dx;dy;dtheta];
        end
        
        function [T, X] = traj(obj, tspan, x0, odeoptions)
            % Numerically solves the system state trajectory using ode45
            
            [T, X] = ode45(@(t, x) obj.vectorfield2(t,x), tspan, x0, odeoptions); 
        end

%%
    % Object Functions -  Motion Polygon Functions

        function P = motionbound(obj, x0, goal)
            if nargin < 3
                goal = zeros(1,2);
            end
            x0 = reshape(x0,1,3);
            goal = reshape(goal,1,2);
            k_e = obj.EpsilonGain;

            x0(3) = x0(3) + 1e-8*(rand); %% Singularity Correction for convexhull function
            x0(1:2) = x0(1:2) + 1e-8*(rand(size(x0(1:2)))); %% Singularity Correction for convexhull function
            x0(1:2) = x0(1:2) - goal;

            switch lower(obj.MotionPolygonMethod)
                case 'circular'
                    [V, F] = unicyclesys.adaptiveheadwaycontrolCircular(x0, k_e, obj.MotionPolygonResolution);
                    V = V + goal(1,:);
                    P = V(F(1,:),:);
                case 'triangular'
                    [V, F] = unicyclesys.adaptiveheadwaycontrolTriangularBound(x0, k_e);
                    V = V + goal(1,:);
                    P = V(F(1,:),:);
                otherwise
                    error('Unknown motion bound method!');
            end
        end

        function P = motionpolygon(obj, x0, goal)
            
            if nargin < 3
                goal = zeros(1,2);
            end
            x0 = reshape(x0,1,3);
            goal = reshape(goal,1,2);
            k_e = obj.EpsilonGain;
            k_r = obj.ReferenceGain;

            x0(3) = x0(3) + 1e-8*(rand); %% Singularity Correction for convexhull function
            x0(1:2) = x0(1:2) + 1e-8*(rand(size(x0(1:2)))); %% Singularity Correction for convexhull function
            x0(1:2) = x0(1:2) - goal;

            switch lower(obj.MotionPolygonMethod)
                case 'simulated'
                    [V, F] = unicyclesys.adaptiveheadwaycontrolSimulated(x0, k_e, k_r, 0.01, 0.01);
                    V = V + goal(1,:);
                    P = V(F(1,:),:);
                case 'circular'
                    [V, F] = unicyclesys.adaptiveheadwaycontrolCircular(x0, k_e, obj.MotionPolygonResolution);
                    V = V + goal(1,:);
                    P = V(F(1,:),:);
                case 'triangular'
                    [V, F] = unicyclesys.adaptiveheadwaycontrolTriangular(x0, k_e);
                    V = V + goal(1,:);
                    P = V(F(1,:),:);
                otherwise
                    error('Unknown motion prediction method!');
            end
        end

        function P = augmentedmotionpolygon(obj, x0, RobotPolygon, goal)

            P = obj.motionpolygon(x0, goal);
            R = RobotPolygon;
            
            switch lower(obj.MotionPolygonMethod)
                case 'simulated'
                    k_e = obj.EpsilonGain;
                    k_r = obj.ReferenceGain;
                    [V, F] = unicyclesys.adaptiveheadwaycontrolSimulated(x0, k_e, k_r, 0.01, 0.01);
                    V = V + goal(1,:);
                    P = V(F(1,:),:);
                    for k = 1:size(F,2)
                        Polytemp = [P(k,1)+R(:,1), P(k,2)+R(:,2)];
                        polyvec(k) = polyshape(Polytemp);
                    end
                        pout = union(polyvec);
                        P = pout.Vertices;
                case 'circular'
                    [P1, P2] = geom.cvxpolysum(P(:,1), P(:,2), R(:,1), R(:,2));
                    P = [P1(:), P2(:)];
                case 'triangular'
                    [P1, P2] = geom.cvxpolysum(P(:,1), P(:,2), R(:,1), R(:,2));
                    P = [P1(:), P2(:)];
                otherwise
                    error('Unknown motion prediction method!');
            end
        end
        
    end
    
    
end





