% Demo for safe unicycle robot navigation via feedback motion prediction 
% and safety assessment.

% Change arguments to test the general motion framework with
% different motion prediction methods and example environments

% A demonstration of time-governed safe path following 
%
% Example:
%   mapID = 1; % 1 - Corridor | 2 - Office
%   MotionPrediction = 'Circular' : %'Circular' | 'Triangular'
%   demo_timegovernedpathfollower('mapID', mapID, 'MotionPrediction', MotionPrediction, 'saveFigure', false, 'saveVideo', false)

function demo_timegovernedpathfollower(varargin)
% Author: Aykut Isleyen, a.isleyen@tue.nl
% Created: February 15, 2023
% Modified: August 09, 2023

    % Default Settings
    startOrientation = -pi/2+0.1;
    mapID = 2;
    MotionPrediction = 'Triangular'; % 'Circular' | 'Triangular'
    saveFigure = false; 
    saveVideo = false;

    for k = 2:2:numel(varargin)
        switch lower(varargin{k-1})
            case 'mapid'
                mapID = varargin{k};
            case 'motionprediction'
                MotionPrediction = varargin{k};
            case 'savefigure'
                saveFigure = varargin{k};
            case 'savevideo'
                saveVideo = varargin{k};
            otherwise
                % Do nothing
        end
    end


    %% Simulation Settings
    % Map Settings
    %   1 - 10 x 6 Straight Corridor Environment and Linear Path
    %   2 - 10 x 10 Office Environment and Linear Path
    settings.scenario.mapID = mapID; 

    % Advanced Settings
    settings = advanced_settings(settings);
    myplanner = timeGovernor.TimeGovernedPathFollower(settings.myrobot, settings.mymap, settings.mypath);
    myplanner.Robot.Control.MotionPolygonMethod = MotionPrediction;
    myplanner.SafetyGain = settings.SafetyGain;
    myplanner.MaxRate = settings.MaxRate; % Maximum Parameter Rate
    myplanner.ConvergenceRate = settings.ConvergenceRate; % Parameter Convergence Rate 

    %% Numerical Solution of Time Governed Path Following 
    k0 = myplanner.Path.Interval(1);
    x0 = [myplanner.Path.value(k0) startOrientation];
    odeOption = settings.ode.option;
    tspan = settings.ode.tspan;
    [T, X, K] = myplanner.ode45(tspan, x0, k0, odeOption);
    
    %% Visualization
    settings.saveVideo = saveVideo;
    settings.saveFigure = saveFigure;
    
    settings.filename = sprintf("%s_%s_%s_m%d_r%d_Gain25", settings.filename, ...
        MotionPrediction, settings.scenario.mapID);

    h.figure = figure(settings.figure{:});
    h.axes = axes(settings.axes{:});
    h.confSpace = patchConfSpace(myplanner.Map, myplanner.Robot.BodyRadius, settings.map.patchConfSpace{:});
    h.map = patch(myplanner.Map, settings.map.patch{:});
    h.path = plot(myplanner.Path, K, settings.pathPlot{:});
    h.robotPath = plot(X(1,1), X(1,2), settings.robotPathPlot{:});
    
    thwl = linspace(5*pi/6, pi/6, 60); % Angle samples for the visualization of the left robot wheel
    thwr = linspace(7*pi/6, 11*pi/6, 60); % Angle sample for the visualization of the right robot wheel
    R = settings.myrobot.BodyRadius;
    h.robot.start{1} = patch('XData', X(1,1) + myplanner.Robot.BodyPolygon(:,1),...
      'YData', X(1,2) + myplanner.Robot.BodyPolygon(:,2),   settings.startPatch{:});
    h.robot.start{2} = patch('XData', X(1,1) + R*cos(thwl+X(1,3)), 'YData',  X(1,2) + R*sin(thwl+X(1,3)), 'FaceColor', 'k');
    h.robot.start{3} = patch('XData', X(1,1) + R*cos(thwr+X(1,3)), 'YData',  X(1,2) + R*sin(thwr+X(1,3)), 'FaceColor', 'k');
    thick = settings.DirectionThickness;
    thdir = [ pi/2, -pi/2, linspace(-asin(thick/R), asin(thick/R), 60)];
    mdir = [repmat(thick, 1, 2), repmat(R, 1, 60)];
    h.robot.start{4} = patch('XData', X(1,1)+mdir.*cos(thdir+X(1,3)),'YData', X(1,2) + mdir.*sin(thdir+X(1,3)), 'FaceColor', 'k'); 


    goalPoint = myplanner.Path.value(myplanner.Path.Interval(2)); 
    h.goal = patch('XData', goalPoint(1,1) + myplanner.Robot.BodyPolygon(:,1), ...
        'YData', goalPoint(1,2) + myplanner.Robot.BodyPolygon(:,2), settings.goalPatch{:});
    
    robotMotionPolygon = myplanner.bodyMotionPolygon(X(1,:), K(1,:));
    h.robotMotion = patch('XData', robotMotionPolygon(:,1), ...
        'YData', robotMotionPolygon(:,2), settings.robotMotionPatch{:});
    h.robotBody = patch('XData', X(1,1) + myplanner.Robot.BodyPolygon(:,1),...
                          'YData', X(1,2) + myplanner.Robot.BodyPolygon(:,2),...
                          settings.robotBodyPatch{:});
    h.robotwl = patch('XData', X(1,1) + R*cos(thwl+X(1,3)), 'YData',  X(1,2) + R*sin(thwl+X(1,3)), 'FaceColor', 'k');
    h.robotwr = patch('XData', X(1,1) + R*cos(thwr+X(1,3)), 'YData',  X(1,2) + R*sin(thwr+X(1,3)), 'FaceColor', 'k');
    h.robotdir= patch('XData', X(1,1)+mdir.*cos(thdir+X(1,3)),'YData', X(1,2) + mdir.*sin(thdir+X(1,3)), 'FaceColor', 'k');

    
    
    pathPoint = myplanner.Path.value(K(1,1));
    h.pathPoint = scatter(pathPoint(1), pathPoint(2), settings.pathPointScatter{:}); 
    
    if settings.saveVideo   
        if ispc
            VideoProfile = 'MPEG-4';
        else
            VideoProfile = 'Uncompressed AVI';
        end
        h.writerobj = VideoWriter(settings.filename, VideoProfile);
        h.writerobj.FrameRate = settings.FrameRate;
        open(h.writerobj);
        for k = 1:2*h.writerobj.FrameRate
           writeVideo(h.writerobj,getframe(gcf));
        end
    end   
    
    tPrevious = T(1);
    for tCounter = 1:length(T)
        if (T(tCounter) - tPrevious >= 1/settings.FrameRate)
            tPrevious = T(tCounter);

            set(h.robotPath, 'XData', X(1:tCounter,1), 'YData', X(1:tCounter,2));
            pathPoint = myplanner.Path.value(K(tCounter,1));
            set(h.pathPoint, 'XData', pathPoint(1), 'YData', pathPoint(2));
            
            robotMotionPolygon = myplanner.bodyMotionPolygon(X(tCounter,:), K(tCounter,:));
            set(h.robotMotion, 'XData', robotMotionPolygon(:,1), 'YData', robotMotionPolygon(:,2));
            
            set(h.robotBody, 'XData', X(tCounter,1) + myplanner.Robot.BodyPolygon(:,1), 'YData', X(tCounter,2)+ myplanner.Robot.BodyPolygon(:,2));
            set(h.robotwl, 'XData', X(tCounter,1) + R*cos(thwl+X(tCounter,3)),...
                             'YData', X(tCounter,2) + R*sin(thwl+X(tCounter,3)));
            set(h.robotwr, 'XData', X(tCounter,1) + R*cos(thwr+X(tCounter,3)),...
                                 'YData', X(tCounter,2) + R*sin(thwr+X(tCounter,3)) );
            set(h.robotdir,'XData', X(tCounter,1) + mdir.*cos(thdir+X(tCounter,3)),...
                                 'YData', X(tCounter,2) + mdir.*sin(thdir+X(tCounter,3)) );

    
            if settings.saveVideo
              writeVideo(h.writerobj,getframe(gcf));
            end
            pause(1/settings.FrameRate);
        end
    end
    
    if settings.saveVideo
        for k = 1:2*h.writerobj.FrameRate
         writeVideo(h.writerobj,getframe(gcf));
        end
        close(h.writerobj);
        disp('Simulation video is saved!!!');    
    end

    if settings.saveFigure
      set(gcf, 'Renderer', 'painters')
      saveas(gcf, settings.filename, 'png');
      disp('Simulation figure is saved!!!');
    end

end
%% Advanced Demo Settings

function settings = advanced_settings(settings)
    
    settings.FrameRate = 10;
    settings.PathSpacing = 0.2;

    settings.folder = 'temp';
    settings.filename = [settings.folder '/demo_timegovernedpathfollower'];
    if not(isfolder(settings.folder))
        mkdir(settings.folder);
    end

    % Visualization Options
    screenSize = get(0, "ScreenSize");
    screenAspectRatio =screenSize(3)/screenSize(4); 
    settings.figure = {'Color', [1 1 1], 'Units', 'normalized', ...
        'Position', [0.25, 0.25, 0.25, 0.25*screenAspectRatio]};
    settings.axes = {'NextPlot', 'add', 'DataAspectRatio', [1, 1, 1],...
        'Position', [0 0 1 1], 'Box', 'on', 'XGrid', 'on', 'YGrid', 'on', ...
        'XLim', [0, 10], 'XTick', 0:10, 'XTickLabels', [],...
        'YLim', [0, 10], 'YTick', 0:10, 'YTickLabels', []};

    settings.map.patch = {'FaceColor', 'k', 'EdgeColor', 'k', 'LineWidth', 1};
    settings.map.patchConfSpace = {'FaceColor', 0.8*[1 1 1], 'EdgeColor', 'none'};

    settings.pathPlot = {'Color', [0.6350, 0.0780, 0.1840], 'LineWidth', 2};
    settings.pathPointScatter= {[], 'r','filled'};
    settings.robotPathPlot = {'Color', [0, 0.4470, 0.7410], 'LineWidth', 2};
    settings.robotBodyPatch = {'FaceColor', [77/255, 190/255, 238/255], 'EdgeColor', 'none'};
    settings.robotMotionPatch = {'FaceColor', [0.9290, 0.6940, 0.1250], 'EdgeColor', 'none', 'FaceAlpha', 0.5};
    settings.startPatch = {'FaceColor', [77/255, 190/255, 238/255], 'EdgeColor', 'none'};
    settings.goalPatch = {'FaceColor', 	[0.8, 0, 0], 'EdgeColor', 'none'};

    % Time Governor Settings
    settings.ConvergenceRate = 4.0;
    settings.SafetyGain = 4.0;
    settings.MaxRate = 4.0;

    % ode Settings
    settings.ode.option = odeset('RelTol', 1e-3, 'AbsTol', 1e-6, 'MaxStep', 0.05);
    settings.ode.tspan = [0, 120];

    switch settings.scenario.mapID
        
        case 1 % Straight Corridor and Piecewise Linear Path

            % Map Settings
            boundary = [0 0; 0 6; 10 6; 10 0];
            obstacle{1} = [0 0; 0 6; 0.25 6; 0.25 0];
            obstacle{2} = [0 6; 10 6; 10 5.75; 0 5.75];
            obstacle{3} = [10 6; 10 0; 9.75 0; 9.75 6];
            obstacle{4} = [0 0; 0 0.25; 10 0.25; 10 0];
            obstacle{5} = [2.25 2.25; 2.25 3.75; 7 3.75; 7 2.25];
            settings.mymap = world.PolygonMap2D(boundary, obstacle);
            % Path Settings
            xy = [4 1.25; 1.25 1.25; 1.25 4.75; 8.5 4.75; 8.5 1.25; 6 1.25];
            settings.mypath = path.Path(xy);
            
            settings.figure = {'Color', [1 1 1], 'Units', 'normalized', ...
                'Position', [0.25, 0.25, 0.25, 0.15*screenAspectRatio]};
            settings.axes = {'NextPlot', 'add', 'DataAspectRatio', [1, 1, 1],...
                'Position', [0 0 1 1], 'Box', 'on', 'XGrid', 'on', 'YGrid', 'on', ...
                'XLim', [0, 10], 'XTick', 0:1:10, 'XTickLabels', [],...
                'YLim', [0, 6], 'YTick', 0:1:6, 'YTickLabels', []};

        case 2 % Office Environment and Linear Path

            % Map Settings
            boundary = [0 0; 0 10; 10 10; 10 0];
            obstacle{1} = [0 0; 0 10; 0.25 10; 0.25 0];
            obstacle{2} = [0 10; 10 10; 10 9.75; 0 9.75];
            obstacle{3} = [10 10; 10 0; 9.75 0; 9.75 10];
            obstacle{4} = [0 0; 0 0.25; 10 0.25; 10 0];
            obstacle{5} = [4 10; 4.5 10; 4.5 8; 4 8];
            obstacle{6} = [8 8; 10 8; 10 7.5; 8 7.5];
            obstacle{7} = [4 2; 4 6; 4.5 6; 4.5 2];
            obstacle{8} = [7 2; 7 2.5; 10 2.5; 10 2];
            obstacle{9} = [4.5 4; 4.5 4.5; 6 4.5; 6 4];
            obstacle{10} = [0 2; 0 2.5; 2 2.5; 2 2];
            obstacle{11} = [2 6; 2 6.5; 4.5 6.5; 4.5 6];
            settings.mymap = world.PolygonMap2D(boundary, obstacle);
            
            % Path Settings
            xy = [3 9; 3 8; 1.125 8; 1.125 4.5; 3 4.5; 3 1.125; 5.75 1.125; 5.75 3; 8 4; 8 6; 6 8; 6 9; 7 9];
            settings.mypath = path.Path(xy);
            
            % Visualization Settings
            settings.figure = {'Color', [1 1 1], 'Units', 'normalized', ...
                'Position', [0.25, 0.25, 0.25, 0.25*screenAspectRatio]};
            settings.axes = {'NextPlot', 'add', 'DataAspectRatio', [1, 1, 1],...
                'Position', [0 0 1 1], 'Box', 'on', 'XGrid', 'on', 'YGrid', 'on', ...
                'XLim', [0, 10], 'XTick', 0:1:10, 'XTickLabels', [],...
                'YLim', [0, 10], 'YTick', 0:1:10, 'YTickLabels', []};
    end

    % Robot Settings
    settings.myrobot = unicyclerobot.UnicycleRobot();
    settings.myrobot.Control = unicyclesys.UnicycleHeadwayControl();
    settings.myrobot.Control.EpsilonGain = 0.5;
    settings.myrobot.Control.ReferenceGain = 1;

    settings.myrobot.BodyRadius = 0.3; % Robot Body Radius
    settings.DirectionThickness = 0.05;

end

