% Demo for unicycle feedback motion "predictions" that bound the closed-loop 
% unicycle motion trajectory under adaptive headway control
% towards a given goal position.
% Motion Prediction Methods: circular, triangle

% Author: Aykut Isleyen, a.isleyen@tue.nl
% Created: February 15, 2023
% Modified: August 09, 2023

%% Start  a clean simulation environment
close all % Close all figures
clear variables % Clear all imports and variables

%% Scenario Settings
x0 = [2 2 pi/2.5];
epsilon_gain = 0.5;
saveFigure = false;

settings = advanced_settings();
myrobot = unicyclesys.UnicycleHeadwayControl();
settings.x0 = x0;
settings.gain = epsilon_gain;
myrobot.EpsilonGain = settings.gain;
myrobot.MotionPolygonResolution = settings.MotionPolygonResolution;
x0  = settings.x0;
goal = settings.goal;
%% Demo Settings
tspan = [0 10]; 
odeOptions = odeset('MaxStep', 0.1, 'RelTol', 1e-3, 'AbsTol', 1e-6);
[T, X] = myrobot.traj(tspan, x0, odeOptions);
[Thead, Xhead] = myrobot.headway_traj(tspan, x0, odeOptions);

%% Motion Visualization

h.figure = figure(settings.figure{:});
h.axes = axes(settings.axes{:});

myrobot.MotionPolygonMethod = 'Circular';
P = myrobot.motionpolygon(x0, goal);
patch('XData', P(:,1), 'YData',P(:,2), settings.CirclemotionPatch{:});
myrobot.MotionPolygonMethod = 'Triangular';
P = myrobot.motionpolygon(x0, goal);
patch('XData', P(:,1), 'YData',P(:,2), settings.TrianglemotionPatch{:});

plot(X(:,1), X(:,2), settings.pathPlot{:});
plot(Xhead(:,1), Xhead(:,2), settings.headwaypathPlot{:});
scatter(goal(1), goal(2), settings.scatter{:});
scatter(Xhead(1,1), Xhead(1,2), settings.headwayposscatter{:});

xe = myrobot.extended_position(x0, goal);
xp = myrobot.projected_position(x0, goal);
scatter(xp(1), xp(2), settings.headwayprojectedscatter{:});
scatter(xe(1), xe(2), settings.headwayextendedscatter{:});

% Robot Visualization
th = linspace(0,2*pi,100); % Angle samples for the visualization of robot body 
thwl = linspace(5*pi/6, pi/6, 60); % Angle samples for the visualization of the left robot wheel
thwr = linspace(7*pi/6, 11*pi/6, 60); % Angle sample for the visualization of the right robot wheel
R = settings.RobotRadius ;
robot1 = patch('XData', X(1,1) + R*cos(th),'YData', X(1,2) + R*sin(th), 'FaceColor',  [77/255, 190/255, 238/255]);
robot2 = patch('XData', X(1,1) + R*cos(thwl+X(1,3)), 'YData',  X(1,2) + R*sin(thwl+X(1,3)), 'FaceColor', 'k');
robot3 = patch('XData', X(1,1) + R*cos(thwr+X(1,3)), 'YData',  X(1,2) + R*sin(thwr+X(1,3)), 'FaceColor', 'k');
thick = settings.DirectionThickness;
thdir = [ pi/2, -pi/2, linspace(-asin(thick/R), asin(thick/R), 60)];
mdir = [repmat(thick, 1, 2), repmat(R, 1, 60)];
robot4 = patch('XData', X(1,1)+mdir.*cos(thdir+X(1,3)),'YData', X(1,2) + mdir.*sin(thdir+X(1,3)), 'FaceColor', 'k');

%% Saving Options
if saveFigure
  saveas(gcf, settings.filename, 'png');
  disp('Simulation figure is saved!!!');
end

%% Advanced Settings
function settings = advanced_settings(settings)
    
    % Visualization Options
    settings.filename = sprintf('temp/motion_bound_demo');
    screenSize = get(0, "ScreenSize");
    screenAspectRatio =screenSize(3)/screenSize(4); 
    settings.figure = {'Units', 'normalized', 'Position', [0.35, 0.35, 0.3, 0.3*screenAspectRatio], 'Color', [1 1 1]};
    settings.axes = {'NextPlot', 'add', 'DataAspectRatio', [1, 1, 1],...
        'LineWidth', 1.0, 'Position', [0 0 1 1], 'Box', 'on',...
        'XGrid', 'on', 'YGrid', 'on', 'Layer', 'top' ...
        'XLim', [-3.5, 3.5], 'XTick', -4:4, 'XTickLabels', [],...
        'YLim', [-3.5, 3.5], 'YTick', -4:4, 'YTickLabels', []};

    settings.RobotRadius = 0.3;
    settings.DirectionThickness = 0.05;

    settings.pathPlot = {'Color', [0 0 0], 'LineStyle', '-', 'LineWidth', 2};
    settings.headwaypathPlot = {'Color', [0 0 0], 'LineStyle', '--', 'LineWidth', 2};
    settings.scatter = {50, 'r', 'filled', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r'};
    settings.headwayposscatter = {50, 'filled', 'MarkerFaceColor', [207/255 159/255 255/255], 'MarkerEdgeColor', 'k'};
    settings.headwayprojectedscatter = {50, 'filled', 'MarkerFaceColor', [0.4660 0.6740 0.1880], 'MarkerEdgeColor', 'k'};
    settings.headwayextendedscatter = {50, 'filled', 'MarkerFaceColor',  [0.3010, 0.7450, 0.9330], 'MarkerEdgeColor', 'k'};
    settings.CirclemotionPatch = {'FaceColor', [0.9804, 0.9137, 0.7490], 'EdgeColor', 'none', 'FaceAlpha', 1};
    settings.TrianglemotionPatch = {'FaceColor', [1 0.6 0.6]	, 'EdgeColor', 'k', 'FaceAlpha', 1};

    settings.goal = [0; 0];
    settings.MotionPolygonResolution = 60;
end