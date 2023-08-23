% Example:
%   n = 40;
%   X2 = geom.shape.circlePoint(2, n);
%   figure; hold on; box on; grid on; axis equal;
%   scatter(X2(:,1), X2(:,2),[], 'r', 'filled');
%   X3 = geom.shape.circlePoint(3, n);
%   figure; hold on; box on; grid on; axis equal;
%   scatter3(X3(:,1), X3(:,2), X3(:,3), [], 'r', 'filled');

function X = circlePoint(r, n)

    th = flipud(linspace(0,2*pi,n)');
    X = r*[cos(th), sin(th)];
end