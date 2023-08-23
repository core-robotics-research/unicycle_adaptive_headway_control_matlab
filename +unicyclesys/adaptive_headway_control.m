% Author: Aykut Isleyen, a.isleyen@tue.nl
% Created: March 02, 2023
% Modified: March 12, 2023
function [v, w] = adaptive_headway_control(Pose, goal, k_e, k_r)

    Pose = reshape(Pose, [], 1);
    goal = reshape(goal, [], 1);

    x = Pose(1:2);
    theta = Pose(3);

    if norm(goal - x) > 1e-5
        v = k_r * norm(goal-x) * ([cos(theta) sin(theta)]*(goal-x)/norm(goal-x)-k_e) / (1-k_e*[cos(theta) sin(theta)]*(goal-x)/norm(goal-x));
        w = (k_r/k_e)*( ([-sin(theta) cos(theta)]*(goal-x))/norm(goal-x) );
    else
        v = 0;
        w = 0;
    end

end