function [V, F, vol] = adaptiveheadwaycontrolSimulated(Pose, k_e, k_r, dx, dth)
% Author: Aykut Isleyen, a.isleyen@tue.nl
% Created: March 02, 2023
% Modified: March 12, 2023

    vol = 0;
    Pose = reshape(Pose, [], 3);
    goal = [0 0];

    [T, X] = unicyclesys.adaptive_headway_control_simulated_traj(Pose, goal, k_e, k_r, dx, dth);

    V = [X(:,1:2); flipud(X(1:end-1,1:2))];
    F = [1:(numel(V)/2)];

end