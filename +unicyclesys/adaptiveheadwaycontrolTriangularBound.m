function [X, K, vol] = adaptiveheadwaycontrolTriangularBound(Pose, k_e)
% Author: Aykut Isleyen, a.isleyen@tue.nl
% Created: Ferbruary 14, 2022
% Modified: March 12, 2023

    Pose = reshape(Pose, 3, 1);
    headwaydist = k_e*norm(Pose(1:2));
    headway_position = Pose(1:2) + headwaydist*[cos(Pose(3)); sin(Pose(3))];
    
    if norm(Pose(1:2)) ~= 0
        t_e = (headway_position)/norm(headway_position);
    else
        t_e = [0; 0];
    end

    if (Pose(1:2)'*[-sin(Pose(3)); cos(Pose(3)) ] > 0 )
        n_e = [cos(pi/2), -sin(pi/2); sin(pi/2), cos(pi/2)]*t_e;
    else
        n_e = [cos(-pi/2), -sin(-pi/2); sin(-pi/2), cos(-pi/2)]*t_e;
    end

    projected_pos = t_e*t_e'*Pose(1:2);
    projected_distance = norm(projected_pos);        
    extended_pos = projected_pos + (k_e/sqrt(1-k_e^2))*projected_distance*n_e;    

    X = [[0 0]; extended_pos'; projected_pos'];
    if [cos(Pose(3)), sin(Pose(3))]*(-Pose(1:2)/norm(Pose(1:2))) >= k_e
            X = [Pose(1:2)'; headway_position'; [0 0]];
    end

    [K, vol] =  geom.convhulln(X);
    K = K(:,1)';
end