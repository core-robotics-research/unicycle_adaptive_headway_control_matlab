% [X, K, vol] = adaptiveheadwaycontrolTriangular(Pose, k_e) returns the 
% traingle vertices that contain a 2D Unicycle kinematic robot motion
% controlled via adaptive headway control. 
% Please refer to the reference page of convhull to learn more about K.
%
% Usage:
%   import unicyclesys.adaptiveheadwaycontrolTriangular
%   [K, X, vol] = adaptiveheadwaycontrolTriangular(Pose,res)
% Input:
%   Pose: Unicycle robot pose, (2+1) x 1 matrix
%   k_e: Scalar control gain for adaptive headway distance, float
% Output:
%   X: Triangle vertices points, 3 x 2 matrix
%   K: Boundary point indicies defining their convex hull, 1 x mline vector or mtri x 3 matrix 
%   vol: Convex hull volume 


function [X, K, vol] = adaptiveheadwaycontrolTriangular(Pose, k_e)
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
    extended_reflected = -extended_pos + 2*extended_pos'*projected_pos/norm(projected_pos)*(projected_pos/norm(projected_pos));

    X = [[0 0]; extended_pos'; extended_reflected'];
    if [cos(Pose(3)), sin(Pose(3))]*(-Pose(1:2)/norm(Pose(1:2))) >= k_e
            x3 = headway_position + (1 - [cos(Pose(3)), sin(Pose(3))]*(-Pose(1:2)/norm(Pose(1:2))))/(1-k_e)*headwaydist*[cos(Pose(3)); sin(Pose(3))];
            X = [Pose(1:2)'; x3'; [0 0]];
    end

    [K, vol] =  geom.convhulln(X);
    K = K(:,1)';
end