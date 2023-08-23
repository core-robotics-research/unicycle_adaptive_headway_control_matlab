% [X, K, vol] = adaptiveheadwaycontrolCircular(Pose, k_e, res) returns the critical boundary
% points and their convex hull that contains a 2D Unicycle kinematic robot motion
% controlled via adaptive headway control. 
% Please refer to the reference page of convhull to learn more about K.
%
% Usage:
%   import unicyclesys.adaptiveheadwaycontrolCircular
%   [K, X, vol] = adaptiveheadwaycontrolCircular(Pose,res)
% Input:
%   Pose: Unicycle robot pose, (2+1) x 1 matrix
%   k_e: Scalar control gain for adaptive headway distance, float
%   res: Number of samples of the circles, integer
% Output:
%   X: Lyapunov Circle Boundary points, (res) x 2 matrix
%   K: Boundary point indicies defining their convex hull, 1 x mline vector or mtri x 3 matrix 
%   vol: Convex hull volume 


function [X, K, vol] = adaptiveheadwaycontrolCircular(Pose, k_e, res)
% Author: Aykut Isleyen, a.isleyen@tue.nl
% Created: Ferbruary 14, 2022
% Modified: March 12, 2023
    
    Pose = reshape(Pose, [], 1);
    
    headwaydist = k_e*norm(Pose(1:2));
    headway_position = Pose(1:2) + headwaydist*[cos(Pose(3)); sin(Pose(3))];
    projected_distance = dot(Pose(1:2),headway_position)/norm(headway_position);
    extended_distance = projected_distance/sqrt(1-k_e^2);

    motionpolygon_radius = extended_distance;
    if [cos(Pose(3)), sin(Pose(3))]*(-headway_position) >= 0
        motionpolygon_radius = norm(Pose(1:2));
    end
    
    [X, K, vol] =  geom.shape.circleConvhull(motionpolygon_radius, res);
    K = K(:,1)';
end