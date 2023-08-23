function [T, X] = adaptive_headway_control_simulated_traj(Pose, goal, k_e, k_r, dx, dth)
   

    Pose = reshape(Pose, [], 3);
    goal = reshape(goal, [], 2);
    
    x = Pose(1:2);
    theta = Pose(3);   

    stopCond = dx;

    k = 1;
    T = [0];
    while(norm(x(k,:)-goal) > stopCond)

        [v, w] = unicyclesys.adaptive_headway_control([x(k,:), theta(k)], goal, k_e, k_r);
        dt = min( abs(dx/v) , abs(dth/w));
        T = [T; T(end) + dt];

        x(k+1,:) = x(k,:) + dt*v*[cos(theta(k)) sin(theta(k))];
        theta(k+1,:)= theta(k,:)+ w*dt;

        k = k+1;
    end
    X = [x, theta];

end