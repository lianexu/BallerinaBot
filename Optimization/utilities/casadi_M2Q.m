function [Q_M] = casadi_M2Q(M, w, q_dot)
    % M is an external moment vector applied to a body
    % w is the angular velocity vector of the body where M is being applied
    % q_dot is the vector of generalized velocities

    Q_M = simplify(jacobian(w, q_dot)'*M);
end