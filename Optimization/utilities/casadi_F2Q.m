function [Q_F] = casadi_F2Q(F, r, q)
    % F is a linear force vector
    % r is the position vector where F is being applied
    % q is a vector ofthe generalized coordinates

    Q_F = simplify(jacobian(r, q)'*F);
end