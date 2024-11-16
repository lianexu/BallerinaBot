function [r_dot] = casadi_ddt(r, x, x_dot)
    % r  is the quantity to differentiate through time, a function of x
    % x:     [q; q_dot] is the state vector of the system 
    % x_dot: [q_dot; q_ddot] is the time derivative of x

    r_dot = simplify(jacobian(r, x) * x_dot);
end