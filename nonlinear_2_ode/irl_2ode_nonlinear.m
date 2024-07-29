function xdot = irl_2ode_nonlinear(~, x)
    global Q; global R;
    global weights;

    x = [x(1) x(2)]';
    dphi = phi_dot(x);
    
    gx = [0; cos(2*x(1)) + 2];

    u = -0.5*R^(-1)*gx'*dphi'*weights;

    f = [-x(1) + x(2);
        -0.5*x(1) - 0.5*x(2)*(1 - (cos(2*x(1)) + 2)^2)] + [0; cos(2*x(1)) + 2]*u;
    
    xdot = [f; x'*Q*x + u'*R*u];

end