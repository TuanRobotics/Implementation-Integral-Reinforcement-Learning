function xdot = lqr_ode(~,x, uu_lqr)
% parameters
global Kopt; global u_lqr; global A; global B; global Q; global R;
% global uu_lqr;

x = [x(1) x(2) x(3) x(4)]';

% calculate the control signal
u_lqr = -Kopt*x;

xdot = [A*x + B*u_lqr; x'*Q*x + u_lqr'*R*u_lqr];

end