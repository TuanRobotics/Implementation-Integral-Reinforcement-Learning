function xdot = irl_2ode_nonlinear(~,x)
    global K; global u; global fx; global gx; global Q; global R;
    global M; global m; global g; global L; global weights; global dphi_w0;
    global k;

    x1 = x(1); % theta 
    x2 = x(2); % theta_dot
    x3 = x(3); %x 
    x4 = x(4); % x_dot
    x = [x1 x2 x3 x4]';
    
    dphi_w0 = nn_dphi_w0(x);

    g2 = cos(x1)/(m*L*cos(x1)^2 - (M+m)*L);
    g4 = 1/(M + m - m*cos(x1)^2);
    gx = [0;
        g2;
        0; 
        g4];
    
    u = -0.5*inv(R)*gx'*dphi_w0'*weights; % 0.01*(rand(1,1)-0.5)

    u_new= u; % + exp(-0.001*t)*37*(sin(t)^2*cos(t)+sin(2*t)^2*cos(0.1*t)+sin(-1.2*t)^2*cos(0.5*t)+sin(t)^5+sin(1.12*t)^2+cos(2.4*t)*sin(2.4*t)^3);
    
    fx = [x2;
       (-(M+m)*g*sin(x1) + m*L*(cos(x1)*sin(x1))*x2^2)/(m*L*cos(x1)^2 - (M+m)*L);
       x4;
       (m*L*sin(x1)*x2^2 - m*g*cos(x1)*sin(x1))/(M + m - m*cos(x1)^2)];

    g2 = cos(x1)/(m*L*cos(x1)^2 - (M+m)*L);
    g4 = 1/(M + m - m*cos(x1)^2);
    gx = [0;
        g2;
        0; 
        g4];

    xdot = [fx + gx*u_new; x'*Q*x + u_new'*R*u_new];

end
