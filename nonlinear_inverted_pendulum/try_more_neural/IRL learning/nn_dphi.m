function dphi = nn_dphi(x)
    x1 = x(1); x2 = x(2); x3 = x(3); x4 = x(4);
    dphi = [2*x1 0 0 0;
        x2 x1 0 0;
        x3 0 x1 0;
        x4 0 0 x1;
        0 2*x2 0 0;
        0 x3 x2 0;
        0 x4 0 x2;
        0 0 2*x3 0;
        0 0 x4 x3;
        0 0 0 2*x4;
        3*x1^2 0 0 0;
        0 3*x2^2 0 0;
        0 0 3*x3^2 0;
        0 0 0 3*x4^2;
        4*x1^3 0 0 0;
        0 4*x2^3 0 0;
        0 0 4*x3^3 0;
        0 0 0 4*x4^3]; % 18x4
%     3*x1^2*x2^2 2*x1^3*x2 0 0;
%         3*x1^2*x3^2 0 2*x1^3*x2 0;
%         3*x1^2*x4^2 0 0 x1^3*2*x4;
%         0 3*x2^2*x3^2 x2^3*2*x3 0;
%         0 3*x2^2*x4^2 0 x2^3*2*x4;
%         0 0 3*x3^2*x4^2 x3^3*2*x4

end