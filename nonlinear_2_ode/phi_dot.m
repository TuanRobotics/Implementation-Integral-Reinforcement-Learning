function a = phi_dot(x)
x1 = x(1);
x2 = x(2);
a = [2*x1 0;
    x2 x1;
    0 2*x2]; % 3x2
end