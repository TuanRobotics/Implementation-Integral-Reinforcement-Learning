function phi = phi_fn(x)
    x1 = x(1); x2 = x(2);
    phi = [x1^2 x1*x2 x2^2]';
end