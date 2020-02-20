function xdot = dragArc(x,z,K)
    g = 9.81;
    eps = x(1);
    theta = x(2);
    xdot = [-K*eps*sec(theta)-g; g*tan(theta)/(2*eps)];
end