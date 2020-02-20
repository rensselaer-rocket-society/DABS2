
K = 0.02;
v0 = 100;
theta0 = 0;
thetaTest = 5*pi/180
endpoint = 150

x0 = [v0^2/2; deg2rad(theta0)];

[zstraight, xstraight] = ode45(@(z,x) dragArc(x,z,K), [0, endpoint], x0);

x0(2) = thetaTest;

[zcurve, xcurve] = ode45(@(z,x) dragArc(x,z,K), [0, endpoint], x0);


x0(2) = 0;
K = K*sec(thetaTest);

[zadj, xadj] = ode45(@(z,x) dragArc(x,z,K), [0, endpoint], x0);

figure(1);
plot(xstraight(:,1),zstraight,xcurve(:,1),zcurve,xadj(:,1),cos(thetaTest)*zadj)
xlim([0, 100])
figure(2);
plot(xcurve(:,2),zcurve);