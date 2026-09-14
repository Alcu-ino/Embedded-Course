g = 9.81;
l = 1;
M = 0.7;
m = 0.2;
k = 0;
c = 0;

tau = sqrt(g/l);
gamma = M/m;
mu = k*l/(g*m);
rho = tau*c*l/(g*m);

F = ode;
F.InitialValue = [pi/2 0 0 0]';

F.MassMatrix = @(t,y)...
 [
 1 0 0 0;
 0 1 0 0;
 0 0 1/3 0.5*cos(y(1));
 0 0 0.5*cos(y(1)) (gamma + 1)
 ];   


F.ODEFcn = @(t, y)...
    [
    y(3);
    y(4);
    -0.5*sin(y(1));
    0.5*y(3)^2*sin(y(1)) - rho*y(4) - mu*y(2)
    ];

S = solve(F, 0, 10);

y1 = S.Solution(1, :);
y2 = S.Solution(2, :);
y3 = S.Solution(3, :);
y4 = S.Solution(4, :);

plot(S.Time, 180*y1/pi, S.Time, 180*y2/pi);
legend("theta", "x", Location="southeast");

plot(S.Time, y3, S.Time, y4);
legend("theta", "x", Location="southeast");