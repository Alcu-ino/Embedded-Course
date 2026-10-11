% Acceleration-input reference model matching pendulum_control.c.
% This script checks the ideal LINEAR upright design, not nonlinear swing-up.
% For executable nonlinear regression, run simulation/run_simulation.py.
% Geometry is an unverified uniform-rod interpretation of the original file.
g = 9.81;
l1 = 0.35;
l2 = 0.65;
m = 0.1;
d = abs(l2 - l1) / 2;                  % pivot-to-COM distance [m]
J = m * (l1^2 - l1*l2 + l2^2) / 3;    % pivot inertia [kg m^2]
L_eff = J / (m*d);                    % dynamic effective length [m]
E_upright = 2*m*g*d;                  % down-to-up energy [J]

% Coordinates: theta=0 DOWN, increasing toward +cart x near down.
% e=wrap(theta-pi); rightward bob lean at upright has NEGATIVE e.
% Nonlinear plant: theta_ddot=-(g*sin(theta)+a*cos(theta))/L_eff.
% States [x; v; e; omega], input cart acceleration a [m/s^2].
A = [0 1 0 0; 0 0 0 0; 0 0 0 1; 0 0 g/L_eff 0];
B = [0; 1; 0; 1/L_eff];

% Feedback a=kx*x+kv*v-kt*e-kw*omega; polynomial (s+2)^2*(s+3)^2.
kx = 36*L_eff/g;
kv = 60*L_eff/g;
kw = L_eff*(10+kv);
kt = L_eff*(37+kx)+g;
K = [-kx, -kv, kt, kw];             % a=-K*state
A_closed = A-B*K;
assert(max(abs(poly(A_closed)-[1 10 37 60 36])) < 1e-8);
fprintf('COM %.6f m, effective length %.6f m, upright energy %.6f J\n', d, L_eff, E_upright);
fprintf('Feedback [kx kv kt kw] = %.6f %.6f %.6f %.6f\n', kx, kv, kt, kw);
disp('Ideal continuous closed-loop eigenvalues:'); disp(eig(A_closed));

% No Control System Toolbox needed. This unsaturated ideal calculation omits
% quantization, motor speed/acceleration limits, rail limits and friction.
t = 0:0.001:10;
z = zeros(4, length(t));
z(:,1) = [0.02; 0; 0.03; 0];
transition = expm(A_closed*0.001);
for i=2:length(t)
    z(:,i) = transition*z(:,i-1);
end
figure(1);
labels = {'cart x [m]', 'cart v [m/s]', 'upright error [rad]', 'angular rate [rad/s]'};
for i=1:4
    subplot(4,1,i); plot(t,z(i,:)); ylabel(labels{i}); grid on;
end
xlabel('time [s]');
