g = 9.81;
l = 1;
M = 0.7;
m = 0.2;
k = 0.5;

tau = sqrt(g/l);
gamma = M/m;
mu = k*l/(g*m);

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
    0.5*y(3)^2*sin(y(1)) - mu*y(4)
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

A = [
    0 1 0 0; 
    (6*g*(m+M))/(l*(m+4*M)) 0 0 -6*k/(l*(m+4*M)); 
    0 0 0 1; 
    3*g*m/(m+4*M) 0 0 -4*k/(m+4*M) 
    ];

Bu = [
    0; 
    -4/(m+4*M); 
    0; 
    -6/(l*(m+4*M))
    ];

Bd = [
    0;
    -3*m*g*pi/(m+4*M);
    0;
    -6*g*pi*(m+M)/(l*m+4*M)
     ];

B = [Bu Bd];

C = [
    1 0 0 0
    0 1 0 0
    0 0 1 0
    0 0 0 1
    ];

D = [
    0;
    0;
    0;
    0;
    ]
%%

Q = [
    400 0 0 0;
    0 1 0 0;
    0 0 500 0;
    0 0 0 1
];
R = 1;

[K, S, P] = lqr(A, Bu, Q, R);

sys = ss(A - Bu*K, Bd, C, D);

%% Simulazione
t  = linspace(0, 10, 1000)';   % COLONNA (con apostrofo) e più punti
x0 = [0; 0; 0; 0];              % COLONNA (con punti e virgola)
u  = ones(length(t), 1);        % COLONNA — disturbo costante unitario

[y, t_out, x_out] = lsim(sys, u, t, x0);

%% Plot degli stati
figure;
plot(t_out, x_out, 'LineWidth', 1.5);
legend('ANGOLO θ', 'VEL. ANGOLARE θ̇', 'POSIZIONE x', 'VELOCITA ẋ', ...
       'Location', 'best');
xlabel('Tempo [s]');
ylabel('Stati');
title('Risposta del sistema chiuso con LQR');
grid on;