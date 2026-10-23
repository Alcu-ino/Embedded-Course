    g = 9.81
    l1 = 0.35
    l2 = 0.65
    L = 2/3*(l1^2-l1*l2+l2^2)/abs(l2-l1)
    m = 0.1
    M = 1.0
    
    Api = [0 1 0 0
           0 0 -m*g/M 0
           0 0 0 1
           0 0 (M+m)*g/(L*M) 0];
    Bpi = [0
         1/M
         0
         -1/(L*M)
        ];
    
    C = [1 0 0 0
         0 1 0 0
         0 0 1 0
         0 0 0 1
        ];
    D = [0
         0
         0
         0
        ];
    
    sys = ss(Api,Bpi,C,D);
    dx0 = [0.45; 0; -0.279253; 0.4];
    [y,t,x] = initial(sys,dx0,10);
    
    % ritorno alle variabili fisiche
    pos   = y(:,1);
    vel = y(:,2);
    theta = y(:,3) + pi;
    w = y(:,4);
    figure(1)
    subplot(4,1,1); plot(t,pos);   ylabel('x [m]');       grid on
    subplot(4,1,2); plot(t,theta); ylabel('\theta [rad]'); grid on
    xlabel('t [s]')
    subplot(4,1,3); plot(t,vel);   ylabel('v [m/s]');       grid on
    subplot(4,1,4); plot(t,w); ylabel('\omega [rad/s]'); grid on
    xlabel('t [s]')
    
    eig(Api)
    Q = [100 0 0 0; 0 1000 0 0;0 0 2000 0;0 0 0 1000];
    R = 1;
    [K,S,p] = lqr(sys,Q,R)
    sys = ss(Api-Bpi*K,zeros(4,1),C,D);
    [y,t,x] = initial(sys,dx0,10);
    
    % ritorno alle variabili fisiche
    pos   = y(:,1);
    vel = y(:,2);
    theta = y(:,3) + pi;
    w = y(:,4);
    figure(2)
    subplot(4,1,1); plot(t,pos);   ylabel('x [m]');       grid on
    subplot(4,1,2); plot(t,theta); ylabel('\theta [rad]'); grid on
    xlabel('t [s]')
    subplot(4,1,3); plot(t,vel);   ylabel('v [m/s]');       grid on
    subplot(4,1,4); plot(t,w); ylabel('\omega [rad/s]'); grid on
    xlabel('t [s]')
    
    Ka = K/M + [0 0 m*g/M 0]%DA ACC
    %a = Ka*dx
    %astep = a/m_per_step, VA PASSATA A FUNZIONE MOTORE

    %%
    %OSSERVATORE DI STATO
    Erif = m*g*L
    K = 1200
    Erif*K