%% pendolo_lqr_offset.m
%  Pendolo inverso su carrello con stepper.
%  Parte dallo script originale (modello in forza, LQR a 4 stati, conversione
%  in accelerazione) e aggiunge:
%    - lo stato integrale z = integrale di (p - r) sulla posizione del carrello
%    - la simulazione con offset dell'encoder e riferimento di posizione
%    - la stima dell'offset dell'encoder ricavata da z
%
%  Stato (scostamenti dalla verticale alta): dx = [p; v; phi; w]
%  Richiede il Control System Toolbox.

clear; clc; close all;

%% 1. Parametri ----------------------------------------------------------------
g  = 9.81;
l1 = 0.35;                 % [m] tratto dell'asta sotto il perno
l2 = 0.65;                 % [m] tratto dell'asta sopra il perno
L  = 2/3*(l1^2 - l1*l2 + l2^2)/abs(l2 - l1);   % [m] lunghezza equivalente
m  = 0.1;                  % [kg] massa del pendolo
M  = 1.0;                  % [kg] massa del carrello

% ATTENZIONE: con l1 = 0.38, l2 = 0.62 risulta L = 0.8144 m.
% Nel firmware (lqr.h, swingup.h) c'e' L = 0.7056 m (0.35 / 0.65).
% Le due lunghezze devono coincidere: misura l'asta e usa lo stesso valore.
fprintf('L = %.4f m   (nel firmware LQR_L = 0.7056 m)\n', L);

q_enc = 2*pi/2400;         % [rad] un conteggio dell'encoder
u_max = 8.0;               % [m/s^2] LQR_ACC_MAX

%% 2. Modello in FORZA (come nello script originale) ---------------------------
Api = [0 1 0               0
       0 0 -m*g/M          0
       0 0 0               1
       0 0 (M+m)*g/(L*M)   0];
Bpi = [0
       1/M
       0
      -1/(L*M)];
C = eye(4);
D = zeros(4,1);

sys = ss(Api, Bpi, C, D);
fprintf('Poli ad anello aperto: %s\n', mat2str(eig(Api).', 4));

%% 3. Risposta libera ad anello aperto ------------------------------------------
dx0 = [0.45; 0; -0.279253; 0.4];
[y, t] = initial(sys, dx0, 2);          % 2 s bastano: diverge
grafico_stati(1, t, y, 'Anello aperto (instabile)');

%% 4. LQR a 4 stati (come nello script originale) -------------------------------
Q = diag([1000 100 2000 100]);        % diag([1000 1000 2000 1000]); pesi su [p v phi w]
R = 10;
K = lqr(sys, Q, R);                     % guadagni in forza: F = -K*dx

% Conversione in ACCELERAZIONE (lo stepper impone a, non F):
%   a = F/M - (m*g/M)*phi   ->   a = -Ka*dx
Ka = K/M + [0 0 m*g/M 0];
fprintf('\nKa a 4 stati = %s\n', mat2str(Ka, 6));

fprintf('Spostamento del carrello per 1 conteggio di offset: %.1f cm\n', ...
        100*abs(Ka(3)/Ka(1))*q_enc);

sys_cl = ss(Api - Bpi*K, zeros(4,1), C, D);
[y, t] = initial(sys_cl, dx0, 10);
grafico_stati(2, t, y, 'LQR a 4 stati');

%% 5. LQR con azione integrale sulla posizione del carrello ---------------------
%  Stato aumentato [dx; z], con z' = p - r
Cz  = [1 0 0 0];                        % si integra SOLO la posizione
Aa  = [Api zeros(4,1); Cz 0];
Ba  = [Bpi; 0];
q_z = 200;                              % peso su z: parti basso e alza
Qa  = blkdiag(Q, q_z);

fprintf('\nControllabilita'' con z sulla posizione: rango %d su 5\n', rank(ctrb(Aa, Ba)));
fprintf('Controllabilita'' con z sull''angolo:    rango %d su 5\n', ...
        rank(ctrb([Api zeros(4,1); [0 0 1 0] 0], Ba)));

K5  = lqr(Aa, Ba, Qa, R);               % in forza: F = -K5*[dx; z]
Ka5 = K5/M + [0 0 m*g/M 0 0];           % in accelerazione
Kap = Ka5(1:4);
Kai = Ka5(5);
fprintf('\nKa aumentato = %s\n', mat2str(Ka5, 6));

%% 6. Simulazione con offset dell'encoder e riferimento di posizione ------------
%  Impianto come lo vede lo stepper (ingresso = accelerazione a):
%     p'' = a ,   phi'' = (g/L)*phi - a/L
%  L'encoder misura phi + delta (delta = offset fisso, ignoto al controllore).
%  Legge:  a = -Kap*([p - r; v; phi + delta; w]) - Kai*z
Aacc = [0 1 0 0; 0 0 0 0; 0 0 0 1; 0 0 g/L 0];
Bacc = [0; 1; 0; -1/L];

delta     = 2*q_enc;                    % [rad] offset: 2 conteggi
r_gradino = 0.05;                       % [m] riferimento dopo t_gradino
t_gradino = 20;                         % [s]
T_fin     = 40;                         % [s]
x0        = [0; 0; 0.03; 0];            % aggancio con 0.03 rad di errore

t = (0:0.004:T_fin).';
ingressi = [delta*ones(size(t)), r_gradino*(t >= t_gradino)];   % [delta, r]

% Anello chiuso SENZA integratore (stato dx)
A4  = Aacc - Bacc*Ka;
B4  = [-Bacc*Ka(3), Bacc*Ka(1)];
y4  = lsim(ss(A4, B4, eye(4), zeros(4,2)), ingressi, t, x0);

% Anello chiuso CON integratore (stato [dx; z])
A5  = [Aacc - Bacc*Kap, -Bacc*Kai; Cz, 0];
B5  = [-Bacc*Kap(3), Bacc*Kap(1); 0, -1];
y5  = lsim(ss(A5, B5, eye(5), zeros(5,2)), ingressi, t, [x0; 0]);
z   = y5(:,5);
fprintf('Poli anello chiuso con integratore: %s\n', mat2str(eig(A5).', 4));

% Comando di accelerazione (per controllare che resti sotto u_max)
a5 = -(y5(:,1:4) - ingressi(:,2)*[1 0 0 0] + ingressi(:,1)*[0 0 1 0])*Kap.' - Kai*z;
fprintf('Accelerazione massima richiesta: %.2f m/s^2 (limite %.1f)\n', max(abs(a5)), u_max);

%% 7. Stima dell'offset dell'encoder a partire da z -----------------------------
%  A regime (carrello sul riferimento, asta dritta) il comando e' nullo:
%     Kap(3)*delta + Kai*z = 0   ->   delta_stimato = -Kai*z/Kap(3)
delta_stimato = -Kai*z/Kap(3);

figure(3); set(gcf, 'Color', 'w', 'Name', 'Offset encoder e riferimento di posizione');
subplot(3,1,1);
plot(t, 100*ingressi(:,2), 'k--', t, 100*y4(:,1), t, 100*y5(:,1), 'LineWidth', 1.2);
ylabel('p [cm]'); grid on;
legend('riferimento', '4 stati', 'con integratore', 'Location', 'best');
title(sprintf('Offset encoder = %.0f conteggi,  q_z = %g', delta/q_enc, q_z));

subplot(3,1,2);
plot(t, rad2deg(y4(:,3)), t, rad2deg(y5(:,3)), 'LineWidth', 1.2);
ylabel('\phi vero [gradi]'); grid on;

subplot(3,1,3);
plot(t, delta_stimato/q_enc, t, delta/q_enc*ones(size(t)), 'k--', 'LineWidth', 1.2);
ylabel('offset [conteggi]'); xlabel('t [s]'); grid on;
legend('stimato da z', 'vero', 'Location', 'best');

i1 = find(t < t_gradino, 1, 'last');
fprintf('\nCarrello prima del gradino (riferimento 0):   4 stati %+6.2f cm, con integratore %+6.2f cm\n', ...
        100*y4(i1,1), 100*y5(i1,1));
fprintf('Carrello a fine prova (riferimento %.0f cm):    4 stati %+6.2f cm, con integratore %+6.2f cm\n', ...
        100*r_gradino, 100*y4(end,1), 100*y5(end,1));
fprintf('Offset stimato prima del gradino: %.2f conteggi (vero: %.0f)\n', ...
        delta_stimato(i1)/q_enc, delta/q_enc);

%% 8. Righe da copiare in lqr.h --------------------------------------------------
fprintf('\n/* ---- da copiare in lqr.h ---- */\n');
fprintf('#define LQR_L      %.4ff\n', L);
fprintf('#define LQR_K1     (%.4ff)   /* su p   [m]     */\n', Ka5(1));
fprintf('#define LQR_K2     (%.4ff)   /* su v   [m/s]   */\n', Ka5(2));
fprintf('#define LQR_K3     (%.4ff)   /* su phi [rad]   */\n', Ka5(3));
fprintf('#define LQR_K4     (%.4ff)   /* su w   [rad/s] */\n', Ka5(4));
fprintf('#define LQR_K5     (%.4ff)   /* su z   [m*s]   */\n', Ka5(5));
fprintf('#define LQR_Z_MAX  (%.4ff)   /* [m*s] = LQR_ACC_MAX/|K5| */\n', u_max/abs(Kai));

%% 9. Swing-up: energia di riferimento -------------------------------------------
%  (dallo script originale; rinominato per non sovrascrivere il K dell'LQR)
MGL   = m*g*L;             % [J] SWINGUP_MGL; fermo in alto l'energia vale 2*MGL
K_su  = 1200;
fprintf('\nm*g*L = %.4f J,   K_su*m*g*L = %.1f\n', MGL, K_su*MGL);

%% ============================================================================
function grafico_stati(n, t, y, titolo)
    % y = [p v phi w] come scostamenti; l'angolo si riporta a theta = phi + pi
    figure(n); set(gcf, 'Color', 'w', 'Name', titolo);
    subplot(4,1,1); plot(t, y(:,1));      ylabel('x [m]');          grid on; title(titolo);
    subplot(4,1,2); plot(t, y(:,3) + pi); ylabel('\theta [rad]');   grid on;
    subplot(4,1,3); plot(t, y(:,2));      ylabel('v [m/s]');        grid on;
    subplot(4,1,4); plot(t, y(:,4));      ylabel('\omega [rad/s]'); grid on;
    xlabel('t [s]');
end