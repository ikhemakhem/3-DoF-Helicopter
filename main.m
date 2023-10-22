clear all;
close all;

syms alpha beta gamma dalpha_dt dbeta_dt dgamma_dt u_F u_B real

%%
% Definition Konstanten
L = 0.655;
l = 0.355/2;
g = 9.81;
k = 0.3; % viskoser Reibkoeffizient
off_1 = 42*10^-3;
off_2 = 19*10^-3;
Fg = 2.124*beta+1.105;

% Trägheiten
[I_alpha, I_beta, I_gamma] = get_inertias(beta, gamma);

% Koeffizienten Polynom Motorkennlinie (u --> F)
P_front_u_F = [-0.0018    0.0183   -0.0109    0.0332   -0.0017];
P_back_u_F = [-0.0046    0.0366   -0.0375    0.0442   -0.0018];

% Koeffizienten Polynom F --> u
P_front = [-79.0309  129.6621  -75.5450   21.8917    0.1664];
P_back = [-66.4438  116.2250  -70.6523   20.7182    0.1758];


% Umrechnung von u --> F
F_F = [u_F^4 u_F^3 u_F^2 u_F 1]*P_front_u_F';
F_B = [u_B^4 u_B^3 u_B^2 u_B 1]*P_back_u_F';

% Zustandsvektor
x = [alpha beta gamma dalpha_dt dbeta_dt dgamma_dt]';
u = [u_F u_B]';

dx = [dalpha_dt;...
      dbeta_dt;...
      dgamma_dt;...
      (-L*cos(beta)*sin(gamma)*(F_F + F_B) - dalpha_dt*k)/I_alpha;...
      (L*cos(gamma)*(F_F + F_B) + Fg*(-L*cos(beta) - sin(beta)*(off_1 + 0.5*off_2)))/I_beta;...
      l*(F_F - F_B)/I_gamma];

%% Linearisierung
% Berechnung des stationären Eingangs
x_s = [0, deg2rad(-22), 0, 0, 0, 0];
dx_s = subs(dx, alpha, x_s(1));
dx_s = subs(dx_s, beta, x_s(2));
dx_s = subs(dx_s, gamma, x_s(3));
dx_s = subs(dx_s, dalpha_dt, x_s(4));
dx_s = subs(dx_s, dbeta_dt, x_s(5));
dx_s = subs(dx_s, dgamma_dt, x_s(6));

u_s = solve(dx_s == 0, [u_F u_B]);
% Erster Eintrag, da einziger physikalisch sinnvoller Wert
u_Fs = double(u_s.u_F(1));
u_Bs = double(u_s.u_B(1));

% Jacobi-Linearisierung
A = jacobian(dx, x);
B = jacobian(dx, u);

% Stationäre Werte
Fg_s = subs(Fg, beta, x_s(2));

% A-Matrix
A = subs(A, alpha, x_s(1));
A = subs(A, beta, x_s(2));
A = subs(A, gamma, x_s(3));
A = subs(A, dalpha_dt, x_s(4));
A = subs(A, dbeta_dt, x_s(5));
A = subs(A, dgamma_dt, x_s(6));
A = subs(A, u_F, u_Fs);
A = subs(A, u_B, u_Bs);
A = double(A);

% B-Matrix
B = subs(B, alpha, x_s(1));
B = subs(B, beta, x_s(2));
B = subs(B, gamma, x_s(3));
B = subs(B, dalpha_dt, x_s(4));
B = subs(B, dbeta_dt, x_s(5));
B = subs(B, dgamma_dt, x_s(6));
B = subs(B, u_F, u_Fs);
B = subs(B, u_B, u_Bs);
B = double(B); 

C = [1, 0, 0, 0, 0, 0;...
    0, 1, 0, 0, 0, 0;...
    0, 0, 1, 0, 0, 0];

Ctrl = ctrb(A,B);
rank(Ctrl)

O = obsv(A,C);
rank(O)

A_hat = [A, zeros(6,2);...
        -C(1:2,:), zeros(2,2)];
B_hat = [B; zeros(2,2)];

Q = 0.5*diag([30, 50, 5, 0.1, 0.1, 0.1, 200, 200]);  %good values
% Q = diag([50, 50, 20, 0.1, 0.1, 0.1, 200, 200]);
R = 2*eye(2);
% sys = ss(A,B,C,D);
% [K,S,e] = lqi(sys,Q,R);
[K, S, P] = lqr(A_hat,B_hat,Q,R);
% Ki = [0, -0.1, 0;...
%     0.1, 0, 0];

%observer
% P = eig(A - B*K(:, 1:6));
P_beo = [-20, -21, -22, -23, -24, -25];
H = place(A', C', P_beo)';

%% %Trajectory
%% Abschnitt 1
[alpha1, ~, ~] = lspb(0, 0, 10000);
[beta1, ~, ~] = lspb(-27, 0, 10000);

[alpha2, ~, ~] = lspb(0, 90, 10000);
[beta2, ~, ~] = lspb(0, 0, 10000);

[alpha3, ~, ~] = lspb(90, 90, 10000);
[beta3, ~, ~] = lspb(0, -22, 10000);

pos1 = deg2rad([alpha1, beta1; alpha2, beta2; alpha3, beta3]);

%% Abschnitt 2
[alpha1, ~, ~] = lspb(90, 90, 10000);
[beta1, ~, ~] = lspb(-22, 0, 10000);

[alpha2, ~, ~] = lspb(90, 450, 20000);
[beta2, ~, ~] = lspb(0, 0, 20000);


[alpha3, ~, ~] = lspb(450, 450, 10000);
[beta3, ~, ~] = lspb(0, -22, 10000);

pos2 = deg2rad([alpha1, beta1; alpha2, beta2; alpha3, beta3]);

%% Abschnitt 3
[alpha1, ~, ~] = lspb(450, 450, 10000);
[beta1, ~, ~] = lspb(-22, 1, 10000);

[alpha2, ~, ~] = lspb(450, 0, 30000);
[beta2, ~, ~] = lspb(1, 1, 30000);

[alpha3, ~, ~] = lspb(0, 0, 10000);
[beta3, ~, ~] = lspb(1, -27, 10000);

pos3 = deg2rad([alpha1, beta1; alpha2, beta2; alpha3, beta3]);
%% Anti_Windup
K_w = K(:,7:8)\eye(2);