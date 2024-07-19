%% Matrices y observador
clc
clear

T = 0.01;
a = 17.75;
b = 1.145;
c = 75.57;
k1 = 120.1;
k2 = -0.6455;

A = [0 1 0 0;-k1 -a 0 0; 0 0 0 1; -k1*k2 -k2*a -c -b];
B = [0;k1;0;k1*k2];
C = [0 0 1 0;1 0 0 0];
D = zeros(2,1);

Ad = [1 T 0 0;-k1*T -a*T+1 0 0;0 0 1 T;-k1*k2*T -k2*a*T -c*T -b*T+1];
Bd = [0;k1*T;0;k1*k2*T];
Cd = C;
Dd = D;

pL = [0.4 0.25 0.96 0.9];
L = place(Ad',Cd', pL)';

for j = 1:4
    nombre1 = strcat('float l',num2str(j), num2str(1), '=');
    nombre2 = strcat('float l',num2str(j), num2str(2), '=');
    disp(strcat(nombre1, num2str(L(j,1)), ';'))
    disp(strcat(nombre2, num2str(L(j,2)), ';'))
end
%% Realimentación
clc
%p = [0.8886 0.9919 0.9775+0.0975*i 0.9775-0.0975*i];
pK = [0.7879 0.9895 0.9876+0.0974i 0.9876-0.0974i];
K = - place(Ad, Bd, pK);

for j = 1:4
    nombre = strcat('float c',num2str(j), '=');
    disp(strcat(nombre, num2str(K(j)), ';'))
end
%% Feed-Forward
clc
F = pinv((Cd*(eye(4) - (Ad+Bd*K))^(-1)*Bd));
%% Acción integral
clc
p_ext = [0.7870 0.9952+0.0106i 0.9952-0.0106i 0.9876+0.0976i 0.9876-0.0976i];

Ad_ext = [Ad zeros(4,1); [1 0 0 0] 1];
Bd_ext = [Bd; 0];

KH = -place(Ad_ext, Bd_ext, p_ext);
H = -KH(end);
K_int = KH(1:end-1);
disp(strcat('float K_int[] = {',num2str(KH(1)),',', num2str(KH(2)),',',num2str(KH(3)),',', num2str(KH(4)),  '};'));
disp(strcat('float H = ', num2str(H), ';'))
%% Simulaciones
% Cargar las variables y luego correr el simulink correspondiente
clc

load('medicion_obs.mat');

theta_est = out.theta_est(1:800);
theta_p_est = out.theta_p_est(1:800);
phi_est = out.phi_est(1:800);
phi_p_est = out.phi_p_est(1:800);

n = length(theta_est);
t = (T:T:T*n)';

theta_est = [t theta_est];
theta_p_est = [t theta_p_est];
phi_est = [t phi_est];
phi_p_est = [t phi_p_est];

load('Pe_torque.mat')
load('brazo.mat')
Brazo = tf1;

clear out
load('H escalon2.mat');

theta_est_he = out.theta_est;
theta_p_est_he = out.theta_p_est;
phi_est_he = out.phi_est;
phi_p_est_he = out.phi_p_est;

n = length(theta_est_he);
t = (T:T:T*n)';

theta_est_he = [t theta_est_he];
theta_p_est_he = [t theta_p_est_he];
phi_est_he = [t phi_est_he];
phi_p_est_he = [t phi_p_est_he];

clear out
load('H golpe.mat');

theta_est_hi = out.theta_est;
theta_p_est_hi = out.theta_p_est;
phi_est_hi = out.phi_est;
phi_p_est_hi = out.phi_p_est;

n = length(theta_est_hi);
t = (T:T:T*n)';

theta_est_hi = [t theta_est_hi];
theta_p_est_hi = [t theta_p_est_hi];
phi_est_hi = [t phi_est_hi];
phi_p_est_hi = [t phi_p_est_hi];

clear out
load('f golpe.mat');

theta_est_fi = out.theta_est;
theta_p_est_fi = out.theta_p_est;
phi_est_fi = out.phi_est;
phi_p_est_fi = out.phi_p_est;

n = length(theta_est_fi);
t = (T:T:T*n)';

theta_est_fi = [t theta_est_fi];
theta_p_est_fi = [t theta_p_est_fi];
phi_est_fi = [t phi_est_fi];
phi_p_est_fi = [t phi_p_est_fi];

clear out
load('f escalon.mat');

theta_est_fe = out.theta_est;
theta_p_est_fe = out.theta_p_est;
phi_est_fe = out.phi_est;
phi_p_est_fe = out.phi_p_est;

n = length(theta_est_fe);
t = (T:T:T*n)';

theta_est_fe = [t theta_est_fe];
theta_p_est_fe = [t theta_p_est_fe];
phi_est_fe = [t phi_est_fe];
phi_p_est_fe = [t phi_p_est_fe];