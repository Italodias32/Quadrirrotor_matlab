%Universidade Federal de Minas Gerais
%Departamento de Engenharia Eletrônica
%Técnicas de Controle de Processos Industriais

%Inicialização
clear

%Parametros do quadrirrotor
m = 2.24; %massa do quadrirrotor
g = 9.81; %aceleração da gravidade
Ixx = 0.0363; %momento de inercia em torno do eixo x
Iyy = 0.0363; %momento de inercia em torno do eixo y
Izz = 0.0615; %momento de inercia em torno do eixo z
Jr = 5.168e-6; %momento de inercia em torno do eixo do rotor
tau_phi_d = 0.01;  
tau_theta_d = 0.01; 
tau_psi_d = 0.01; 
ax = 0.01;
ay = 0.01;
az = 0.01;

%%
A_l = [zeros(6,3) [0 g 0;-g 0 0;0 0 0; zeros(3)]];
Am = [zeros(6) eye(6); A_l zeros(6)];
Bm = [zeros(8,4); 1/m 0 0 0;0 1/Ixx 0 0;0 0 1/Iyy 0;0 0 0 1/Izz];
Cm = [eye(6) zeros(6)];
Dm = zeros(6,4);
Km = [zeros(6,12);
      0 0 0 0 0 0 1/m 0    0    0      0      0;
      0 0 0 0 0 0 0   1/m  0    0      0      0;
      0 0 0 0 0 0 0   0    1/m  0      0      0;
      0 0 0 0 0 0 0   0    0    1/Ixx  0      0;
      0 0 0 0 0 0 0   0    0    0      1/Iyy  0;
      0 0 0 0 0 0 0   0    0    0      0      1/Izz];

%%
%Observador de ordem mínima
R = [zeros(6) eye(6)];
X = [Cm; R];
Y = inv(X);
Y1 = Y(:,1:6);
Y2 = Y(:,7:12);
Ab = (X*Am)/X;
Bb = X*Bm;
Cb = Cm/X;
Db = Dm;

% Capturando as submatrizes de interesse para o pendulo invertido analisado
A11 = Ab(1:6,1:6);
A12 = Ab(1:6,7:12);
A21 = Ab(7:12,1:6);
A22 = Ab(7:12,7:12);
B1 = Bb(1:6,:);
B2 = Bb(7:12,:);

%Ganhos do obsevador
Lc = eye(6);
polo = 10;
F = [zeros(5,1) eye(5)];
F = [F; -1*polo, -6*polo, -15*polo, -20*polo, -15*polo, -6*polo];
T = lyap(-F,A22,-Lc*A12);
Lb = inv(T)*Lc;

%%
%Controle de atitude
A = [0 1 0 0 0 0;
     0 0 0 0 0 0;
     0 0 0 1 0 0;
     0 0 0 0 0 0;
     0 0 0 0 0 1;
     0 0 0 0 0 0];
 
 B = [0     0     0;
      1/Ixx 0     0;
      0     0     0;
      0     1/Iyy 0;
      0     0     0;
      0     0     1/Izz];
  
  R = eye(3)*0.5;
  Q = eye(6);
  
  G = -B*inv(R)*B';
  
  [P, K, L] = care(A,B,Q,R);
  K = inv(R)*B'*P;
%   K = lqr(A,B,Q,R);
  
  %%
  %Controladores PID
%   Hx = tf([1],[m 0 0]);
%   figure(1)
%   rlocus(Hx)
%   saveas(gcf,'rlocus1.png')
%   
%   Hy = tf([-1],[m 0 0]);
%   figure(2)
%   rlocus(Hy)
%   saveas(gcf,'rlocus2.png')
%   
%   Hz = tf([1],[m 0 0]);
%   figure(3)
%   rlocus(Hy)
%   saveas(gcf,'rlocus3.png')
%   
%   Xc = tf([39.95 190.9 230],[m 0 0 0]);
%   figure(4)
%   rlocus(Xc)
%   saveas(gcf,'rlocusc1.png')
%   
%   Yc = tf([39.95 190.9 230],[m 0 0 0]);
%   figure(5)
%   rlocus(Yc)
%   saveas(gcf,'rlocusc2.png')
%   
%   Zc = tf(-[34 153 170],[m 0 0 0]);
%   figure(6)
%   rlocus(Xc)
%   saveas(gcf,'rlocusc3.png')