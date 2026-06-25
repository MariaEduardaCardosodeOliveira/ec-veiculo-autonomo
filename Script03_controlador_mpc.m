%% ================================================================
%  ESTUDO DE CASO - RASTREAMENTO DE TRAJETORIA DE VEICULO AUTONOMO
%  Script 03: Controlador 2 - MPC (Model Predictive Control)
%  Disciplina: Controle de Sistemas - UTFPR Apucarana
%  Requer: Model Predictive Control Toolbox
%  Gera as Figuras 5 e 7 do artigo + metricas do MPC
%% ================================================================
clear; close all; clc;

% --- Parametros ---
L = 2.7; v_nom = 15; v_pert = 18;
Ts = 0.05; T = 15;
t = (0:Ts:T)'; N = length(t);
r = 0.5*ones(N,1);
C = [1 0]; D = 0;

% --- MPC nominal ---
A = [0 v_nom;0 0]; B = [0;v_nom/L];
sysd = c2d(ss(A,B,C,D),Ts);
mpcobj = mpc(sysd,Ts,20,5);
mpcobj.Weights.OutputVariables = 1;
mpcobj.Weights.ManipulatedVariablesRate = 0.1;
mpcobj.MV.Min = -0.52; mpcobj.MV.Max = 0.52;
mpcobj.MV.RateMin = -0.1; mpcobj.MV.RateMax = 0.1;
mpcobj.Model.Noise = ss(eye(1));

x = [0;0]; y_nom = zeros(N,1); u_nom = zeros(N,1);
xmpc = mpcstate(mpcobj); xmpc.Plant = [0;0]; xmpc.LastMove = 0;
for k = 1:N
    y_nom(k) = C*x;
    u_nom(k) = mpcmove(mpcobj,xmpc,y_nom(k),r(k));
    x = sysd.A*x + sysd.B*u_nom(k);
end

% --- MPC perturbado ---
A2 = [0 v_pert;0 0]; B2 = [0;v_pert/L];
sysd2 = c2d(ss(A2,B2,C,D),Ts);
mpcobj2 = mpc(sysd2,Ts,20,5);
mpcobj2.Weights.OutputVariables = 1;
mpcobj2.Weights.ManipulatedVariablesRate = 0.1;
mpcobj2.MV.Min = -0.52; mpcobj2.MV.Max = 0.52;
mpcobj2.MV.RateMin = -0.1; mpcobj2.MV.RateMax = 0.1;
mpcobj2.Model.Noise = ss(eye(1));

x = [0;0]; y_pert = zeros(N,1); u_pert = zeros(N,1);
xmpc2 = mpcstate(mpcobj2); xmpc2.Plant = [0;0]; xmpc2.LastMove = 0;
for k = 1:N
    y_pert(k) = C*x;
    u_pert(k) = mpcmove(mpcobj2,xmpc2,y_pert(k),r(k));
    x = sysd2.A*x + sysd2.B*u_pert(k);
end

%% --- Figura 5: saida + acao de controle ---
figure('Position',[100 100 1000 600]);
subplot(2,1,1);
plot(t,r,'k--','LineWidth',1.5); hold on;
plot(t,y_nom,'b-','LineWidth',2);
plot(t,y_pert,'r--','LineWidth',1.5);
grid on; xlabel('Tempo (s)'); ylabel('Erro lateral e_y (m)'); ylim([-0.1 0.7]);
title('Rastreamento de Referência — Controlador MPC');
legend('Referência r(t)','Saída y(t) — Nominal','Saída y(t) — Perturbado (+20% v)','Location','northeast');
subplot(2,1,2);
plot(t,u_nom,'b-','LineWidth',2); hold on;
plot(t,u_pert,'r--','LineWidth',1.5);
yline(0.52,'k:','LineWidth',1.2); yline(-0.52,'k:','LineWidth',1.2);
grid on; xlabel('Tempo (s)'); ylabel('\delta (rad)'); ylim([-0.6 0.6]);
title('Ação de Controle — Ângulo de Esterçamento \delta(t)');
legend('u(t) — Nominal','u(t) — Perturbado','Restrições \pm0.52','Location','northeast');
sgtitle('Controlador MPC — Rastreamento (v=15 m/s, L=2.7 m)');
saveas(gcf,'06_mpc_resposta.png');

%% --- Figura 7: espectrograma ---
fs = 1/Ts;
figure('Position',[100 100 1000 450]);
subplot(1,2,1);
spectrogram(u_nom,hamming(64),32,128,fs,'yaxis');
title('Espectrograma u_{MPC}(t) — Nominal'); xlabel('Tempo (s)'); ylabel('Frequência (Hz)');
subplot(1,2,2);
spectrogram(u_pert,hamming(64),32,128,fs,'yaxis');
title('Espectrograma u_{MPC}(t) — Perturbado'); xlabel('Tempo (s)'); ylabel('Frequência (Hz)');
sgtitle('Espectrograma do Sinal de Controle — MPC');
saveas(gcf,'07_mpc_espectrograma.png');

%% --- Metricas ---
e_nom = r - y_nom; e_pert = r - y_pert;
f = @(e) [sqrt(mean(e.^2)), trapz(t,abs(e)), trapz(t,e.^2), trapz(t,t.*abs(e))];
mn = f(e_nom); mp = f(e_pert);
fprintf('MPC Nominal      RMSE=%.4f  IAE=%.4f  ISE=%.4f  ITAE=%.4f\n',mn);
fprintf('MPC Perturbado   RMSE=%.4f  IAE=%.4f  ISE=%.4f  ITAE=%.4f\n',mp);
save('metricas_mpc.mat','t','r','y_nom','y_pert','u_nom','u_pert');
fprintf('\nScript 03 concluido.\n');
