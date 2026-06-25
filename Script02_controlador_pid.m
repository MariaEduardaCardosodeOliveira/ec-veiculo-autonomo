%% ================================================================
%  ESTUDO DE CASO - RASTREAMENTO DE TRAJETORIA DE VEICULO AUTONOMO
%  Script 02: Controlador 1 - PID (saturacao fisica + anti-windup)
%  Disciplina: Controle de Sistemas - UTFPR Apucarana
%  Gera as Figuras 4 e 6 do artigo + metricas do PID
%% ================================================================
clear; close all; clc;

% --- Parametros ---
L  = 2.7; v = 15; Ts = 0.05; T = 15;
t  = (0:Ts:T)'; N = numel(t);
r  = 0.5*ones(N,1);
umin = -0.52; umax = 0.52;
Kp = 0.5; Ki = 0.4; Kd = 0.3; Nf = 20; Tt = 0.25;

% --- Simulacoes ---
[y_nom ,u_nom ] = pid_sat(v    ,L,Ts,N,r,Kp,Ki,Kd,Nf,Tt,umin,umax);
[y_pert,u_pert] = pid_sat(v*1.2,L,Ts,N,r,Kp,Ki,Kd,Nf,Tt,umin,umax);

%% --- Figura 4: saida + acao de controle ---
figure('Position',[100 100 1000 600]);
subplot(2,1,1);
plot(t,r,'k--','LineWidth',1.5); hold on;
plot(t,y_nom,'b-','LineWidth',2);
plot(t,y_pert,'r--','LineWidth',1.5);
grid on; xlabel('Tempo (s)'); ylabel('Erro lateral e_y (m)'); ylim([-0.1 0.75]);
title('Rastreamento de Referência — Controlador PID');
legend('Referência r(t)','Saída y(t) — Nominal','Saída y(t) — Perturbado (+20% v)','Location','northeast');
subplot(2,1,2);
plot(t,u_nom,'b-','LineWidth',2); hold on;
plot(t,u_pert,'r--','LineWidth',1.5);
yline(0.52,'k:','LineWidth',1.2); yline(-0.52,'k:','LineWidth',1.2);
grid on; xlabel('Tempo (s)'); ylabel('\delta (rad)'); ylim([-0.6 0.6]);
title('Ação de Controle — Ângulo de Esterçamento \delta(t)');
legend('u(t) — Nominal','u(t) — Perturbado','Restrições \pm0.52','Location','northeast');
sgtitle('Controlador PID — Rastreamento (v=15 m/s, L=2.7 m)');
saveas(gcf,'04_pid_resposta.png');

%% --- Figura 6: espectrograma ---
fs = 1/Ts;
figure('Position',[100 100 1000 450]);
subplot(1,2,1);
spectrogram(u_nom,hamming(64),32,128,fs,'yaxis');
title('Espectrograma u_{PID}(t) — Nominal'); xlabel('Tempo (s)'); ylabel('Frequência (Hz)');
subplot(1,2,2);
spectrogram(u_pert,hamming(64),32,128,fs,'yaxis');
title('Espectrograma u_{PID}(t) — Perturbado'); xlabel('Tempo (s)'); ylabel('Frequência (Hz)');
sgtitle('Espectrograma do Sinal de Controle — PID');
saveas(gcf,'05_pid_espectrograma.png');

%% --- Metricas ---
e_nom = r - y_nom; e_pert = r - y_pert;
imprime('PID Nominal'   ,t,e_nom);
imprime('PID Perturbado',t,e_pert);
save('metricas_pid.mat','t','r','y_nom','y_pert','u_nom','u_pert');
fprintf('\nScript 02 concluido.\n');

%% ---------------- FUNCOES LOCAIS ----------------
function [y,u] = pid_sat(v,L,Ts,N,r,Kp,Ki,Kd,Nf,Tt,umin,umax)
    A = [0 v;0 0]; B = [0;v/L]; C = [1 0];
    sysd = c2d(ss(A,B,C,0),Ts);
    Ad = sysd.A; Bd = sysd.B; Cd = sysd.C;
    x = [0;0]; y = zeros(N,1); u = zeros(N,1);
    I = 0; eprev = 0; Tf = 1/Nf; Dprev = 0;
    for k = 1:N
        y(k) = Cd*x;
        e = r(k) - y(k);
        D = (Tf/(Tf+Ts))*Dprev + (Kd/(Tf+Ts))*(e-eprev);  % derivada filtrada
        vu = Kp*e + I + D;                                 % saida nao saturada
        us = min(max(vu,umin),umax);                       % saturacao
        I = I + Ki*e*Ts + (Ts/Tt)*(us-vu);                 % integral + anti-windup
        u(k) = us; eprev = e; Dprev = D;
        x = Ad*x + Bd*us;
    end
end

function imprime(nome,t,e)
    rmse = sqrt(mean(e.^2));
    iae  = trapz(t,abs(e));
    ise  = trapz(t,e.^2);
    itae = trapz(t,t.*abs(e));
    fprintf('%-16s RMSE=%.4f  IAE=%.4f  ISE=%.4f  ITAE=%.4f\n',nome,rmse,iae,ise,itae);
end
