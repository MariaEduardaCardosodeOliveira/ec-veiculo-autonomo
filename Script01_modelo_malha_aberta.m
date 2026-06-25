%% ================================================================
%  ESTUDO DE CASO - RASTREAMENTO DE TRAJETORIA DE VEICULO AUTONOMO
%  Script 01: Modelagem e Analise de Malha Aberta
%  Disciplina: Controle de Sistemas - UTFPR Apucarana
%  Gera as Figuras 1, 2 e 3 do artigo
%% ================================================================
clear; close all; clc;

% --- Parametros nominais ---
L  = 2.7;    % [m]   distancia entre eixos (wheelbase)
v  = 15;     % [m/s] velocidade longitudinal nominal
Ts = 0.05;   % [s]   periodo de amostragem

A = [0 v; 0 0];
B = [0; v/L];
C = [1 0];
D = 0;

sys = ss(A, B, C, D);
sys_tf = tf(sys);
fprintf('=== Funcao de Transferencia (Malha Aberta) ===\n');
sys_tf

%% --- Figura 3: resposta temporal de malha aberta (degrau + impulso) ---
t = 0:Ts:10;
figure('Position',[100 100 1000 450]);
subplot(1,2,1);
[ys,ts] = step(sys_tf,t);
plot(ts,ys,'b-','LineWidth',2); grid on;
xlabel('Tempo (s)'); ylabel('Erro lateral e_y (m)');
title('Resposta ao Degrau — Malha Aberta');
subplot(1,2,2);
[yi,ti] = impulse(sys_tf,t);
plot(ti,yi,'r-','LineWidth',2); grid on;
xlabel('Tempo (s)'); ylabel('Erro lateral e_y (m)');
title('Resposta ao Impulso — Malha Aberta');
sgtitle('Análise de Malha Aberta — Veículo Autônomo (v=15 m/s, L=2.7 m)');
saveas(gcf,'01_malha_aberta_temporal.png');

%% --- Figura 2: Bode + Nyquist ---
figure('Position',[100 100 1000 700]);
subplot(2,2,[1 2]);
bode(sys_tf); grid on;
title('');
subplot(2,2,3);
w = logspace(-1,1,500);
nyquist(sys_tf,w);
xlim([-3 1]); ylim([-3 3]); grid on;
title('Diagrama de Nyquist — Malha Aberta');
hold on; plot(-1,0,'r+','MarkerSize',12,'LineWidth',2);
subplot(2,2,4);
[mag,~,wout] = bode(sys_tf); mag = squeeze(mag);
semilogx(wout,20*log10(mag),'k-','LineWidth',2); grid on;
xlabel('\omega (rad/s)'); ylabel('|G(j\omega)| (dB)');
title('Magnitude — Malha Aberta');
sgtitle('Análise em Frequência — Malha Aberta');
saveas(gcf,'02_malha_aberta_frequencia.png');

%% --- Figura 1: Lugar das raizes + margem de estabilidade ---
figure('Position',[100 100 1100 500]);
subplot(1,2,1);
rlocus(sys_tf); grid on;
title('Lugar das Raízes — Malha Aberta');
subplot(1,2,2);
margin(sys_tf); grid on;
saveas(gcf,'03_lugar_raizes_margens.png');

[Gm,Pm,Wcg,Wcp] = margin(sys_tf);
fprintf('\n=== Margens de Estabilidade (Malha Aberta) ===\n');
fprintf('Margem de Ganho : %.4f dB (freq %.4f rad/s)\n', 20*log10(Gm), Wcg);
fprintf('Margem de Fase  : %.4f graus (freq %.4f rad/s)\n', Pm, Wcp);
fprintf('Polos: '); disp(pole(sys_tf));

fprintf('\nScript 01 concluido.\n');
