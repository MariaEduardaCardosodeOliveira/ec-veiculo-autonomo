%  Script 05: RUNNER - Executa todos os scripts na sequencia
%  Disciplina: Analise de Sistemas Lineares - UTFPR Apucarana
%
%  COMO USAR: abra esta pasta no MATLAB e rode  run('Script05_runner.m')
clear; close all; clc;

fprintf('=================================================\n');
fprintf(' EC - Rastreamento de Trajetoria: PID vs MPC\n');
fprintf(' UTFPR Apucarana - Controle de Sistemas\n');
fprintf('=================================================\n\n');

fprintf('[1/4] Modelo de Malha Aberta...\n');
run('Script01_modelo_malha_aberta.m'); close all;

fprintf('[2/4] Controlador PID...\n');
run('Script02_controlador_pid.m'); close all;

fprintf('[3/4] Controlador MPC...\n');
run('Script03_controlador_mpc.m'); close all;

fprintf('[4/4] Comparacao e Tabela 4...\n');
run('Script04_comparacao_metricas.m');

fprintf('\n=================================================\n');
fprintf(' Estudo de Caso concluido! Figuras .png geradas.\n');
fprintf('=================================================\n');
