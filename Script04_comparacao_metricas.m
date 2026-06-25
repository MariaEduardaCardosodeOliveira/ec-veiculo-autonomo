%% ================================================================
%  ESTUDO DE CASO - RASTREAMENTO DE TRAJETORIA DE VEICULO AUTONOMO
%  Script 04: Comparacao PID vs MPC - Tabela 4 de Metricas
%  Disciplina: Controle de Sistemas - UTFPR Apucarana
%  Requer: rodar Script 02 e Script 03 antes (geram os .mat)
%% ================================================================
clear; clc;

P = load('metricas_pid.mat');
M = load('metricas_mpc.mat');
t = P.t;

f = @(e) [sqrt(mean(e.^2)), trapz(t,abs(e)), trapz(t,e.^2), trapz(t,t.*abs(e))];
pid_n = f(P.r - P.y_nom);
pid_p = f(P.r - P.y_pert);
mpc_n = f(M.r - M.y_nom);
mpc_p = f(M.r - M.y_pert);

nomes = {'RMSE','IAE','ISE','ITAE'};

fprintf('\n========================================================\n');
fprintf('   TABELA 4 - METRICAS DE DESEMPENHO: PID vs MPC\n');
fprintf('========================================================\n');
fprintf('%-6s %10s %10s %10s %10s\n','Metr','PID-Nom','PID-Pert','MPC-Nom','MPC-Pert');
for i = 1:4
    fprintf('%-6s %10.4f %10.4f %10.4f %10.4f\n', ...
        nomes{i}, pid_n(i), pid_p(i), mpc_n(i), mpc_p(i));
end
fprintf('========================================================\n');

fprintf('\nRobustez (degradacao nominal -> perturbado):\n');
for i = 1:4
    dpid = (pid_p(i)-pid_n(i))/pid_n(i)*100;
    dmpc = (mpc_p(i)-mpc_n(i))/mpc_n(i)*100;
    fprintf('  %-5s PID: %+8.1f%%   MPC: %+8.1f%%\n', nomes{i}, dpid, dmpc);
end

fprintf('\nScript 04 concluido.\n');
