L = 2.7; v_nom = 15; v_pert = 18;
Ts = 0.05; T = 15;
t = (0:Ts:T)';
N = length(t);
r = 0.5 * ones(N, 1);
C = [1 0]; D = 0;

A = [0 v_nom; 0 0]; B = [0; v_nom/L];
sysd = c2d(ss(A,B,C,D), Ts);
mpcobj = mpc(sysd, Ts, 20, 5);
mpcobj.Weights.OutputVariables = 1;
mpcobj.Weights.ManipulatedVariablesRate = 0.1;
mpcobj.MV.Min = -0.52; mpcobj.MV.Max = 0.52;
mpcobj.MV.RateMin = -0.1; mpcobj.MV.RateMax = 0.1;
mpcobj.Model.Noise = ss(eye(1));

x = [0;0];
y_nom = zeros(N,1); u_nom = zeros(N,1);
xmpc = mpcstate(mpcobj);
xmpc.Plant = [0;0];
xmpc.LastMove = 0;
for k = 1:N
    y_nom(k) = C*x;
    u_nom(k) = mpcmove(mpcobj, xmpc, y_nom(k), r(k));
    x = sysd.A*x + sysd.B*u_nom(k);
end

A2 = [0 v_pert; 0 0]; B2 = [0; v_pert/L];
sysd2 = c2d(ss(A2,B2,C,D), Ts);
mpcobj2 = mpc(sysd2, Ts, 20, 5);
mpcobj2.Weights.OutputVariables = 1;
mpcobj2.Weights.ManipulatedVariablesRate = 0.1;
mpcobj2.MV.Min = -0.52; mpcobj2.MV.Max = 0.52;
mpcobj2.MV.RateMin = -0.1; mpcobj2.MV.RateMax = 0.1;
mpcobj2.Model.Noise = ss(eye(1));

x = [0;0];
y_pert = zeros(N,1); u_pert = zeros(N,1);
xmpc2 = mpcstate(mpcobj2);
xmpc2.Plant = [0;0];
xmpc2.LastMove = 0;
for k = 1:N
    y_pert(k) = C*x;
    u_pert(k) = mpcmove(mpcobj2, xmpc2, y_pert(k), r(k));
    x = sysd2.A*x + sysd2.B*u_pert(k);
end

figure;
subplot(2,1,1);
plot(t, r, 'k--', t, y_nom, 'b-', t, y_pert, 'r--', 'LineWidth', 1.5);
xlabel('Tempo (s)'); ylabel('e_y (m)');
title('Saída MPC — Nominal vs Perturbado');
legend('Referência r(t)', 'Nominal (v=15)', 'Perturbado (v=18)');
grid on;
subplot(2,1,2);
plot(t, u_nom, 'b-', t, u_pert, 'r--', 'LineWidth', 1.5);
yline(0.52,'k--'); yline(-0.52,'k--');
xlabel('Tempo (s)'); ylabel('\delta (rad)');
title('Ação de Controle MPC — \delta(t)');
legend('u nominal', 'u perturbado', 'Restrições ±0.52');
grid on;
sgtitle('Controlador MPC — Rastreamento de Trajetória (v=15 m/s, L=2.7 m)');
