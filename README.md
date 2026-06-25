# 🚗 Rastreamento de Trajetória de Veículo Autônomo — PID vs MPC

Estudo de Caso da disciplina de **Controle de Sistemas** — UTFPR Apucarana (2026.1).

Análise comparativa entre os controladores **PID** (Proporcional-Integral-Derivativo) e **MPC** (Model Predictive Control) aplicados ao **controle lateral** de um veículo autônomo — o problema de manter o carro centrado na faixa, corrigindo o volante automaticamente.

O veículo é representado pelo **modelo de bicicleta cinemático linearizado**, e a comparação é feita nos domínios do tempo e da frequência, em condições nominais e sob incerteza paramétrica.

---

## 📌 Objetivo

Responder, com simulação e métricas quantitativas: **qual controlador mantém melhor o veículo na trajetória de referência** — o PID, simples e amplamente usado, ou o MPC, que antecipa o futuro e trata restrições nativamente?

---

## 🔑 Principais resultados

Sob **restrições físicas idênticas** de esterçamento (±0,52 rad) aplicadas a ambos os controladores, o **MPC supera o PID em todas as métricas avaliadas**:

| Métrica | PID Nom. | PID Pert. | MPC Nom. | MPC Pert. |
|---------|---------:|----------:|---------:|----------:|
| **RMSE** | 0,0692 | 0,1288 | **0,0595** | **0,0564** |
| **IAE**  | 0,3598 | 1,6574 | **0,1184** | **0,1044** |
| **ISE**  | 0,0658 | 0,2430 | **0,0471** | **0,0416** |
| **ITAE** | 0,4441 | 11,9592 | **0,0160** | **0,0124** |

> *Nom. = nominal (v = 15 m/s) · Pert. = perturbado (v = 18 m/s, +20%)*

**Destaques:**
- 🎯 A vantagem do MPC é **mais expressiva no ITAE** (convergência transitória rápida).
- 🛡️ **Robustez:** sob perturbação de +20% na velocidade, o PID degrada severamente (ITAE de 0,44 → 11,96), enquanto o MPC permanece praticamente inalterado.
- ⚙️ O MPC gera uma **ação de controle mais suave**, reduzindo o desgaste mecânico dos atuadores de direção.

---

## 📁 Estrutura do repositório

| Arquivo | Descrição | Figuras geradas |
|---------|-----------|-----------------|
| `Script01_modelo_malha_aberta.m` | Modelagem e análise de malha aberta (lugar das raízes, Bode, Nyquist, resposta temporal) | Figuras 1, 2, 3 |
| `Script02_controlador_pid.m` | Controlador PID com saturação ±0,52 rad e anti-windup | Figuras 4, 6 |
| `Script03_controlador_mpc.m` | Controlador MPC (horizonte de predição e controle, restrições) | Figuras 5, 7 |
| `Script04_comparacao_metricas.m` | Cálculo da Tabela 4 (RMSE, IAE, ISE, ITAE) e análise de robustez | — |
| `Script05_runner.m` | Executa todos os scripts na ordem correta | todas |

---

## ▶️ Como executar

1. Abra a pasta do projeto no **MATLAB** (R2024a ou superior).
2. Execute o runner:
   ```matlab
   run('Script05_runner.m')
   ```
   Ou rode cada script individualmente, na ordem 01 → 02 → 03 → 04.
3. As figuras `.png` são salvas automaticamente na pasta, e a Tabela 4 é impressa no Command Window.

---

## 🧰 Requisitos

- **MATLAB** R2024a+
- **Control System Toolbox**
- **Model Predictive Control Toolbox**
- **Signal Processing Toolbox**

---

## 📐 Modelo

Espaço de estados linearizado (erro lateral `e_y` e erro de orientação `e_ψ`):

```
ẋ = A·x + B·δ ,   y = C·x

A = [0  v;  0  0]    B = [0;  v/L]    C = [1  0]
```

Função de transferência de malha aberta — um **duplo integrador** (planta tipo 2):

```
G(s) = v² / (L·s²)
```

| Parâmetro | Valor |
|-----------|-------|
| Distância entre eixos (L) | 2,7 m |
| Velocidade longitudinal (v) | 15 m/s |
| Período de amostragem (Ts) | 0,05 s |
| Referência lateral (r) | 0,5 m |

---

## 👥 Autores

Arieli Leandro Gutierres · Dirceu Morais da Costa Júnior · Maria Eduarda Cardoso de Oliveira · Nathália Rodrigues Nunes Gonçalves

Universidade Tecnológica Federal do Paraná — UTFPR Apucarana
