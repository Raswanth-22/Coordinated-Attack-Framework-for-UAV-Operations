function [M, C, U_k] = MPC(A, B, N, x_k, x_k_bias, Q, R, F, lb, ub)
%MPC  Condensed quadratic‑programming formulation of Model Predictive Control.
%   [M, C, U_k] = MPC(A, B, N, x_k, x_k_bias, Q, R, F, lb, ub) solves a
%   finite‑horizon MPC problem with *zero reference* and returns the first
%   optimal input sequence U_k as well as the intermediate matrices M and C
%   used in the condensed QP.
%
%   Inputs
%   ------
%   A, B       : Discrete‑time system matrices (x_{k+1} = A x_k + B u_k).
%   N          : Prediction horizon length (integer).
%   x_k        : Current state vector.
%   x_k_bias   : Stacked reference trajectory ( (N+1)*n × 1 ).
%   Q, R       : Stage cost weighting matrices.
%   F          : Terminal cost weight.
%   lb, ub     : Lower / upper bounds on inputs (stacked for horizon).
%
%   Outputs
%   -------
%   M, C       : Matrices of the condensed QP formulation.
%   U_k        : Optimal control vector (stacked) for the horizon.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Dimensions -------------------------------------------------------------
n = size(A,1);      % State dimension (A is n×n)
p = size(B,2);      % Input dimension (B is n×p)

%% Build M matrix ( (N+1)n × n ) ----------------------------------------
M = [eye(n); zeros(N*n, n)];  % Top block is identity, rest zeros

%% Build C matrix ( (N+1)n × Np ) ---------------------------------------
C = zeros((N+1)*n, N*p);

tmp = eye(n);                 % Accumulator for A^i
for i = 1:N
    rows = i*n + (1:n);                       % Row indices of block i
    C(rows,:) = [tmp*B, C(rows-n, 1:end-p)];  % Fill C block by block
    tmp       = A * tmp;                      % Update A^i
    M(rows,:) = tmp;                          % Fill M with A^i
end

%% Build block‑diagonal weight matrices Q_bar and R_bar ------------------
S_q   = size(Q,1);
S_r   = size(R,1);
Q_bar = kron(eye(N+1), Q);   % (N+1) blocks of Q on diagonal
Q_bar(end-S_q+1:end, end-S_q+1:end) = F;  % Replace last block with F

R_bar = kron(eye(N), R);      % N blocks of R on diagonal

%% Condensed QP matrices --------------------------------------------------
G = M' * Q_bar * M;
E = C' * Q_bar * M;
H = C' * Q_bar * C + R_bar;

%% Linear term of the cost function --------------------------------------
f = (x_k' * E')' - (x_k_bias' * Q_bar * C)';

%% Solve the quadratic program -------------------------------------------
options = optimoptions('quadprog', 'Display', 'off');
U_k = quadprog(H, f, [], [], [], [], lb, ub, [], options);
end
