function [M,C,U_k] = MPC(A,B,N,x_k,x_k_bias,Q,R,F,lb,ub)
% -------------------------------------------------------------------------
% Model Predictive Control (MPC) solver with a zero reference point.
% -------------------------------------------------------------------------
% Inputs:
%   A, B      : System matrices (x_{k+1} = A x_k + B u_k)
%   N         : Prediction horizon length
%   x_k       : Current state
%   x_k_bias  : Reference trajectory (stacked)
%   Q, R      : Stage cost weighting matrices
%   F         : Terminal cost matrix
%   lb, ub    : Input bounds (lower / upper)
%
% Outputs:
%   U_k : Optimal control input sequence (stacked)
%   M, C: Intermediate matrices used in the condensed QP formulation
% -------------------------------------------------------------------------

% Dimensions
n = size(A,1);                % State dimension (A is n x n)
p = size(B,2);                % Input dimension (B is n x p)

% Build M matrix ( (N+1)*n  x  n )
M = [eye(n); zeros(N*n, n)];  % Top block identity, rest zeros

% Build C matrix ( (N+1)*n  x  N*p )
C = zeros((N+1)*n, N*p);

tmp = eye(n);                 % Temporary A^i accumulator
for i = 1:N
    rows = i*n + (1:n);       % Row indices for current block
    % Populate C: [A^{i-1}B  ...  B]
    C(rows,:) = [tmp*B, C(rows-n,1:end-p)];
    tmp       = A * tmp;      % Update A^i
    M(rows,:) = tmp;          % Populate M with A^i
end

% Build Q_bar (block‑diagonal of Q with terminal F)
S_q   = size(Q,1);
Q_bar = zeros((N+1)*S_q, (N+1)*S_q);
for i = 0:N
    Q_bar(i*S_q+1:(i+1)*S_q, i*S_q+1:(i+1)*S_q) = Q;
end
Q_bar(N*S_q+1:(N+1)*S_q, N*S_q+1:(N+1)*S_q) = F; % Terminal cost

% Build R_bar (block‑diagonal of R)
S_r   = size(R,1);
R_bar = zeros(N*S_r, N*S_r);
for i = 0:N-1
    R_bar(i*S_r+1:(i+1)*S_r, i*S_r+1:(i+1)*S_r) = R;
end

% Condensed QP matrices
G = M' * Q_bar * M;
E = C' * Q_bar * M;
H = C' * Q_bar * C + R_bar;

% Linear term of the cost function
f = (x_k' * E')' - (x_k_bias' * Q_bar * C)';

% Solve the QP
options = optimoptions('quadprog','Display','off');
U_k = quadprog(H, f, [], [], [], [], lb, ub, [], options);
end
