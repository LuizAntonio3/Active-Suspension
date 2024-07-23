function [K,h2,P] = h2_discrete(A,B,C,Du,Dw,Bw,u)

%% Variáveis
nv = length(A);
nx = length(A{1});
nu = length(u);
nw = size(Dw,2);
nz = size(Dw,1);

nb = size(B,2);

%% settings SDP
solver = {'sdpt3','mosek'};
optSolver = sdpsettings;
optSolver.verbose = 1;
optSolver.solver = solver{2};

%% define decision variables
Q = sdpvar(nx,nx,'symmetric');
Z = sdpvar(nu,nx,'full');
V = sdpvar(nu,nx,'full');
U = sdpvar(nu,nu,'symmetric');
rho = sdpvar(1, 1,'full');

% H = sdpvar(nx,nx,'full');
% Sd = sdpvar(nu,nx,'full');
% Pd = sdpvar(nx,nx,'symmetric');

%% define LMI constraints
eps = 1e-9;                 % tolerance of the semi-definite problem

lmis = [];
S = Z - V;
for i = 1:nu
    lmis = lmis + ( [rho, Bw{i}'; ...
                     Bw{i} , Q] >= eps*eye(nx+1) );
end

% alpha = 0.92 ta me dando os ganhos com norma 264 - lmi geral
alpha = 0.90;
for i = 1:nv
    lmis = lmis + ( [-alpha*Q , A{i}*Q + B{i}*Z; ...
                     Q'*A{i}' + Z'*B{i}', -alpha*Q] <= -eps*eye(2*nx) );
    
    lmis = lmis + ( [Q               , Q*A{i}' + Z'*B{i}' , Q*C' + Z'*Du'; ...
                     A{i}*Q + B{i}*Z , Q                  , zeros(nx, nz); ...
                     C*Q + Du*Z      , zeros(nz, nx)      , eye(nz)]        >= eps*eye(2*nx+nz) );
    
%     lmis = lmis + ( [alpha*Q               , Q*A{i}' + Z'*B{i}' , Q*C' + Z'*Du'; ...
%                      A{i}*Q + B{i}*Z , alpha*Q                  , zeros(nx, nz); ...
%                      C*Q + Du*Z      , zeros(nz, nx)      , eye(nz)]        >= eps*eye(2*nx+nz) );
end     

%% solution
optimize(lmis,trace(rho),optSolver);
Q = value(Q);               % convert the matrix to double type
Z = value(Z);
rho = value(rho);
% Sd = value(Sd);
% H = value(H);
% Pd = value(Pd);

%% recovering gains
% K2 = Sd/H;
% K2 = [0];
K = Z/Q;
h2 = sqrt(rho);
P = inv(Q);
end