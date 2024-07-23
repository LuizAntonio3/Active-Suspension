function [K,hinf,P] = hinf_discrete_sat_V2(A,B,C,Du,Dw,Bw,u)

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
rho = sdpvar(1,1,'full');

% H = sdpvar(nx,nx,'full');
% Sd = sdpvar(nu,nx,'full');
% Pd = sdpvar(nx,nx,'symmetric');

%% define LMI constraints
eps = 1e-9;                 % tolerance of the semi-definite problem

lmis = [];
S = Z - V;
for i = 1:nu
    lmis = lmis + ( [Q, S(i,:)'; ...
                     S(i,:), u(i)] >= eps*eye(nx+1) );
end

alpha = 1;
for i = 1:nv
%     M11 = kron(R{1},Pd) + kron(R{2},(A{i}*H + B{i}*Sd)) + kron(R{2}',(A{i}*H + B{i}*Sd)');
%     M22 = kron(R{3}, (Pd -H -H'));
%     M12 = kron(R{2}', (Pd -H')) + kron(R{3},(A{i}*H + B{i}*Sd));
%     lmis = lmis + ([M11, M12; ...
%                     M12', M22] <= -eps*eye(2*nr*nx));

%     lmis = lmis + ( [-alpha*Q, Q*A{i}' + Z'*B{i}', Q*C' + Z'*Du', zeros(nx,nw); ...
%                      A{i}*Q + B{i}*Z, - alpha*Q, zeros(nx,nz), Bw{i}; ...
%                      C*Q + Du*Z, zeros(nz,nx), - eye(nz), Dw; ...
%                      zeros(nw,nx), Bw{i}', Dw', - rho*eye(nw)] <= -eps*eye(2*nx+nz+nw) );
                 
         lmis = lmis + ( [-alpha*Q, Q*A{i}' + Z'*B{i}', V', Q*C' + Z'*Du', zeros(nx,nw); ...
                     A{i}*Q + B{i}*Z, - alpha*Q, - B{i}*U, zeros(nx,nz), Bw{i}; ...
                     V, - U*B{i}', - 2*U, - U*Du', zeros(nu,nw); ...
                     C*Q + Du*Z, zeros(nz,nx), - Du*U, - eye(nz), Dw; ...
                     zeros(nw,nx), Bw{i}', zeros(nw,nu), Dw', - rho*eye(nw)] <= -eps*eye(2*nx+nu+nz+nw) );
end     

%% solution
optimize(lmis,rho,optSolver);
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
hinf = sqrt(rho);
P = inv(Q);
end