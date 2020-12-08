function [log] = QP_based_MPC_Tracking(i, K, dt, p, v, a, p_0, v_0, a_0,v_u,v_d,a_u,a_d,t)

w1 = 1000;
w2 = 1;
w3 = 1;
w4 = 1;
w5 = 10000;

%% Construct the prediction matrix
[Tp, Tv, Ta, Bp, Bv,Ba] = getPredictionMatrix(K, dt, p_0, v_0, a_0);

%% Construct the optimization problem

% with hard constraints
%H = w4*eye(K)+w1*(Tp'*Tp)+w2*(Tv'*Tv)+w3*(Ta'*Ta);
%F = w1*Bp'*Tp+w2*Bv'*Tv+w3*Ba'*Ta;
%A = [Tv;-Tv;Ta;-Ta];
%b = [ones(20,1)-Bv; ones(20,1)+Bv; ones(20,1)-Ba; ones(20,1)+Ba];

% with soft constraints and tack the reference traj. : p,v,a
H = blkdiag(w4*eye(K)+w1*(Tp'*Tp)+w2*(Tv'*Tv)+w3*(Ta'*Ta), w5*eye(K));
F = [w1*(Bp-p)'*Tp+w2*(Bv-v)'*Tv+w3*(Ba-a)'*Ta zeros(1,K)];

A = [Tv zeros(K);-Tv -eye(K);Ta zeros(K); -Ta zeros(K); zeros(size(Ta)) -eye(K)];
b = [v_u*ones(K,1)-Bv;-v_d*ones(K,1)+Bv;a_u*ones(K,1)-Ba;-a_d*ones(K,1)+Ba; zeros(K,1)];


%% Solve the optimization problem
J = quadprog(H, F, A, b);

%% Apply the control
if i == 3
    j = 0;
else
    j = J(1);
end

p_0 = p_0 + v_0*dt + 0.5*a_0*dt^2 + 1/6*j*dt^3;

if i == 3
    v_0 = -0.5;
    a_0 = 0;
else
    v_0 = v_0 + a_0*dt + 0.5*j*dt^2;
    a_0 = a_0 + j*dt;
end

%% Log the state
log = [t p_0 v_0 a_0];

end