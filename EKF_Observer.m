current_state =
current_input = 
dt = 


Q_k = 
P_bk =





calculated_accelerations = accel_func_generated( ...
    current_state, ...
    current_input, ...
    param_values_num.M, ...
    param_values_num.mp, ...
    param_values_num.IM, ...
    param_values_num.Ip, ...
    param_values_num.l, ...
    param_values_num.L, ...
    param_values_num.L_m, ...
    param_values_num.R, ...
    param_values_num.g, ...
    param_values_num.m_w, ...
    param_values_num.I_wheel ...
);

ddx = calculated_accelerations(1);
ddtheta = calculated_accelerations(2);
ddphi = calculated_accelerations(3);

dx_pred = current_state(2)+dt*ddx;
dtheta_pred = current_state(4)+dt*ddtheta;
dphi_pred = current_state(6)+dt*ddphi;

%% estimation 
x_pred = current_state(1)+dt*dx_pred;
theta_pred =  current_state(3)+dt*dtheta_pred;
phi_pred = current_state(5)+dt*dphi_pred;
%% X_K_Hat: The prediction of the next state 
X_k_hat = [x_pred;dx_pred;theta_pred;dtheta_pred;phi_pred;dphi_pred];

param_cell = struct2cell(param_values_num);
A_c_numerical = jacobian_Ac_generated(x_hat_prev, u_prev, param_cell{:});
%% FK: state transition matrix
Fk = eye(6) + dt * A_c_numerical;

%% Pk: Current convariance matrix 
Pk = Fk*P_bk*Fk.'+Q_k;






