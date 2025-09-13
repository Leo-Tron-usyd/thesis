clear; clc;

% 1. Define all symbolic variables and parameters
syms M mp IM Ip l L L_m R T Tp g m_w I_wheel real

% BEST PRACTICE: Use distinct, simple placeholder variables from the start
syms x_s dx_s theta_s dtheta_s phi_s dphi_s T_s Tp_s real
syms ddx ddtheta ddphi real % Variables to solve for

% Create vectors for states and inputs using these simple placeholders
state_vec_sym = [x_s; dx_s; theta_s; dtheta_s; phi_s; dphi_s];
input_vec_sym = [T_s; Tp_s];

% 2. Define intermediate expressions using the simple placeholder variables
Nm = M*(l*sin(phi_s)*dphi_s^2     - sin(theta_s)*(L + L_m)*dtheta_s^2   + ddx - ddphi*l*cos(phi_s) + ddtheta*cos(theta_s)*(L + L_m));
N = M*(l*sin(phi_s)*dphi_s^2 - sin(theta_s)*(L + L_m)*dtheta_s^2 + ddx - ddphi*l*cos(phi_s) + ddtheta*cos(theta_s)*(L + L_m)) + mp*(- L*sin(theta_s)*dtheta_s^2 + ddx + L*ddtheta*cos(theta_s));
PM = M*g - M*(l*cos(phi_s)*dphi_s^2 + cos(theta_s)*(L + L_m)*dtheta_s^2 + ddphi*l*sin(phi_s) + ddtheta*sin(theta_s)*(L + L_m));
P = M*g - M*(l*cos(phi_s)*dphi_s^2 + cos(theta_s)*(L + L_m)*dtheta_s^2 + ddphi*l*sin(phi_s) + ddtheta*sin(theta_s)*(L + L_m)) + g*mp - mp*(L*cos(theta_s)*dtheta_s^2 + L*ddtheta*sin(theta_s));

% 3. Set up the system of equations 
% Note: Using placeholder T_s and Tp_s now
eq5 = ddtheta == ((P*L + L_m*PM)*sin(theta_s) - (N*L + Nm*L_m)*cos(theta_s) - T_s + Tp_s) / Ip;
eq8 = ddphi == (Nm*l*cos(phi_s) + PM*l*sin(phi_s) + Tp_s) / IM;
eq9 = ddx == (T_s - N*R) / (m_w*R + I_wheel/R);

% 4. Solve the coupled system for the accelerations
fprintf('Solving the symbolic system for accelerations...\n');
sol = solve([eq5, eq8, eq9], [ddx, ddtheta, ddphi]);
fprintf('Done.\n');

% Extract the solved, explicit expressions for accelerations
accel_exprs = [sol.ddx; sol.ddtheta; sol.ddphi];

% 5. Define the full continuous-time state derivative vector, x_dot = f_c(x, u)
f_c = [dx_s; accel_exprs(1); dtheta_s; accel_exprs(2); dphi_s; accel_exprs(3)];

% 6. Calculate the Jacobian of the CONTINUOUS model
fprintf('Calculating the Jacobian matrix A_c...\n');
A_c_sym = jacobian(f_c, state_vec_sym);
fprintf('Done.\n');

% 7. Create and save the numerical functions
params_list = {M, mp, IM, Ip, l, L, L_m, R, g, m_w, I_wheel};

% Create the function for accelerations
accel_func = matlabFunction( ...
    accel_exprs, ...
    'File', 'accel_func_generated', ...
    'Vars', {state_vec_sym, input_vec_sym, params_list{:}} ...
);

% Create the function for the continuous Jacobian A_c
% --- THIS IS THE CORRECTED LINE ---
jacobian_Ac_func = matlabFunction( ...
    A_c_sym, ...
    'File', 'jacobian_Ac_generated', ...
    'Vars', {state_vec_sym, input_vec_sym, params_list{:}} ...
);

disp('Successfully created and saved both accel_func and jacobian_Ac_func.');


%% --------------------------------------------------------------------------------------------------------------------------------------

% % --- 1) Symbols ---
% clear; clc;
% 
% % parameters
% syms M mp g l L L_m real
% 
% % time-varying functions (use distinct names)
% syms X(t) TH(t) PH(t)
% 
% % scalar placeholders for state/derivatives
% syms x_s dx ddx theta_s dtheta ddtheta phi_s dphi ddphi real
% 
% % --- 2) Build expressions with time functions ---
% Nm_t = M * diff( X(t) + (L+L_m)*sin(TH(t)) - l*sin(PH(t)), t, 2 );
% N_t  = mp*diff( X(t) + L*sin(TH(t)), t, 2 ) + Nm_t;
% 
% PM_t = g*M + M*diff( (L+L_m)*cos(TH(t)) + l*cos(PH(t)), t, 2 );
% P_t  = mp*diff( L*cos(TH(t)), t, 2 ) + g*mp + PM_t;
% 
% % --- 3) Substitute placeholders for derivatives & functions ---
% old_vars = [ ...
%     diff(X(t),t,2),   diff(TH(t),t,2),   diff(PH(t),t,2), ...
%     diff(X(t),t),     diff(TH(t),t),     diff(PH(t),t),   ...
%     X(t),             TH(t),             PH(t)            ];
% 
% new_vars = [ ...
%     ddx,              ddtheta,           ddphi, ...
%     dx,               dtheta,            dphi,  ...
%     x_s,              theta_s,           phi_s ];
% 
% Nm = subs(Nm_t, old_vars, new_vars);
% N  = subs(N_t,  old_vars, new_vars);
% PM = subs(PM_t, old_vars, new_vars);
% P  = subs(P_t,  old_vars, new_vars);
% 
% % --- 4) Show results ---
% fprintf('Nm after substitution:\n'); disp(Nm);
% fprintf('N after substitution:\n');  disp(N);
% fprintf('PM after substitution:\n'); disp(PM);
% fprintf('P after substitution:\n');  disp(P);

