    syms X(t) theta(t) phi(t) real
    syms M mp IM Ip l L L_m R T Tp g m_w I_wheel real
    

    Nm = M * diff( X + (L+L_m)*sin(theta) - l*sin(phi), t, 2 );  
    N = mp * diff( X + L*sin(theta), t, 2 ) + Nm;
    PM = g*M + M * diff( (L+L_m)*cos(theta) + l*cos(phi), t, 2 );
    P = mp * diff( L*cos(theta), t, 2 ) + g*mp + PM;
    

    eq5 = diff(theta, t, 2) == ...
        ((P*L + L_m*PM)*sin(theta) - ...
         (N*L + Nm*L_m)*cos(theta) - ...
          T + Tp) / Ip;
    
    eq8 = diff(phi, t, 2) == ...
        (Nm*l*cos(phi) + PM*l*sin(phi) + Tp) / IM;
    
    eq9 = diff(X, t, 2) == ...
        (T - N*R)/ (m_w*R + I_wheel/R);
    
    % ============= 4. Get ddX, ddtheta, ddalpha =============
    ddtheta_expr = rhs(eq5);
    ddphi_expr = rhs(eq8);
    ddX_expr = rhs(eq9);
    
    syms x1 x2 x3 x4 x5 x6 ddottheta ddotphi ddotx real
    syms T_n Tp_n real 
    

    f1 = x2;   % dot{x1} = ddx
    
    f2 = subs(ddX_expr, {diff(theta,t,t),diff(phi,t,t),diff(X,t), diff(theta,t), diff(phi,t),X,theta,phi}, {ddottheta,ddotphi,x2, x4, x6,x1,x3,x5});
    f2 = subs(f2, {T, Tp}, {T_n, Tp_n});
    
    f3 = x4;   % dot{x3} = ddtheta
    
    f4 = subs(ddtheta_expr, {diff(X,t,t),diff(phi,t,t),diff(X,t), diff(theta,t), diff(phi,t),X,theta,phi}, {ddotx,ddotphi,x2, x4, x6,x1,x3,x5});
    f4 = subs(f4, {T, Tp}, {T_n, Tp_n});
    
    f5 = x6;   % dot{x5} = ddphi
    
    f6 = subs(ddphi_expr, {diff(theta,t,t),diff(X,t,t),diff(X,t), diff(theta,t), diff(phi,t),X,theta,phi}, {ddottheta,ddotx,x2, x4, x6,x1,x3,x5});
    f6 = subs(f6, {T, Tp}, {T_n, Tp_n});
    
    %% apply the small angle analysis that the square of the angular velocity.ddotphi
    f2 = subs(f2, ...
        {sin(x3),    cos(x3),    sin(x5),    cos(x5), x4^2  , x6^2}, ...
        {x3,         1,          x5,         1,         0        0});
    
    f4 = subs(f4, ...
        {sin(x3),    cos(x3),    sin(x5),    cos(x5) , x4^2  , x6^2}, ...
        {x3,         1,          x5,         1,         0        0});
    
    f6 = subs(f6, ...
        {sin(x3),    cos(x3),    sin(x5),    cos(x5) , x4^2  , x6^2}, ...
        {x3,         1,          x5,         1,         0        0});
    
    
    % make the functions back to equation
    f2 = ddotx == f2;
    f4 = ddottheta == f4;
    f6 = ddotphi == f6;
    sol = solve([f2,f4,f6], [ddotx,ddottheta,ddotphi]);
    
    sol2 = simplify(sol.ddotx);
    sol4 = simplify(sol.ddottheta);
    sol6 = simplify(sol.ddotphi);

    disp(sol2);
    disp(sol4);
    disp(sol6);

    f = [f1;sol2;f3;sol4;f5;sol6];
    A_sym = jacobian(f, [x1, x2, x3, x4, x5, x6]);  
    B_sym = jacobian(f, [T_n, Tp_n]);

    %% steady state signal
    x_star = [0, 0, 0 ,0 ,0 ,0];
    u_star = [0 , 0];

    x_aug_star = [0,0,0,0,0,0,0];
    u_aug_star = [0,0];



    Cv = zeros(1, 6);
    Cv(1, 2) = 1; %state 2, x_dot
    A_aug = [0,Cv;zeros(6,1),A_sym];
    B_aug = [ zeros(1, 2);B_sym];



    
    %% 5) substitute in at steady point
    A_lin = subs(A_sym, [x1, x2, x3, x4, x5, x6, T_n, Tp_n], [x_star, u_star]);
    B_lin = subs(B_sym, [x1, x2, x3, x4, x5, x6, T_n, Tp_n], [x_star, u_star]);
    A_aug_lin = subs(A_aug, [x1, x2, x3, x4, x5, x6, T_n, Tp_n], [x_star, u_star]);
    B_aug_lin = subs(B_aug, [x1, x2, x3, x4, x5, x6, T_n, Tp_n], [x_star, u_star]);
    
    param_values = {
        M, 1.2;   % Body Mass 10 kg
        mp, 1.6;    % support Mass 5 kg
        m_w, 0.3;
        IM, 0.00725;  % Rotation Inertia 0.2 kg·m²
        Ip, 0.03346;  
        I_wheel, 0.00038 ;  % wheel inertia 0.05 kg·m²
        l, 0.05;    % Center of mass of body to the joint 0.05 m
        L, 0.0903;    % wheel joint  1.0 m
        L_m, 0.0897;  
        R, 0.05;   % wheel radius 0.05 m
        g, 9.81    % 9.81 m/s²
    };

     param_values_num = {
        'M', 1.2;   % Body Mass 10 kg
        'mp', 1.6;    % support Mass 5 kg
        'm_w', 0.3;
        'IM', 0.00725;  % Rotation Inertia 0.2 kg·m²
        'Ip', 0.03346;  % Load moment of inertia 0.05 kg·m²
        'I_wheel', 0.00038 ;  % wheel inertia 0.05 kg·m²
        'l', 0.05;    % Center of mass of body to the joint
        'L', 0.0903;    % wheel joint  1.0 m
        'L_m', 0.0897;  
        'R', 0.05;   % wheel radius 0.15 m
        'g', 9.81    % 9.81 m/s²
    };

    % Create an empty struct to hold the parameters
    param_num = struct();
    
    % Loop through each row of the cell array
    for i = 1:size(param_values_num, 1)
        % Get the field name (e.g., 'M') and its value (e.g., 1.2)
        fieldName = param_values_num{i, 1};
        fieldValue = param_values_num{i, 2};
        
        % Add the new field to the struct
        param_num.(fieldName) = fieldValue;
    end


    % Create an empty struct to hold the parameters
    params_val = struct();
    

    params = param_values(:, 1);  % params
    values = param_values(:, 2);  % values
    
    % 将参数值代入 A_lin 和 B_lin
    A_lin_num = double(subs(A_lin, params, values));
    B_lin_num = double(subs(B_lin, params, values));

    A_aug_lin_num = double(subs(A_aug_lin, params, values));
    B_aug_lin_num = double(subs(B_aug_lin, params, values));



    
  % 显示结果
    disp('线性化后的系统矩阵 A:');
    disp(A_lin_num);
    
    disp('线性化后的输入矩阵 B:');
    disp(B_lin_num);

    disp('线性化后的系统矩阵 A aug:');
    disp(A_aug_lin_num);
    
    disp('线性化后的输入矩阵 B aug:');
    disp(B_aug_lin_num);


    % A_lin_num and B_lin_num calculated by previous steps
    A = A_lin_num;
    B = B_lin_num;
    
    X_desired = 0.6;  % desired position
    
    % weight matrix
    Q = diag([1e-6, 100, 1, 1, 5000, 1]);  
    R = diag([100 25]); 

    [K, ~, ~] = lqr(A, B, Q, R);
        
   % Build Augmented state space model

   dX_desired = 0.2;
   dx_desired = [0;0;dX_desired;0;0;0;0];
   Q_1 = diag([600 ,1e-6, 100, 1, 1, 5000, 1]);  
   R_1 = diag([100 25]);   

   [K_1, ~, ~] = lqr(A_aug_lin_num, B_aug_lin_num, Q_1, R_1);



