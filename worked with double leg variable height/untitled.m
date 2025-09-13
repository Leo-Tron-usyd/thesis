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
    

    base_params = struct();
    base_params.M = 1.2;
    base_params.mp = 1.6 *2;
    base_params.m_w = 0.3 * 2;
    base_params.IM = 0.00725;
    base_params.I_wheel = 0.00038 * 2;
    base_params.l = 0.05;
    base_params.R = 0.05;
    base_params.g = 9.81;

    results = struct('params', {}, 'K_gain', {});

    for L_val = 0.13:0.01:0.28
 %   for L_val = 0.18
        current_params = base_params;
        % Call the calculator function, which now returns the inertia AND the angles.
        [inertia_val, l_num, l_m_num] = inertia_calculator(L_val);
        current_params.Ip = inertia_val * 2;
        current_params.L = l_num;
        current_params.L_m = l_m_num;
    
        A_aug_lin_num = double(subs(A_lin, current_params));
        B_aug_lin_num = double(subs(B_lin, current_params));
   
       Q_1 = diag([1e-6, 600, 1, 1, 5000, 1]);  
       R_1 = diag([100 25]);   
    
       [K_1, ~, ~] = lqr(A_aug_lin_num, B_aug_lin_num, Q_1, R_1);

        % --- 2. CREATE A STRUCT TO PAIR THE DATA ---
        new_result.params = current_params;
        new_result.K_gain = K_1;
        
        % --- 3. APPEND THE PAIRED DATA TO YOUR RESULTS ARRAY ---
        results(end+1) = new_result;
    end

    disp(results)

    

    % fitting the matrix using polygons
    %% 1. Data Preparation
    % Extract L values and K matrices from the results struct.
    num_points = length(results);
    L_values = zeros(1, num_points);
    % Get the size of the K matrix (assuming all K matrices are the same size).
    [k_rows, k_cols] = size(results(1).K_gain);
    % Use a cell array to store the data for each element of the K matrix as L changes.
    K_elements_data = cell(k_rows, k_cols);
    
    for i = 1:num_points
        L_values(i) = results(i).params.L;
        for r = 1:k_rows
            for c = 1:k_cols
                K_elements_data{r, c}(i) = results(i).K_gain(r, c);
            end
        end
    end
    
    %% 2. Perform the Polynomial Fit
    % =========================================================================
    % ===                THIS IS A KEY TUNING PARAMETER                     ===
    % =========================================================================
    polynomial_order = 4; % <<<<<< Choose the order 'n' of the polynomial. Try 2, 3, 4, 5...
    % =========================================================================
    
    p_models = cell(k_rows, k_cols); % Initialize the cell array to store models
    for r = 1:k_rows
        for c = 1:k_cols
            % Use polyfit to find the polynomial coefficients
            p_models{r, c} = polyfit(L_values, K_elements_data{r, c}, polynomial_order);
        end
    end


    [k_rows, k_cols] = size(p_models);
    num_coeffs = polynomial_order + 1;
    
    % Initialize an empty matrix to store the coefficients
    p_matrix = zeros(k_rows * k_cols, num_coeffs);
    
    % Loop through the cell array and stack the coefficient vectors into the matrix
    idx = 1;
    for r = 1:k_rows
        for c = 1:k_cols
            p_matrix(idx, :) = p_models{r, c};
            idx = idx + 1;
        end
    end

    
    fprintf('Completed %d-order polynomial fitting for %d elements of the K matrix.\n', polynomial_order, k_rows*k_cols);
    
    %% 3. Visualization and Verification
    % Create a new, finer L vector for smooth plotting.
    L_fine = linspace(min(L_values), max(L_values), 200);
    
    figure('Name', 'Polynomial Fit Results for K Matrix Elements', 'Position', [50, 50, 1000, 600]);
    plot_idx = 1;
    for r = 1:k_rows
        for c = 1:k_cols
            subplot(k_rows, k_cols, plot_idx);
            
            % Plot the original data points.
            plot(L_values, K_elements_data{r, c}, 'bo', 'MarkerFaceColor', 'b', 'DisplayName', 'Original Data Points');
            hold on;
            
            % Calculate and plot the fitted curve.
            K_fit_fine = polyval(p_models{r, c}, L_fine);
            plot(L_fine, K_fit_fine, 'r-', 'LineWidth', 2, 'DisplayName', ['Order ' num2str(polynomial_order) ' Fit']);
            
            grid on;
            title(sprintf('K(%d, %d) vs. L', r, c));
            xlabel('Parameter L Value');
            ylabel(sprintf('K(%d, %d) Value', r, c));
            legend('show');
            
            plot_idx = plot_idx + 1;
        end
    end



function [I,LH,LL] = inertia_calculator(L_input)
    %#codegen
    % Solves for theta1, theta2 from:
    %   2*L1*sin(theta1) + 2*L2*sin(theta2) = L_input
    %   0.5*LR + 2*L1*cos(theta1) - 2*L2*cos(theta2) = 0
    % using a few Newton steps (no function handles, codegen-safe).
    
    % Constants (same as your previous function)
    L1 = 0.07;  L2 = 0.105;
    M1 = 0.3;   M2 = 0.3;    % not used directly but retained for clarity
    M3 = 0.5;   M4 = 0.5;
    LR = 0.2;
    
    % Initial guess (could be persisted across calls for faster convergence)
    theta1 = 0.2;
    theta2 = 0.0;
    
    % Newton iterations (small, damped)
    for it = 1:10
        s1 = sin(theta1); c1 = cos(theta1);
        s2 = sin(theta2); c2 = cos(theta2);
    
        f1 = 2*L1*s1 + 2*L2*s2 - L_input;
        f2 = 0.5*LR + 2*L1*c1 - 2*L2*c2;
    
        % Jacobian
        J11 =  2*L1*c1;   J12 =  2*L2*c2;
        J21 = -2*L1*s1;   J22 =  2*L2*s2;
    
        % Solve J * d = -f  (2x2 solve)
        detJ = J11*J22 - J12*J21;
        if abs(detJ) < 1e-12
            break; % singular Jacobian; leave current guess
        end
        d1 = (-f1*J22 + f2*J12)/detJ;
        d2 = (-J11*f2 + J21*f1)/detJ;
    
        % Damping + update
        alpha = 0.7;
        theta1 = theta1 + alpha*d1;
        theta2 = theta2 + alpha*d2;
    
        % Clamp to bounds (your original lb/ub)
        if theta1 < 0,       theta1 = 0;     end
        if theta1 > pi,      theta1 = pi;    end
        if theta2 < -pi/2,   theta2 = -pi/2; end
        if theta2 >  pi/2,   theta2 =  pi/2; end
    
        if (abs(d1)+abs(d2)) < 1e-9
            break;
        end
    end
    
    % Geometry
    I_l = 2827482.39e-9;   % long leg inertia
    I_h = 586653.56e-9;    % short leg inertia
    
    y1 = L1*sin(theta1);
    y3 = 2*y1 + L2*sin(theta2);
    
    % COM split
    LH = (y1*M1 + y1*M2 + y3*M3 + y3*M4) / (M1 + M2 + M3 + M4);
    LL = L_input - LH;
    
    % Distances
    LHR = sqrt((LH - y1)^2 + (LR/2 + L1*cos(theta1))^2);
    LLR = sqrt((y3 - LH)^2 + (L2*cos(theta2))^2);
    
    % Inertia
    I = 2*(I_h + M1*LHR^2) + 2*(I_l + M3*LLR^2);
end