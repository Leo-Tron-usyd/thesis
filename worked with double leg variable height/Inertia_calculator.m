% --- Function Definition ---
% It's good practice to keep function definitions at the end of a script file.
function [I,LH,LL ] = inertia_calculator(L_input)
    % This function calculates the inertia 'I' for a given input length 'L'.
    % It also returns the solved angles theta1 and theta2.

    % --- Define Constants ---
    L1 = 0.07;
    L2 = 0.105;
    M1 = 0.3;
    M2 = 0.3;
    M3 = 0.5;
    M4 = 0.5;
    LR = 0.2;

    % --- Define System of Equations for the Solver ---
    % 'fun' returns the error for a given set of angles 'theta'.
    % lsqnonlin will try to make these errors as close to zero as possible.
    fun = @(theta) [
        2*L1*sin(theta(1)) + 2*L2*sin(theta(2)) - L_input;
        0.5*LR + 2*L1*cos(theta(1)) - 2*L2*cos(theta(2))
        ];
    
    % --- Initial Guess and Bounds for the solver ---
    theta0 = [0.2, 0]; % Initial guess for [theta1, theta2]
    lb = [0, -pi/2];         % Lower bounds [0, 0]
    ub = [pi, pi/2];   % Upper bounds [pi/3, pi/3]
    
    % --- Solve the System using lsqnonlin ---
    % Suppress the solver's default output for a cleaner command window.
    options = optimoptions('lsqnonlin', 'Display', 'off');
    theta_sol = lsqnonlin(fun, theta0, lb, ub, options);
    
    % --- Extract Solution ---
    theta1 = theta_sol(1);
    theta2 = theta_sol(2);
    
    % --- Subsequent Calculations for Inertia ---
    I_l = 2827482.39 * 1e-9;   % long leg
    I_h = 586653.56 * 1e-9;    % short leg
    
    y1 = L1*sin(theta1);
    y2 = y1;
    y3 = 2*y1 + L2*sin(theta2);
    y4 = y3;
    
    LH = (y1*M1 + y2*M2 + y3*M3 + y4*M4) / (M1 + M2 + M3 + M4);
    LL = L_input-LH;
    LHR = sqrt((LH - y1)^2 + (LR/2 + L1*cos(theta1))^2);
    LLR = sqrt((y3 - LH)^2 + (L2*cos(theta2))^2);
    
    % Final inertia calculation
    I = 2*(I_h + M1*LHR^2) + 2*(I_l + M3*LLR^2);
end