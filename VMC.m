syms phi1 phi2 phi3 phi4 real

L1 = 0.14;
L2 = 0.25;
L3 = 0.25;
L4 = 0.14;
L5 = 0.21;  % between A and E

% Define coordinates
XB = -L1*cos(pi-phi1);
YB = L1*sin(pi-phi1);
XD = L5+L4*cos(phi4);
YD = L4*sin(phi4);

A = 2*L2*(XD-XB);
B = 2*L2*(YD-YB);
LBD2=(XB-XD)^2+(YB-YD)^2;
C = L2^2+LBD2-L3^2;

% Check for reachability
discriminant = A^2 + B^2 - C^2;
% if discriminant < 0
%     % This configuration is not reachable
%     L0 = 0;
%     phi0 = 0;
%     return; % Exit the function
% end
% Now it's safe to calculate phi2
phi2 = 2*atan((B+sqrt(discriminant))/(A+C));

% Compute end effector point
XC = XB + L2*cos(phi2);
YC = YB + L2*sin(phi2);


L0 = sqrt((L5/2 - XC)^2 + YC^2);

phi0 = piecewise(XC <= L5/2, pi - asin(YC / L0), ... % Condition 1, Value 1
                 XC > L5/2,  asin(YC / L0));      % Condition 2, Value 2


% Compute Jacobian
J = jacobian([L0, phi0], [phi1, phi4]);

% Simplify the result
J_simplified = simplify(J);

% Display result
disp('Jacobian Matrix:');
disp(J_simplified);

% Define the input arguments for the function (phi1 and phi4)
vars = [phi1, phi4];

% Use matlabFunction to convert the symbolic expression to a .m file
matlabFunction(J_simplified, 'File', 'calculateJacobian', 'Vars', vars);

disp(' ');
disp('Successfully created the function file: calculateJacobian.m');
