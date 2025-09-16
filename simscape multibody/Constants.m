Damping_coeff = 0.0001;
Mass_upper_leg = 0.3;
Mass_lower_leg = 0.5;
Mass_body = 1.2;
Mass_wheel = 0.3;
X0_hat = zeros(6,1);
X0_hat(1)  = 0.0217;
P0 = zeros(6,6);
deg = pi/180;
R_k = diag([1e-8, 1e-8, 1e-8]);   % but keep SPD!

sigma_ax   = 0.7;         % m/s^2
sigma_add  = 30*deg;      % rad/s^2  (for both theta, phi)
qx  = sigma_ax^2;         % 0.25
qth = sigma_add^2;        % ~0.1219
qphi= sigma_add^2;        % ~0.1219

