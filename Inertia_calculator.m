% --- 1. Define Known Parameters ---

% Properties of the central rod
m_rod = 1.7;      % Assume total mass of the central rod (kg)
L_rod = 0.0125;       % Assume total length of the central rod (m)

% Geometric position of the side planes/rods
% Assume the center of mass of the four side planes is at a distance d
d_planes = 0.005;  % Distance from the center of mass of each side plane to the central axis (m)


% --- 2. Calculate the Moment of Inertia of the Central Rod ---

% For a slender rod rotating about its center, the moment of inertia is (1/12)*m*L^2
I_rod = (1/12) * m_rod * L_rod^2;

fprintf('Total moment of inertia of the central rod I_rod = %.4f kg*m^2\n\n', I_rod);


% --- 3. Distribute Mass and Inertia to the Four Side Planes ---

% Assumption 1: The total mass is evenly distributed among the four side planes
m_plane = m_rod / 4;
fprintf('After distribution, the mass of each side plane m_plane = %.4f kg\n', m_plane);

% Assumption 2: The geometry and position of the four side planes are identical

% According to the Parallel Axis Theorem, the total moment of inertia of the four side planes is:
% I_total_planes = 4 * (I_plane_cm + m_plane * d_planes^2)
% We require I_total_planes == I_rod

% Therefore, we can solve for the required moment of inertia of each side plane about its own center of mass, I_plane_cm
% I_rod = 4 * I_plane_cm + 4 * m_plane * d_planes^2
% I_rod = 4 * I_plane_cm + m_rod * d_planes^2
% 4 * I_plane_cm = I_rod - m_rod * d_planes^2
I_plane_cm = (I_rod - m_rod * d_planes^2) / 4;


% --- 4. Output and Check the Results ---

fprintf('Distance from the center of mass of each side plane to the central axis d = %.4f m\n', d_planes);

% Check if the calculated moment of inertia is positive
if I_plane_cm < 0
    fprintf('\nError: The calculated moment of inertia for the side planes is negative (%.4f kg*m^2).\n', I_plane_cm);
    fprintf('This means that for the current mass distribution, the distance d (%.2f m) is too large.\n', d_planes);
    fprintf('Please decrease the value of d or reconsider the mass distribution.\n');
else
    fprintf('Calculated moment of inertia for each side plane about its own CoM should be I_plane_cm = %.4f kg*m^2\n', I_plane_cm);
    
    % Verification
    I_total_equivalent = 4 * (I_plane_cm + m_plane * d_planes^2);
    fprintf('\nVerification: The total moment of inertia of the equivalent system = %.4f kg*m^2\n', I_total_equivalent);
    fprintf('This matches the moment of inertia of the central rod (%.4f kg*m^2).\n', I_rod);
end
