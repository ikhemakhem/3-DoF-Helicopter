function [I_alpha, I_beta, I_gamma] = get_inertias(beta, gamma)
I_alpha = (cos(beta))^2 + 0.095*cos(beta)*sin(beta) + 0.097; % [kgm^2]
I_beta = -0.04*(cos(gamma))^2 + 1.1; % [kgm^2]
I_gamma = 0.0415; % [kgm^2]
end