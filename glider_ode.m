function dydt = glider_ode(t, y, CL, m, S, CD0, K)
    % State Vector y: [V; gamma; H; R]
    % V: Velocity, gamma: Flight path angle, H: Altitude, R: Range
    V = y(1);
    gam = y(2);
    H = y(3);
    % R = y(4); % Not used in derivatives explicitly but tracked

    % Constants
    g = 9.81;
    rho0 = 1.225;
    
    % Atmosphere Model (Exponential approx for density)
    % rho = rho0 * exp(-H / 9296); 
    % For simple assignment, check if standard atmosphere is required. 
    % If H=3000m is constant-ish, use standard rho at h.
    % Let's use the standard troposphere formula:
    if H > 0
       
        rho = rho0 * exp(-H/9296);
    else
        rho = 1.225; % Sea level or crashed
    end

    % Aerodynamics
    CD = CD0 + K * CL^2;
    L = 0.5 * rho * V^2 * S * CL;
    D = 0.5 * rho * V^2 * S * CD;

    % Equations of Motion (Glider: Thrust T = 0)
    % dV/dt = (-D - m*g*sin(gamma)) / m
    dVdt = (-D - m*g*sin(gam)) / m;
    
    % dGamma/dt = (L - m*g*cos(gamma)) / (m*V)
    dGamdt = (L - m*g*cos(gam)) / (m*V);
    
    % dH/dt = V * sin(gamma)
    dHdt = V * sin(gam);
    
    % dR/dt = V * cos(gamma)
    dRdt = V * cos(gam);

    dydt = [dVdt; dGamdt; dHdt; dRdt];
end
