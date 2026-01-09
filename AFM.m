clear; clc; close all;

%% Aircraft Characterization
m = 750;            %kg
S = 12.47;          %m
CD0 = 0.036;        
e = 0.8; 
AR = 8.8; 
K = 1/(pi*e*AR);
H_i = 3000;
R_i = 0;
t_lim = [0 2000 ]; 
CL0=0.365;
CL_alpha=4.2;
CL_max=1.5;
g=9.81;
b=9296;
w=m*g;

% --- 2. Define Cases ---
% Case 1: Min Glide Angle (Max Range)
CL1 = sqrt(CD0 / K); 
CD1 = CD0 + K*CL1^2;
gam1_est = -atan(CD1/CL1); 
V1_est = sqrt((2*m*g)/(1.225*S*CL1*cos(gam1_est))); % Approx V for initial guess

% Case 2: Min Sink Rate (Max Endurance)
% Calculated CL is ~1.545, which is > CL_max (1.5). 
% We use the theoretical value for the math check, but physically limit to 1.5.
CL2 = sqrt(3*CD0/K); 

% Case 3: Max CL
CL3 = 1.5;

% Store Cases in a Struct for Looping
cases(1).name = 'Min Glide Angle'; cases(1).CL = CL1;
cases(2).name = 'Min Sink Rate';   cases(2).CL = CL2; 
cases(3).name = 'Max CL';          cases(3).CL = CL3;

% --- 3. Simulation Loop ---
% --- Setup Figures (Run this BEFORE the loop) ---
% We create 4 separate figure windows and store their "handles" (ID numbers)
fig_V = figure('Name', 'Velocity vs Time'); hold on; grid on;
xlabel('Time (s)'); ylabel('Velocity (m/s)'); title('Velocity vs Time');

fig_Gam = figure('Name', 'Flight Path Angle'); hold on; grid on;
xlabel('Time (s)'); ylabel('Gamma (deg)'); title('Flight Path Angle vs Time');

fig_Alt = figure('Name', 'Altitude vs Time'); hold on; grid on;
xlabel('Time (s)'); ylabel('Altitude (m)'); title('Altitude vs Time');

fig_Traj = figure('Name', 'Trajectory'); hold on; grid on;
xlabel('Range (km)'); ylabel('Altitude (m)'); title('Trajectory (H vs R)');

fig_CL = figure('Name', 'Lift Coefficient vs Time'); hold on; grid on;
xlabel('Time (s)'); ylabel('C_L'); title('Lift Coefficient vs Time');
ylim([0.5 1.6]); % Set limits for better visibility

fig_Alpha = figure('Name', 'Angle of Attack vs Time'); hold on; grid on;
xlabel('Time (s)'); ylabel('Alpha (deg)'); title('Angle of Attack vs Time');

colors = {'b', 'r', 'g'};

for i = 1:3
    current_CL = cases(i).CL;
    

    % We strat at initial conditons given in assignment v_inf=50 and
    % h_i=3000m

   
    rho_init = 1.225 *exp(-H_i/b);
    
    % Iteratively solve for gamma and V using the initial conditions
    % specified
  
    CD_curr = CD0 + K*current_CL^2;
    gam_init = -atan(CD_curr / current_CL);
    V_init = 50;
    
    y0 = [V_init; gam_init; H_i; R_i];
    
    % Run ODE45
    % Note: We pass anonymous function to handle parameters
    odeFun = @(t,y) glider_ode(t, y, current_CL, m, S, CD0, K);
    
    % Use Event function to stop when altitude H = 0 (Ground)
    options = odeset('Events', @ground_event);
    [t, y] = ode45(odeFun, t_lim, y0, options);


    % --- Calculate Alpha Vector ---
    % Formula: CL = CL0 + CL_alpha * alpha (in radians)
    % Alpha = (CL - CL0) / CL_alpha
    alpha_rad = (current_CL - CL0) / CL_alpha;
    alpha_deg = alpha_rad * 180/pi;
    
    % Create vectors for plotting (same length as time t)
    CL_vector = ones(size(t)) * current_CL;
    Alpha_vector = ones(size(t)) * alpha_deg;


     % --- Plotting (Modified) ---
    
    % 1. Velocity Plot
    figure(fig_V); % Switch focus to Velocity window
    plot(t, y(:,1), colors{i}, 'LineWidth', 1.5);
    
    % 2. Gamma Plot
    figure(fig_Gam); % Switch focus to Gamma window
    plot(t, y(:,2)*180/pi, colors{i}, 'LineWidth', 1.5);
    
    % 3. Altitude Plot
    figure(fig_Alt); % Switch focus to Altitude window
    plot(t, y(:,3), colors{i}, 'LineWidth', 1.5);
    
    % 4. Trajectory Plot
    figure(fig_Traj); % Switch focus to Trajectory window
    plot(y(:,4)/1000, y(:,3), colors{i}, 'LineWidth', 1.5);
    
    %5.Cl plot vs time
    figure(fig_CL);    
    plot(t, CL_vector, colors{i}, 'LineWidth', 1.5);


    
    %6. Alpha plot vs time
    figure(fig_Alpha); 
    plot(t, Alpha_vector, colors{i}, 'LineWidth', 1.5);
    ylim([0 ,20])
    
    fprintf('Case: %s | Range: %.2f km | Time: %.2f s | CL: %.3f | Alpha: %.2f deg\n', ...
            cases(i).name, y(end,4)/1000, t(end), current_CL, alpha_deg);
end
    


 figure(fig_CL);  
    yline(CL_max, 'w--', 'CL_{max}', 'LineWidth', 1);

legend_names = {cases.name};

figure(fig_V);    legend(legend_names, 'Location', 'best');
figure(fig_Gam);  legend(legend_names, 'Location', 'best');
figure(fig_Alt);  legend(legend_names, 'Location', 'best');
figure(fig_Traj); legend(legend_names, 'Location', 'best');
figure(fig_CL); legend(legend_names, 'Location', 'best');
figure(fig_Alpha); legend(legend_names, 'Location', 'best');

% --- Helper Function: Ground Detection ---
function [value, isterminal, direction] = ground_event(t, y)
    value = y(3);     % Stop when Altitude (y(3)) = 0
    isterminal = 1;   % Stop the integration
    direction = -1;   % Only when decreasing (falling)
end




