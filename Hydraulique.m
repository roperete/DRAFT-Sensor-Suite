
clear
close all

losses = struct();
loss_names = {};

Q_history = [];          
DP_history = [];         
P_available_history = []; 

Q_nom = 594/3600;      % max flow rate [m³/s]
P_max = 120;           % max pressure [Pa]
P_available = @(Q) P_max - P_max*(Q/Q_nom).^0.551958;

rho = 1.1;              % air density [kg/m³]
mu = 1.8e-5;            % dynamic viscosity [Pa·s]
D = 0.094;              % inner tube diameter [m]
A = pi*(D/2)^2;         % tube cross-section [m²]
v = Q_nom / A;          % initial velocity [m/s]

tol = 1e-3;             % tolerance on pressure difference [Pa]
DP = 0;                 % initial total pressure (losses side)
Q = Q_nom;
n = 0;
max_iter = 100;        % iteration limit

while abs(P_max - DP) > tol && n < max_iter
    DP = 0;  % Reset losses at each iteration
    losses = struct(); % Reset losses structure
    
    % Calculate velocities in each section                      % velocity in main tube (94mm)
    v = Q / A;                              % velocity in main tube (94mm)
    v_upstream = Q / (pi * (0.148/2)^2);   % velocity in 148mm section
    v_100mm = Q / (pi * (0.1/2)^2);        % velocity in 100mm section
    v_75mm = Q / (pi * (0.075/2)^2);       % velocity in 75mm section
    v_50mm = Q / (pi * (0.05/2)^2);        % velocity in 50mm section
    v_120mm = Q / (pi * (0.120/2)^2);      % velocity in 120mm section
    
    % Loss 1: Contraction from fan to tube
    loss0 = sudden_expansion(0.148, 0.149, rho, v_upstream);
    loss1 = contraction_angle(0.149, 0.1, rho, v_upstream, 10.17);
    loss2 = sudden_contraction(0.1, D, rho, v_100mm);
    losses = add_loss(losses, 'fan_to_tube', loss1 + loss2+loss0);
    DP = DP + losses.fan_to_tube;

    %Loss 2: Sensor suite
    % loss1 = sudden_contraction(D, 0.091, rho, v);
    % loss2 = expansion_angle(0.091, 0.100, rho, v, 20);
    % loss3 = singular_loss(0.5, v_100mm, rho);
    % loss4 = sudden_contraction(0.095, 0.091, rho, v);
    % loss5 = sudden_contraction(0.1, D, rho, v_100mm);
    % losst=loss1+loss2+loss3+loss4+loss5;
    % losses = add_loss(losses, 'sensor_suite_1', losst);
    % DP=DP+losses.sensor_suite_1;

    % Loss 3: 3-way junction with single active path
    % losses = add_loss(losses, 'junction_3way', singular_loss(0.7, v, rho));
    % DP = DP + losses.junction_3way;
    
    % Loss 14: Valve
    % losses=add_loss(losses,'Valve',singular_loss(0.5, v_100mm, rho));
    % DP=DP+losses.Valve;

    % Loss 4: Elbows group 1
    elbow_loss_1 = singular_loss(0.7, v, rho);
    losses = add_loss(losses, 'elbows_group_1', 2 * elbow_loss_1);
    DP = DP + losses.elbows_group_1;
    
    % Loss 5: Soil filter - to be implemented with Darcy's law
    % losses = add_loss(losses, 'regolith_filter', 100);
    % DP = DP + losses.regolith_filter;
    
   
    
    % Loss 6: Elbows group 2
    elbow_loss_1 = singular_loss(0.7, v, rho);
    losses = add_loss(losses, 'elbows_group_2', 2 * elbow_loss_1);
    DP = DP + losses.elbows_group_2;
    
    
    %Loss 7: 3-way reunion 
    % losses = add_loss(losses, 'reunion_3way', singular_loss(0.7, v, rho));
    % DP = DP + losses.reunion_3way;
    
    % Loss 8: venturi tube + sensors
    loss3 = sudden_expansion(D, 0.1, rho, v);
    loss4 = contraction_angle(0.1, 0.05, rho, v_100mm, 20);
    loss5 = expansion_angle(0.05, 0.1, rho, v_50mm, 20);
    loss6 = sudden_contraction(0.1, D, rho, v_100mm);
    losses = add_loss(losses, 'venturi_sensors', loss3 + loss4 + loss5 + loss6);
    DP = DP + losses.venturi_sensors;
    
    % Loss 8: venturi tube + sensors
    loss3 = sudden_expansion(D, 0.1, rho, v);
    loss4 = contraction_angle(0.1, 0.075, rho, v_100mm, 20);
    loss5 = expansion_angle(0.075, 0.1, rho, v_75mm, 20);
    loss6 = sudden_contraction(0.1, D, rho, v_100mm);
    losses = add_loss(losses, 'venturi_sensors2', loss3 + loss4 + loss5 + loss6);
    DP = DP + losses.venturi_sensors2;
    
    %Loss 9: Sensor suite
    % loss1 = sudden_contraction(D, 0.091, rho, v);
    % loss2 = expansion_angle(0.091, 0.100, rho, v, 20);
    % loss3 = singular_loss(0.5, v_100mm, rho);
    % loss4 = sudden_contraction(0.095, 0.091, rho, v);
    % loss5 = sudden_contraction(0.1, D, rho, v_100mm);
    % losst=loss1+loss2+loss3+loss4+loss5;
    % losses = add_loss(losses, 'sensor_suite_2', losst);
    % DP=DP+losses.sensor_suite_2;

    
    % Loss 10: Additional Elbows
    % losses = add_loss(losses, 'elbows_section_change', 2 * singular_loss(0.7, v, rho));
    % DP = DP + losses.elbows_section_change;

    % Loss 11: change of section + spiral
    % loss1 = sudden_contraction(D, 0.075, rho, v);
    % loss2 = singular_loss(4, v, rho);
    % loss3 = sudden_expansion(0.075, D, rho, v_75mm);
    % loss4 = straight_line_loss(rho, mu, v_75mm, 0.075, 0.4);
    % losst=loss1+loss2+loss3+loss4;
    % losses = add_loss(losses, 'section_changes_and_spiral', losst);
    % DP = DP + losses.section_changes_and_spiral;

    % Loss 12: HEPA filter + expansion

    losses = add_loss(losses, 'hepa_filter', hepa_filter(Q));
    losses = add_loss(losses, 'hepa_expansion', sudden_expansion(D, 0.100, rho, v));
    losses = add_loss(losses, 'hepa_expansion2', sudden_expansion(0.100, 0.120, rho, v_100mm));
    DP = DP + losses.hepa_filter + losses.hepa_expansion+ losses.hepa_expansion2;

    % Loss 13: Long straight tube (2m)
    losses = add_loss(losses, 'straight_pipe', straight_line_loss(rho, mu, v, D, 1));
    DP = DP + losses.straight_pipe;
    

    
    % Loss 13: Honeycomb K=0.4
    losses = add_loss(losses, 'honeycomb', singular_loss(0.4, v, rho)+sudden_expansion(D-0.04, D, rho, v)+sudden_contraction(D, D-0.04, rho, v));
    DP = DP + losses.honeycomb;
    
    % Loss 14 : Open tube K=1 add it if the hepa filter is not on !
    % losses = add_loss(losses, 'open_tube', singular_loss(1, v, rho));
    % DP = DP + losses.open_tube;
    % Spiral (if needed): 
    % losses = add_loss(losses, 'spiral', singular_loss(5, v, rho));
    % DP = DP + losses.spiral;
        
    % Verification & adjustment of v if necessary (iterative)
    if DP > P_max
        v = v * (0.95 + 0.049*n/max_iter);         
        Q = v * A;            
        P_max = P_available(Q);
    else
        v = v * ( 1.05 - 0.049*n/max_iter); 
        Q = v * A;
        P_max = P_available(Q);
    end

    n = n + 1;

    Q_history(end+1) = Q;
    DP_history(end+1) = DP;
    P_available_history(end+1) = P_max;

end

% Convergence check
if n >= max_iter
    warning('Convergence not reached after %d iterations', max_iter);
end

% Results display
fprintf('=== HYDRAULIC CALCULATION RESULTS ===\n');
fprintf('Convergence reached in %d iterations\n', n);
fprintf('Final flow rate: %.4f m³/s (%.0f CFM)\n', Q, Q*2118.88);
fprintf('Final velocity: %.2f m/s\n', v);
fprintf('Available pressure: %.1f Pa\n', P_max);
fprintf('Total losses: %.1f Pa\n', DP);
fprintf('Difference: %.2f Pa\n', abs(P_max - DP));

% Plot convergence history
figure;
subplot(2,1,1);
plot(1:length(Q_history), Q_history*2118.88, 'b-o');
xlabel('Iteration'); ylabel('Flow Rate [CFM]');
title('Flow Rate Convergence');
grid on;

subplot(2,1,2);
plot(1:length(DP_history), DP_history, 'r-o', 1:length(P_available_history), P_available_history, 'g-o');
xlabel('Iteration'); ylabel('Pressure [Pa]');
legend('Total Losses', 'Available Pressure');
title('Pressure Convergence');
grid on;

% Automatic loss analysis and plotting
plot_losses(losses);

%% FUNCTIONS

function losses = add_loss(losses, name, value)
    % Automatically adds a loss to the system
    losses.(name) = value;
end

function plot_losses(losses)
    % Automatic extraction and plotting of loss data
    field_names = fieldnames(losses);
    loss_values = [];
    loss_labels = {};
    
    for i = 1:length(field_names)
        loss_values(i) = losses.(field_names{i});
        % Automatic name formatting (replace _ with space, capitalize)
        label = strrep(field_names{i}, '_', ' ');
        label = regexprep(label, '\<\w', '${upper($0)}'); % First letter uppercase
        loss_labels{i} = label;
    end
    
    % Create plots
    figure;
    subplot(1,2,1);
    pie(loss_values, loss_labels);
    title('Pressure Loss Distribution');
    
    subplot(1,2,2);
    bar(loss_values);
    set(gca, 'XTickLabel', loss_labels);
    xtickangle(45);
    ylabel('Pressure Loss [Pa]');
    title('Individual Pressure Losses');
    grid on;
    
    % Automatic value display
    fprintf('\n=== PRESSURE LOSS BREAKDOWN ===\n');
    total_loss = sum(loss_values);
    for i = 1:length(loss_values)
        percentage = (loss_values(i)/total_loss)*100;
        fprintf('%-25s: %6.1f Pa (%4.1f%%)\n', loss_labels{i}, loss_values(i), percentage);
    end
    fprintf('%-25s: %6.1f Pa\n', 'TOTAL', total_loss);
end

function dP = hepa_filter(Q)
    
    filter_area = 0.095;  % effective area [m²]
    v_face = Q / filter_area;  % air speed at the filter [m/s]
   
    a = 300;    % [Pa·s/m]
    b = 600;    % [Pa·s²/m²]
    
    dP = a * v_face + b * v_face^2;
end

function DP = straight_line_loss(rho, mu, v, D, L)
    Re = rho * v * D / mu;
    if Re < 1200
        f = 64/Re;  % laminar flow
    else
        f = 0.3164/(Re^0.25);  % turbulent flow (Blasius)
    end
    DP = f * (L / D) * 0.5 * rho * v^2;
end

function dP = singular_loss(K, V, rho)
    % K = loss coefficient (dimensionless)
    % V = air velocity (m/s)
    % rho = air density (kg/m³)
    dP = K * 0.5 * rho * V^2;
end

function dP = filter_loss(A, B, Q)
    % A = linear coefficient [Pa·s/m³]
    % B = quadratic coefficient [Pa·s²/m⁶]
    % Q = volumetric flow rate [m³/s]
    dP = A * Q + B * Q^2;
end

function dP = contraction_angle(D1, D2, rho, V, alpha)
    % D1 = upstream diameter [m] (larger)
    % D2 = downstream diameter [m] (smaller)
    % rho = fluid density [kg/m³]
    % V = velocity in upstream section [m/s]
    % alpha = contraction angle [deg]
    K = 0.8 * sind(alpha/2) * (1 - (D2/D1)^2);
    dP = K * 0.5 * rho * V^2;
end

function dP = expansion_angle(D1, D2, rho, V, alpha)
    % D1 = upstream diameter [m] (smaller)
    % D2 = downstream diameter [m] (larger)
    % rho = fluid density [kg/m³]
    % V = velocity in upstream section [m/s]
    % alpha = expansion angle [deg]
    K = 2.6 * sind(alpha/2) * (1 - (D1/D2)^2)^2;
    dP = K * 0.5 * rho * V^2;
end

function dP = sudden_contraction(D1, D2, rho, V1)
    % D1 = upstream diameter [m] (larger)
    % D2 = downstream diameter [m] (smaller)  
    % rho = fluid density [kg/m³]
    % V1 = velocity in upstream section [m/s]
    
    % Verification
    if D2 >= D1
        error('For contraction: D2 must be < D1');
    end
    
    % Area ratio
    beta = (D2/D1)^2;
    
    % Loss coefficient (empirical formula)
    if beta >= 0.8
        K = 0.1 * (1 - beta);
    elseif beta >= 0.6
        K = 0.18;
    elseif beta >= 0.4
        K = 0.37;
    elseif beta >= 0.2
        K = 0.50;
    else
        K = 0.62;
    end
    
    % Loss calculation (based on downstream velocity V2)
    V2 = V1 / beta;  % flow conservation
    dP = K * 0.5 * rho * V2^2;
end

function dP = sudden_expansion(D1, D2, rho, V1)
    % D1 = upstream diameter [m] (smaller)
    % D2 = downstream diameter [m] (larger)
    % rho = fluid density [kg/m³]
    % V1 = velocity in upstream section [m/s]
    
    % Verification
    if D2 <= D1
        error('For expansion: D2 must be > D1');
    end
    
    % Borda-Carnot formula
    beta = (D1/D2)^2;  % area ratio A1/A2
    K = (1 - beta)^2;
    
    % Loss based on upstream velocity V1
    dP = K * 0.5 * rho * V1^2;
end
