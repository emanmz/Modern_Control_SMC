clc; clear; close all;

% Time span
tspan = [0 10];
x0 = [0.1; -0.1; 0; 0];

% Run simulations
[t1, x1] = ode45(@(t,x) robot_dynamics(t,x,'Classic_SMC'), tspan, x0);
[t2, x2] = ode45(@(t,x) robot_dynamics(t,x,'Smooth_SMC'), tspan, x0);

% --- Recompute control inputs for plotting ---
tau1 = zeros(length(t1),2);
tau2 = zeros(length(t2),2);

for i = 1:length(t1)
    [~, tau_tmp] = robot_dynamics(t1(i), x1(i,:)', 'Classic_SMC');
    tau1(i,:) = tau_tmp';
end

for i = 1:length(t2)
    [~, tau_tmp] = robot_dynamics(t2(i), x2(i,:)', 'Smooth_SMC');
    tau2(i,:) = tau_tmp';
end

%% ===== Publication-Quality Plotting =====
clc;

% Global figure settings (applies to all plots)
set(groot, 'defaultAxesFontName', 'Times New Roman');
set(groot, 'defaultTextFontName', 'Times New Roman');
set(groot, 'defaultAxesFontSize', 12);
set(groot, 'defaultTextFontSize', 12);
set(groot, 'defaultLineLineWidth', 1.5);
set(groot, 'defaultAxesLineWidth', 1);
set(groot, 'defaultFigureColor', 'w');

% Create figure
figure('Units','inches','Position',[1 1 6.5 5]); % journal column width

tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

%% ===== Tracking Error =====
nexttile;
plot(t1, x1(:,1) - sin(t1), 'r-', 'LineWidth', 1.6); hold on;
plot(t2, x2(:,1) - sin(t2), 'b--', 'LineWidth', 1.6);

ylabel('Error (rad)');
title('Tracking Error for Joint 1');

legend({'Classic SMC','Smooth SMC'}, ...
    'Location','northeast', ...
    'Box','off');

grid on;
box on;

%% ===== Control Input =====
nexttile;
plot(t1, tau1(:,1), 'r-', 'LineWidth', 1.2); hold on;
plot(t2, tau2(:,1), 'b--', 'LineWidth', 1.6);

xlabel('Time (s)');
ylabel('Torque (Nm)');
title('Control Input \tau_1');

legend({'Classic SMC','Smooth SMC'}, ...
    'Location','northeast', ...
    'Box','off');

grid on;
box on;

%% ===== Export (VERY IMPORTANT) =====
exportgraphics(gcf, 'smc_comparison.pdf', 'ContentType','vector');
exportgraphics(gcf, 'smc_comparison.png', 'Resolution', 600);