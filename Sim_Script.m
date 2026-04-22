clc; clear; close all;

% Time span
tspan = 0:0.05:10; % Fixed step for smoother animation
x0 = [0.1; -0.1; 0; 0];

% Run simulations
[t1, x1] = ode45(@(t,x) robot_dynamics(t,x,'Classic_SMC'), tspan, x0);
[t2, x2] = ode45(@(t,x) robot_dynamics(t,x,'Smooth_SMC'), tspan, x0);

%% ===== GIF Generation & Animation =====
filename = 'SMC_Comparison.gif';
h = figure('Color', 'w', 'Position', [100, 100, 800, 400]);

for k = 1:length(tspan)
    % Left Subplot: Tracking Visualization
    subplot(1,2,1);
    % Plot desired vs actual (Joint 1)
    plot(tspan(1:k), sin(tspan(1:k)), 'k:', 'LineWidth', 1); hold on;
    plot(tspan(1:k), x1(1:k,1), 'r-', 'LineWidth', 1.5);
    plot(tspan(1:k), x2(1:k,1), 'b--', 'LineWidth', 1.5);
    hold off;
    title('Joint 1 Trajectory');
    xlabel('Time (s)'); ylabel('Angle (rad)');
    legend('Desired', 'Classic', 'Smooth', 'Location', 'southoutside', 'Orientation', 'horizontal');
    grid on; axis([0 10 -1.5 1.5]);

    % Right Subplot: Phase Portrait (Error vs Change in Error)
    subplot(1,2,2);
    e1 = x1(1:k,1) - sin(tspan(1:k))';
    de1 = x1(1:k,3) - cos(tspan(1:k))';
    e2 = x2(1:k,1) - sin(tspan(1:k))';
    de2 = x2(1:k,3) - cos(tspan(1:k))';
    
    plot(e1, de1, 'r-'); hold on;
    plot(e2, de2, 'b--');
    hold off;
    title('Phase Portrait (e vs \dot{e})');
    xlabel('Error'); ylabel('d/dt Error');
    grid on; axis([-0.2 0.2 -1 1]);
    
    drawnow;
    
    % Capture the plot as a frame
    frame = getframe(h);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    
    % Write to the GIF File
    if k == 1
        imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.05);
    else
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
    end
end

fprintf('GIF saved as %s\n', filename);

%% ===============================================================
% ADD THIS AFTER YOUR EXISTING SIMULATION CODE
% Generates:
%   1) Real robot movement GIF (2-link arm motion)
%   2) Side-by-side comparison:
%         Computed Torque vs Classic SMC
%
% Assumes robot_dynamics() already exists
%% ===============================================================

clc; clear; close all;

%% ---------------- SIM SETTINGS ----------------
tspan = 0:0.03:10;
x0 = [0.1; -0.1; 0; 0];

%% ---------------- RUN CONTROLLERS ----------------
[t_ct, x_ct] = ode45(@(t,x) robot_dynamics_compare(t,x,'Torque'), tspan, x0);
[t_smc, x_smc] = ode45(@(t,x) robot_dynamics_compare(t,x,'SMC'), tspan, x0);

%% ===============================================================
% GIF #1 : REAL ROBOT MOVEMENT (SMC ONLY)
%% ===============================================================
gif1 = 'robot_motion.gif';

L1 = 0.5;
L2 = 0.5;

fig1 = figure('Color','w','Position',[100 100 600 600]);

for k = 1:length(tspan)

    q1 = x_smc(k,1);
    q2 = x_smc(k,2);

    % Forward kinematics
    p0 = [0 0];
    p1 = [L1*cos(q1), L1*sin(q1)];
    p2 = [p1(1)+L2*cos(q1+q2), p1(2)+L2*sin(q1+q2)];

    clf
    hold on

    plot([p0(1) p1(1)], [p0(2) p1(2)], 'r-', 'LineWidth',6);
    plot([p1(1) p2(1)], [p1(2) p2(2)], 'b-', 'LineWidth',6);

    plot(p0(1),p0(2),'ko','MarkerSize',10,'MarkerFaceColor','k');
    plot(p1(1),p1(2),'ko','MarkerSize',10,'MarkerFaceColor','k');
    plot(p2(1),p2(2),'go','MarkerSize',12,'MarkerFaceColor','g');

    title(sprintf('Real Robot Motion (SMC)   t = %.2f s', tspan(k)))
    axis equal
    axis([-1.1 1.1 -1.1 1.1])
    grid on

    drawnow

    frame = getframe(fig1);
    im = frame2im(frame);
    [A,map] = rgb2ind(im,256);

    if k==1
        imwrite(A,map,gif1,'gif','LoopCount',inf,'DelayTime',0.03);
    else
        imwrite(A,map,gif1,'gif','WriteMode','append','DelayTime',0.03);
    end
end

fprintf('Saved: %s\n',gif1);

%% ===============================================================
% GIF #2 : COMPARISON
% Left = Computed Torque
% Right = Sliding Mode Control
%% ===============================================================
gif2 = 'torque_vs_smc.gif';

fig2 = figure('Color','w','Position',[100 100 1000 500]);

for k = 1:length(tspan)

    clf

    %% -------- LEFT : TORQUE CONTROL --------
    subplot(1,2,1)

    q1 = x_ct(k,1);
    q2 = x_ct(k,2);

    p0 = [0 0];
    p1 = [L1*cos(q1), L1*sin(q1)];
    p2 = [p1(1)+L2*cos(q1+q2), p1(2)+L2*sin(q1+q2)];

    hold on
    plot([p0(1) p1(1)], [p0(2) p1(2)], 'r-', 'LineWidth',6);
    plot([p1(1) p2(1)], [p1(2) p2(2)], 'b-', 'LineWidth',6);
    plot(p2(1),p2(2),'go','MarkerSize',10,'MarkerFaceColor','g');

    title('Computed Torque')
    axis equal
    axis([-1.1 1.1 -1.1 1.1])
    grid on

    %% -------- RIGHT : SMC --------
    subplot(1,2,2)

    q1 = x_smc(k,1);
    q2 = x_smc(k,2);

    p1 = [L1*cos(q1), L1*sin(q1)];
    p2 = [p1(1)+L2*cos(q1+q2), p1(2)+L2*sin(q1+q2)];

    hold on
    plot([0 p1(1)], [0 p1(2)], 'r-', 'LineWidth',6);
    plot([p1(1) p2(1)], [p1(2) p2(2)], 'b-', 'LineWidth',6);
    plot(p2(1),p2(2),'go','MarkerSize',10,'MarkerFaceColor','g');

    title('Sliding Mode Control')
    axis equal
    axis([-1.1 1.1 -1.1 1.1])
    grid on

    sgtitle(sprintf('Robot Comparison   t = %.2f s',tspan(k)))

    drawnow

    frame = getframe(fig2);
    im = frame2im(frame);
    [A,map] = rgb2ind(im,256);

    if k==1
        imwrite(A,map,gif2,'gif','LoopCount',inf,'DelayTime',0.03);
    else
        imwrite(A,map,gif2,'gif','WriteMode','append','DelayTime',0.03);
    end
end

fprintf('Saved: %s\n',gif2);

%% ===============================================================
% DYNAMICS FOR COMPARISON
%% ===============================================================
function dx = robot_dynamics_compare(t,x,type)

q = x(1:2);
dq = x(3:4);

m1=1; m2=0.8;
l1=0.5; l2=0.5;
g=9.81;

M = [(m1+m2)*l1^2 + m2*l2^2 + 2*m2*l1*l2*cos(q(2)), ...
     m2*l2^2 + m2*l1*l2*cos(q(2));
     m2*l2^2 + m2*l1*l2*cos(q(2)), ...
     m2*l2^2];

C = [-m2*l1*l2*sin(q(2))*dq(2), ...
     -m2*l1*l2*sin(q(2))*(dq(1)+dq(2));
      m2*l1*l2*sin(q(2))*dq(1), 0];

G = [(m1+m2)*g*l1*cos(q(1)) + m2*g*l2*cos(q(1)+q(2));
     m2*g*l2*cos(q(1)+q(2))];

qd   = [sin(t); cos(t)];
dqd  = [cos(t); -sin(t)];
ddqd = [-sin(t); -cos(t)];

e  = q - qd;
de = dq - dqd;

lambda = 5;

if strcmp(type,'Torque')

    Kp = 35;
    Kd = 12;

    tau = M*(ddqd - Kd*de - Kp*e) + C*dq + G;

else

    s = de + lambda*e;
    K = 15;

    tau = M*(ddqd - lambda*de) + C*dq + G - K*sign(s);

end

dist = [2*sin(5*t); 2*cos(5*t)];

ddq = M\(tau + dist - C*dq - G);

dx = [dq; ddq];

end