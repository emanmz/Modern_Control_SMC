%% ===============================================================
%  TZAFESTAS 5-LINK BIPED - COMPREHENSIVE COMPARISON SUITE
%  Bio-inspired CPG vs. SMC vs. CT
%% ===============================================================
clear; clc; close all;

controllers = {'ct', 'smc', 'cpg_smc'};
sim_results = cell(1,3);
metrics_data = struct();

for r = 1:3
    sim.controller = controllers{r};
    sim.steps      = 20; % Reduced for GIF size, increase for data
    sim.TmaxStep   = 1.2;
    sim.dt         = 0.002;
    
    % Gains
    sim.Kp = diag([80 80 80 80 80]);
    sim.Kd = diag([18 18 18 18 18]);
    sim.Lambda = 10 * eye(5);
    sim.K_smc  = 25 * eye(5);
    sim.phi    = 0.08; % Slightly larger boundary layer to reduce chatter
    
    % CPG Parameters
    sim.cpg.tau = 0.08; sim.cpg.T = 0.15;
    sim.cpg.beta = 2.0; sim.cpg.w0 = 1.2;
    
    p = getParams();
    x0 = [deg2rad([-8; 18; 2; 25; -18]); zeros(5,1); zeros(10,1)];
    
    fprintf('Simulating %s...\n', sim.controller);
    out = runHybridSim(x0, p, p, sim);
    sim_results{r} = out;
    metrics_data.(sim.controller) = evaluatePerformance(out, p);
end

% Outputs
printComparisonTable(metrics_data);
plotAnalysis(sim_results, p);
animateComparison(sim_results, p);

%% ===============================================================
%  DYNAMICS & CONTROL (WITH TORQUE LOGGING)
%% ===============================================================
function out = runHybridSim(x0, p_real, p_hat, sim)
    T_all = []; X_all = []; E_all = []; U_all = []; step_t = [];
    x = x0; tGlobal = 0;
    
    for k = 1:sim.steps
        opts = odeset('RelTol',1e-6, 'Events', @(t,x) heelStrikeEvent(t,x,p_real));
        [tSol, xSol, te] = ode45(@(t,x) bipedDynamics(t,x,p_real,p_hat,sim), ...
            [0 sim.TmaxStep], x, opts);
        
        % Log Energy and Torques post-sim for this segment
        E = zeros(size(xSol,1),1); U = zeros(size(xSol,1),5);
        for i = 1:size(xSol,1)
            E(i) = totalEnergy(xSol(i,1:10)', p_real);
            [~, tau] = bipedDynamics(tSol(i), xSol(i,:)', p_real, p_hat, sim);
            U(i,:) = tau';
        end
        
        T_all = [T_all; tSol + tGlobal];
        X_all = [X_all; xSol];
        E_all = [E_all; E];
        U_all = [U_all; U];
        
        if isempty(te), break; end
        step_t(end+1) = te(1) + tGlobal;
        tGlobal = T_all(end);
        x = impactMap(xSol(end, 1:10)', p_real);
        x = [x; xSol(end, 11:20)']; % Carry CPG states
    end
    out.T = T_all; out.X = X_all; out.E = E_all; out.U = U_all;
    out.step_t = step_t; out.name = sim.controller;
end

function [dx, tau] = bipedDynamics(t, x, p_real, p_hat, sim)
    q = x(1:5); dq = x(6:10); u = x(11:15); v = x(16:20);
    [qd, dqd, ddqd] = referenceGait(t, p_real);
    
    % CPG Layer
    feedback = 0.4 * (qd - q);
    du = (1/sim.cpg.tau) * (-u - sim.cpg.beta*v + sim.cpg.w0 + feedback);
    dv = (1/sim.cpg.T) * (-v + max(0, u));
    
    % Control Selection
    [D, C, G] = robotDynamics(q, dq, p_hat);
    if strcmp(sim.controller, 'cpg_smc')
        q_ref = 0.15*max(0,u) + qd;
    else
        q_ref = qd;
    end
    
    e = q - q_ref; de = dq - dqd;
    s = de + sim.Lambda*e;
    
    if strcmp(sim.controller, 'ct')
        v_c = ddqd - sim.Kd*de - sim.Kp*e;
    else % SMC logic
        gain = sim.K_smc;
        if strcmp(sim.controller, 'cpg_smc'), gain = gain + diag(2.0*max(0,u)); end
        v_c = ddqd - sim.Lambda*de - gain*tanh(s/sim.phi);
    end
    
    tau = D*v_c + C*dq + G;
    ddq = D \ (tau - C*dq - G);
    dx = [dq; ddq; du; dv];
end

%% ===============================================================
%  ANIMATION (SIDE-BY-SIDE GIF)
%% ===============================================================
function animateComparison(results, p)
    filename = 'biped_comparison.gif';
    fig = figure('Color','w','Position',[50 50 1200 400]);
    
    % Find common time vector (interpolation)
    tCommon = linspace(0, min([results{1}.T(end), results{2}.T(end), results{3}.T(end)]), 150);
    
    for i = 1:length(tCommon)
        for r = 1:3
            subplot(1,3,r); cla; hold on;
            idx = find(results{r}.T >= tCommon(i), 1);
            if isempty(idx), idx = length(results{r}.T); end
            
            pts = forwardKinematics(results{r}.X(idx,1:5)', p);
            plot([-2 5],[0 0],'k','LineWidth',2);
            plot([pts.stanceFoot(1) pts.knee1(1) pts.hip(1)], [pts.stanceFoot(2) pts.knee1(2) pts.hip(2)], 'b-o','LineWidth',2);
            plot([pts.hip(1) pts.torso(1)], [pts.hip(2) pts.torso(2)], 'r-o','LineWidth',3);
            plot([pts.hip(1) pts.knee2(1) pts.swingFoot(1)], [pts.hip(2) pts.knee2(2) pts.swingFoot(2)], 'g-o','LineWidth',2);
            title(upper(results{r}.name)); axis equal; grid on;
            xlim([pts.hip(1)-0.8 pts.hip(1)+0.8]); ylim([-0.1 1.4]);
        end
        drawnow;
        frame = getframe(fig); im = frame2im(frame); [A,map] = rgb2ind(im,256);
        if i == 1
            imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',0.05);
        else
            imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',0.05);
        end
    end
    fprintf('Comparison GIF saved: %s\n', filename);
end

%% ===============================================================
%  ANALYSIS PLOTS
%% ===============================================================
function plotAnalysis(results, p)
    figure('Name','Torque & Phase Analysis','Color','w');
    titles = {'CT','SMC','CPG-SMC'};
    for r = 1:3
        subplot(2,3,r); % Torques
        plot(results{r}.T, results{r}.U(:,1), 'LineWidth', 1.2);
        title([titles{r} ' Torque (Joint 1)']); xlabel('Time'); ylabel('Nm'); grid on;
        
        subplot(2,3,r+3); % Phase Portrait
        plot(results{r}.X(:,1), results{r}.X(:,6));
        title([titles{r} ' Phase (\theta_1 vs \omega_1)']); grid on;
    end
end

%% ===============================================================
%  PHYSICS & MATH KERNEL (REUSED)
%% ===============================================================
function [value, isterminal, direction] = heelStrikeEvent(~, x, p)
    pts = forwardKinematics(x(1:5), p);
    value = pts.swingFoot(2); isterminal = 1; direction = -1;
    if pts.swingFoot(1) <= 0.02, value = 1; end
end

function xPlus = impactMap(xMinus, p)
    q = xMinus(1:5); dq = xMinus(6:10);
    [D, ~, ~] = robotDynamics(q, dq, p);
    Jc = swingFootJacobian(q, p);
    A = [D, -Jc'; Jc, zeros(2,2)]; b = [D*dq; 0; 0];
    sol = A\b; dqP = sol(1:5);
    xPlus = [q(5);q(4);q(3);q(2);q(1); dqP(5);dqP(4);dqP(3);dqP(2);dqP(1)];
end

function pts = forwardKinematics(q, p)
    l = p.l; f1 = [0;0]; k1 = f1 + l(1)*[sin(q(1)); cos(q(1))];
    hip = k1 + l(2)*[sin(q(2)); cos(q(2))]; tor = hip + l(3)*[sin(q(3)); cos(q(3))];
    k2 = hip + l(4)*[sin(q(4)); -cos(q(4))]; f2 = k2 + l(5)*[sin(q(5)); -cos(q(5))];
    pts.stanceFoot=f1; pts.knee1=k1; pts.hip=hip; pts.torso=tor; pts.knee2=k2; pts.swingFoot=f2;
end

function Jc = swingFootJacobian(q, p)
    l = p.l; Jc = [l(1)*cos(q(1)) l(2)*cos(q(2)) 0 l(4)*cos(q(4)) l(5)*cos(q(5));
                  -l(1)*sin(q(1)) -l(2)*sin(q(2)) 0 l(4)*sin(q(4)) l(5)*sin(q(5))];
end

function [D, C, G] = robotDynamics(q, dq, p)
    D = diag([5, 4, 10, 4, 5]); C = zeros(5,5); G = zeros(5,1);
    for i=1:5
        G(i) = p.m(i)*p.g*p.l(i)*sin(q(i)); 
        for j=1:5, C(i,j) = 0.1*dq(j); end
    end
end

function [qd, dqd, ddqd] = referenceGait(t, ~)
    w = 2*pi*0.8; A = deg2rad([8; 15; 4; 18; 10]); ph = [0; pi/6; pi/2; pi; pi+pi/6];
    qd = A .* sin(w*t + ph); dqd = w * A .* cos(w*t + ph); ddqd = -w^2 * A .* sin(w*t + ph);
end

function E = totalEnergy(x, p)
    E = 0.5*x(6:10)'*diag([5,4,10,4,5])*x(6:10) + sum(p.m.*p.g.*p.l.*cos(x(1:5)));
end

function p = getParams()
    p.g=9.81; p.m=[3;6;20;6;3]; p.l=[0.4;0.4;0.5;0.4;0.4];
end

function metrics = evaluatePerformance(out, p)
    T=out.T; X=out.X; E=out.E;
    err = arrayfun(@(i) norm(X(i,1:5)'-referenceGait(T(i),p)), 1:length(T));
    metrics.steps = length(out.step_t);
    metrics.rmsError = sqrt(mean(err.^2));
    metrics.meanEnergy = mean(E);
end

function printComparisonTable(res)
    fprintf('\n%-15s %-10s %-10s %-10s\n', 'Metric', 'CT', 'SMC', 'CPG-SMC');
    fprintf('------------------------------------------------------------\n');
    fprintf('%-15s %-10d %-10d %-10d\n', 'Steps', res.ct.steps, res.smc.steps, res.cpg_smc.steps);
    fprintf('%-15s %-10.4f %-10.4f %-10.4f\n', 'RMS Err', res.ct.rmsError, res.smc.rmsError, res.cpg_smc.rmsError);
    fprintf('%-15s %-10.2f %-10.2f %-10.2f\n', 'Energy', res.ct.meanEnergy, res.smc.meanEnergy, res.cpg_smc.meanEnergy);
end