%% ===============================================================
%  TZAFESTAS 1996 FIVE-LINK BIPED – FULL REPORT VERSION
%  Sections VI (A/B/C) and VII (A/B/C) — LaTeX-ready plots
%% ===============================================================
clear; clc; close all;

%% -------- CONFIGURATION --------
controllers = {'ct', 'smc'};
unc_levels  = [0.0, 0.20, 0.60];
unc_labels  = {'0\%', '20\%', '60\%'};
unc_tags    = {'0', '20', '60'};

%% -------- COLLECT ALL RESULTS --------
allResults = struct();

for ui = 1:length(unc_levels)
    unc = unc_levels(ui);
    for ci = 1:length(controllers)
        ctrl = controllers{ci};

        sim.controller = ctrl;
        sim.steps      = 5;
        sim.TmaxStep   = 1.5;
        sim.dt         = 0.002;
        sim.Kp         = diag([80 80 80 80 80]);
        sim.Kd         = diag([18 18 18 18 18]);
        sim.Lambda     = 10 * eye(5);
        sim.K_smc      = 25 * eye(5);
        sim.phi        = 0.05;

        p_real = getParams();
        p_hat  = getParams();
        p_hat.m = p_real.m .* (1 + unc*[-1;1;-1;1;-1]);
        p_hat.I = p_real.I .* (1 + unc*[1;-1;1;-1;1]);
        p_hat.l = p_real.l .* (1 + 0.15*unc*[1;1;-1;-1;1]);

        q0  = deg2rad([-8; 18; 2; 25; -18]);
        dq0 = deg2rad([ 5; -8;  0; -10;  6]);
        x0  = [q0; dq0];

        fprintf('Running %s | unc=%d%%...\n', upper(ctrl), round(unc*100));
        out     = runHybridSim(x0, p_real, p_hat, sim);
        metrics = evaluatePerformance(out, p_real);

        tag = sprintf('unc%s_%s', unc_tags{ui}, ctrl);
        allResults.(tag).out     = out;
        allResults.(tag).metrics = metrics;
        allResults.(tag).p_real  = p_real;
    end
end

fprintf('\nAll simulations complete. Generating plots...\n');

%% ================================================================
%  GLOBAL STYLE
%% ================================================================
col_ct  = [0.00 0.45 0.70];   % blue  — Computed Torque
col_smc = [0.84 0.37 0.00];   % orange — Sliding Mode

set(groot, 'defaultAxesFontSize',   9);
set(groot, 'defaultTextInterpreter','tex');
set(groot, 'defaultLegendInterpreter','tex');
set(groot, 'defaultAxesTickLabelInterpreter','tex');

%% ================================================================
%  SECTION VI — PER-UNCERTAINTY PLOTS (A = 0%, B = 20%, C = 60%)
%  For each level: (1) tracking, (2) error, (3) control input
%% ================================================================

sec_labels = {'A','B','C'};

for ui = 1:3

    tag_ct  = sprintf('unc%s_ct',  unc_tags{ui});
    tag_smc = sprintf('unc%s_smc', unc_tags{ui});
    out_ct  = allResults.(tag_ct).out;
    out_smc = allResults.(tag_smc).out;
    p       = allResults.(tag_ct).p_real;
    T       = out_ct.T;

    % Pre-compute reference and errors for every joint
    N = length(T);
    qd_mat    = zeros(N,5);
    err_ct_m  = zeros(N,5);
    err_smc_m = zeros(N,5);

    for k = 1:N
        [qd_k,~,~]    = referenceGait(T(k), p);
        qd_mat(k,:)   = qd_k';
        err_ct_m(k,:) = out_ct.X(k,1:5)  - qd_k';
        err_smc_m(k,:)= out_smc.X(k,1:5) - qd_k';
    end

    % ---- (1) TRACKING PERFORMANCE --------------------------------
    fig = figure('Units','inches','Position',[0.5 0.5 6.5 9]);
    sgtitle(sprintf('VI.%s — Joint Tracking: %s Uncertainty', ...
        sec_labels{ui}, strrep(unc_labels{ui},'\','')),...
        'FontSize',11,'FontWeight','bold');

    for j = 1:5
        ax = subplot(5,1,j); hold on; box on;

        h1 = plot(T, rad2deg(out_ct.X(:,j)),  '--', 'Color', col_ct,  'LineWidth', 1.5);
        h2 = plot(T, rad2deg(out_smc.X(:,j)), ':',  'Color', col_smc, 'LineWidth', 1.5);
        h3 = plot(T, rad2deg(qd_mat(:,j)),    '-k', 'LineWidth', 1.0);

        % Mark heel-strike events
        for st = out_ct.step_t
            xline(st, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 0.7, ...
                'HandleVisibility','off');
        end

        ylabel(sprintf('q_{%d} (deg)', j));
        xlim([T(1) T(end)]);
        grid on; ax.GridAlpha = 0.25;

        if j == 1
            legend([h1 h2 h3], 'CT','SMC','Reference', ...
                'FontSize',8, 'Location','northeast', 'NumColumns',3);
        end
    end
    xlabel('Time (s)');

    exportgraphics(fig, sprintf('fig_VI%s_tracking.pdf', sec_labels{ui}), ...
        'ContentType','vector');
    exportgraphics(fig, sprintf('fig_VI%s_tracking.png', sec_labels{ui}), ...
        'Resolution',200);
    fprintf('Saved fig_VI%s_tracking\n', sec_labels{ui});

    % ---- (2) TRACKING ERROR --------------------------------------
    fig = figure('Units','inches','Position',[0.5 0.5 6.5 9]);
    sgtitle(sprintf('VI.%s — Tracking Error: %s Uncertainty', ...
        sec_labels{ui}, strrep(unc_labels{ui},'\','')),...
        'FontSize',11,'FontWeight','bold');

    for j = 1:5
        ax = subplot(5,1,j); hold on; box on;

        h1 = plot(T, rad2deg(err_ct_m(:,j)),  'Color', col_ct,  'LineWidth', 1.5);
        h2 = plot(T, rad2deg(err_smc_m(:,j)), 'Color', col_smc, 'LineWidth', 1.5, ...
            'LineStyle',':');
        yline(0, 'k--', 'LineWidth', 0.7, 'HandleVisibility','off');

        for st = out_ct.step_t
            xline(st, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 0.7, ...
                'HandleVisibility','off');
        end

        ylabel(sprintf('e_{%d} (deg)', j));
        xlim([T(1) T(end)]);
        grid on; ax.GridAlpha = 0.25;

        if j == 1
            legend([h1 h2],'CT','SMC','FontSize',8,'Location','northeast');
        end
    end
    xlabel('Time (s)');

    exportgraphics(fig, sprintf('fig_VI%s_error.pdf', sec_labels{ui}), ...
        'ContentType','vector');
    exportgraphics(fig, sprintf('fig_VI%s_error.png', sec_labels{ui}), ...
        'Resolution',200);
    fprintf('Saved fig_VI%s_error\n', sec_labels{ui});

    % ---- (3) CONTROL INPUT ---------------------------------------
    fig = figure('Units','inches','Position',[0.5 0.5 6.5 3.2]);
    sgtitle(sprintf('VI.%s — Control Input Norm: %s Uncertainty', ...
        sec_labels{ui}, strrep(unc_labels{ui},'\','')),...
        'FontSize',11,'FontWeight','bold');

    hold on; box on;

    tau_ct_norm  = vecnorm(out_ct.tau,  2, 2);
    tau_smc_norm = vecnorm(out_smc.tau, 2, 2);

    h1 = plot(T, tau_ct_norm,  'Color', col_ct,  'LineWidth', 1.5);
    h2 = plot(T, tau_smc_norm, 'Color', col_smc, 'LineWidth', 1.5, 'LineStyle',':');

    for st = out_ct.step_t
        xline(st, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 0.7, ...
            'HandleVisibility','off');
    end

    xlabel('Time (s)');
    ylabel('||\tau|| (N{\cdot}m)');
    legend([h1 h2],'CT','SMC','FontSize',9,'Location','best');
    xlim([T(1) T(end)]);
    grid on;

    exportgraphics(fig, sprintf('fig_VI%s_control.pdf', sec_labels{ui}), ...
        'ContentType','vector');
    exportgraphics(fig, sprintf('fig_VI%s_control.png', sec_labels{ui}), ...
        'Resolution',200);
    fprintf('Saved fig_VI%s_control\n', sec_labels{ui});

end % ui loop

%% ================================================================
%  SECTION VII.A — CT vs SMC CONCEPTUAL OVERVIEW
%  Three-panel: s-surface, tanh saturation, phase portrait
%% ================================================================
fig = figure('Units','inches','Position',[0.5 0.5 6.5 4.5]);
sgtitle('VII.A — CT vs. SMC: Controller Concepts',...
    'FontSize',11,'FontWeight','bold');

% Panel 1: Sliding surface s = de + lambda*e
ax1 = subplot(1,3,1); hold on; box on;
e_vec  = linspace(-0.6, 0.6, 200);
de_vec = linspace(-0.6, 0.6, 200);
[EG,DEG] = meshgrid(e_vec, de_vec);
SG = DEG + 10*EG;
contourf(EG, DEG, SG, [-1.5 -0.5 0 0.5 1.5], 'LineWidth',0.5);
colormap(ax1, bluewhiteorange());
plot([-0.6 0.6],[-6 6],'k-','LineWidth',2);
xlabel('e (rad)'); ylabel('{\ite} (rad/s)');
title('SMC Sliding Surface');
text(0.05, 0.4, 's = 0','FontSize',8,'Rotation',75);
axis tight;

% Panel 2: tanh saturation vs sign (chattering)
ax2 = subplot(1,3,2); hold on; box on;
s_vec = linspace(-0.3, 0.3, 400);
phi   = 0.05;
plot(s_vec, sign(s_vec),              'k--', 'LineWidth',1.2,'DisplayName','sign(s)');
plot(s_vec, tanh(s_vec/phi),          '-',   'Color', col_smc, 'LineWidth',1.8,'DisplayName','tanh(s/\phi)');
plot(s_vec, zeros(size(s_vec))+0,     'k-',  'LineWidth',0.5,'HandleVisibility','off');
xline(0,'k-','LineWidth',0.5,'HandleVisibility','off');
xlabel('s'); ylabel('Switching term');
title('Chattering Reduction');
legend('FontSize',7,'Location','southeast');
ylim([-1.3 1.3]); xlim([-0.3 0.3]); grid on;

% Panel 3: CT vs SMC conceptual error decay
ax3 = subplot(1,3,3); hold on; box on;
t_c = linspace(0,5,300);
e_ct_ideal  = 0.5*exp(-2*t_c).*cos(3*t_c);
e_smc_ideal = 0.5*exp(-2.5*t_c).*max(tanh(3*t_c)-0.05*randn(size(t_c)),0).*sign(cos(3*t_c));
plot(t_c, rad2deg(e_ct_ideal),  '-', 'Color', col_ct,  'LineWidth',1.8,'DisplayName','CT (ideal)');
plot(t_c, rad2deg(e_smc_ideal), '-', 'Color', col_smc, 'LineWidth',1.8,'DisplayName','SMC');
yline(0,'k--','LineWidth',0.7,'HandleVisibility','off');
xlabel('Time (s)'); ylabel('Error (deg)');
title('Error Convergence (Conceptual)');
legend('FontSize',8,'Location','northeast');
grid on;

exportgraphics(fig, 'fig_VIIA_concepts.pdf', 'ContentType','vector');
exportgraphics(fig, 'fig_VIIA_concepts.png', 'Resolution',200);
fprintf('Saved fig_VIIA_concepts\n');

%% ================================================================
%  SECTION VII.B — COMPARATIVE TRACKING / ERROR ACROSS ALL LEVELS
%% ================================================================
fig = figure('Units','inches','Position',[0.5 0.5 6.5 7.5]);
sgtitle('VII.B — CT vs. SMC: Tracking Error Across Uncertainty Levels',...
    'FontSize',11,'FontWeight','bold');

for ui = 1:3
    tag_ct  = sprintf('unc%s_ct',  unc_tags{ui});
    tag_smc = sprintf('unc%s_smc', unc_tags{ui});

    err_ct  = trackingErrorNorm(allResults.(tag_ct).out,  allResults.(tag_ct).p_real);
    err_smc = trackingErrorNorm(allResults.(tag_smc).out, allResults.(tag_smc).p_real);
    T       = allResults.(tag_ct).out.T;

    ax = subplot(3,1,ui); hold on; box on;

    h1 = plot(T, rad2deg(err_ct),  '-',  'Color', col_ct,  'LineWidth', 1.5);
    h2 = plot(T, rad2deg(err_smc), '-.', 'Color', col_smc, 'LineWidth', 1.5);

    for st = allResults.(tag_ct).out.step_t
        xline(st, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 0.7, ...
            'HandleVisibility','off');
    end

    title(sprintf('Uncertainty = %s', strrep(unc_labels{ui},'\','')),...
        'FontSize',10);
    ylabel('||e|| (deg)');
    xlim([T(1) T(end)]);
    grid on; ax.GridAlpha = 0.25;

    if ui == 1
        legend([h1 h2],'CT','SMC','FontSize',8,'Location','northeast');
    end

    % Annotate RMS
    rms_ct_val  = rad2deg(allResults.(tag_ct).metrics.rmsError);
    rms_smc_val = rad2deg(allResults.(tag_smc).metrics.rmsError);
    text(0.02, 0.85, sprintf('RMS  CT=%.2f°  SMC=%.2f°', rms_ct_val, rms_smc_val), ...
        'Units','normalized','FontSize',7.5,'Color',[0.2 0.2 0.2]);
end
xlabel('Time (s)');

exportgraphics(fig, 'fig_VIIB_tracking.pdf', 'ContentType','vector');
exportgraphics(fig, 'fig_VIIB_tracking.png', 'Resolution',200);
fprintf('Saved fig_VIIB_tracking\n');

%% ================================================================
%  SECTION VII.C — CONTROL EFFORT + METRICS BAR CHART
%% ================================================================

% --- (1) Control effort time-series, 3 uncertainty levels --------
fig = figure('Units','inches','Position',[0.5 0.5 6.5 7.5]);
sgtitle('VII.C — Control Effort: CT vs. SMC',...
    'FontSize',11,'FontWeight','bold');

for ui = 1:3
    tag_ct  = sprintf('unc%s_ct',  unc_tags{ui});
    tag_smc = sprintf('unc%s_smc', unc_tags{ui});
    T       = allResults.(tag_ct).out.T;

    tau_ct_n  = vecnorm(allResults.(tag_ct).out.tau,  2, 2);
    tau_smc_n = vecnorm(allResults.(tag_smc).out.tau, 2, 2);

    ax = subplot(3,1,ui); hold on; box on;
    h1 = plot(T, tau_ct_n,  '-',  'Color', col_ct,  'LineWidth', 1.5);
    h2 = plot(T, tau_smc_n, '-.', 'Color', col_smc, 'LineWidth', 1.5);

    for st = allResults.(tag_ct).out.step_t
        xline(st, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 0.7, ...
            'HandleVisibility','off');
    end

    title(sprintf('Uncertainty = %s', strrep(unc_labels{ui},'\','')),...
        'FontSize',10);
    ylabel('||\tau|| (N{\cdot}m)');
    xlim([T(1) T(end)]);
    grid on; ax.GridAlpha = 0.25;

    if ui == 1
        legend([h1 h2],'CT','SMC','FontSize',8,'Location','northeast');
    end
end
xlabel('Time (s)');

exportgraphics(fig, 'fig_VIIC_effort.pdf', 'ContentType','vector');
exportgraphics(fig, 'fig_VIIC_effort.png', 'Resolution',200);
fprintf('Saved fig_VIIC_effort\n');

% --- (2) Summary metrics bar chart --------------------------------
fig = figure('Units','inches','Position',[0.5 0.5 6.5 4]);
sgtitle('VII.C — Summary: RMS Error & Mean Control Effort',...
    'FontSize',11,'FontWeight','bold');

rms_ct  = zeros(3,1); rms_smc  = zeros(3,1);
mef_ct  = zeros(3,1); mef_smc  = zeros(3,1);

for ui = 1:3
    tag_ct  = sprintf('unc%s_ct',  unc_tags{ui});
    tag_smc = sprintf('unc%s_smc', unc_tags{ui});
    rms_ct(ui)  = rad2deg(allResults.(tag_ct).metrics.rmsError);
    rms_smc(ui) = rad2deg(allResults.(tag_smc).metrics.rmsError);
    mef_ct(ui)  = mean(vecnorm(allResults.(tag_ct).out.tau,  2, 2));
    mef_smc(ui) = mean(vecnorm(allResults.(tag_smc).out.tau, 2, 2));
end

x    = 1:3;
xlbls = {'0%','20%','60%'};
bw   = 0.35;

subplot(1,2,1); hold on; box on;
b1 = bar(x - bw/2, rms_ct,  bw, 'FaceColor', col_ct,  'EdgeColor','none');
b2 = bar(x + bw/2, rms_smc, bw, 'FaceColor', col_smc, 'EdgeColor','none');
set(gca,'XTick',x,'XTickLabel',xlbls);
xlabel('Parameter Uncertainty');
ylabel('RMS Tracking Error (deg)');
title('Tracking Accuracy');
legend([b1 b2],'CT','SMC','FontSize',8,'Location','northwest');
grid on; ylim([0 max([rms_ct;rms_smc])*1.25]);

subplot(1,2,2); hold on; box on;
b3 = bar(x - bw/2, mef_ct,  bw, 'FaceColor', col_ct,  'EdgeColor','none');
b4 = bar(x + bw/2, mef_smc, bw, 'FaceColor', col_smc, 'EdgeColor','none');
set(gca,'XTick',x,'XTickLabel',xlbls);
xlabel('Parameter Uncertainty');
ylabel('Mean ||\tau|| (N{\cdot}m)');
title('Control Effort');
legend([b3 b4],'CT','SMC','FontSize',8,'Location','northwest');
grid on; ylim([0 max([mef_ct;mef_smc])*1.25]);

exportgraphics(fig, 'fig_VIIC_metrics.pdf', 'ContentType','vector');
exportgraphics(fig, 'fig_VIIC_metrics.png', 'Resolution',200);
fprintf('Saved fig_VIIC_metrics\n');

fprintf('\n=== All plots exported ===\n');
fprintf('LaTeX usage guide:\n');
fprintf('  VI.A: fig_VIA_tracking, fig_VIA_error, fig_VIA_control\n');
fprintf('  VI.B: fig_VIB_tracking, fig_VIB_error, fig_VIB_control\n');
fprintf('  VI.C: fig_VIC_tracking, fig_VIC_error, fig_VIC_control\n');
fprintf('  VII.A: fig_VIIA_concepts\n');
fprintf('  VII.B: fig_VIIB_tracking\n');
fprintf('  VII.C: fig_VIIC_effort, fig_VIIC_metrics\n');

%% ================================================================
%  HELPER: blue-white-orange colormap for VII.A
%% ================================================================
function cmap = bluewhiteorange()
    n = 64;
    c1 = [0.00 0.45 0.70];
    c2 = [1.00 1.00 1.00];
    c3 = [0.84 0.37 0.00];
    cmap = [interp1([0 1], [c1; c2], linspace(0,1,n/2));
            interp1([0 1], [c2; c3], linspace(0,1,n/2))];
end

%% ================================================================
%  SIMULATION ENGINE
%% ================================================================
function out = runHybridSim(x0, p_real, p_hat, sim)
    T_all=[]; X_all=[]; E_all=[]; tau_all=[]; step_t=[];
    x = x0; tGlobal = 0;

    for k = 1:sim.steps
        tspan = (0:sim.dt:sim.TmaxStep)';
        N     = length(tspan);
        X     = zeros(N, 10);
        Tau   = zeros(N, 5);
        x_curr = x;

        for i = 1:N
            [dx, tau]  = swingPhaseDynamics(tspan(i), x_curr, p_real, p_hat, sim);
            x_curr     = x_curr + sim.dt * dx;
            X(i,:)     = x_curr';
            Tau(i,:)   = tau';
        end

        T_all   = [T_all;   tspan + tGlobal];
        X_all   = [X_all;   X];
        tau_all = [tau_all; Tau];

        E = zeros(N,1);
        for i = 1:N
            E(i) = totalEnergy(X(i,:)', p_real);
        end
        E_all = [E_all; E];

        step_t(end+1) = T_all(end);
        tGlobal       = T_all(end);
        x             = impactMap(X(end,:)', p_real);
    end

    out.T      = T_all;
    out.X      = X_all;
    out.E      = E_all;
    out.tau    = tau_all;
    out.step_t = step_t;
end

function [dx, tau] = swingPhaseDynamics(t, x, p_real, p_hat, sim)
    q = x(1:5); dq = x(6:10);
    [Dreal,Creal,Greal] = robotDynamics(q, dq, p_real);
    [Dhat, Chat, Ghat]  = robotDynamics(q, dq, p_hat);
    [qd, dqd, ddqd]     = referenceGait(t, p_real);

    e  = q  - qd;
    de = dq - dqd;

    switch lower(sim.controller)
        case 'ct'
            v = ddqd - sim.Kd*de - sim.Kp*e;
        case 'smc'
            s = de + sim.Lambda*e;
            v = ddqd - sim.Lambda*de - sim.K_smc*tanh(s/sim.phi);
    end

    tau  = Dhat*v + Chat*dq + Ghat;
    ddq  = Dreal \ (tau - Creal*dq - Greal);
    dx   = [dq; ddq];
end

%% ================================================================
%  ROBOT MODEL
%% ================================================================
function p = getParams()
    p.g = 9.81;
    p.m = [3.2; 6.8; 20.0; 6.8; 3.2];
    p.l = [0.40; 0.40; 0.55; 0.40; 0.40];
    p.I = p.m .* p.l.^2 / 12;
end

function [qd, dqd, ddqd] = referenceGait(t, ~)
    w   = 2*pi*0.85;
    A   = deg2rad([8; 15; 4; 18; 10]);
    ph  = [0; pi/6; pi/2; pi; pi+pi/6];
    q0  = deg2rad([-5; 12; 1; 14; -12]);
    qd   = q0 + A.*sin(w*t + ph);
    dqd  = w * A .* cos(w*t + ph);
    ddqd = -w^2 * A .* sin(w*t + ph);
end

function [D, C, G] = robotDynamics(q, dq, p)
    m = p.m; l = p.l; I = p.I; g = p.g;
    s = sin(q); c = cos(q);

    J  = cell(5,1);
    Jw = cell(5,1);

    J{1}=zeros(2,5); J{1}(:,1)=0.5*l(1)*[c(1);-s(1)]; Jw{1}=[1 0 0 0 0];
    J{2}=zeros(2,5); J{2}(:,1)=l(1)*[c(1);-s(1)];
    J{2}(:,2)=0.5*l(2)*[c(2);-s(2)]; Jw{2}=[1 1 0 0 0];
    J{3}=zeros(2,5); J{3}(:,1)=l(1)*[c(1);-s(1)];
    J{3}(:,2)=l(2)*[c(2);-s(2)];
    J{3}(:,3)=0.5*l(3)*[c(3);-s(3)]; Jw{3}=[1 1 1 0 0];
    J{4}=zeros(2,5); J{4}(:,1)=l(1)*[c(1);-s(1)];
    J{4}(:,2)=l(2)*[c(2);-s(2)];
    J{4}(:,4)=0.5*l(4)*[c(4);s(4)]; Jw{4}=[1 1 0 1 0];
    J{5}=zeros(2,5); J{5}(:,1)=l(1)*[c(1);-s(1)];
    J{5}(:,2)=l(2)*[c(2);-s(2)];
    J{5}(:,4)=l(4)*[c(4);s(4)];
    J{5}(:,5)=0.5*l(5)*[c(5);s(5)]; Jw{5}=[1 1 0 1 1];

    D = zeros(5,5);
    for i = 1:5
        D = D + m(i)*(J{i}'*J{i}) + I(i)*(Jw{i}'*Jw{i});
    end
    C = christoffelCoriolis(q, dq, p);
    G = zeros(5,1);
    for i = 1:5
        G = G - m(i)*g*J{i}(2,:)';
    end
    G = -G;
end

function C = christoffelCoriolis(q, dq, p)
    n = 5; h = 1e-5;
    C = zeros(n,n); dDdq = zeros(n,n,n);
    for k = 1:n
        qf = q; qf(k) = qf(k)+h;
        qb = q; qb(k) = qb(k)-h;
        dDdq(:,:,k) = (massMatrix(qf,p) - massMatrix(qb,p)) / (2*h);
    end
    for i = 1:n
        for j = 1:n
            for k = 1:n
                C(i,j) = C(i,j) + 0.5*(dDdq(i,j,k)+dDdq(i,k,j)-dDdq(j,k,i))*dq(k);
            end
        end
    end
end

function D = massMatrix(q, p)
    m = p.m; l = p.l; I = p.I;
    s = sin(q); c = cos(q);
    J  = cell(5,1); Jw = cell(5,1);
    J{1}=zeros(2,5); J{1}(:,1)=0.5*l(1)*[c(1);-s(1)]; Jw{1}=[1 0 0 0 0];
    J{2}=zeros(2,5); J{2}(:,1)=l(1)*[c(1);-s(1)];
    J{2}(:,2)=0.5*l(2)*[c(2);-s(2)]; Jw{2}=[1 1 0 0 0];
    J{3}=zeros(2,5); J{3}(:,1)=l(1)*[c(1);-s(1)];
    J{3}(:,2)=l(2)*[c(2);-s(2)];
    J{3}(:,3)=0.5*l(3)*[c(3);-s(3)]; Jw{3}=[1 1 1 0 0];
    J{4}=zeros(2,5); J{4}(:,1)=l(1)*[c(1);-s(1)];
    J{4}(:,2)=l(2)*[c(2);-s(2)];
    J{4}(:,4)=0.5*l(4)*[c(4);s(4)]; Jw{4}=[1 1 0 1 0];
    J{5}=zeros(2,5); J{5}(:,1)=l(1)*[c(1);-s(1)];
    J{5}(:,2)=l(2)*[c(2);-s(2)];
    J{5}(:,4)=l(4)*[c(4);s(4)];
    J{5}(:,5)=0.5*l(5)*[c(5);s(5)]; Jw{5}=[1 1 0 1 1];
    D = zeros(5,5);
    for i = 1:5
        D = D + m(i)*(J{i}'*J{i}) + I(i)*(Jw{i}'*Jw{i});
    end
end

function xPlus = impactMap(xMinus, p)
    q  = xMinus(1:5); dq = xMinus(6:10);
    [D,~,~] = robotDynamics(q, dq, p);
    Jc      = swingFootJacobian(q, p);
    A       = [D, -Jc'; Jc, zeros(2,2)];
    sol     = A \ [D*dq; zeros(2,1)];
    dqP     = sol(1:5);
    xPlus   = [q(5);q(4);q(3);q(2);q(1); ...
               dqP(5);dqP(4);dqP(3);dqP(2);dqP(1)];
end

function Jc = swingFootJacobian(q, p)
    l = p.l;
    Jc = zeros(2,5);
    Jc(1,1)= l(1)*cos(q(1)); Jc(2,1)=-l(1)*sin(q(1));
    Jc(1,2)= l(2)*cos(q(2)); Jc(2,2)=-l(2)*sin(q(2));
    Jc(1,4)= l(4)*cos(q(4)); Jc(2,4)= l(4)*sin(q(4));
    Jc(1,5)= l(5)*cos(q(5)); Jc(2,5)= l(5)*sin(q(5));
end

function E = totalEnergy(x, p)
    q = x(1:5); dq = x(6:10);
    m = p.m; l = p.l; g = p.g;
    [D,~,~] = robotDynamics(q, dq, p);
    T  = 0.5 * dq' * D * dq;
    c  = cos(q);
    y(1) = 0.5*l(1)*c(1);
    y(2) = l(1)*c(1) + 0.5*l(2)*c(2);
    y(3) = l(1)*c(1) + l(2)*c(2) + 0.5*l(3)*c(3);
    y(4) = l(1)*c(1) + l(2)*c(2) - 0.5*l(4)*c(4);
    y(5) = l(1)*c(1) + l(2)*c(2) - l(4)*c(4) - 0.5*l(5)*c(5);
    V = sum(m .* g .* y');
    E = T + V;
end

function err = trackingErrorNorm(out, p)
    N   = length(out.T);
    err = zeros(N, 1);
    for i = 1:N
        [qd,~,~] = referenceGait(out.T(i), p);
        err(i)   = norm(out.X(i,1:5)' - qd);
    end
end

function metrics = evaluatePerformance(out, p)
    err = trackingErrorNorm(out, p);
    metrics.steps       = length(out.step_t);
    metrics.totalTime   = out.T(end);
    metrics.rmsError    = sqrt(mean(err.^2));
    metrics.maxError    = max(err);
    metrics.meanEnergy  = mean(out.E);
    metrics.energySwing = max(out.E) - min(out.E);
end