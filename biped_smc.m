%% ===============================================================
%  TZAFESTAS 1996 FIVE-LINK BIPED  –  CORRECTED RESEARCH-GRADE VERSION
%
%  Fixes applied vs. original:
%   [1] D(q), C(q,dq), G(q) derived from full Lagrangian (pinned-foot base)
%   [2] Impact map from impulsive momentum equations (Hurmuzlu & Marghitu 1994)
%   [3] Forward kinematics with consistent absolute-angle convention
%   [4] Foot-strike event with forward-progress guard
%   [5] SMC gains correctly separated (Lambda vs K_smc)
%   [6] Reference gait amplitude physically feasible
%   [7] Energy monitor added
%   [8] Animated camera tracks hip
%
%  Coordinate convention
%   q(i) = absolute angle of link i from VERTICAL (positive CCW)
%   Links:  1=stance shin, 2=stance thigh, 3=torso,
%           4=swing thigh,  5=swing shin
%   Stance foot pinned to origin.
%% ===============================================================
clear; clc; close all;
%% ===============================================================

clear; clc; close all;

controllers = {'ct','smc'};
results = struct();

for r = 1:2
    
    sim.controller = controllers{r};
    sim.steps      = 5;
    sim.TmaxStep   = 1.5;
    sim.dt         = 0.002;

    % CT gains
    sim.Kp  = diag([80 80 80 80 80]);
    sim.Kd  = diag([18 18 18 18 18]);

    % SMC gains
    sim.Lambda = 10 * eye(5);
    sim.K_smc  = 25 * eye(5);
    sim.phi    = 0.05;

    % Parameters
    p_real = getParams();
    p_hat  = getParams();

    unc = 0.0;

    p_hat.m = p_real.m .* (1 + unc*[-1;1;-1;1;-1]);
    p_hat.I = p_real.I .* (1 + unc*[1;-1;1;-1;1]);
    p_hat.l = p_real.l .* (1 + 0.15*unc*[1;1;-1;-1;1]);

    % Initial condition
    q0  = deg2rad([-8; 18; 2; 25; -18]);
    dq0 = deg2rad([ 5; -8; 0; -10; 6]);
    x0  = [q0; dq0];

    fprintf('\n=====================================================\n');
    fprintf(' Running %s controller...\n', upper(sim.controller));
    fprintf('=====================================================\n');

    out = runHybridSim(x0,p_real,p_hat,sim);
    
    % ---- PERFORMANCE METRICS ----
    metrics = evaluatePerformance(out,p_real);

    results.(sim.controller) = metrics;

    fprintf('\n%s RESULTS:\n', upper(sim.controller));
    fprintf('Completed Steps       : %d\n', metrics.steps);
    fprintf('Simulation Time       : %.3f sec\n', metrics.totalTime);
    fprintf('RMS Tracking Error    : %.4f rad\n', metrics.rmsError);
    fprintf('Max Tracking Error    : %.4f rad\n', metrics.maxError);
    fprintf('Mean Energy           : %.2f J\n', metrics.meanEnergy);
    fprintf('Energy Variation      : %.2f J\n', metrics.energySwing);
end

%% ===============================================================
%  FINAL COMPARISON TABLE
%% ===============================================================

ct  = results.ct;
smc = results.smc;

fprintf('\n\n=====================================================\n');
fprintf(' FINAL CONTROLLER COMPARISON\n');
fprintf('=====================================================\n');

fprintf('%-22s %-12s %-12s\n','Metric','CT','SMC');
fprintf('-----------------------------------------------------\n');
fprintf('%-22s %-12d %-12d\n','Completed Steps',ct.steps,smc.steps);
fprintf('%-22s %-12.3f %-12.3f\n','Sim Time (s)',ct.totalTime,smc.totalTime);
fprintf('%-22s %-12.4f %-12.4f\n','RMS Error',ct.rmsError,smc.rmsError);
fprintf('%-22s %-12.4f %-12.4f\n','Max Error',ct.maxError,smc.maxError);
fprintf('%-22s %-12.2f %-12.2f\n','Mean Energy',ct.meanEnergy,smc.meanEnergy);
fprintf('%-22s %-12.2f %-12.2f\n','Energy Swing',ct.energySwing,smc.energySwing);

fprintf('=====================================================\n');

if smc.rmsError < ct.rmsError
    fprintf('Winner (tracking): SMC\n');
else
    fprintf('Winner (tracking): CT\n');
end

if smc.energySwing < ct.energySwing
    fprintf('Winner (energy smoothness): SMC\n');
else
    fprintf('Winner (energy smoothness): CT\n');
end



%% -------- SIMULATION SETTINGS --------
sim.controller = 'ct';   % 'ct'  or  'smc'
sim.steps      = 5;
sim.TmaxStep   = 1.5;     % max time allowed per step (s)
sim.dt         = 0.002;   % for energy-monitor sampling only

% Computed-torque gains
sim.Kp  = diag([80 80 80 80 80]);
sim.Kd  = diag([18 18 18 18 18]);

% SMC gains  (Lambda ~ 8-12 is physically sensible)
sim.Lambda = 10 * eye(5);   % sliding-surface slope
sim.K_smc  = 25 * eye(5);   % robustness gain
sim.phi    = 0.05;          % boundary-layer thickness (tanh saturation)

%% -------- PARAMETERS --------
p_real = getParams();
p_hat  = getParams();

% Add uncertainty to controller model
unc = 0.0;     % 0% uncertainty


p_hat.m = p_real.m .* (1 + unc*[-1;1;-1;1;-1]);
p_hat.I = p_real.I .* (1 + unc*[1;-1;1;-1;1]);
p_hat.l = p_real.l .* (1 + 0.15*unc*[1;1;-1;-1;1]);
%% -------- INITIAL CONDITION --------
% q = [stance-shin, stance-thigh, torso, swing-thigh, swing-shin]  (rad, absolute)
q0  = deg2rad([-8; 18; 2; 25; -18]);
dq0 = deg2rad([ 5; -8;  0; -10;  6]);   % small non-zero seed velocities

x0 = [q0; dq0];

%% -------- RUN HYBRID SIMULATION --------
out = runHybridSim(x0, p_real, p_hat, sim);
%% -------- PLOTS --------
plotResults(out, p_real);

%% -------- ANIMATION --------
animateBiped(out, p_real);


%% ===============================================================
%%  HYBRID SIMULATOR
%% ===============================================================
function out = runHybridSim(x0, p_real, p_hat, sim)

    T_all = [];
    X_all = [];
    E_all = [];
    step_t = [];

    x       = x0;
    tGlobal = 0;

    for k = 1:sim.steps

        opts = odeset( ...
            'RelTol',1e-7, ...
            'AbsTol',1e-9, ...
            'Events', @(t,x) heelStrikeEvent(t,x,p_real));

        [tSol,xSol,te,~,~] = ode45( ...
            @(t,x) swingPhaseDynamics(t,x,p_real,p_hat,sim), ...
            [0 sim.TmaxStep], x, opts);

        % energy uses REAL robot
        E = zeros(length(tSol),1);
        for i = 1:length(tSol)
            E(i) = totalEnergy(xSol(i,:)', p_real);
        end

        T_all = [T_all; tSol + tGlobal];
        X_all = [X_all; xSol];
        E_all = [E_all; E];

        if isempty(te)
            warning('Step %d: heel strike not detected.',k);
            break
        end

        step_t(end+1) = te(1) + tGlobal;
        tGlobal = T_all(end);

        xMinus = xSol(end,:)';

        % impact uses REAL robot
        x = impactMap(xMinus, p_real);
    end

    out.T = T_all;
    out.X = X_all;
    out.E = E_all;
    out.step_t = step_t;
end



%% ===============================================================
%  SWING-PHASE ODE  (pinned stance foot)
% ===============================================================
function dx = swingPhaseDynamics(t,x,p_real,p_hat,sim)

    q  = x(1:5);
    dq = x(6:10);

    % Real robot
    [Dreal,Creal,Greal] = robotDynamics(q,dq,p_real);

    % Estimated model used by controller
    [Dhat,Chat,Ghat] = robotDynamics(q,dq,p_hat);

    % reference gait
    [qd,dqd,ddqd] = referenceGait(t,p_real);

    e  = q  - qd;
    de = dq - dqd;

    switch lower(sim.controller)

        case 'ct'

            v = ddqd - sim.Kd*de - sim.Kp*e;
            tau = Dhat*v + Chat*dq + Ghat;

        case 'smc'

            s = de + sim.Lambda*e;

            v = ddqd ...
              - sim.Lambda*de ...
              - sim.K_smc*tanh(s/sim.phi);

            tau = Dhat*v + Chat*dq + Ghat;

        otherwise
            error('Unknown controller');

    end

    % REAL plant motion
    ddq = Dreal \ (tau - Creal*dq - Greal);

    dx = [dq; ddq];
end

%% ===============================================================
%  HEEL-STRIKE EVENT
%  Triggers when swing foot y-coordinate crosses zero (descending)
%  AND swing foot is ahead of stance foot (positive x-direction).
% ===============================================================
function [value, isterminal, direction] = heelStrikeEvent(~, x, p)

    q   = x(1:5);
    pts = forwardKinematics(q, p);

    swingY = pts.swingFoot(2);
    swingX = pts.swingFoot(1);   % must be > 0 (forward step)

    % Primary trigger: swing foot at ground level
    value      = swingY;
    isterminal = 1;
    direction  = -1;   % only detect descending zero-crossing

    % Guard: if swing foot is behind stance foot, do not terminate
    if swingX <= 0.01
        value = 1;     % keep positive → event will not fire
    end
end


%% ===============================================================
%  IMPACT MAP  (Hurmuzlu & Marghitu 1994 / Grizzle et al.)
%
%  Assumptions
%   - Perfectly plastic impact (new stance foot does not rebound)
%   - Old stance foot lifts off instantaneously
%   - Torso and upper-body angular momentum preserved through impact
%
%  Equations
%   D(q) * dq+ = D(q) * dq-  +  Jc(q)' * F_imp
%   Jc(q) * dq+ = 0          (new stance foot velocity = 0)
%
%  This gives:
%   [D   -Jc'] [dq+  ]   [D*dq-]
%   [Jc   0  ] [F_imp] = [0    ]
% ===============================================================
function xPlus = impactMap(xMinus, p)

    q   = xMinus(1:5);
    dq  = xMinus(6:10);

    [D, ~, ~] = robotDynamics(q, dq, p);

    % Constraint Jacobian of new stance foot (currently swing foot)
    Jc = swingFootJacobian(q, p);   % 2×5

    % Solve KKT system
    n  = 5;
    nc = 2;   % x and y velocity of new stance foot = 0

    A = [D,  -Jc';
         Jc,  zeros(nc, nc)];
    b = [D * dq;
         zeros(nc, 1)];

    sol   = A \ b;
    dqPlus = sol(1:n);

    % Relabel legs (swap stance ↔ swing)
    qPlus  = [q(5);  q(4);  q(3);  q(2);  q(1)];
    dqPlus = [dqPlus(5); dqPlus(4); dqPlus(3); dqPlus(2); dqPlus(1)];

    xPlus = [qPlus; dqPlus];
end


%% ===============================================================
%  FORWARD KINEMATICS
%  Consistent absolute-angle convention; stance foot pinned at origin.
%  Positive angles = CCW from vertical.
%
%  Link layout (proximal → distal):
%   Foot → knee1 → hip → (torso tip)
%                  hip → knee2 → swing foot
% ===============================================================
function pts = forwardKinematics(q, p)

    l = p.l;

    % --- Stance leg (foot pinned at origin) ---
    foot1  = [0; 0];

    % shin goes from foot upward at angle q(1) from vertical
    knee1  = foot1  + l(1) * [sin(q(1));  cos(q(1))];

    % thigh continues from knee upward
    hip    = knee1  + l(2) * [sin(q(2));  cos(q(2))];

    % torso tip
    torso  = hip    + l(3) * [sin(q(3));  cos(q(3))];

    % --- Swing leg (from hip downward) ---
    % swing thigh hangs from hip
    knee2  = hip    + l(4) * [sin(q(4)); -cos(q(4))];

    % swing shin hangs from swing knee
    foot2  = knee2  + l(5) * [sin(q(5)); -cos(q(5))];

    pts.stanceFoot = foot1;
    pts.knee1      = knee1;
    pts.hip        = hip;
    pts.torso      = torso;
    pts.knee2      = knee2;
    pts.swingFoot  = foot2;
end


%% ===============================================================
%  SWING-FOOT JACOBIAN  (2×5)
%  Partial derivatives of swing foot position w.r.t. each joint angle.
%  Used in the impact map constraint.
% ===============================================================
function Jc = swingFootJacobian(q, p)

    l = p.l;

    % Swing foot position (see forwardKinematics):
    %   foot2 = [l1*sin(q1) + l2*sin(q2) + l4*sin(q4) + l5*sin(q5) ;
    %            l1*cos(q1) + l2*cos(q2) - l4*cos(q4) - l5*cos(q5) ]
    %
    % ∂foot2/∂q:

    Jc = zeros(2, 5);

    % d(foot2_x)/dqi  ,  d(foot2_y)/dqi
    Jc(1,1) =  l(1)*cos(q(1));   Jc(2,1) = -l(1)*sin(q(1));
    Jc(1,2) =  l(2)*cos(q(2));   Jc(2,2) = -l(2)*sin(q(2));
    Jc(1,3) =  0;                 Jc(2,3) =  0;
    Jc(1,4) =  l(4)*cos(q(4));   Jc(2,4) =  l(4)*sin(q(4));
    Jc(1,5) =  l(5)*cos(q(5));   Jc(2,5) =  l(5)*sin(q(5));
end


%% ===============================================================
%  LAGRANGIAN ROBOT DYNAMICS  D(q)*ddq + C(q,dq)*dq + G(q) = tau
%
%  Derived via Euler-Lagrange with absolute joint angles.
%  Centre of mass of each link at its midpoint.
%
%  Link CoM positions (absolute coordinates):
%   r1 = 0.5*l1*[sin(q1); cos(q1)]
%   r2 = l1*[sin(q1);cos(q1)] + 0.5*l2*[sin(q2);cos(q2)]
%   r3 = l1*[s1;c1] + l2*[s2;c2] + 0.5*l3*[s3;c3]
%   r4 = l1*[s1;c1] + l2*[s2;c2] + 0.5*l4*[s4;-c4]
%   r5 = l1*[s1;c1] + l2*[s2;c2] + l4*[s4;-c4] + 0.5*l5*[s5;-c5]
% ===============================================================
function [D, C, G] = robotDynamics(q, dq, p)

    m = p.m;   % 5×1 link masses
    l = p.l;   % 5×1 link lengths
    I = p.I;   % 5×1 link moments of inertia about CoM
    g = p.g;

    s = sin(q);
    c = cos(q);

    %% ---- Jacobians of each link's CoM ----
    % J{i} is 2×5 translational Jacobian; Jw{i} is 1×5 rotational Jacobian
    % (rotational Jacobian = 1 for joints proximal to link i, 0 otherwise,
    %  with sign +1 for stance-side links, -1 for swing-side links below hip)

    J  = cell(5,1);
    Jw = cell(5,1);

    % Link 1 (stance shin)  CoM = 0.5*l1 from foot
    J{1} = zeros(2,5);
    J{1}(:,1) = 0.5*l(1)*[ c(1); -s(1)];
    Jw{1} = [1 0 0 0 0];

    % Link 2 (stance thigh)  CoM = l1 + 0.5*l2
    J{2} = zeros(2,5);
    J{2}(:,1) = l(1)*[ c(1); -s(1)];
    J{2}(:,2) = 0.5*l(2)*[ c(2); -s(2)];
    Jw{2} = [1 1 0 0 0];

    % Link 3 (torso)  CoM = l1 + l2 + 0.5*l3
    J{3} = zeros(2,5);
    J{3}(:,1) = l(1)*[ c(1); -s(1)];
    J{3}(:,2) = l(2)*[ c(2); -s(2)];
    J{3}(:,3) = 0.5*l(3)*[ c(3); -s(3)];
    Jw{3} = [1 1 1 0 0];

    % Link 4 (swing thigh)  hangs from hip; CoM = l1+l2 + 0.5*l4 downward
    J{4} = zeros(2,5);
    J{4}(:,1) = l(1)*[ c(1); -s(1)];
    J{4}(:,2) = l(2)*[ c(2); -s(2)];
    J{4}(:,4) = 0.5*l(4)*[ c(4);  s(4)];   % downward branch: +sin, +cos
    Jw{4} = [1 1 0 1 0];

    % Link 5 (swing shin)  CoM = l1+l2 + l4 + 0.5*l5 downward
    J{5} = zeros(2,5);
    J{5}(:,1) = l(1)*[ c(1); -s(1)];
    J{5}(:,2) = l(2)*[ c(2); -s(2)];
    J{5}(:,4) = l(4)*[ c(4);  s(4)];
    J{5}(:,5) = 0.5*l(5)*[ c(5);  s(5)];
    Jw{5} = [1 1 0 1 1];

    %% ---- Mass matrix  D = Σ (m_i * J_i' * J_i + I_i * Jw_i' * Jw_i) ----
    D = zeros(5,5);
    for i = 1:5
        D = D + m(i) * (J{i}' * J{i}) + I(i) * (Jw{i}' * Jw{i});
    end

    %% ---- Coriolis / centripetal  C  (Christoffel symbols) ----
    %  C_ij = Σ_k  Γ_{ijk} * dq_k
    %  Γ_{ijk} = 0.5*(∂D_ij/∂q_k + ∂D_ik/∂q_j - ∂D_jk/∂q_i)
    %
    %  Efficient computation via:  C*dq = dD/dt * dq  -  0.5 * ∂(dq'*D*dq)/∂q
    %  We use the direct symbolic-differentiation approach for clarity.

    C = christoffelCoriolis(q, dq, p);

    %% ---- Gravity  G = -Σ m_i * g * ∂(y_CoM_i)/∂q ----
    % y-component of each CoM Jacobian:
    G = zeros(5,1);
    for i = 1:5
        G = G - m(i) * g * J{i}(2,:)';
    end
    % Note: G is defined such that D*ddq + C*dq + G = tau
    % i.e., G already carries the negative sign from ∂V/∂q = -∂(mgy)/∂q
    % Re-check sign: V = Σ m_i*g*y_i  →  ∂V/∂q > 0 for upright posture
    G = -G;   % correct sign: EL eqn uses +G on left side
end


%% ===============================================================
%%  CHRISTOFFEL CORIOLIS (numerical finite-difference for D)
%%  Computes C(q,dq) such that (D_dot - 2C) is skew-symmetric.
%% ===============================================================
function C = christoffelCoriolis(q, dq, p)

    n  = 5;
    h  = 1e-5;   % finite-difference step

    C  = zeros(n, n);

    % Numerical partial derivatives ∂D/∂q_k
    dDdq = zeros(n, n, n);
    for k = 1:n
        qf = q; qf(k) = qf(k) + h;
        qb = q; qb(k) = qb(k) - h;
        Df = massMatrix(qf, p);
        Db = massMatrix(qb, p);
        dDdq(:,:,k) = (Df - Db) / (2*h);
    end

    % Christoffel symbols Γ_{ijk} and C matrix
    for i = 1:n
        for j = 1:n
            for k = 1:n
                Gamma_ijk = 0.5 * ( dDdq(i,j,k) + dDdq(i,k,j) - dDdq(j,k,i) );
                C(i,j) = C(i,j) + Gamma_ijk * dq(k);
            end
        end
    end
end


%% ===============================================================
%%  MASS MATRIX ONLY  (used by Christoffel finite-difference)
%% ===============================================================
function D = massMatrix(q, p)

    m = p.m;
    l = p.l;
    I = p.I;

    s = sin(q);  %#ok<NASGU>
    c = cos(q);

    J  = cell(5,1);
    Jw = cell(5,1);

    J{1} = zeros(2,5);
    J{1}(:,1) = 0.5*l(1)*[ c(1); -s(1)];
    Jw{1} = [1 0 0 0 0];

    J{2} = zeros(2,5);
    J{2}(:,1) = l(1)*[ c(1); -s(1)];
    J{2}(:,2) = 0.5*l(2)*[ c(2); -s(2)];
    Jw{2} = [1 1 0 0 0];

    J{3} = zeros(2,5);
    J{3}(:,1) = l(1)*[ c(1); -s(1)];
    J{3}(:,2) = l(2)*[ c(2); -s(2)];
    J{3}(:,3) = 0.5*l(3)*[ c(3); -s(3)];
    Jw{3} = [1 1 1 0 0];

    J{4} = zeros(2,5);
    J{4}(:,1) = l(1)*[ c(1); -s(1)];
    J{4}(:,2) = l(2)*[ c(2); -s(2)];
    J{4}(:,4) = 0.5*l(4)*[ c(4);  s(4)];
    Jw{4} = [1 1 0 1 0];

    J{5} = zeros(2,5);
    J{5}(:,1) = l(1)*[ c(1); -s(1)];
    J{5}(:,2) = l(2)*[ c(2); -s(2)];
    J{5}(:,4) = l(4)*[ c(4);  s(4)];
    J{5}(:,5) = 0.5*l(5)*[ c(5);  s(5)];
    Jw{5} = [1 1 0 1 1];

    D = zeros(5,5);
    for i = 1:5
        D = D + m(i) * (J{i}' * J{i}) + I(i) * (Jw{i}' * Jw{i});
    end
end


%% ===============================================================
%%  REFERENCE GAIT
%%  Smooth, physically feasible periodic joint trajectories.
%%  Amplitudes kept within human-like ROM:
%%   shin ~±8°, thigh ~±15°, torso ~±5°
%% ===============================================================
function [qd, dqd, ddqd] = referenceGait(t, ~)

    w  = 2*pi * 0.85;   % gait frequency (rad/s)  ≈ 1 step/s

    % Desired angles (rad)
    A  = deg2rad([ 8;  15;  4;  18;  10]);   % amplitudes
    ph = [0;  pi/6;  pi/2;  pi;  pi + pi/6]; % phase offsets
    q0 = deg2rad([-5;  12;  1;  14;  -12]);  % offsets (nominal posture)

    qd   = q0 + A .* sin(w*t + ph);
    dqd  =  w  * A .* cos(w*t + ph);
    ddqd = -w^2 * A .* sin(w*t + ph);
end


%% ===============================================================
%%  TOTAL MECHANICAL ENERGY
%%  E = T + V  where T = 0.5*dq'*D*dq,  V = Σ m_i*g*y_CoM_i
%% ===============================================================
function E = totalEnergy(x, p)

    q  = x(1:5);
    dq = x(6:10);

    m = p.m;
    l = p.l;
    g = p.g;

    [D, ~, ~] = robotDynamics(q, dq, p);

    T = 0.5 * dq' * D * dq;

    % Potential energy: y-coordinate of each link CoM
    s = sin(q);
    c = cos(q);

    yCoM = zeros(5,1);
    yCoM(1) = 0.5*l(1)*c(1);
    yCoM(2) = l(1)*c(1) + 0.5*l(2)*c(2);
    yCoM(3) = l(1)*c(1) + l(2)*c(2) + 0.5*l(3)*c(3);
    yCoM(4) = l(1)*c(1) + l(2)*c(2) - 0.5*l(4)*c(4);
    yCoM(5) = l(1)*c(1) + l(2)*c(2) - l(4)*c(4) - 0.5*l(5)*c(5);

    V = sum(m .* g .* yCoM);

    E = T + V;
end


%% ===============================================================
%%  PARAMETERS
%% ===============================================================
function p = getParams()

    p.g = 9.81;

    % Masses (kg):  [stance-shin, stance-thigh, torso, swing-thigh, swing-shin]
    p.m = [3.2;  6.8;  20.0;  6.8;  3.2];

    % Link lengths (m)
    p.l = [0.40;  0.40;  0.55;  0.40;  0.40];

    % Moments of inertia about CoM (kg·m²) — slender rod: I = m*l²/12
    p.I = p.m .* p.l.^2 / 12;
end


%% ===============================================================
%%  PLOTS
%% ===============================================================
function plotResults(out, p)
    T = out.T;
    X = out.X;
    E = out.E;
    labels = {'\theta_1 (stance shin)', '\theta_2 (stance thigh)', ...
              '\theta_3 (torso)', '\theta_4 (swing thigh)', '\theta_5 (swing shin)'};
    colors  = lines(5);

    %% -- Joint Tracking Error (NEW) --
    figure('Name','Tracking Errors','NumberTitle','off');
    hold on;
    for i = 1:5
        subplot(5,1,i);
        % Calculate error for this joint over all time steps
        q_actual = X(:,i);
        q_desired = zeros(length(T),1);
        for k = 1:length(T)
            [qd_vec,~,~] = referenceGait(T(k), p);
            q_desired(k) = qd_vec(i);
        end
        
        plot(T, rad2deg(q_actual - q_desired), 'Color', colors(i,:), 'LineWidth', 1.5);
        ylabel('Error (deg)', 'FontSize', 8);
        grid on;
        if i == 1, title('Joint Tracking Errors (q_{actual} - q_{desired})'); end
        if i == 5, xlabel('Time (s)'); end
        
        % Mark impacts
        for ts = out.step_t
            xline(ts, 'k--', 'Alpha', 0.3);
        end
    end

    %% -- Joint angles --
    figure('Name','Joint Angles','NumberTitle','off');
    for i = 1:5
        subplot(5,1,i);
        % Calculate desired for overlay
        q_desired = zeros(length(T),1);
        for k = 1:length(T)
            [qd_vec,~,~] = referenceGait(T(k), p);
            q_desired(k) = qd_vec(i);
        end
        
        plot(T, rad2deg(X(:,i)), 'Color', colors(i,:), 'LineWidth', 1.5); hold on;
        plot(T, rad2deg(q_desired), 'k:', 'LineWidth', 1.0); % Desired as dotted black
        ylabel(labels{i}, 'FontSize', 8);
        grid on;
        if i == 1, title('Joint Angles: Solid (Actual) vs Dotted (Reference)'); end
        if i == 5, xlabel('Time (s)'); end
        for ts = out.step_t, xline(ts, 'k--', 'Alpha', 0.4); end
    end

    %% -- Joint velocities --
    figure('Name','Joint Velocities','NumberTitle','off');
    for i = 1:5
        subplot(5,1,i);
        plot(T, rad2deg(X(:,i+5)), 'Color', colors(i,:), 'LineWidth', 1.5);
        ylabel(['\omega_' num2str(i)], 'FontSize', 8);
        grid on;
        if i == 1, title('Joint Angular Velocities (deg/s)'); end
        if i == 5, xlabel('Time (s)'); end
    end

    %% -- Phase portraits --
    figure('Name','Phase Portraits','NumberTitle','off');
    for i = 1:5
        subplot(2,3,i);
        plot(rad2deg(X(:,i)), rad2deg(X(:,i+5)), ...
             'Color', colors(i,:), 'LineWidth', 1.2);
        xlabel(['\theta_' num2str(i) ' (deg)']);
        ylabel(['\omega_' num2str(i) ' (deg/s)']);
        title(labels{i}, 'FontSize', 8);
        grid on;
    end

    %% -- Total energy --
    figure('Name','Mechanical Energy','NumberTitle','off');
    plot(T, E, 'k', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('E_{total} (J)');
    title('Total Mechanical Energy (T + V)');
    grid on;
    for ts = out.step_t
        xline(ts, 'r--', 'LineWidth', 1, 'Label', 'impact');
    end

    %% -- Swing foot height --
    footY = zeros(length(T),1);
    for k = 1:length(T)
        pts    = forwardKinematics(out.X(k,1:5)', p);
        footY(k) = pts.swingFoot(2);
    end
    figure('Name','Swing Foot Height','NumberTitle','off');
    plot(T, footY, 'b', 'LineWidth', 1.5);
    yline(0, 'k--', 'Ground');
    xlabel('Time (s)');
    ylabel('Swing foot y (m)');
    title('Swing Foot Ground Clearance');
    grid on;
end

%% ===============================================================
%%  ANIMATION  (camera tracks hip)
%% ===============================================================
function animateBiped(out, p)

    gifName = 'biped_walk_0_ct.gif';   % output filename

    figure('Name','Biped Animation','NumberTitle','off', ...
           'Color','w', 'Position',[100 100 900 500]);

    ax = axes;
    axis(ax,'equal');
    grid(ax,'on');
    ylim(ax,[-0.15 1.6]);

    firstFrame = true;

    for k = 1:6:length(out.T)

        q   = out.X(k,1:5)';
        pts = forwardKinematics(q,p);

        hipX = pts.hip(1);

        xlim(ax,[hipX-1.0 hipX+1.0]);

        cla(ax); hold(ax,'on');

        % Ground
        plot(ax,[hipX-2 hipX+2],[0 0],'k-','LineWidth',2);

        % Stance leg
        drawSegment(ax,pts.stanceFoot,pts.knee1,[0.15 0.35 0.75]);
        drawSegment(ax,pts.knee1,pts.hip,[0.15 0.35 0.75]);

        % Torso
        drawSegment(ax,pts.hip,pts.torso,[0.70 0.10 0.10]);

        % Swing leg
        drawSegment(ax,pts.hip,pts.knee2,[0.10 0.60 0.20]);
        drawSegment(ax,pts.knee2,pts.swingFoot,[0.10 0.60 0.20]);

        % Joints
        jpts = [pts.stanceFoot pts.knee1 pts.hip pts.torso ...
                pts.knee2 pts.swingFoot];

        plot(ax,jpts(1,:),jpts(2,:),'o', ...
            'MarkerSize',6, ...
            'MarkerFaceColor','k', ...
            'MarkerEdgeColor','k');

        title(ax,sprintf('t = %.3f s   step %d', ...
            out.T(k),sum(out.step_t <= out.T(k))+1));

        drawnow;

        % ===== CAPTURE FRAME =====
        frame = getframe(gcf);
        im = frame2im(frame);
        [A,map] = rgb2ind(im,256);

        if firstFrame
            imwrite(A,map,gifName,'gif', ...
                'LoopCount',Inf, ...
                'DelayTime',0.03);
            firstFrame = false;
        else
            imwrite(A,map,gifName,'gif', ...
                'WriteMode','append', ...
                'DelayTime',0.03);
        end

        pause(0.018);
    end

    fprintf('GIF saved as %s\n', gifName);

end

function drawSegment(ax, a, b, col)
    plot(ax, [a(1) b(1)], [a(2) b(2)], '-', ...
         'Color', col, 'LineWidth', 3.5);
end

%% ===============================================================
% ADD THIS FUNCTION AT BOTTOM OF FILE
%% ===============================================================
function metrics = evaluatePerformance(out,p)

T = out.T;
X = out.X;
E = out.E;

N = length(T);
err = zeros(N,1);

for i = 1:N
    [qd,~,~] = referenceGait(T(i),p);
    e = X(i,1:5)' - qd;
    err(i) = norm(e);
end

metrics.steps       = length(out.step_t);
metrics.totalTime   = T(end);
metrics.rmsError    = sqrt(mean(err.^2));
metrics.maxError    = max(err);
metrics.meanEnergy  = mean(E);
metrics.energySwing = max(E)-min(E);

end