function [dqdt, tau] = robot_dynamics(t, x, controller_type)

% Ensure column vector
x = x(:);

% State split
q = x(1:2);
dq = x(3:4);

% --- Parameters ---
m1 = 1.0; m2 = 0.8;
l1 = 0.5; l2 = 0.5;
g = 9.81;

% --- Dynamics ---
M = [(m1+m2)*l1^2 + m2*l2^2 + 2*m2*l1*l2*cos(q(2)), ...
     m2*l2^2 + m2*l1*l2*cos(q(2));
     m2*l2^2 + m2*l1*l2*cos(q(2)), ...
     m2*l2^2];

C = [-m2*l1*l2*sin(q(2))*dq(2), ...
     -m2*l1*l2*sin(q(2))*(dq(1)+dq(2));
      m2*l1*l2*sin(q(2))*dq(1), 0];

G = [(m1+m2)*g*l1*cos(q(1)) + m2*g*l2*cos(q(1)+q(2));
     m2*g*l2*cos(q(1)+q(2))];

% --- Desired trajectory ---
qd   = [sin(t);  cos(t)];
dqd  = [cos(t); -sin(t)];
ddqd = [-sin(t); -cos(t)];

% --- Errors ---
e  = q  - qd;
de = dq - dqd;

lambda = 5;
s = de + lambda*e;

% --- Control ---
if strcmp(controller_type, 'Classic_SMC')
    K = 15;
    tau_v = -K * sign(s);
else
    Phi = 0.1;
    tau_v = -15 * (s ./ (abs(s) + Phi));
end

% Computed torque + robust term
tau = M*ddqd + C*dq + G - M*lambda*de + tau_v;

% Disturbance
tau_dist = [2*sin(5*t); 2*cos(5*t)];

% --- Dynamics ---
ddq = M \ (tau + tau_dist - C*dq - G);

% Return derivative
dqdt = [dq; ddq];
end