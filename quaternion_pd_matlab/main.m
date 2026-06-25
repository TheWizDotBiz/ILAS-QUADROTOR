function main
clc; close all;

P = parameters;
t = 0:P.dt:P.T;
N = numel(t);

q = zeros(4,N);
omega = zeros(3,N);
q(:,1) = P.q0;
omega(:,1) = P.omega0;

for k = 1:N-1
    tau = control_law(q(:,k),omega(:,k),P);

    omega_dot = P.J \ (tau - cross(omega(:,k),P.J*omega(:,k)));
    q_dot = quaternion_kinematics(q(:,k),omega(:,k));

    omega(:,k+1) = omega(:,k) + P.dt*omega_dot;
    q(:,k+1) = q(:,k) + P.dt*q_dot;
    q(:,k+1) = q(:,k+1)/norm(q(:,k+1));
end

animate_attitude(t,q,P);
end

function q_dot = quaternion_kinematics(q,omega)
G = [-q(2), -q(3), -q(4);
      q(1), -q(4),  q(3);
      q(4),  q(1), -q(2);
     -q(3),  q(2),  q(1)];

q_dot = 0.5*G*omega;
end
