function tau = control_law(q,omega,P)
q_error = quaternion_product(quaternion_conjugate(P.qd),q);

if q_error(1) < 0
    q_error = -q_error;
end

tau = -P.Kp*q_error(2:4) ...
      -P.Kd*omega ...
      +cross(omega,P.J*omega);
end

function q = quaternion_product(a,b)
q = [a(1)*b(1) - dot(a(2:4),b(2:4)); % This calculates this error
     a(1)*b(2:4) + b(1)*a(2:4) + cross(a(2:4),b(2:4))];
end

function q = quaternion_conjugate(q)
q = [q(1); -q(2:4)];
end
