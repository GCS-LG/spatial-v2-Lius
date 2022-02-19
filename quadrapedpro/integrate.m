function [state] = integrate(fb,  Joint_q,  Joint_qd,  xd_fb, Joint_qdd,  dt)
%integration to get the latest state of robot

state.qd = Joint_qd + Joint_qdd * dt;
state.q   = Joint_q   + state.qd * dt;

state.bodyvelocity = fb(8:end) + xd_fb(8:end) * dt;
Omega_body = state.bodyvelocity(1:3);
V_body = state.bodyvelocity(4:6);

state.bodyorientation = integrateQuatImplicit(fb(1:4), Omega_body, dt);
dbodypostion = quat2dcm(fb(1:4)')' * V_body; 
state.bodyposition = fb(5:7) + dbodypostion * dt;


end

function [quatNew] =  integrateQuatImplicit(quat, omega, dt)

ang = norm(omega);
if(ang > 0)
    axis = omega / ang;
else
    axis = [1,1,1]';
end
 ang = ang * dt;
 ee = sin(ang / 2) * axis;
 quatD = [cos(ang / 2), ee(1), ee(2), ee(3)]';
 quatNew = quatProduct(quat, quatD);
 quatNew = quatNew / norm(quatNew);
end

function  [quat] = quatProduct(q1, q2)

r1 = q1(1);
r2 = q2(1);

v1 = [q1(2), q1(3), q1(4)]';
v2 = [q2(2), q2(3), q2(4)]';

r = r1 * r2 - dot(v1, v2);
v = r1 * v2 + r2 * v1 + cross(v1, v2);

quat = [r, v(1), v(2), v(3)];






end