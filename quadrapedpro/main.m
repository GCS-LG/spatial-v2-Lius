clc;
clear;
quadraped;
w = 2;
dt = 0.001;
fr_qd_des_old = zeros(3, 1);

q = [ 0;    0.8721;   -1.5741; 
         0;    0.0;   0; 
         0;    0;   0; 
        0;    0;   0 ];

qd = zeros(12, 1);

qdd = zeros(12, 1);

q_data = q;
qd_data = qd;
qdd_data = qdd;
tau_des = zeros(12 ,1);

for t = 0.001:0.001:0.005
    % planner
    fr_p_des = [sin(2*pi*w*t), -0.1329,  -0.4 + 0.05*cos(2*pi*w*t)]';
    fr_v_des = [2*pi*w*cos(2*pi*w*t), 0, -2*pi*w*0.05*sin(2*pi*w*t)]';
    [fr_q_des] = InverseKinematics(fr_p_des, 0, L);
    [fr_p, fr_J] = ForwardKinematics(fr_q_des, 0, L);
    fr_qd_des  = fr_J \ fr_v_des;
    fr_qdd_des = (fr_qd_des - fr_qd_des_old)./0.001;
    fr_qd_des_old = fr_qd_des;

    floatingbase = [angle2quat(0,0,0, 'ZYX'),  0, 0, 0.4,  0, 0, 0,  0 ,0 ,0 ]';
    q_des = q;
    q_des(1:3) = fr_q_des;
    qd_des = qd;
    qd_des(1:3) = fr_qd_des;
    qdd_des = qdd;
    qdd_des(1:3) = fr_qdd_des;
    %Controller 
%     [xdfb_1,tau_des] = IDfb( fbmodel, floatingbase, q_des, qd_des, qdd_des);
     fr_force = 1*(fr_q_des - q_data(1:3)) + 0.1*( fr_qd_des - qd_data(1:3) );
     fr_tau_des = fr_J' * fr_force;
     tau_des(1:3) = fr_tau_des;
    
    [xdfb_2,  qdd] = FDfb(fbmodel, floatingbase, q_data, qd_data, tau_des);%这一步有问题
    
    state = integrate(floatingbase, q_data, qd_data, xdfb_2, qdd, dt);
 
    q_data = state.q;
    qd_data = state.qd;
    show_bodyorientation = state.bodyorientation;
    
    showAnimation;
end