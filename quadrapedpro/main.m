clc;
clear;
close all;
pic_num = 1;
k=1;
quadraped;
w = 2;
dt = 0.001;
fr_qd_des_old = zeros(3, 1);

%height == 0.4
q = [ 0;    0.6435;   -1.287; 
         0;    0.6435;   -1.287; 
        0;    -0.6435;   1.287;  
        0;    -0.6435;   1.287 ];
qd = zeros(12, 1);
qdd = zeros(12, 1);

q_data = q;
qd_data = qd;
qdd_data = qdd;
tau_des = zeros(12 ,1);
i = 1;

for t = 0.001:0.001:0.1
    % planner
    
    fr_p_des = [0.05*sin(2*pi*w*t), -0.01,  -0.35 + 0.05*cos(2*pi*w*t + pi)]';
    fr_v_des = [2*pi*w*0.05*cos(2*pi*w*t), 0, -2*pi*w*0.05*sin(2*pi*w*t + pi)]';
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
    
%      fr_force = 1*(fr_q_des - q_data(1:3)) + 0.1*( fr_qd_des - qd_data(1:3) );
%      fr_tau_des = fr_J' * fr_force;
%      tau_des(1:3) = fr_tau_des;
    f_ext = {zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) ...
                 zeros(6,1) zeros(6,1) [0;0;0;0;0;0] ...
                 zeros(6,1) zeros(6,1) [0;0;0;0;0;400/3] ...
                 zeros(6,1) zeros(6,1) [0;0;0;0;0;400/3] ...
                 zeros(6,1) zeros(6,1) [0;0;0;0;0;400/3] };
         
    [xdfb_1,tau_des] = IDfb( fbmodel, floatingbase, q_des, qd_des, qdd_des, f_ext);
    
    [xdfb_2,  qdd] = FDfb(fbmodel, floatingbase, q_des, qd_des, tau_des, f_ext);
    
    state = integrate(floatingbase, q_data, qd_data, xdfb_2, qdd, dt);
 
    q_data = state.q;
    qd_data = state.qd;
    show_bodyorientation = state.bodyorientation;
    [fr_p_data, fr_J] = ForwardKinematics(q_data(1:3), 0, L);
    showAnimation;
    i = i+1;
%     F = getframe;
%     drawnow;
%     im(pic_num) = getframe(1);
%     pic_num = pic_num+1;
end

% filename = 'swingstracking.gif';
% for idx = 1:size(im ,2)
%     [A, map] =rgb2ind(frame2im(im(idx)), 256);
%     if idx == 1
%         imwrite(A, map, filename, 'gif', 'LoopCount', Inf, 'DelayTime', 1e-1);
%     else
%         imwrite(A, map, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 1e-1);
%     end
% end