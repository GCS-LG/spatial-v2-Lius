clc;
clear;
quadraped;

q = [ 0;    0.6435;   -1.287; 
         0;    0.6435;   -1.287; 
        0;    -0.6435;   1.287;  
        0;    -0.6435;   1.287 ];

qd = zeros(12, 1);

qdd = zeros(12, 1);

q_des = q;

qd_des = qd;

qdd_des = qdd;

floatingbase = [angle2quat(0,0,0, 'ZYX'),  0, 0, 0.4,  0, 0, 0,  0 ,0 ,0 ]';

f_ext = {zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) zeros(6,1) ...
             zeros(6,1) zeros(6,1) [0;0;0;0;0;100] ...
             zeros(6,1) zeros(6,1) [0;0;0;0;0;100] ...
             zeros(6,1) zeros(6,1) [0;0;0;0;0;100] ...
             zeros(6,1) zeros(6,1) [0;0;0;0;0;100] };

 [xdfb_1,tau_des] = IDfb( fbmodel, floatingbase, q_des, qd_des, qdd_des, f_ext);