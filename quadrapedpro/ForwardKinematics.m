function [P,J] = ForwardKinematics(leg_q, leg, L, body_ori)

body_L = L(1);
body_W = L(2);
a_L = L(3);
h_L = L(4);
k_L = L(5);

c1 = cos(leg_q(1));
s1 = sin(leg_q(1));
c2 = cos(leg_q(2));
s2 = sin(leg_q(2));
c3 = cos(leg_q(3));
s3 = sin(leg_q(3));
c23 = c2*c3 - s2*s3;
s23 = s2*c3 +c2*s3; 

sign = [1, -1, 1, -1];

T_rotz = [-1  0  0  0;
                0   -1  0  0;
                0   0  1  0;
                0   0  0  1];%??z?180
                
if nargin > 3
     T_w2b = eye(4);
     T_w2b(1:3, 1:3) = body_ori;
     T_w2b(1:3, 4) = [0; 0; 0.4];

    T_01 = [1  0  0   0;
                 0 c1 -s1 0;
                 0 s1 c1  0;
                 0  0  0  1];%body to abad
    if(leg == 0) 
     T_01(1:3, 4)  = [body_L; -body_W; 0];
    elseif(leg ==1)
      T_01(1:3, 4) = [body_L;  body_W; 0];
    elseif(leg == 2)
      T_01(1:3, 4) = [-body_L; -body_W; 0];
    else
      T_01(1:3, 4) = [-body_L; body_W; 0];
    end
    
    T_12 = [c2  0  s2   0;
                0   1  0   sign(leg+1) * a_L;
               -s2  0  c2   0;
                0   0  0    1 ];
    T_12 =  T_rotz * T_12; %abad to hip

    T_23 = [c3   0   s3  0;
                 0    1   0   0;
               -s3   0  c3 -h_L ;
                 0  0  0   1];%hip to knee

    if( leg == 0 || leg == 2 )
        Hip_inW = T_w2b * T_01 * [0; -a_L; 0; 1];
    elseif ( leg == 1 || leg == 3 )
        Hip_inW =  T_w2b * T_01 * [0; a_L; 0; 1];
    end

    Knee_inW =  T_w2b * T_01 * T_12 * [0; 0; -h_L; 1];
    Foot_inW =   T_w2b * T_01 * T_12 * T_23 * [0; 0; -k_L; 1];

    P = [Hip_inW(1:3)';  Knee_inW(1:3)';  Foot_inW(1:3)'];
else
    T_01 = [1  0  0   0;
                 0 c1 -s1 0;
                0 s1 c1  0;
                0  0  0  1];%body to abad   
     T_12 = [c2  0  s2   0;
            0   1  0   sign(leg+1) * a_L;
           -s2  0  c2   0;
            0   0  0    1 ];
    T_12 =  T_rotz * T_12; %abad to hip

    T_23 = [c3   0   s3  0;
            0    1   0   0;
           -s3   0  c3 -h_L ;
            0  0  0   1];%hip to knee

    if( leg == 0 || leg == 2 )
        Hip_inW = T_01 * [0; -a_L; 0; 1];
    elseif ( leg == 1 || leg == 3 )
        Hip_inW = T_01 * [0; a_L; 0; 1];
    end

    Knee_inW =  T_01 * T_12 * [0; 0; -h_L; 1];
    Foot_inW =   T_01 * T_12 * T_23 * [0; 0; -k_L; 1];

    P = [Hip_inW(1:3)';  Knee_inW(1:3)';  Foot_inW(1:3)'];
    J = zeros(3,3);
    sideSign = [1, -1, 1, -1];
%     J(1, 1) = 0;
%     J(1, 2) = - k_L*c23 - h_L*c2;
%     J(1, 3) = -k_L*c23;
%     J(2, 1) = k_L*(c1*c2*c3 - c1*s2*s3) -  sideSign(leg+1) *a_L*s1 + h_L*c1*c2;
%     J(2, 2) = -s1*(k_L*s23 + h_L*s2);
%     J(2, 3) = -k_L*s23*s1;
%     J(3, 1) =  sideSign(leg+1) *a_L*c1 - k_L*(s1*s2*s3 - c2*c3*s1) + h_L*c2*s1;
%     J(3, 2) = c1*(k_L*s23 + h_L*s2);
%     J(3, 3) = k_L*s23*c1;
    
    J(1, 1) = 0;
    J(1, 2) =  k_L*c23 + h_L*c2;
    J(1, 3) = k_L*c23;
    J(2, 1) = k_L * c1 * c23 + h_L * c1 * c2 - a_L * sideSign(leg+1) * s1;
    J(2, 2) = -s1*(k_L*s23 + h_L*s2);
    J(2, 3) = -k_L*s23*s1;
    J(3, 1) =  k_L * s1 * c23 + h_L * c2 * s1 + a_L * sideSign(leg+1) * c1;
    J(3, 2) = c1*(k_L*s23 + h_L*s2);
    J(3, 3) = k_L*s23*c1;
end

end
% 
%         J->operator()(0, 0) = 0;
%     J->operator()(0, 1) = l3 * c23 + l2 * c2;
%     J->operator()(0, 2) = l3 * c23;
%     J->operator()(1, 0) = l3 * c1 * c23 + l2 * c1 * c2 - (l1 + l4) * sideSign * s1;
%     J->operator()(1, 1) = -l3 * s1 * s23 - l2 * s1 * s2;
%     J->operator()(1, 2) = -l3 * s1 * s23;
%     J->operator()(2, 0) = l3 * s1 * c23 + l2 * c2 * s1 + (l1 + l4) * sideSign * c1;
%     J->operator()(2, 1) = l3 * c1 * s23 + l2 * c1 * s2;
%     J->operator()(2, 2) = l3 * c1 * s23;
    