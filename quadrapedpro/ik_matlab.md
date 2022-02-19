```matlab
%% four IK
clear;
clc;
leg = 3;
p = [0.0, 0.1329, -0.25];
l1 = 0.1329;
if( leg == 0 || leg == 1 )
  l2 = 0.226;
  l3 = 0.2629;
end

if( leg == 2 || leg == 3 )
  l2 = 0.271;
  l3 = 0.2621;
end
sideSign = [-1, 1. -1, 1];
square_alpha = p(3)^2 + p(2)^2 - l1^2;
alpha = sqrt(square_alpha);
solution_q = zeros(4,3);

solution_q(1,1) = atan2(alpha, sideSign(leg+1)*l1) - atan2(abs(p(3)), p(2));
solution_q(2,1) = solution_q(1,1);
solution_q(3,1) = solution_q(1,1);
solution_q(4,1) = solution_q(1,1);

temp_c3 = (square_alpha + p(1)^2 - l2^2 - l3^2)/(2*l2*l3);
temp_s3 = sqrt(1-temp_c3^2);

solution_q(1,3) = atan2(-temp_s3,temp_c3);
solution_q(2,3) = atan2(-temp_s3,temp_c3);
solution_q(3,3) = atan2(temp_s3,temp_c3);
solution_q(4,3) = atan2(temp_s3,temp_c3);

if(leg == 0 || leg == 1) 
  temp_s3 = -temp_s3; 
end
temp_s2 = ((l2 + l3 * temp_c3) * p(1) - l3 * temp_s3 * alpha)/(square_alpha + p(1)^2);
temp_c2 = ((l2 + l3 * temp_c3) * alpha + l3 * temp_s3 * p(1))/(square_alpha + p(1)^2);
solution_q(1,2) = atan2(temp_s2, temp_c2);
solution_q(2,2) = atan2(temp_s2, temp_c2);
solution_q(3,2) = atan2(-temp_s2, temp_c2);
solution_q(4,2) = atan2(-temp_s2, temp_c2);


if(leg == 0)
 leg_q = solution_q(1,1:3)
elseif (leg==1)
 leg_q = solution_q(2,1:3)
elseif (leg == 2)
 leg_q = solution_q(3,1:3)
elseif (leg == 3)
 leg_q = solution_q(4,1:3)
end
%% FK
a_L = sideSign(leg+1) * l1;
h_L = l2;
k_L = l3;
q1 = leg_q(1); q2 = leg_q(2); q3 = leg_q(3);
x = h_L*sin(q2) + k_L*cos(q2)*sin(q3) + k_L*cos(q3)*sin(q2);
y = a_L*cos(q1) + h_L*cos(q2)*sin(q1) - k_L*sin(q1)*sin(q2)*sin(q3) + k_L*cos(q2)*cos(q3)*sin(q1);
z = a_L*sin(q1) - h_L*cos(q1)*cos(q2) - k_L*cos(q1)*cos(q2)*cos(q3) + k_L*cos(q1)*sin(q2)*sin(q3);

p_hat = [x, y, z]
```

