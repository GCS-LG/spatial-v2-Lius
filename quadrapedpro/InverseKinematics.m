function [leg_q]  = InverseKinematics(p, leg, L)

% four IK

l1 =L(3);
l2 = L(4);
l3 = L(5);

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
 leg_q = solution_q(1,1:3)';
elseif (leg==1)
 leg_q = solution_q(2,1:3)';
elseif (leg == 2)
 leg_q = solution_q(3,1:3)';
elseif (leg == 3)
 leg_q = solution_q(4,1:3)';
end

end