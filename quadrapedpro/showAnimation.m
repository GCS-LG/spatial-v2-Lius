% for plot  animation
FR_abad = bodybase + withLegSigns(abadLocation, 0);
FL_abad = bodybase + withLegSigns(abadLocation, 1);
HR_abad = bodybase + withLegSigns(abadLocation, 2);
HL_abad = bodybase + withLegSigns(abadLocation, 3);


plot3( [FR_abad(1), HL_abad(1) ],  [ FR_abad(2), HL_abad(2) ], [ FR_abad(3), HL_abad(3) ] , 'k'); hold on;
plot3( [FL_abad(1), HR_abad(1)], [FL_abad(2), HR_abad(2)], [FL_abad(3), HR_abad(3)], 'k'); hold on;

fr_q = state.q(1:3);
fl_q = state.q(4:6);
hr_q = state.q(7:9);
hl_q = state.q(10:12);
bodyori = quat2dcm(state.bodyorientation);

fr_p = ForwardKinematics(fr_q, 0, L, bodyori);
fl_p = ForwardKinematics( fl_q, 1, L, bodyori);
hr_p = ForwardKinematics(hr_q, 2, L, bodyori);
hl_p = ForwardKinematics(hl_q, 3, L, bodyori);

FR_hip_inW = fr_p(1, 1:3);
FR_knee_inW = fr_p(2, 1:3);
FR_foot_inW = fr_p(3, 1:3);

FL_hip_inW    = fl_p(1, 1:3);
FL_knee_inW = fl_p(2, 1:3);
FL_foot_inW  = fl_p(3, 1:3);

HR_hip_inW    = hr_p(1, 1:3);
HR_knee_inW = hr_p(2, 1:3);
HR_foot_inW  = hr_p(3, 1:3);

HL_hip_inW    = hl_p(1, 1:3);
HL_knee_inW = hl_p(2, 1:3);
HL_foot_inW  = hl_p(3, 1:3);

plot3( [FR_abad(1), FR_hip_inW(1) ],  [ FR_abad(2), FR_hip_inW(2) ], [ FR_abad(3), FR_hip_inW(3) ] , 'k'); hold on;
plot3( [FR_hip_inW(1), FR_knee_inW(1) ],  [ FR_hip_inW(2), FR_knee_inW(2) ], [ FR_hip_inW(3), FR_knee_inW(3) ] , 'k'); hold on;
plot3( [FR_knee_inW(1), FR_foot_inW(1) ],  [ FR_knee_inW(2), FR_foot_inW(2) ], [ FR_knee_inW(3), FR_foot_inW(3) ] , 'k'); hold on;

plot3( [FL_abad(1), FL_hip_inW(1) ],  [ FL_abad(2), FL_hip_inW(2) ], [ FL_abad(3), FL_hip_inW(3) ] , 'k'); hold on;
plot3( [FL_hip_inW(1), FL_knee_inW(1) ],  [ FL_hip_inW(2), FL_knee_inW(2) ], [ FL_hip_inW(3), FL_knee_inW(3) ] , 'k'); hold on;
plot3( [FL_knee_inW(1), FL_foot_inW(1) ],  [ FL_knee_inW(2), FL_foot_inW(2) ], [ FL_knee_inW(3), FL_foot_inW(3) ] , 'k'); hold on;

plot3( [HR_abad(1), HR_hip_inW(1) ],  [ HR_abad(2), HR_hip_inW(2) ], [ HR_abad(3), HR_hip_inW(3) ] , 'k'); hold on;
plot3( [HR_hip_inW(1), HR_knee_inW(1) ],  [ HR_hip_inW(2), HR_knee_inW(2) ], [ HR_hip_inW(3), HR_knee_inW(3) ] , 'k'); hold on;
plot3( [HR_knee_inW(1), HR_foot_inW(1) ],  [ HR_knee_inW(2), HR_foot_inW(2) ], [ HR_knee_inW(3), HR_foot_inW(3) ] , 'k'); hold on;

plot3( [HL_abad(1), HL_hip_inW(1) ],  [ HL_abad(2), HL_hip_inW(2) ], [ HL_abad(3), HL_hip_inW(3) ] , 'k'); hold on;
plot3( [HL_hip_inW(1), HL_knee_inW(1) ],  [ HL_hip_inW(2), HL_knee_inW(2) ], [ HL_hip_inW(3), HL_knee_inW(3) ] , 'k'); hold on;
plot3( [HL_knee_inW(1), HL_foot_inW(1) ],  [ HL_knee_inW(2), HL_foot_inW(2) ], [ HL_knee_inW(3), HL_foot_inW(3) ] , 'k'); hold on;