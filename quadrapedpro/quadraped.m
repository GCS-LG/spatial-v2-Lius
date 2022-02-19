
modelconfig;

fbmodel.gravity = [0 0 -9.81];

fbmodel.NB = 18;

fbmodel.jtype = {'Px' 'Py' 'Pz' 'Rx' 'Ry' 'Rz' 'Rx' 'Ry' 'Ry' 'Rx' 'Ry' 'Ry' 'Rx' 'Ry' 'Ry' 'Rx' 'Ry' 'Ry'};
                              

fbmodel.parent = [0 1 2 3 4 5  6 7 8  6  10 11   6  13 14   6 16 17];

modelXtree;

fbmodel.Xtree = Xtree;

modelInertia;

fbmodel.I = Inertial;

floatingbase = [angle2quat(0,0,0, 'ZYX'),  0, 0, 0.4,  0, 0, 0,  0 ,0 ,0 ]';

q = [ 0;    0.6916;   -1.2595; 
        0;    0.6916;   -1.2595; 
        0;    -0.6916;   1.2595; 
        0;    -0.6916;   1.2595];

qd = zeros(12, 1);

tau  = zeros(12, 1);

[xdfb,  qdd] = FDfb(fbmodel, floatingbase, q, qd, tau);

state = integrate(floatingbase, q, qd, xdfb, qdd, dt);

showAnimation;
