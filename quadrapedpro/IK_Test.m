% for IK test

modelconfig;
p = [0, -abadLinkLength, -0.4];
q = InverseKinematics(p, 0, L);
q
