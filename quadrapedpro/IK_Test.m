% for IK test

bodyLength = 0.2225;
bodyWidth = 0.0725;
abadLinkLength = 0.1329;
hipLinkLength = 0.226;
kneeLinkLength = 0.268;
L = [0.5*bodyLength, 0.5*bodyWidth, abadLinkLength, hipLinkLength, kneeLinkLength];
p = [0, -abadLinkLength, -0.35];
q = InverseKinematics(p, 0, L);
q
