dt = 0.001;
bodybase = [0,  0,  0.4];
bodyLength = 1;
bodyWidth = 0.01;
abadLinkLength = 0.01;
hipLinkLength = 0.25;
kneeLinkLength = 0.25;
L = [0.5*bodyLength, 0.5*bodyWidth, abadLinkLength, hipLinkLength, kneeLinkLength];

abadLocation = 0.5 * [bodyLength, bodyWidth, 0];
hipLocation = [0, abadLinkLength, 0];
kneeLocation = [0, 0, -hipLinkLength];