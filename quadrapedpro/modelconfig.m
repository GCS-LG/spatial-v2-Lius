bodybase = [0,  0,  0.4];
bodyLength = 0.2225;
bodyWidth = 0.0725;
abadLinkLength = 0.1329;
hipLinkLength = 0.226;
kneeLinkLength = 0.268;
L = [0.5*bodyLength, 0.5*bodyWidth, abadLinkLength, hipLinkLength, kneeLinkLength];

abadLocation = 0.5 * [bodyLength, bodyWidth, 0];
hipLocation = [0, abadLinkLength, 0];
kneeLocation = [0, 0, -kneeLinkLength];