bodymass = 40;
abadmass = 1;
hipmass = 1;
kneemass = 1;

bodycom = [0.00288,0.00209,0.00209];
abadcom = [-0.0069, 0.00429, -0.0009];
hipcom = [-0.00048, -0.02536, -0.03611];
kneecom = [0.00075, -0.00067, -0.20263];

BodyRotationalInertia = [1, 0, 0; 0, 1, 0; 0, 0, 1] * 1e-6;

fr_abad_RotationalInertia = [1, 0, 0; 0, 1, 0; 0, 0, 1]* 1e-6;
fr_hip_RotationalInertia = [1, 0, 0; 0, 1, 0; 0, 0, 1]* 1e-6;
fr_knee_RotationalInertia = [1, 0, 0; 0, 1, 0; 0, 0, 1]* 1e-6;

fl_abad_RotationalInertia = [1, 0, 0; 0, 1, 0; 0, 0, 1]* 1e-6;
fl_hip_RotationalInertia = [1, 0, 0; 0, 1, 0; 0, 0, 1]* 1e-6;
fl_knee_RotationalInertia = [1, 0, 0; 0, 1, 0; 0, 0, 1]* 1e-6;

hr_abad_RotationalInertia = [1, 0, 0; 0, 1, 0; 0, 0, 1]* 1e-6;
hr_hip_RotationalInertia = [1, 0, 0; 0, 1, 0; 0, 0, 1]* 1e-6;
hr_knee_RotationalInertia = [1, 0, 0; 0, 1, 0; 0, 0, 1]* 1e-6;

hl_abad_RotationalInertia = [1, 0, 0; 0, 1, 0; 0, 0, 1]* 1e-6;
hl_hip_RotationalInertia = [1, 0, 0; 0, 1, 0; 0, 0, 1]* 1e-6;
hl_knee_RotationalInertia = [1, 0, 0; 0, 1, 0; 0, 0, 1]* 1e-6;

bodyInertia = SpatialInertia(bodymass, bodycom, BodyRotationalInertia);

fr_abad_Inertia = SpatialInertia(abadmass, abadcom, fr_abad_RotationalInertia);
fr_hip_Inertia = SpatialInertia(hipmass, hipcom, fr_hip_RotationalInertia);
fr_knee_Inertia = SpatialInertia(kneemass, kneecom, fr_knee_RotationalInertia);

fl_abad_Inertia = SpatialInertia(abadmass, abadcom, fl_abad_RotationalInertia);
fl_hip_Inertia    = SpatialInertia(hipmass, hipcom,    fl_hip_RotationalInertia);
fl_knee_Inertia = SpatialInertia(kneemass, kneecom, fl_knee_RotationalInertia);

hr_abad_Inertia = SpatialInertia(abadmass, abadcom, hr_abad_RotationalInertia);
hr_hip_Inertia    = SpatialInertia(hipmass, hipcom,    hr_hip_RotationalInertia);
hr_knee_Inertia = SpatialInertia(kneemass, kneecom, hr_knee_RotationalInertia);

hl_abad_Inertia = SpatialInertia(abadmass, abadcom, hl_abad_RotationalInertia);
hl_hip_Inertia    = SpatialInertia(hipmass, hipcom,    hl_hip_RotationalInertia);
hl_knee_Inertia = SpatialInertia(kneemass, kneecom, hl_knee_RotationalInertia);

Inertial = {zeros(6) zeros(6) zeros(6) zeros(6) zeros(6) bodyInertia, ...
                fr_abad_Inertia,  fr_hip_Inertia,  fr_knee_Inertia, ...
                fl_abad_Inertia,  fl_hip_Inertia,   fl_knee_Inertia, ...
                hr_abad_Inertia, hr_hip_Inertia, hr_knee_Inertia, ...
                hl_abad_Inertia,  hl_hip_Inertia, hl_knee_Inertia};