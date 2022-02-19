I3 = eye(3);

fr_abad_xtree = createSXform(I3, withLegSigns(abadLocation, 0));
fl_abad_xtree = createSXform(I3, withLegSigns(abadLocation, 1));
hr_abad_xtree = createSXform(I3, withLegSigns(abadLocation, 2));
hl_abad_xtree = createSXform(I3, withLegSigns(abadLocation, 3));

fr_hip_xtree = createSXform(Rotation('z', pi), withLegSigns(hipLocation, 0));
fl_hip_xtree = createSXform(Rotation('z', pi), withLegSigns(hipLocation, 1));
hr_hip_xtree = createSXform(Rotation('z', pi), withLegSigns(hipLocation, 2));
hl_hip_xtree = createSXform(Rotation('z', pi), withLegSigns(hipLocation, 3));

fr_knee_xtree = createSXform(I3, withLegSigns(kneeLocation, 0));
fl_knee_xtree = createSXform(I3, withLegSigns(kneeLocation, 1));
hr_knee_xtree = createSXform(I3, withLegSigns(kneeLocation, 2));
hl_knee_xtree = createSXform(I3, withLegSigns(kneeLocation, 3));

Xtree = { eye(6) eye(6) eye(6) eye(6) eye(6) eye(6), ...
                fr_abad_xtree,  fr_hip_xtree,  fr_knee_xtree, ...
               fl_abad_xtree,  fl_hip_xtree,   fl_knee_xtree, ...
              hr_abad_xtree, hr_hip_xtree, hr_knee_xtree, ...
              hl_abad_xtree,  hl_hip_xtree, hl_knee_xtree};
          


function Rot = Rotation(coor, ang)

c = cos(ang);
s = sin(ang);

if(coor == 'x') %rotx
    Rot = [1, 0, 0; 0, c, s; 0, -s, c];
elseif(coor == 'y') %roty
    Rot = [c, 0, -s; 0, 1, 0; s, 0, c];
elseif(coor == 'z') %rotz
    Rot = [c, s, 0; -s, c, 0; 0, 0, 1];
end

end
