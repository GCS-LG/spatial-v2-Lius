function  [inertia_]= SpatialInertia(m ,com ,inertia)

cSkew = vectorToSkewMat(com);
inertia_ = zeros(6);
inertia_(1:3, 1:3) = inertia + m * (cSkew * cSkew');
inertia_(1:3, 4:6) = m * cSkew;
inertia_(4:6, 1:3) = m * cSkew';
inertia_(4:6, 4:6) = m * eye(3);

end