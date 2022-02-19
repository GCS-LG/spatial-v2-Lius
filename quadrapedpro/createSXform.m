function [x] = createSXform(R, r)

x_ =zeros(6);
x_(1:3, 1:3) = R;
x_(4:6, 4:6) = R;
x_(4:6, 1:3) = -R * vectorToSkewMat(r);
x = x_;

end