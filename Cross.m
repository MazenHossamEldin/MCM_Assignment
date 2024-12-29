function h_wedge  = Cross(h)
% This function implements the cross product operator defined as a
% skew-symmetric matrix from its axial vector
% Input : axial vector 
% Output : Skew-symmeteric matrix operator
h_wedge = [0 -h(3) h(2) ; h(3) 0 -h(1) ; -h(2) h(1) 0];
end