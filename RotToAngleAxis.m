function [h,theta] = RotToAngleAxis(R)
%EULER REPRESENTATION: Given a tensor rotation matrices this function  output the 
% equivalent angle-axis representation values, respectively 'theta' (angle), 'h' (axis) 

%% Check number of input arguments
if nargin ~= 1
    error(' RotToAngleAxis function takes One input argument') ;
end

%% Check the dimension of the input matrix
if ~isequal(size(R),[3,3])
    error('Rotation matrix input is not a 3x3 matrix') 
end

% % Check Validity of the input matrix as a Rotation matrix 
% if (R*R.' ~= eye(3) )
%     error('The input matrix is not a valid rotation matrix')
% end
 
%% Obtaining the h and theta 
theta = acos((trace(R)-1)/2) ;

if theta == 0
    disp(' From RotToAngleAxis function , regarding the output ')
    disp(' At the case where θ = 0, then h is an arbitrary vector and a Nan vector is returned ')
    h = NaN(3,1);
end

if theta == pi 

    h = zeros(3,1);

    h(1) =  sqrt((R(1,1)+1)/2) ;
    if h(1) ~= 0
        h(2) =  sign(h(1)) * sign(R(1,2)) * sqrt((R(2,2)+1)/2) ;
        h(3) =  sign(h(1)) * sign(R(1,3)) * sqrt((R(3,3)+1)/2) ;
        return
    end

    h(2) = sqrt((R(2,2)+1)/2) ;
    if h(2) ~= 0
         h(3) =  sign(h(2)) * sign(R(2,3)) * sqrt((R(3,3)+1)/2) ;
         return
    end

    h(3) =  sqrt((R(3,3)+1)/2) ;
end

if theta > 0 && theta <pi
    h = 0.5*[ R(3,2)-R(2,3) ; R(1,3)-R(3,1) ; R(2,1)-R(1,2)]/sin(theta) ;
end

end