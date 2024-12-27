function [R] = YPRToRot(varargin)
% The function computes the rotation matrix using the YPR (yaw-pitch-roll)
% convention, given psi, theta, phi or a single 3D input vector.
% Input:
% - Three scalars: psi (yaw), theta (pitch), phi (roll), OR
% - One 3D vector [psi, theta, phi]
% Output:
% - R: rotation matrix

%% Input Handling
if nargin == 1
    % Single input case: 3D vector
    angles = varargin{1};
    if ~isvector(angles) || length(angles) ~= 3
        error('If providing one argument, it must be a 3-element vector [psi, theta, phi].');
    end
    psi = angles(1);
    theta = angles(2);
    phi = angles(3);
elseif nargin == 3
    % Three scalar inputs
    psi = varargin{1};
    theta = varargin{2};
    phi = varargin{3};
else
    error('YPRToRot function takes either one 3D vector or three scalar inputs.');
end

%% Checking the input arguments
if any(abs([psi, theta, phi]) > 2*pi)
    error('All input angles must be in radians and within [-2*pi, 2*pi].');
end

%% Preparing quantities for the Rotation matrix
aRb_11 = cos(psi) * cos(theta);
aRb_12 = -sin(psi) * cos(phi) + cos(psi) * sin(theta) * sin(phi);
aRb_13 = sin(psi) * sin(phi) + cos(psi) * cos(phi) * sin(theta);
aRb_21 = sin(psi) * cos(theta);
aRb_22 = cos(psi) * cos(phi) + sin(phi) * sin(theta) * sin(psi);
aRb_23 = -cos(psi) * sin(phi) + sin(theta) * sin(psi) * cos(phi);
aRb_31 = -sin(theta);
aRb_32 = cos(theta) * sin(phi);
aRb_33 = cos(theta) * cos(phi);

%% Constructing the Rotation matrix from Euler angles
R = [aRb_11, aRb_12, aRb_13;
     aRb_21, aRb_22, aRb_23;
     aRb_31, aRb_32, aRb_33];
end
