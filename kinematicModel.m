%% Kinematic Model Class - GRAAL Lab
classdef kinematicModel < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        J    % Jacobian
    end

    methods
        % Constructor to initialize the geomModel property
        function self = kinematicModel(gm)
            if nargin > 0
                self.gm = gm;
                self.J = zeros(6, self.gm.jointNumber);
                self.
            else
                error('Not enough input arguments (geometricModel)')
            end
        end
        function updateJacobian(self)
        %% Update Jacobian function
        % The function update:
        % - J: end-effector jacobian matrix

        % Extract the position vector of the end-effector 
        bTe = self.gm.getTransformWrtBase(self.gm.jointNumber) ;  %Transformation matrix from end-effector to base 
        bre = bTe(1:3,4) ;                                                                       % End-effector position in the base frame

        for i=1:self.gm.jointNumber
            
            % Getting the transformation matrix form link i to base
            bTi = self.gm.getTransformWrtBase(i);                       
            % Position vector of the i-th joint relative to the base 
            bri = bTi(1:3,4);
            % Rotation matrix of the i-th joint relative to the base
            iRj = bTi(1:3 , 1:3) ;
            % Axis of motion for the i-th joint (z-axis of joint frame)
            % represented in base frame
            Ki =  iRj * [0 ; 0 ; 1] ;

             % Compute Jacobian components based on joint type
            if self.gm.jointType(i) == 0 % Revoulte joint
                % Computing Angular Velocity Jacobian
                self.J(1:3,i) = Ki ;
                % Position vector of the i-th joint to the end-effector 
                ire = bre - bri ;
                % Cross function is a function that returns cross product operator defined as a skew-matrix operator
                ire_cross = Cross(ire).' ;   % Tanspose of skew-matrix = negative the skew-matrix
                % Computing Linear Velocity Jacobian
                self.J(4:6,i) = ire_cross * (iRj *[0 ; 0 ; 1]);
            elseif self.gm.jointType(i) == 1    % Prismatic joint
                % Angular velocity Jacobian
                self.J(1:3,i) = [0 ; 0 ; 0] ;
                % Linear velocity Jacobian
                self.J(4:6,i) = Ki ;
            end
        end    
        end
        function RBJ =RigidBodyJacobian(self)
            ert = self.gm.eTt(1:3,4) ;
            RBJ = [eye(3)           zeros(3,3)
                        Cross(ert).'    eye(3)      ] ;
        end
        end
end