%% Geometric Model Class - GRAAL Lab
classdef geometricModel < handle
    % iTj_0 is an object containing the trasformations from the frame <i> to <i'> which
    % for q = 0 is equal to the trasformation from <i> to <i+1> = >j>

    % jointType is a vector containing the type of the i-th joint (0 rotation, 1 prismatic)
    % jointNumber is a int and correspond to the number of joints
    % q is a given configuration of the joints
    % iTj is  vector of matrices containing the transformation matrices from link i to link j for the input q.
    % The size of iTj is equal to (4,4,numberOfLinks)
    % eTt (OPTIONAL) add a tool to the model rigid attached to the end-effector
    properties
        iTj_0
        jointType
        jointNumber
        iTj
        q
        eTt
    end

    methods
        % Constructor to initialize the geomModel property
        function self = geometricModel(iTj_0,jointType,eTt)
            if nargin > 1
                 if ~exist('eTt','var')
                     % third parameter does not exist, so default it to something
                      eTt = eye(4);
                 end
                self.iTj_0 = iTj_0;
                self.iTj = iTj_0;
                self.jointType = jointType;
                self.jointNumber = length(jointType);
                self.q = zeros(self.jointNumber,1);
                self.eTt = eTt;
            else
                error('Not enough input arguments (iTj_0) (jointType)')
            end
        end

        function updateDirectGeometry(self, q)
            %%% GetDirectGeometryFunction
            % This method update the matrices iTj.
            % Inputs: q : joints current position ;

            % The function updates:
            % - iTj: vector of matrices containing the transformation matrices from link i to link j for the input q.
            % The size of iTj is equal to (4,4,numberOfLinks)

             % Check if the input joint configuration has the correct length
            if length(q) ~= self.jointNumber
                error('Input joint configuration q must have the same length as the number of joints.');
            end

            for i=1:self.jointNumber

                if self.jointType(i) == 0  % Revolute joint
                    % Compute position vector for revolute joint
                    irj_0 = self.iTj_0(1:3,4,i) ;
                    % Rotation about Z-axis
                    R_z = [ cos(q(i)) -sin(q(i)) 0 
                                 sin(q(i))   cos(q(i)) 0
                                    0               0         1] ;
                    % Extract current rotation matrix
                    iRidash = self.iTj_0(1:3, 1:3, i);
            
                     % Compute new rotation matrix
                    iRj = iRidash * R_z;
                    
                    % Update iTj for revolute joint
                     self.iTj(:, :, i) = [  iRj          irj_0
                                                0 0 0          1       ];
                end
                if self.jointType(i) == 1
                    % Prismatic joint
                    % Extract current rotation matrix (unchanged for prismatic joint)
                    iRj = self.iTj_0(1:3, 1:3, i);

                    % Compute position vector for prismatic joint
                    irj_0 = self.iTj_0(1:3,4,i) +iRj* [0 ; 0 ;  1] * q(i)  ;

                    % Update iTj for prismatic joint
                    self.iTj(:, :, i) = [iRj         irj_0
                                              0 0 0          1   ] ;
                end
            end
        end

        function [bTk] = getTransformWrtBase(self,k)
            %% GetTransformatioWrtBase function
            % Inputs : k: the idx for which computing the transformation matrix
            % outputs : bTk : transformation matrix from the manipulator base to the k-th joint in
            % the configuration identified by iTj.

            if k > self.jointNumber
                error("Value is bigger than the number of joints");
            end
            bTk = eye(4) ;
            for i=1:k
                temp_bTi = self.iTj(:,:,i) ;
                bTk = bTk * temp_bTi ;
                clear temp_bTi ;
            end
        end

        %% getToolTransformWrtBase function
        function [bTt] = getToolTransformWrtBase(self)
            % Inputs : None 
            % bTt : transformation matrix from the manipulator base to the tool

            bTk = eye(4) ;
            for i=1:self.jointNumber
                temp_bTi = self.iTj(:,:,i) ;
                bTe = bTk * temp_bTi ;
                clear temp_bTi ;
            end
            bTt = bTe * self.eTt ;
        end

    end
end


