%% Kinematic Model Class - GRAAL Lab
classdef cartesianControl < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        k_a
        k_l
    end

    methods
        % Constructor to initialize the geomModel property
        function self = cartesianControl(gm,angular_gain,linear_gain)
            if nargin > 2
                self.gm = gm;
                self.k_a = angular_gain;
                self.k_l = linear_gain;
            else
                error('Not enough input arguments (cartesianControl)')
            end
        end

        %% getCartesianReference function
        function [x_dot] = getCartesianReference(self, bTg)
        % Compute the Cartesian error between the current end-effector frame and the goal frame
        bTt = self.gm.getToolTransformWrtBase();
    
        % Position error
        r_error = bTg(1:3, 4) - bTt(1:3, 4);
    
        % Compute relative rotation matrix
        tRg_e = bTt(1:3, 1:3)' * bTg(1:3, 1:3);
         [U, ~, V] = svd(tRg_e); % Ensure orthogonality
         tRg_e = U * V'; 
    
        % Extract orientation error (as angle-axis representation)
        [h_error,theta_error] = RotToAngleAxis(tRg_e) ;
        rho_e = h_error *theta_error ;
        b_rho_e = bTt(1:3 , 1:3) *rho_e ;
    
        % Combine errors
        e = [b_rho_e; r_error];
    
        % Gain matrix
        Lambda = diag([self.k_a, self.k_a, self.k_a, self.k_l, self.k_l, self.k_l]);
    
        % Compute the desired velocities with feedforward term
        x_dot = Lambda * e ;
        end
    end
end

