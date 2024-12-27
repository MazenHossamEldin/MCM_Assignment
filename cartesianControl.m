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

        % Orientation error (using Euler angles)
        current_rho = rotm2eul(bTt(1:3, 1:3));
        goal_rho = rotm2eul(bTg(1:3, 1:3));
        rho_error = goal_rho' - current_rho';

        % Combine errors
        e = [r_error; rho_error];

        % Apply proportional control gains
        x_dot = [self.k_a * e(4:6); self.k_l * e(1:3)];
    end

    % function [x_dot] = getCartesianReference(self, bTg)
    %     % Compute the Cartesian error between the current end-effector frame and the goal frame
    %     bTt = self.gm.getToolTransformWrtBase();
    % 
    %     % Position error
    %     position_error = bTg(1:3, 4) - bTt(1:3, 4);
    % 
    %     % Orientation error (using Euler angles)
    %     current_orientation = rotm2eul(bTt(1:3, 1:3));
    %     goal_orientation = rotm2eul(bTg(1:3, 1:3));
    %     orientation_error = goal_orientation' - current_orientation';
    % 
    %     % Combine errors
    %     e = [orientation_error; position_error];
    % 
    %     % Generalized Velocity 
    %     nu_desired = [ goal_orientation' ; bTg(1:3,4)] ;
    % 
    %     % Gain matrix
    %     Lambda = diag([self.k_a, self.k_a, self.k_a, self.k_l, self.k_l, self.k_l]);
    % 
    %     % Compute the desired velocities with feedforward term
    %     x_dot = Lambda * e + nu_desired;
    % end
    end
end

