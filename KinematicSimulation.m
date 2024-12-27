%% Kinematic Simulation function
function q = KinematicSimulation(q, q_dot, dt, qmin, qmax)
% Inputs
% - q current robot configuration
% - q_dot joints velocity
% - ts sample time
% - q_min lower joints bound
% - q_max upper joints bound
%
% Outputs
% - q new joint configuration

    % Integrate joint velocities
    q = q + q_dot * dt;
    
    % Ensure joint positions are within bounds
    q = max(qmin, min(qmax, q));
end