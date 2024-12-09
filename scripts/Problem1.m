%% Add path of utils folder
clc;clear
addpath("utils\","Kinova_Gen3\")

%% ---> Importing the robot URDF file
robot = importrobot('Kinova_Gen3.urdf');
robot.DataFormat = 'column';

%% 1. Screw axis at home config
% Base and EE Frame
baseFrame = 's_frame'; % Replace with actual base link name if different
eeFrame = 'b_frame';

% Links Bodies
bodies = robot.Bodies;

% Initialize screw axes
S0 = [];

% Loop through each joint to calculate the screw axis
for i = 1:length(bodies)
    joint = bodies{i}.Joint;

    if ~strcmp(joint.Type, 'fixed') % Only move joints
        % Transformation from base frame to the joint (child) frame
        T = getTransform(robot, robot.homeConfiguration, bodies{i}.Name, baseFrame);

        % Extract rotation and position
        R = T(1:3, 1:3);
        p = T(1:3, 4);
        
        % Joint axis w.r.t joint frame 
        axis = joint.JointAxis';

        % Joint axis w.r.t base frame
        axisBase = R*axis;
        
        % Compute screw axis 
        if strcmp(joint.Type, 'revolute')
            screw = screw2axis(axisBase,p,0);
        elseif strcmp(joint.Type, 'prismatic')
            screw = screw2axis(axisBase,p,inf);
        end

        % Append to list of Screw axis 
        S0 = [S0, screw];
    end
end

% Tsb at home position from URDF
M0 = getTransform(robot, robot.homeConfiguration, eeFrame, baseFrame);