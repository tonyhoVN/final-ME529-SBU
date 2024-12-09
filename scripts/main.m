%% Add path of utils folder
clc;clear
addpath("utils\","Kinova_Gen3\")

% Importing the robot URDF file
robot = importrobot('Kinova_Gen3.urdf');
robot.DataFormat = 'column';

% Extract important info of robot 
% Base and EE Frame
baseFrame = 's_frame'; % Replace with actual base link name if different
eeFrame = 'b_frame';

% Screw axis and Tsb0
[S0, B0, M0, jointLimit] = robot_info(robot);
numJoint = 7;

% Joint velocity and accerelation limit 
jointVelLimit = deg2rad(150);
jointAccLimit = deg2rad(700);
