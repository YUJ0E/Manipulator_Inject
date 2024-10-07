%% Initialization

% Clear
clc;
clear;
close all;

% Load
openExample('urseries/MotionPlanningRBTSimulationUR5eBinPickingManipulatorRRTExample');
ur5eRBT = loadrobot('universalUR5e','DataFormat','row');
ur5e = exampleHelperAddGripper(ur5eRBT);

%% Task 2
% Model the manipulator using the classical D-H representation of UR5e

% Specify the parameters for the UR5e as a matrix.
a = [0, -0.42500, -0.3922, 0, 0, 0]';
d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996]';
alpha = [pi/2, 0, 0, pi/2, -pi/2, 0]';
theta = zeros(6,1);
dhparams = [a,alpha,d,theta];

% Create a rigid body tree object
robot = rigidBodyTree;

% Add DH parameters into robot
bodies = cell(6,1);
joints = cell(6,1);
for i = 1:6
    bodies{i} = rigidBody(['body' num2str(i)]);
    joints{i} = rigidBodyJoint(['jnt' num2str(i)],"revolute");
    setFixedTransform(joints{i},dhparams(i,:),"dh");
    bodies{i}.Joint = joints{i};
    if i == 1   % Add first body to base
        addBody(robot,bodies{i},"base")
    else   % Add current body to previous body by name
        addBody(robot,bodies{i},bodies{i-1}.Name)
    end
end

% Show original robot model
showdetails(robot)
show(robot);
title("UR5e Original Robot Model");
disp(bodies)

% Show robot model
figure;
show(ur5e);
title("UR5e Robot Model");

%% Task 3
% Derivate the forward kinematics

% Initial transformation matrix
T=zeros(4,4);

% Obtain the total transformation matrix
for i=1:6
    T(:,:,i) = [cos(theta(i))  -sin(theta(i))*cos(alpha(i))    sin(theta(i))*sin(alpha(i))    a(i)*cos(theta(i));
                sin(theta(i))   cos(theta(i))*cos(alpha(i))   -cos(theta(i))*sin(alpha(i))    a(i)*sin(theta(i));
                     0                sin(alpha(i))                   cos(alpha(i))                  d(i);
                     0                      0                              0                          1];
end
T06 = T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*T(:,:,6);

% T06 is the forward kinematic matrix
disp('Total Forward Kinematics Transformation Matrix (T06):');
disp(T06);

%% Task 4
% Solve the analytic inverse kinematics problem, which involves finding the set of joint configurations that satisfy the desired position and orientation of the final link

% Target position
targetPosition = [0.5, -0.3, 0.6];
targetOrientation = [1 0 0 0];

% Create an inverse kinematics solver
ik = robotics.InverseKinematics('RigidBodyTree', robot);

% Create target pose
tform = trvec2tform(targetPosition) * quat2tform(targetOrientation); 

% Specify parameters for inverse kinematics
weights = [0.25 0.25 0.25 1 1 1]; % Weight of joint position and attitude
initialGuess = robot.homeConfiguration; % Initial joint angle guess

% Solving Inverse Kinematics
[configSoln,solnInfo] = ik(robot.BodyNames{end}, tform, weights, initialGuess);

% Print joint solutions
for i = 1:length(configSoln)
    fprintf('Joint %d: %f rad\n', i, configSoln(i).JointPosition);
end

% Define home position
homePosition = zeros(1, length(configSoln));  
for i = 1:length(configSoln)
    homePosition(i) = configSoln(i).JointPosition;
end

% Set home position of each joint
ur5e.Bodies{1, 3}.Joint.HomePosition = homePosition(1);
ur5e.Bodies{1, 4}.Joint.HomePosition = homePosition(2);
ur5e.Bodies{1, 5}.Joint.HomePosition = homePosition(3);
ur5e.Bodies{1, 6}.Joint.HomePosition = homePosition(4);
ur5e.Bodies{1, 7}.Joint.HomePosition = homePosition(5);
ur5e.Bodies{1, 8}.Joint.HomePosition = homePosition(6);

% Show robot at home position
f1 = figure;
show(ur5e,homePosition,'Frames','on','PreservePlot',false,'Collisions','off','Visuals','on');
title("UR5e Robot Model with Inverse Kinematics");
hold on

%% Task 5
% Define an upright 3D plane resembling the back of a human body in your workspace, and identify 8 target points on the plane

% Plane parameters (simplified representation of the human back, meters)
planeY = 0.6; % Y distance between the plane and the robot base 
torsoLength = 0.525; % Torso length
shoulderWidth = 0.45; % Shoulder width
waistWidth = 0.35; % Waist circumference

% Define target point
% Assume that the bottom of the plane is flush with the robot base plane and located in front of the robot
zBase = 0.1; 
xCenter = 0;

% Corner points of the plane
planePointsX = [xCenter - shoulderWidth/2, xCenter + shoulderWidth/2, xCenter + shoulderWidth/2, xCenter - shoulderWidth/2];
planePointsZ = [zBase + torsoLength, zBase + torsoLength, zBase, zBase];
planePointsY = [planeY, planeY, planeY, planeY]; 

% Target Points
targetPoints = zeros(8, 3);
targetPoints(1, :) = [xCenter - shoulderWidth/2, planeY, zBase + torsoLength]; % Above the left shoulder
targetPoints(2, :) = [xCenter + shoulderWidth/2, planeY, zBase + torsoLength]; % Above the right shoulder
targetPoints(3, :) = [xCenter - waistWidth/2, planeY, zBase]; % Left waist
targetPoints(4, :) = [xCenter + waistWidth/2, planeY, zBase]; % Right waist
targetPoints(5, :) = [xCenter, planeY, zBase + torsoLength*3/4]; % Upper spine
targetPoints(6, :) = [xCenter, planeY, zBase + torsoLength/2]; % Middle spine
targetPoints(7, :) = [xCenter, planeY, zBase + torsoLength/4]; % Lower spine
targetPoints(8, :) = [xCenter, planeY, zBase + torsoLength]; % Bottom of the neck

% Draw plane with robot
figure;
show(ur5e, homePosition, 'Frames', 'off', 'PreservePlot', false);
hold on;
fill3(planePointsX, planePointsY, planePointsZ, [0.9 0.9 0.9], 'FaceAlpha', 0.5); 

% Draw Points
plot3(targetPoints(:,1), targetPoints(:,2), targetPoints(:,3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

% Set image properties
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
grid on;
axis equal;
view(3);
title('Upright 3D Plane with Target Points');

hold off;