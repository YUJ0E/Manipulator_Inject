%% Initialization
clc
clear
close all
openExample('urseries/MotionPlanningRBTSimulationUR5eBinPickingManipulatorRRTExample');
ur5eRBT = loadrobot('universalUR5e','DataFormat','row');
ur5e = exampleHelperAddGripper(ur5eRBT);

%% Define the robot

% Specify the parameters for the UR5e as a matrix.
a = [0, -0.42500, -0.3922, 0, 0, 0]';
d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996]';
alpha = [pi/2, 0, 0, pi/2, -pi/2, 0]';
theta = zeros(6,1);
dhparams = [a,alpha,d,theta];

% Create a rigid body tree object.
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

% Home Position
homePosition = [0, 0, -pi/2, pi/2, -pi/2, 0];

% Set home position of each joint
ur5e.Bodies{1, 3}.Joint.HomePosition = homePosition(1);
ur5e.Bodies{1, 4}.Joint.HomePosition = homePosition(2);
ur5e.Bodies{1, 5}.Joint.HomePosition = homePosition(3);
ur5e.Bodies{1, 6}.Joint.HomePosition = homePosition(4);
ur5e.Bodies{1, 7}.Joint.HomePosition = homePosition(5);
ur5e.Bodies{1, 8}.Joint.HomePosition = homePosition(6);

%% Define the 3D plane and 8 target points on the plane

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

%% Show robot at home position
f1 = figure;
show(ur5e,homePosition,'Frames','on','PreservePlot',false,'Collisions','off','Visuals','on');
hold on

% Limit the Z-axis
zlim([-0.5 1.5]); 
xlim([-1 1]); 
ylim([-1 1]); 

%% Execute parameter

% targetPositions
targetPositions = [
    -0.225, -0.25, 0.625;
    0, -0.25, 0.625;
    0.225, -0.25, 0.625;
    0, -0.25, 0.49375;
    0, -0.25, 0.3625;
    0, -0.25, 0.23125;
    -0.175, -0.25, 0.1;
    0.175, -0.25, 0.1;
    ];

% Fixed target orientation for all positions
targetOrientation = [0.707 0.707 0 0]; % Quaternion
ik = robotics.InverseKinematics('RigidBodyTree', robot);
weights = [0.25 0.25 0.25 1 1 1]; % Weight of joint position and attitude
initialGuess = robot.homeConfiguration;

%% Execution
home_theta_values = homePosition;

for idx=(1:8)
    Final_pos=targetPositions(idx,:);
    tform = trvec2tform(targetPositions(idx, :)) * quat2tform(targetOrientation); 
    Final_Theta = ik(robot.BodyNames{end}, tform, weights, initialGuess);
    final_theta_values = zeros(1, 6);
    for i = 1:6
        final_theta_values(i) = Final_Theta(i).JointPosition;
    end
    step = 100;
        
    % Time parameters
    T = 1; % Assume the time from start to end is 1 second
    t = linspace(0, T, step); % Generate time vector
    
    % Assume initial velocity and acceleration are both zero, and final velocity and acceleration are also zero
    theta_dot_0 = zeros(1, 6); % Initial velocity
    theta_ddot_0 = zeros(1, 6); % Initial acceleration
    theta_dot_f = zeros(1, 6); % Final velocity
    theta_ddot_f = zeros(1, 6); % Final acceleration
    t_f = 1; % Total time for interpolation

    % Compute coefficients of the quintic polynomial
    a0 = home_theta_values;
    a1 = theta_dot_0;
    a2 = theta_ddot_0 / 2;
    a3 = (20*(final_theta_values - home_theta_values) - (8*theta_dot_f + 12*theta_dot_0)*t_f ...
        - (3*theta_ddot_0 - theta_ddot_f)*t_f^2) / (2*t_f^3);
    a4 = (30*(home_theta_values - final_theta_values) + (14*theta_dot_f + 16*theta_dot_0)*t_f ...
        + (3*theta_ddot_0 - 2*theta_ddot_f)*t_f^2) / (2*t_f^4);
    a5 = (12*(final_theta_values - home_theta_values) - 6*(theta_dot_f + theta_dot_0)*t_f ...
        - (theta_ddot_0 - theta_ddot_f)*t_f^2) / (2*t_f^5);
    
    % Use quintic polynomial to compute joint angles at each time point
    Theta = zeros(step, 6); % Initialize joint angle matrix
    for i = 1:step
        t_i = t(i);
        Theta(i,:) = a0 + a1*t_i + a2*t_i^2 + a3*t_i^3 + a4*t_i^4 + a5*t_i^5;
    end
    
    Total_inser_time = 5;
    Total_insert_len = 0.1;
    Total_insert_step = 30;
    Inser_Theta = zeros(Total_insert_step, 6);
    Inser_Theta(1, :) = Theta(step, :); % Assume Theta contains the initial joint angles
    dt = Total_inser_time / Total_insert_step;
    Insert_Start_point = Final_pos; % Assume Final_pos is the starting position of the end effector

    for i = 1:Total_insert_step-1
        % Update the y-coordinate, assuming movement only in the y direction
        Insert_End_point = Insert_Start_point;  % Copy the current point
        Insert_End_point(2) = Insert_End_point(2) + Total_insert_len / Total_insert_step;    
        % Call the Insert function
        Inser_Theta(i+1, :) = Insert(ur5e, Insert_Start_point, Insert_End_point, Inser_Theta(i,:), dt);
        % Update the insertion starting point
        Insert_Start_point = Insert_End_point;     
    end

    % Show the Video

    rateObj = rateControl(20);
    rateinst= rateControl(1/dt);

    for i = 1 : size(Theta)
        show(ur5e,Theta(i,:),'PreservePlot',false,'Frames','off','Collisions','off','Visuals','on','FastUpdate',true);
        drawPlaneAndPoints(planePointsX, planePointsY, planePointsZ, targetPoints); % Plot the plane and target points
        drawnow
        waitfor(rateObj);
    end
    pause(1);

    for i = 1 : size(Inser_Theta)
        show(ur5e,Inser_Theta(i,:),'PreservePlot',false,'Frames','off','Collisions','off','Visuals','on','FastUpdate',true);
        drawPlaneAndPoints(planePointsX, planePointsY, planePointsZ, targetPoints); % Plot the plane and target points
        drawnow
        waitfor(rateinst);
    end
    pause(1);

    for i = size(Inser_Theta) :-1: 1
        show(ur5e,Inser_Theta(i,:),'PreservePlot',false,'Frames','off','Collisions','off','Visuals','on','FastUpdate',true);
        drawPlaneAndPoints(planePointsX, planePointsY, planePointsZ, targetPoints); % Plot the plane and target points
        drawnow
        waitfor(rateinst);
    end
    pause(1);

    home_theta_values = final_theta_values;

end

%% Draw the plane and points
function drawPlaneAndPoints(planePointsX, planePointsY, planePointsZ, targetPoints)
    fill3(planePointsX, planePointsY, planePointsZ, [0.9 0.9 0.9], 'FaceAlpha', 0.5); 
    hold on;
    plot3(targetPoints(:,1), targetPoints(:,2), targetPoints(:,3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
end

%% Jacobian Method
function [q] = Insert(ur5e, Start_point, End_point, init_q, dt)   
    % Calculate displacement vector
    displacement = (End_point - Start_point) / dt;
    % Compute the Jacobian matrix
    J = geometricJacobian(ur5e, init_q, ur5e.BodyNames{end});      
    J_inv = pinv(J(4:6, :));  % Invert the Jacobian for the last three rows
    % Displacement should be a column vector
    d = displacement';
    % Inverse kinematics calculation for dq
    dq = J_inv * d;  % Note that the displacement should be a column vector
    % Update joint angles
    q = init_q + dq'*dt;   
end





