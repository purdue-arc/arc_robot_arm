%% Define DH Parameters
links = [.05801,0.12015,0.1330,0.097766];
links_urdf = [.056179,0.12015,0.11794,0.097766];
theta = deg2rad([0;0;0;0]);
alpha = deg2rad([90;180;0;-90]);
d = [links(1);0;0;0];
r = [0;links(2);links(3);links(4)];
DH_Table = [theta,d,r,alpha];

% Joint values for testing
q = [-3.11037329855, 0.775891775412, -2.52880388348, -1.1049898623];
q2 = [-3.14159265359, -0.6478029842,  0.109733991066,-0.0795170559746];
home = deg2rad([0;90;90;0]);
home_vec = [-0.186;-0.138;0.247];

%% Pose Transformation Matrices for Testing

orient_home = [0,0,0,1];
rot_mat_home = quat2rotm(orient_home);

home_T = [rot_mat_home, home_vec; 0,0,0,1];

% Test Pose Transf Matrix #1
pos_vec_1 = [-0.2;0.15;-0.1];
orient_vec_1 = [0,0,0,1];
rot_mat_1 = quat2rotm(orient_vec_1);

T1 = [rot_mat_1, pos_vec_1; 0,0,0,1]

% Test Pose Transf Matrix #2
pos_vec_2 = [-0.145075699259;-0.121377287933;0.154914489919];
orient_vec_2 = [0.594215535732,0.545977743754,0.590514487116, 0.0104326854653];
rot_mat_2 = quat2rotm(orient_vec_2);

T2 = [rot_mat_2, pos_vec_2; 0,0,0,1];

%% Initialize Robot + IK + FK
robot = SerialLink(DH_Table);
% IK
q1_IK = robot.ikine(T1, 'mask',[1, 1, 1, 0, 0, 0],'ilimit',1000)
home_IK = robot.ikine(home_T, 'mask',[1, 1, 1, 0, 0, 0],'ilimit',1000);


%% Plot
%figure(1)

robot.plot(deg2rad([180,90,90,0]),'delay', 4)
%robot.plot(home_IK,'delay', 4)
robot.plot(q1_IK,'delay', 4)
%robot.plot(deg2rad(rad2deg(q1_IK)),'delay', 4)
%robot.plot(q1_IK_trans,'delay', 4)
%T = robot.fkine(q1_IK_trans)
%T = robot.fkine(q1_IK);

% Transforms to actual robot
q1_IK_trans = q1_IK;
q1_IK_trans(1) = q1_IK_trans(1) - pi();
q1_IK_trans(2) = q1_IK_trans(2) - pi()/2;
q1_IK_trans(3) = q1_IK_trans(3) - pi()/2;
q1_IK_trans(4) = q1_IK_trans(4);

disp(wrapToPi(q1_IK_trans))
