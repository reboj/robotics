function ass = assignment2(inputTop, inputBottom, inputPCB, leftOrigin, rightOrigin, AssemblyPos)
%% Input position
close all;
clc;
startup_rvc;
set(0,'DefaultFigureWindowStyle','docked')
baseOrigin = [0, 0, 0.31];
%% Arm Locations 

%% Plot Arms and objects (Table, Safety fence, assembly parts) 
SAWYER1 = SAWYER;
SAWYER1.model.base = transl(baseOrigin); 
close all
PlotAndColourRobot(SAWYER1);
hold on; 
q = zeros(1,7); %initial robot orientation
% SAWYER1.model.teach
loadObject('tableandDesktop(1).ply',[0.5, -0.55, -0.09]) %load objects in specified position
hold on;
loadObject('table(arm)(2).ply',[0, 0.35, -0.395])
hold on;
loadObject('PCtower(2).ply',[-0.75, -0.5, 0.175])
hold on;
loadObject('mouse(2).ply',[0.65, -0.25, 0])
hold on;
loadObject('coffeeMug1.ply',[0.65, -0.5, 0.05])
hold on;
loadObject('USBCMale.ply',[0, 1, 0.41])
hold on;
% loadObject('HDMI(female)(1).ply',[-0.55, 0.1, 0])
% hold on;
% loadObject('HDMI(1).ply',[-0.7, 0.2, 0])
% hold on;
loadObject('keyboard3.ply',[0, -0.35, 0.05])
hold on;
loadObject('EmergencyLight.ply',[0.25, 1, 0.31])
hold on;
loadObject('emergencyStop.ply',[-0.2, 1, 0.31])
hold on;
loadObject('safetyFence1.ply',[1.5, -1.25, -0.4])
hold on;
loadObject('safetyFence3.ply',[1.5, 1.75, -0.4])
hold on;
% loadObject('safetyFence1.ply',[0.5, -1.25, -0.4])
% hold on;
% loadObject('safetyFence3.ply',[0.5, 1.75, -0.4])
% hold on;
loadObject('safetyFence90d1.ply',[2.5, -0.5, -0.4])
hold on;
loadObject('safetyFence90d2.ply',[2.5, 1, -0.4])
hold on;
% loadObject('safetyFence90d1.ply',[1.5, -0.5, -0.4])
% hold on;
% loadObject('safetyFence90d2.ply',[1.5, 1, -0.4])
% hold on;
loadObject('safetyFence90d3.ply',[-1.75, -0.5, -0.4])
hold on;
loadObject('safetyFence90d4.ply',[-1.75, 1, -0.4])
hold on;
loadObject('safetyFence2.ply',[-1, -1.25, -0.4])
hold on;
loadObject('safetyFence4.ply',[-1, 1.75, -0.4])
hold on;
% loadObject('safetyFence90d3.ply',[-2.25, -0.5, -0.4])
% hold on;
% loadObject('safetyFence90d4.ply',[-2.25, 1, -0.4])
% hold on;
% loadObject('safetyFence2.ply',[-1.5, -1.25, -0.4])
% hold on;
% loadObject('safetyFence4.ply',[-1.5, 1.75, -0.4])
% hold on;
input('Prepare viewpoints for movements');
%% Task 3 Robot movement


%% Given an end effector pose, determine a joint state
%clear all;
%clc;  
%establish object
%rightUR3 = UR3;
%rightUR3.model.teach
%adjust the position to the appropriate model, or if there is a given matrix
%q = rightUR3.model.getpos();

%The end effector pose is defined as a 4x4 homogeneous transform
% to go further - feed the getpos value into fkine
%JointState = rightUR3.model.ikine(q);
%hold on;
%input('Continue: move robot to required joint');
%% Move robot to required joint states and demonstrate that the joint states satisfy the given pose
% clear all;
% clc;
% leftUR3 = UR3;
% % q = [6x1] matrix
% animate(leftUR3.model,[pi/2,-pi/2,pi/2,pi/2,pi/2,pi/2]); % This is quick method to see the robot position itsef
% 
% %Move the robot required joint states and demonstrate that the joint states
% %satisfy the given pose.
% %Step1: feed the given joint states into either animate or jtrai. 
% %Step2: Run robot teach leftUR3.model.teach
% %Step3: Run getpos() leftUR3.model.getpos()
% 
% %Extra initial step (x,y,z), ikcon is best used
% q = leftUR3.model.ikcon(transl(1,1,1));

