function ass = assignment2(inputTop, inputBottom, inputPCB, leftOrigin, rightOrigin, AssemblyPos)
%% Input position
close all;
clc;
startup_rvc;
set(0,'DefaultFigureWindowStyle','docked')
baseOrigin = [0, 0, 1.06];
%% Arm Locations 

%% Plot Arms and objects (Table, Safety fence, assembly parts) 
SAWYER1 = SAWYER;
SAWYER1.model.base = transl(baseOrigin); 
%close all
PlotAndColourRobot(SAWYER1);
hold on; 
q = zeros(1,7); %initial robot orientation
% SAWYER1.model.teach
loadObject('tableandDesktop(1).ply',[0.5, -0.55, 0.66]) %load objects in specified position
hold on;
loadObject('table(arm)(2).ply',[0, 0.35, 0.355])
hold on;
loadObject('PCtower(2).ply',[-0.75, -0.5, 0.925])
hold on;
loadObject('mouse(2).ply',[0.65, -0.25, 0.75])
hold on;
loadObject('coffeeMug1.ply',[0.65, -0.5, 0.8])
hold on;
%loadObject('USBCMale.ply',[0, 1, 0.41])
%hold on;
loadObject('HDMI(female)(2).ply',[-0.6, -0.3, 1.13])
hold on;
loadObject('HDMIMale(green).ply',[0, 0.5, 1.16])
hold on;
loadObject('keyboard3.ply',[0, -0.35, 0.8])
hold on;
loadObject('EmergencyLight.ply',[0.25, 1, 1.06])
hold on;
loadObject('emergencyStop.ply',[-0.2, 1, 1.06])
hold on;
loadObject('safetyFence1.ply',[1.5, -1.25, 0.35])
hold on;
loadObject('safetyFence3.ply',[1.5, 1.75, 0.35])
hold on;
% loadObject('safetyFence1.ply',[0.5, -1.25, -0.4])
% hold on;
% loadObject('safetyFence3.ply',[0.5, 1.75, -0.4])
% hold on;
loadObject('safetyFence90d1.ply',[2.5, -0.5, 0.35])
hold on;
loadObject('safetyFence90d2.ply',[2.5, 1, 0.35])
hold on;
% loadObject('safetyFence90d1.ply',[1.5, -0.5, -0.4])
% hold on;
% loadObject('safetyFence90d2.ply',[1.5, 1, -0.4])
% hold on;
loadObject('safetyFence90d3.ply',[-1.75, -0.5, 0.35])
hold on;
loadObject('safetyFence90d4.ply',[-1.75, 1, 0.35])
hold on;
loadObject('safetyFence2.ply',[-1, -1.25, 0.35])
hold on;
loadObject('safetyFence4.ply',[-1, 1.75, 0.35])
hold on;
SAWYER1.model.teach;
% loadObject('safetyFence90d3.ply',[-2.25, -0.5, -0.4])
% hold on;
% loadObject('safetyFence90d4.ply',[-2.25, 1, -0.4])
% hold on;
% loadObject('safetyFence2.ply',[-1.5, -1.25, -0.4])
% hold on;
% loadObject('safetyFence4.ply',[-1.5, 1.75, -0.4])
% hold on;
%input('Prepare viewpoints for movements');
%% Task 3 Robot movement
% SAWYER1 = SAWYER;
% close all;
% SAWYER1.model.base = transl([0,0,0]);
% PlotAndColourRobot(SAWYER1); % class function
% % hold on

pos1 = [0, 0.5, 1.16];
pos2 = [-0.6, -0.3, 1.13];
point1 = transl(pos1);
point2 = transl(pos2);
% q= zeros(1,7);
%q= zeros(1,7);
steps = 50;
qlim = SAWYER1.model.qlim;
% qPtop = sawyer.model.ikcon(point1);  
% qBot = sawyer.model.ikcon (point2);
% qB2top = jtraj(q,qPtop,steps);
% qB2bot = jtraj(q,qBot,steps);

q1 = SAWYER1.model.ikcon(point1); 
q2 = SAWYER1.model.ikcon (point2);
qMatrix_0 = jtraj(q,q1,steps);
qMatrix_1 = jtraj(q1,q2,steps);
s = lspb(0,1,steps);                                             	% First, create the scalar function
        qMatrix_1 = nan(steps,7);                                             % Create memory allocation for variables
            for i = 1:steps
                qMatrix_0(i,:) = (1-s(i))*q + s(i)*q1;    
                qMatrix_1(i,:) = (1-s(i))*q1 + s(i)*q2;  
                  
            end
            
velocity = zeros(steps,7);
acceleration  = zeros(steps,7);
for i = 2:steps
    velocity_0(i,:) = qMatrix_0(i,:) - qMatrix_0(i-1,:);
    velocity_1(i,:) = qMatrix_1(i,:) - qMatrix_1(i-1,:); 
    acceleration_0(i,:) = velocity_0(i,:) - velocity_0(i-1,:);
    acceleration_1(i,:) = velocity_1(i,:) - velocity_1(i-1,:); 
end
figure(1)
SAWYER1.model.plot3d(qMatrix_0);             
SAWYER1.model.plot3d(qMatrix_1); 
figure(2)
for i = 1:7
    subplot(4,2,i)
    plot(velocity_0(:,i),'k','LineWidth',1)
%     plot(velocity_1(:,i),'k','LineWidth',5)
    title(['Joint ', num2str(i)])
    xlabel('Step')
    ylabel('Joint Velocity')
end

figure(3)
for i = 1:7
    subplot(4,2,i)
    plot(acceleration_0(:,i),'k','LineWidth',1)
%     plot(acceleration_1(:,i),'k','LineWidth',5)
    title(['Joint ', num2str(i)])
    xlabel('Step')
    ylabel('Joint Acceleration')
end
% sawyer.model.plot(qB2top);  
% sawyer.model.plot(qB2bot);
% input('Select point view');
% figure(1)
% SAWYER1.model.plot3d(qMatrix); 
% SAWYER1.model.plot3d(q1,'delay',100);  
% SAWYER1.model.plot3d(q2,'delay',100);
% SAWYER1.model.plot(qMatrix,'trail','r-')    

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

