function ass = assignment2()
%% Input position, object and variables
close all;
clc;
clear all
startup_rvc;
set(0,'DefaultFigureWindowStyle','docked')
SAWYER1 = SAWYER;
qlim = SAWYER1.model.qlim;
baseOrigin = [0, 0, 1.06];
steps = 100;

%% Old Ply file locatione, may prove useful in the future
%loadObject('USBCMale.ply',[0, 1, 0.41])
%hold on;
% loadObject('safetyFence1.ply',[0.5, -1.25, -0.4])
% hold on;
% loadObject('safetyFence3.ply',[0.5, 1.75, -0.4])
% hold on;
% loadObject('safetyFence90d1.ply',[1.5, -0.5, -0.4])
% hold on;
% loadObject('safetyFence90d2.ply',[1.5, 1, -0.4])
% hold on;
% loadObject('safetyFence90d3.ply',[-2.25, -0.5, -0.4])
% hold on;
% loadObject('safetyFence90d4.ply',[-2.25, 1, -0.4])
% hold on;
% loadObject('safetyFence2.ply',[-1.5, -1.25, -0.4])
% hold on;
% loadObject('safetyFence4.ply',[-1.5, 1.75, -0.4])
% hold on;
%input('Prepare viewpoints for movements');

%% Plot Arms and objects (Table, Safety fence, assembly parts) 
SAWYER1.model.base = transl(baseOrigin); 
PlotAndColourRobot(SAWYER1);
hold on; 
q = zeros(1,7); %initial robot orientation
loadObject('tableandDesktop(1).ply',[0.5, -0.55, 0.66]) %load objects in specified position
hold on;
loadObject('table(arm)(2).ply',[0, 0.35, 0.355])
hold on;
loadObject('PCtower(2).ply',[-0.75, -0.5, 0.925])
hold on;
loadObject('mouse(2).ply',[0.65, -0.25, 0.75])
hold on;
loadObject('coffeeMug1.ply',[0.75, -0.5, 0.8])
hold on;
loadObject('HDMI(female)(1).ply',[-0.65, -0.3, 1.115])
hold on;
loadObject('HDMI(female)(2).ply',[-0.75, -0.3, 1.115])
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
loadObject('safetyFence90d1.ply',[2.5, -0.5, 0.35])
hold on;
loadObject('safetyFence90d2.ply',[2.5, 1, 0.35])
hold on;
loadObject('safetyFence90d3.ply',[-1.75, -0.5, 0.35])
hold on;
loadObject('safetyFence90d4.ply',[-1.75, 1, 0.35])
hold on;
loadObject('safetyFence2.ply',[-1, -1.25, 0.35])
hold on;
loadObject('safetyFence4.ply',[-1, 1.75, 0.35])
hold on;


input ('robot movement')
%% Old Robot Movement code
% qPtop = SAWYER1.model.ikcon(point1);  
% qBot = SAWYER1.model.ikcon (point2);
% qB2top = jtraj(q,qPtop,steps);
% qB2bot = jtraj(q,qBot,steps);

%% Robot movement
% Initial objects established in the input position, object section

HDMIMale = [0, 0.5, 1.16]; %HDMI positions hardcoded for the mean time
HDMIFemale = [-0.6, -0.3, 1.13]; %HDMI positions hardcoded for the mean time
point1 = transl(HDMIMale);
point2 = transl(HDMIFemale);
q1 = SAWYER1.model.ikcon(point1); 
q2 = SAWYER1.model.ikcon (point2);
qMatrix_0 = jtraj(q,q1,steps); % Moving to different trajectory points
qMatrix_1 = jtraj(q1,q2,steps);

%following for loops for the movment of the end effector to position 
% for loops cut up to prevent the issue of position teleporting
for i=1:steps
    SAWYER1.model.animate(qMatrix_0(i,:));
    drawnow()
end

for i=1:steps
    SAWYER1.model.animate(qMatrix_1(i,:));
    drawnow()
end

input('the following will display velocity and acceleration tables')

% s = lspb(0,1,steps);                   % First, create the scalar function
%         qMatrix_0 = nan(steps,7);
%         qMatrix_1 = nan(steps,7);      % Create memory allocation for variables
%             for i = 1:steps
%                 qMatrix_0(i,:) = (1-s(i))*q + s(i)*q1;    
%                 qMatrix_1(i,:) = (1-s(i))*q1 + s(i)*q2;        
%             end            
velocity = zeros(steps,7);
acceleration  = zeros(steps,7);

for i = 2:steps
    velocity_0(i,:) = qMatrix_0(i,:) - qMatrix_0(i-1,:);
    velocity_1(i,:) = qMatrix_1(i,:) - qMatrix_1(i-1,:); 
    acceleration_0(i,:) = velocity_0(i,:) - velocity_0(i-1,:);
    acceleration_1(i,:) = velocity_1(i,:) - velocity_1(i-1,:); 
end


figure(1)
for i = 1:7
    subplot(4,2,i)
    plot(velocity_0(:,i),'k','LineWidth',1)
%     plot(velocity_1(:,i),'k','LineWidth',5)
    title(['Joint ', num2str(i)])
    xlabel('Step')
    ylabel('Joint Velocity')
end

figure(2)
for i = 1:7
    subplot(4,2,i)
    plot(acceleration_0(:,i),'k','LineWidth',1)
%     plot(acceleration_1(:,i),'k','LineWidth',5)
    title(['Joint ', num2str(i)])
    xlabel('Step')
    ylabel('Joint Acceleration')
end
% SAWYER1.model.plot(qB2top);  
% SAWYER1.model.plot(qB2bot);
% input('Select point view');
% figure(1)
% SAWYER1.model.plot3d(qMatrix); 
% SAWYER1.model.plot3d(q1,'delay',100);  
% SAWYER1.model.plot3d(q2,'delay',100);
% SAWYER1.model.plot(qMatrix,'trail','r-')    

%% Collision detection
%forced collision based on the additional file loaded into place
input('this is forced collision simulated working area')

q = zeros(1,7);

point1 = transl(HDMIMale);
point2 = transl(HDMIFemale);
q1 = SAWYER1.model.ikcon(point1); 
q2 = SAWYER1.model.ikcon (point2);


centerpnt = [0,-1.5,0]; % Center point of the testing cube
side = 1; % Side of the testing cube
plotOptions.plotFaces = true; % Refer to RetangularPrism.m
[vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions); % Passing parameteres of the cube 
axis equal % So that you can actually see the cube in the environment
% Get the transform of every joint (start and end of every link)
tr = zeros(4,4,SAWYER1.model.n+1);
tr(:,:,1) = SAWYER1.model.base;
L = SAWYER1.model.links;
for i = 1 : SAWYER1.model.n
   tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end
% Go through each link and also each triangle face
for i = 1 : size(tr,3)-1    
    for faceIndex = 1:size(faces,1)
        vertOnPlane = vertex(faces(faceIndex,1)',:);
        [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
        if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
            plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
            display('Intersection');
        end
    end    
end
%Go through until there are no step sizes larger than 1 degree/ In the

% q1 = [-pi/4,0,0,0,0,0,0];
% q2 = [pi/4,0,0,0,0,0,0];
% 
steps = 100;
while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q,q1,steps)))),1))
    steps = steps + 1;
end
qMatrix = mtraj(@lspb,q,q1,steps);

% 2.7
result = true(steps,1);
for i = 1: steps
   result(i) = IsCollision(SAWYER1,qMatrix(i,:),faces,vertex,faceNormals,false); %is collision 

    SAWYER1.model.animate(qMatrix(i,:));
    drawnow();
    if result(i) == 1
        disp('Collision Detected!');
        break;
    end
end
% THIS IS THE AREA THAT NEEDS DEBUG
% function result = IsCollision(SAWYER1,qMatrix,faces,vertex,faceNormals,returnOnceFound)
% if nargin < 6
%     returnOnceFound = true;
% end
% result = false;
% % 
% for qIndex = 1:size(qMatrix,1)
%    % Get the transform of every joint (i.e. start and end of every link)
%     tr = GetLinkPoses(qMatrix(qIndex,:), SAWYER1);
% 
%     % Go through each link and also each triangle face
%     for i = 1 : size(tr,3)-1    
%         for faceIndex = 1:size(faces,1)
%             vertOnPlane = vertex(faces(faceIndex,1)',:);
%             [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
%             if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
%                plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
%                 display('Intersection');
%                 result = true;
%                 if returnOnceFound
%                     return
%                 end
%             end
%         end    
%     end
% end
% end
% 
% 
% % q - SAWYER1 joint angles
% % SAWYER1 -  seriallink SAWYER1 model
% % transforms - list of transforms
% function [ transforms ] = GetLinkPoses( q, SAWYER1)
% 
% links = SAWYER1.model.links;
% transforms = zeros(4, 4, length(links) + 1);
% transforms(:,:,1) = SAWYER1.model.base;
% 
% for i = 1:length(links)
%     L = links(1,i);
%     
%     current_transform = transforms(:,:, i);
%     
%     current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
%     transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
%     transforms(:,:,i + 1) = current_transform;
% end
% end
% 
% function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)
% 
% u = triangleVerts(2,:) - triangleVerts(1,:);
% v = triangleVerts(3,:) - triangleVerts(1,:);
% 
% uu = dot(u,u);
% uv = dot(u,v);
% vv = dot(v,v);
% 
% w = intersectP - triangleVerts(1,:);
% wu = dot(w,u);
% wv = dot(w,v);
% 
% D = uv * uv - uu * vv;
% 
% % Get and test parametric coords (s and t)
% s = (uv * wv - vv * wu) / D;
% if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
%     result = 0;
%     return;
% end
% 
% t = (uv * wu - uu * wv) / D;
% if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
%     result = 0;
%     return;
% end
% 
% result = 1;                      % intersectP is in Triangle
% end

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

end