close all;
clc;
clear all
startup_rvc;
set(0,'DefaultFigureWindowStyle','docked')
SAWYER1 = SAWYER;
baseOrigin = [0, 0, 1.06];
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
%%Test Collision 
input('this is forced collision simulated working area')
qlim = SAWYER1.model.qlim;

q= zeros(1,7);
steps = 20;
HDMIMale = [0, 0.5, 1.16]; %HDMI positions hardcoded for the mean time
HDMIFemale = [-0.6, -0.3, 1.13]; %HDMI positions hardcoded for the mean time

point1 = transl(HDMIMale);
point2 = transl(HDMIFemale);
q1 = SAWYER1.model.ikcon(point1); 
q2 = SAWYER1.model.ikcon (point2);


centerpnt = [0.25,-0.5,1]; % Center point of the testing cube
side = 0.5; % Side of the testing cube
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
steps = 20;
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

function result = IsCollision(sawyer,qMatrix,faces,vertex,faceNormals,returnOnceFound)
if nargin < 6
    returnOnceFound = true;
end
result = false;

for qIndex = 1:size(qMatrix,1)
   % Get the transform of every joint (i.e. start and end of every link)
    tr = GetLinkPoses(qMatrix(qIndex,:), sawyer);

    % Go through each link and also each triangle face
    for i = 1 : size(tr,3)-1    
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
               plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                display('Intersection');
                result = true;
                if returnOnceFound
                    return
                end
            end
        end    
    end
end
end

%% GetLinkPoses
% q - sawyer joint angles
% sawyer -  seriallink sawyer model
% transforms - list of transforms
function [ transforms ] = GetLinkPoses( q, sawyer)

links = sawyer.model.links;
transforms = zeros(4, 4, length(links) + 1);
transforms(:,:,1) = sawyer.model.base;

for i = 1:length(links)
    L = links(1,i);
    
    current_transform = transforms(:,:, i);
    
    current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
    transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
    transforms(:,:,i + 1) = current_transform;
end
end
%%
function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

u = triangleVerts(2,:) - triangleVerts(1,:);
v = triangleVerts(3,:) - triangleVerts(1,:);

uu = dot(u,u);
uv = dot(u,v);
vv = dot(v,v);

w = intersectP - triangleVerts(1,:);
wu = dot(w,u);
wv = dot(w,v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1;                      % intersectP is in Triangle
end