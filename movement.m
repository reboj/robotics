function  movement(sawyer,hdmi_m1,hdmi_m2)
%% Movement function is Sawyer's movement in the workspace using Trapizoidal 
%FLAGS
portFlag = 0;
hdmiFlag = 0;
methodFlag = 1;
collisionFlag = 0;  

%% Collision
if collisionFlag == 1
    if portFlag == 0
centerpnt = [-1.4, -0.4, 1];
    else
centerpnt = [-1.4, -0.5, 1];   
    end
    
side = 0.7;
plotOptions.plotFaces = true;
[vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
end 

%% velocity and Resolved Rate Motion Control.
%Time Step
t = 5;                                                                      % Total time (s)
deltaT = 0.1;                                                               % Control frequency
steps = t/deltaT; 

%Desired Joint Angles
q_start = zeros(1,7);
p_start  = sawyer.model.fkine(q_start);

%Choose which Port
if portFlag == 1
%PORT B -- Desired position prior to RMRC
p_end = transl([-1.05, -0.5, 1.115])*troty(pi/2);    
else
%PORT A -- Desired position prior to RMRC
p_end = transl([-1.05, -0.4, 1.115])*troty(pi/2); 
end
q_end    = sawyer.model.ikcon(p_end);

%Choose with hdmi Object
if hdmiFlag == 1
hdmi = hdmi_m2;
else
hdmi = hdmi_m1;
end



if methodFlag == 0
%%%%%%%INSERT CABLE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Generate Trajectory from q_start to q_end using Trapizoidal Velocity.
s = lspb(0,1,steps);
q_matrix = zeros(steps,7);
for i = 1:steps
    q_matrix(i,:) = (1-s(i))*q_start + s(i)*q_end;                          %Generate trajectory from q_start to q_end.
    sawyer.model.animate(q_matrix(i,:));
    drawnow();
    objMove = sawyer.model.fkine(q_matrix(i,:));
    hdmi.move(objMove);
    if collisionFlag == 1
    result(i) = IsCollision(sawyer.model,q_matrix(i,:),faces,vertex,faceNormals,false);
    end
    
    end

%Move Sawyer with RMRC.
q_entry = q_matrix(end,:);
position = sawyer.model.fkine(q_entry);

 for i = 1:steps
    q_current = sawyer.model.ikcon(position);
    J         = sawyer.model.jacob0(q_current);
    desired_V = [0.05 0 0  0  0  0 ]';                                      %[vx,vy,vz,ax,ay,az]
    q_dot     = pinv(J) * desired_V;
    q_k       = q_current + (q_dot*deltaT)';
    sawyer.model.animate(q_k);
    drawnow(); 
    position  = sawyer.model.fkine(q_k);
    hdmi.move(position);
    if collisionFlag == 1
    result(i) = IsCollision(sawyer.model,q_matrix(i,:),faces,vertex,faceNormals,false);
    end
    q_k_Matrix(i,:) = q_k;
    
 end

 %Move Sawyer with RMRC.

position = sawyer.model.fkine(q_k_Matrix(end,:));
 for i = 1:steps
    q_current = sawyer.model.ikcon(position);
    J         = sawyer.model.jacob0(q_current);
    desired_V = [-0.05 0 0  0  0  0 ]';                                      %[vx,vy,vz,ax,ay,az]
    q_dot     = pinv(J) * desired_V;
    q_k       = q_current + (q_dot*deltaT)';
    sawyer.model.animate(q_k);
    drawnow(); 
    position  = sawyer.model.fkine(q_k);
    if collisionFlag == 1
    result(i) = IsCollision(sawyer.model,q_matrix(i,:),faces,vertex,faceNormals,false);
    end    
    q_k_Matrix2(i,:) = q_k;
    
 end
 
 
s = lspb(0,1,steps);
q_matrix = zeros(steps,7);
q_end = q_k_Matrix2(end,:);
for i = 1:steps
    q_matrix(i,:) = (1-s(i))*q_end + s(i)*q_start;                          %Generate trajectory from q_start to q_end.
    sawyer.model.animate(q_matrix(i,:));
    drawnow();
    objMove = sawyer.model.fkine(q_matrix(i,:));
    if collisionFlag == 1
    result(i) = IsCollision(sawyer.model,q_matrix(i,:),faces,vertex,faceNormals,false);
    end    
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
else
%%%%%%EJECT CABLE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Generate Trajectory from q_start to q_end using Trapizoidal Velocity.
s = lspb(0,1,steps);
q_matrix = zeros(steps,7);
for i = 1:steps
    q_matrix(i,:) = (1-s(i))*q_start + s(i)*q_end;                          %Generate trajectory from q_start to q_end.
    sawyer.model.animate(q_matrix(i,:));
    drawnow();
    objMove = sawyer.model.fkine(q_matrix(i,:));
    if collisionFlag == 1
    result(i) = IsCollision(sawyer.model,q_matrix(i,:),faces,vertex,faceNormals,false);
    end    
end

%Move Sawyer with RMRC.
q_entry = q_matrix(end,:);
position = sawyer.model.fkine(q_entry);

 for i = 1:steps
    q_current = sawyer.model.ikcon(position);
    J         = sawyer.model.jacob0(q_current);
    desired_V = [0.05 0 0  0  0  0 ]';                                      %[vx,vy,vz,ax,ay,az]
    q_dot     = pinv(J) * desired_V;
    q_k       = q_current + (q_dot*deltaT)';
    sawyer.model.animate(q_k);
    drawnow(); 
    position  = sawyer.model.fkine(q_k);   
    q_k_Matrix(i,:) = q_k;
    if collisionFlag == 1
    result(i) = IsCollision(sawyer.model,q_matrix(i,:),faces,vertex,faceNormals,false);
    end    
 end

 %Move Sawyer with RMRC.

position = sawyer.model.fkine(q_k_Matrix(end,:));
 for i = 1:steps
    q_current = sawyer.model.ikcon(position);
    J         = sawyer.model.jacob0(q_current);
    desired_V = [-0.05 0 0  0  0  0 ]';                                      %[vx,vy,vz,ax,ay,az]
    q_dot     = pinv(J) * desired_V;
    q_k       = q_current + (q_dot*deltaT)';
    sawyer.model.animate(q_k);
    drawnow(); 
    position  = sawyer.model.fkine(q_k);
    q_k_Matrix2(i,:) = q_k;
    hdmi.move(position);
    if collisionFlag == 1
    result(i) = IsCollision(sawyer.model,q_matrix(i,:),faces,vertex,faceNormals,false);
    end    
 end
  
s = lspb(0,1,steps);
q_matrix = zeros(steps,7);
q_end = q_k_Matrix2(end,:);
for i = 1:steps
    q_matrix(i,:) = (1-s(i))*q_end + s(i)*q_start;                          %Generate trajectory from q_start to q_end.
    sawyer.model.animate(q_matrix(i,:));
    drawnow();
    objMove = sawyer.model.fkine(q_matrix(i,:));
    hdmi.move(objMove);
    if collisionFlag == 1
    result(i) = IsCollision(sawyer.model,q_matrix(i,:),faces,vertex,faceNormals,false);
    end
end

end
end

