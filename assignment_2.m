function assignment_2()

close all;
clc;
set(0,'DefaultFigureWindowStyle','docked')
%% SAWYER INTIALIZATION AND PLOT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sawyer = SAWYER;
baseOrigin = [-1.875, -0.6, 1.06];
sawyer.model.base = transl(baseOrigin); 
hold on; 

%% PLOT ENVIRONMENT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COORDINATES
f_hdmi1 = [-0.95, -0.4, 1.115]; 
f_h1E   = [-1, -0.4, 1.115];
f_hdmi2 = [-0.95, -0.5, 1.115];
f_h2E   = [-1, -0.5, 1.115];
m_hdmi  = [-1.875, 0.05, 1.16];
f_h3E   = [-1.05, -0.4, 1.115];
loadObject('HDMI(female)(1).ply',f_hdmi1)
hold on;
loadObject('HDMI(female)(2).ply',f_hdmi2)
hold on;
loadObject('HDMIMale(green).ply',m_hdmi)
hold on;
loadObject('tableandDesktop(1).ply',[0.5, -0.55, 0.66]) 
hold on;
loadObject('table(arm)(2).ply',[-1.875, -0.4, 0.355])
hold on;
loadObject('PCtower(2).ply',[-0.75, -0.5, 0.925])
hold on;
%%%DECORATION%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% loadObject('keyboard3.ply',[0, -0.35, 0.8])
% hold on;
% loadObject('EmergencyLight.ply',[0.25, 1, 1.06])
% hold on;
% loadObject('emergencyStop.ply',[-0.2, 1, 1.06])
% hold on;
% loadObject('safetyFence1.ply',[1.5, -1.25, 0.35])
% hold on;
% loadObject('safetyFence3.ply',[1.5, 1.75, 0.35])
% hold on;
% loadObject('safetyFence90d1.ply',[2.5, -0.5, 0.35])
% hold on;
% loadObject('safetyFence90d2.ply',[2.5, 1, 0.35])
% hold on;
% loadObject('safetyFence90d3.ply',[-1.75, -0.5, 0.35])
% hold on;
% loadObject('safetyFence90d4.ply',[-1.75, 1, 0.35])
% hold on;
% loadObject('safetyFence2.ply',[-1, -1.25, 0.35])
% hold on;
% loadObject('safetyFence4.ply',[-1, 1.75, 0.35])
% hold on;
% loadObject('mouse(2).ply',[0.65, -0.25, 0.75])
% hold on;
% loadObject('coffeeMug1.ply',[0.75, -0.5, 0.8])
% hold on;

%% Robotic Movement using RMRC  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Time Intervals
t = 5;                                                                      % Total time (s)
deltaT = 0.1;                                                               % Control frequency
steps = t/deltaT;                                                           % No. of steps for simulation

%Initialization of q joints. 
q_intial = zeros(1,7);
p_start  = sawyer.model.fkine(q_intial);
q_start  = sawyer.model.ikcon(p_start);
p_end    = transl(f_h3E)*troty(pi/2);
q_end    = sawyer.model.ikcon(p_end);

%% Quintic-polynomial Trajectory. 
% q_matrix = jtraj(q_start,q_end,steps);
% for i = 1:steps
% sawyer.model.animate(q_matrix(i,:));
% drawnow();
% end

%%
%Generate Trajectory from q_start to q_end using Trapizoidal Velocity.
s = lspb(0,1,steps);
q_matrix = zeros(steps,7);
for i = 1:steps
    q_matrix(i,:) = (1-s(i))*q_start + s(i)*q_end;                          %Generate trajectory from q_start to q_end.
    sawyer.model.animate(q_matrix(i,:));
    drawnow();
end

%%
%RMRC from entry point to insert point. 
%Initialise
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
 end

%% Safety --- RMRC                                                         
%If simulated safety symbol detected, go to "retreat state".
%Make your own trajectory. Then move. 
% position = sawyer.model.fkine(q_matrix(end,:));
% for i = 1:steps
%     q_current = sawyer.model.ikcon(position);
%     jacobian  = sawyer.model.jacob0(q_current);
%     desired_V = [0.1 0.1  0  0  0  0]';
%     q_dot     = pinv(jacobian) * desired_V;
%     q_k   = q_current + (q_dot*deltaT);
%     sawyer.model.animate(q_k);
%     drawnow(); 
%     position = sawyer.model.fkine(q_current);
% end

end