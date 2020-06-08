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
% coord = Coordinates();
hdmi_m1 = Cable('HDMIMale(green).ply',[-2.037,0.5146,1.3460]);
hdmi_m2 = Cable('HDMIMale(green).ply',[-2.037,0.5146,1.3460]);
loadEnvironment();

%% Robotic Movement using RMRC  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
movement(sawyer,hdmi_m1,hdmi_m2);




% %Time Intervals
% t = 5;                                                                      % Total time (s)
% deltaT = 0.1;                                                               % Control frequency
% steps = t/deltaT;                                                           % No. of steps for simulation
% 
% %Initialization of q joints. 
% q_intial = zeros(1,7);
% p_start  = sawyer.model.fkine(q_intial);
% q_start  = sawyer.model.ikcon(p_start);
% p_end    = transl([-1.05, -0.4, 1.115])*troty(pi/2);                       %entry coord for first female hdmi
% q_end    = sawyer.model.ikcon(p_end);
% 
% %%
% %Generate Trajectory from q_start to q_end using Trapizoidal Velocity.
% s = lspb(0,1,steps);
% q_matrix = zeros(steps,7);
% for i = 1:steps
%     q_matrix(i,:) = (1-s(i))*q_start + s(i)*q_end;                          %Generate trajectory from q_start to q_end.
%     sawyer.model.animate(q_matrix(i,:));
%     drawnow();
%     objMove = sawyer.model.fkine(q_matrix(i,:));
%     hdmi_m1.move(objMove);
% end
% 
% %%
% %RMRC from entry point to insert point. 
% %Initialise
% q_entry = q_matrix(end,:);
% position = sawyer.model.fkine(q_entry);
%  for i = 1:steps
%     q_current = sawyer.model.ikcon(position);
%     J         = sawyer.model.jacob0(q_current);
%     desired_V = [0.05 0 0  0  0  0 ]';                                      %[vx,vy,vz,ax,ay,az]
%     q_dot     = pinv(J) * desired_V;
%     q_k       = q_current + (q_dot*deltaT)';
%     sawyer.model.animate(q_k);
%     drawnow(); 
%     position  = sawyer.model.fkine(q_k);
%     hdmi_m1.move(position);
%  end

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